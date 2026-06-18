#!/bin/sh
set -e

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
TOOL_DIR="${TOOL_DIR:-$SCRIPT_DIR}"
TOOL="$TOOL_DIR/lme2510_dtmb_libusb"

find_iface() {
  for dev in /sys/bus/usb/devices/*; do
    [ -f "$dev/idVendor" ] || continue
    [ -f "$dev/idProduct" ] || continue
    if [ "$(cat "$dev/idVendor")" = "3344" ] && [ "$(cat "$dev/idProduct")" = "1120" ]; then
      for intf in "$dev":*; do
        [ -e "$intf" ] || continue
        basename "$intf"
        return 0
      done
    fi
  done
  return 1
}

IFACE="${IFACE:-$(find_iface 2>/dev/null || true)}"
IFACE_PATH="/sys/bus/usb/devices/$IFACE"
ORIG_DRIVER=""

if [ -n "$IFACE" ] && [ -L "$IFACE_PATH/driver" ]; then
  ORIG_DRIVER="$(basename "$(readlink "$IFACE_PATH/driver")")"
fi

cleanup() {
  if [ -n "$ORIG_DRIVER" ] && [ -d "/sys/bus/usb/drivers/$ORIG_DRIVER" ]; then
    printf "%s" "$IFACE" > "/sys/bus/usb/drivers/$ORIG_DRIVER/bind" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [ -n "$ORIG_DRIVER" ] && [ "$ORIG_DRIVER" != "usbfs" ] && [ -d "/sys/bus/usb/drivers/$ORIG_DRIVER" ]; then
  printf "%s" "$IFACE" > "/sys/bus/usb/drivers/$ORIG_DRIVER/unbind" 2>/dev/null || true
fi

exec "$TOOL" "$@"
