// SPDX-License-Identifier: GPL-2.0-only
/*
 * libusb user-space controller for LME2510C + LGS8GL5 + MAX2165 DTMB sticks.
 *
 * This is intentionally a libusb tool, not a kernel DVB stack. It mirrors the
 * Linux driver's USB protocol closely enough to upload firmware,
 * initialize the bridge/demod/tuner path, tune a frequency, and capture TS.
 */

#include <errno.h>
#include <inttypes.h>
#include <libusb.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define LME_VID 0x3344
#define LME_PID 0x1120

#define EP_CMD_OUT 0x01
#define EP_FW_OUT  0x01
#define EP_CMD_IN  0x81
#define EP_TS_IN   0x88

#define GL5_SEL     0x32
#define GL5_ALT_SEL 0x36

#define TS_XFER_COUNT 10
#define TS_XFER_SIZE  4096
#define TS_PACKET_SIZE 188
#define TS_SYNC_CONFIRM 5

static int g_verbose = 1;
static uint8_t g_ts_ep = EP_TS_IN;
static bool g_ts_keepalive = true;
static bool g_ts_sync = false;
static volatile sig_atomic_t g_stop;

struct c0_entry {
    uint32_t freq_hz;
    uint8_t cmd[17];
};

static const struct c0_entry c0_table[] = {
    { 586000000U, { 0xC0,0x00,0x30,0x1D,0x55,0x55,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    { 602000000U, { 0xC0,0x00,0x32,0x12,0xAA,0xAA,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    { 626000000U, { 0xC0,0x00,0x34,0x12,0xAA,0xAA,0xB7,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00 } },
    { 468000000U, { 0xC0,0x00,0x27,0xB1,0x3E,0xC1,0xB6,0x8C,0x83,0x2D,0x68,0xB6,0x44,0x3A,0x84,0x8C,0x1E } },
    { 480000000U, { 0xC0,0x00,0x28,0x89,0x00,0x00,0x03,0x03,0x2B,0x96,0x7F,0xF7,0xFF,0xBF,0xFF,0xFF,0xFF } },
    { 492000000U, { 0xC0,0x00,0x29,0x87,0x87,0xB2,0x18,0xA2,0x44,0x46,0x75,0x9E,0xC2,0x59,0xD0,0xB3,0x8E } },
    { 504000000U, { 0xC0,0x00,0x2A,0xF8,0x82,0xBE,0x3A,0xC9,0x21,0x59,0x90,0xA4,0x3C,0x2A,0x48,0x16,0x8D } },
    { 516000000U, { 0xC0,0x00,0x2B,0x96,0xDB,0x7C,0x7B,0xE7,0xCE,0xE6,0xE1,0x31,0xCE,0xEB,0x3C,0x70,0xE9 } },
    { 528000000U, { 0xC0,0x00,0x2C,0x60,0xD6,0xC3,0x33,0x50,0xA8,0x30,0xF2,0x47,0x13,0x8F,0x1F,0x06,0x21 } },
    { 540000000U, { 0xC0,0x00,0x2D,0xED,0x43,0xCD,0x75,0x2E,0x1C,0x45,0x79,0x5B,0x9E,0x01,0xC4,0x4E,0xBC } },
    { 552000000U, { 0xC0,0x00,0x2E,0x4D,0xEE,0xD3,0xD2,0x24,0x41,0xED,0x98,0x8C,0x35,0xF1,0xAC,0x5D,0x4C } },
    { 564000000U, { 0xC0,0x00,0x2F,0x04,0x0F,0xC7,0x10,0x00,0x88,0x4C,0xFB,0x72,0x00,0x9B,0xF8,0x00,0xFE } },
    { 576000000U, { 0xC0,0x00,0x30,0xF7,0x17,0x00,0x00,0x00,0x01,0x08,0x42,0x18,0x47,0xCC,0x67,0x95,0xDC } },
    { 578000000U, { 0xC0,0x00,0x30,0x1D,0x55,0x55,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    { 588000000U, { 0xC0,0x00,0x31,0x09,0xA0,0x13,0x0C,0x3A,0x36,0xA4,0x06,0x20,0x21,0x03,0xD5,0x13,0x00 } },
    { 600000000U, { 0xC0,0x00,0x32,0x60,0x1A,0xD5,0x4A,0x03,0xA7,0xC1,0xEB,0xD1,0x12,0x34,0x8D,0x05,0x93 } },
    { 612000000U, { 0xC0,0x00,0x33,0x8C,0xFC,0xE3,0x5A,0xCE,0x41,0xD4,0x50,0x4C,0xD7,0x23,0xBD,0xDC,0x5A } },
    { 624000000U, { 0xC0,0x00,0x34,0x12,0xAA,0xAA,0xB7,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00 } },
    { 636000000U, { 0xC0,0x00,0x35,0x09,0x9C,0x04,0x2E,0x3A,0x63,0x2B,0x41,0x06,0x6E,0x6C,0xC3,0xC9,0xF2 } },
    { 648000000U, { 0xC0,0x00,0x36,0xA9,0x13,0x5C,0x76,0xD8,0x1D,0x12,0x7C,0x11,0x26,0xA8,0xA0,0x50,0xA0 } },
    { 660000000U, { 0xC0,0x00,0x37,0x54,0x66,0xCD,0x26,0x43,0x0D,0x5C,0xAC,0x8A,0x40,0x8A,0xB9,0x4C,0x5B } },
    { 672000000U, { 0xC0,0x00,0x38,0xD1,0x51,0x3C,0x06,0x50,0x00,0xE0,0x9D,0x00,0x0E,0x49,0xF5,0xF0,0x22 } },
    { 684000000U, { 0xC0,0x00,0x39,0x00,0x73,0xD7,0xD0,0x01,0xF0,0x02,0xCC,0xF7,0x7D,0xF6,0xA0,0x0C,0x08 } },
    { 696000000U, { 0xC0,0x00,0x3A,0x94,0xE7,0x5C,0xB0,0x61,0x4B,0x34,0xEB,0x3F,0xB0,0xF0,0x39,0x64,0x09 } },
    { 708000000U, { 0xC0,0x00,0x3B,0x96,0x01,0x03,0x05,0x1B,0x95,0xFF,0xAD,0xA1,0x00,0x00,0x21,0xB3,0x2D } },
    { 720000000U, { 0xC0,0x00,0x3C,0x08,0x4B,0xBE,0x40,0x03,0xB8,0x00,0x4A,0x9A,0x00,0x1D,0x53,0xEF,0xBF } },
    { 732000000U, { 0xC0,0x00,0x3D,0x47,0x13,0x92,0x3D,0x19,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF } },
    { 744000000U, { 0xC0,0x00,0x3E,0x2F,0x12,0x03,0xE0,0x00,0x00,0x01,0x08,0x12,0x10,0x4C,0x97,0x23,0xFE } },
    { 756000000U, { 0xC0,0x00,0x3F,0x78,0x01,0x40,0x0C,0x40,0x47,0x13,0x89,0x1B,0x32,0x2C,0x34,0x9B,0x83 } },
    { 768000000U, { 0xC0,0x00,0x40,0x18,0x00,0xD0,0xA2,0x58,0x60,0x14,0x02,0x80,0x51,0x04,0xC2,0x12,0x51 } },
    { 780000000U, { 0xC0,0x00,0x41,0x40,0x25,0x64,0x00,0x7A,0x0C,0x00,0x02,0xC5,0x00,0x5D,0x50,0xFF,0x48 } },
    { 792000000U, { 0xC0,0x00,0x42,0x50,0x03,0xC2,0x68,0x06,0x04,0xD0,0x0C,0x89,0xBC,0x06,0x24,0x30,0xD4 } },
    { 804000000U, { 0xC0,0x00,0x43,0x47,0x46,0x64,0x0A,0x00,0x36,0x69,0x00,0x13,0xC1,0xDE,0x90,0x13,0x7F } },
    { 816000000U, { 0xC0,0x00,0x44,0x90,0x28,0x01,0xAE,0x01,0x80,0x02,0x82,0xC0,0x32,0x26,0x94,0x90,0x0C } },
    { 828000000U, { 0xC0,0x00,0x45,0x68,0x22,0x7F,0x91,0x1C,0x02,0x02,0x54,0x46,0x47,0x13,0x8B,0x1F,0x96 } },
    { 840000000U, { 0xC0,0x00,0x46,0x00,0xB0,0x57,0x20,0x0F,0xB9,0xE5,0xD2,0xC7,0xFC,0x3A,0x6B,0x95,0x12 } },
    { 852000000U, { 0xC0,0x00,0x47,0x13,0x8B,0x12,0x7C,0x9A,0xAA,0x2E,0x63,0x36,0x8F,0x93,0x4E,0x23,0x35 } },
    { 864000000U, { 0xC0,0x00,0x48,0x08,0x1F,0x5C,0x00,0xC8,0x00,0x8F,0xDC,0x02,0xAE,0x25,0x75,0xD1,0x78 } },
};

struct dev {
    libusb_context *ctx;
    libusb_device_handle *h;
    uint8_t gl5_hint;
};

struct stream {
    struct dev *d;
    FILE *out;
    uint64_t bytes;
    uint64_t packets47;
    int seconds;
    int active;
    struct timeval deadline;
    struct libusb_transfer *xfers[TS_XFER_COUNT];
    uint8_t *bufs[TS_XFER_COUNT];
};

struct ts_aligner {
    uint8_t buf[TS_XFER_SIZE + TS_PACKET_SIZE * (TS_SYNC_CONFIRM + 2)];
    size_t len;
    bool locked;
    uint64_t written_packets;
    uint64_t dropped_bytes;
    uint64_t relocks;
};

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void msleep(unsigned ms)
{
    usleep(ms * 1000U);
}

static const char *usb_err(int r)
{
    return libusb_error_name(r);
}

static void dump_hex(const char *prefix, const uint8_t *buf, int len)
{
    int n = len < 24 ? len : 24;
    fprintf(stderr, "%s", prefix);
    for (int i = 0; i < n; i++)
        fprintf(stderr, " %02x", buf[i]);
    if (len > n)
        fprintf(stderr, " ...");
    fprintf(stderr, "\n");
}

static int bulk_xfer(struct dev *d, uint8_t ep, uint8_t *buf, int len,
                     int timeout_ms, const char *tag)
{
    int actual = 0;
    int r = libusb_bulk_transfer(d->h, ep, buf, len, &actual, timeout_ms);
    if (g_verbose > 1 || r != 0) {
        fprintf(stderr, "%s ep=0x%02x len=%d actual=%d ret=%s\n",
                tag, ep, len, actual, usb_err(r));
        if (g_verbose > 2 && actual > 0)
            dump_hex("  data:", buf, actual);
    }
    if (r == 0 && actual != len && (ep & LIBUSB_ENDPOINT_OUT))
        return LIBUSB_ERROR_IO;
    return r;
}

static int talk(struct dev *d, const uint8_t *w, int wlen, uint8_t *rbuf, int rlen,
                int timeout_ms, const char *tag)
{
    uint8_t tmp[256];
    int r;

    if (wlen > (int)sizeof(tmp) || rlen > (int)sizeof(tmp))
        return LIBUSB_ERROR_INVALID_PARAM;
    memcpy(tmp, w, wlen);
    if (g_verbose > 1)
        dump_hex("OUT:", w, wlen);
    r = bulk_xfer(d, EP_CMD_OUT, tmp, wlen, timeout_ms, tag);
    if (r != 0)
        return r;
    if (rlen <= 0)
        return 0;
    memset(tmp, 0, rlen);
    r = bulk_xfer(d, EP_CMD_IN, tmp, rlen, timeout_ms, tag);
    if (rbuf)
        memcpy(rbuf, tmp, rlen);
    if (g_verbose > 1)
        dump_hex(" IN:", tmp, rlen);
    return r;
}

static int fw_talk(struct dev *d, const uint8_t *w, int wlen, uint8_t *rbuf,
                   int rlen, int timeout_ms, const char *tag)
{
    uint8_t tmp[256];
    int r;

    memcpy(tmp, w, wlen);
    r = bulk_xfer(d, EP_FW_OUT, tmp, wlen, 500, tag);
    if (r != 0)
        return r;
    if (rlen <= 0)
        return 0;
    memset(tmp, 0, rlen);
    r = bulk_xfer(d, EP_CMD_IN, tmp, rlen, timeout_ms, tag);
    if (r == LIBUSB_ERROR_TIMEOUT) {
        if (rbuf)
            memset(rbuf, 0, rlen);
        return 0;
    }
    if (rbuf)
        memcpy(rbuf, tmp, rlen);
    return r;
}

static int set_alt1(struct dev *d)
{
    int r = libusb_set_interface_alt_setting(d->h, 0, 1);
    if (r != 0)
        fprintf(stderr, "set alt1 ret=%s\n", usb_err(r));
    return r;
}

static int open_dev(struct dev *d)
{
    int r;

    memset(d, 0, sizeof(*d));
    r = libusb_init(&d->ctx);
    if (r != 0)
        return r;

    d->h = libusb_open_device_with_vid_pid(d->ctx, LME_VID, LME_PID);
    if (!d->h)
        return LIBUSB_ERROR_NO_DEVICE;

    libusb_set_auto_detach_kernel_driver(d->h, 1);
    r = libusb_set_configuration(d->h, 1);
    if (r != 0 && r != LIBUSB_ERROR_BUSY && r != LIBUSB_ERROR_NOT_SUPPORTED)
        fprintf(stderr, "set configuration ret=%s\n", usb_err(r));

    r = libusb_claim_interface(d->h, 0);
    if (r != 0) {
        fprintf(stderr, "claim interface ret=%s\n", usb_err(r));
        return r;
    }
    d->gl5_hint = 0;
    return 0;
}

static void close_dev(struct dev *d)
{
    if (d->h) {
        libusb_release_interface(d->h, 0);
        libusb_close(d->h);
    }
    if (d->ctx)
        libusb_exit(d->ctx);
    memset(d, 0, sizeof(*d));
}

static int wait_reopen(struct dev *d, int seconds)
{
    int r = LIBUSB_ERROR_NO_DEVICE;
    for (int i = 0; i < seconds * 2 && !g_stop; i++) {
        r = open_dev(d);
        if (r == 0) {
            set_alt1(d);
            return 0;
        }
        msleep(500);
    }
    return r;
}

static uint8_t cksum(const uint8_t *p, int len)
{
    uint8_t s = 0;
    for (int i = 0; i < len; i++)
        s = (uint8_t)(s + p[i]);
    return s;
}

static int read_file(const char *path, uint8_t **data, size_t *size)
{
    FILE *f = fopen(path, "rb");
    long n;
    uint8_t *buf;

    if (!f) {
        perror(path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n <= 0 || n > 65535) {
        fclose(f);
        return -1;
    }
    buf = malloc((size_t)n);
    if (!buf) {
        fclose(f);
        return -1;
    }
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) {
        free(buf);
        fclose(f);
        return -1;
    }
    fclose(f);
    *data = buf;
    *size = (size_t)n;
    return 0;
}

static int upload_fw_data(struct dev *d, const uint8_t *data, uint16_t size)
{
    uint8_t buf[128], rb[4] = {0};
    const uint8_t pkt = 0x31;
    int ret = 0;

    set_alt1(d);
    for (int seg = 1; seg <= 2; seg++) {
        uint16_t start = seg == 1 ? 0 : 500;
        uint16_t end = seg == 1 ? 500 : size;

        for (uint16_t i = start; i < end; ) {
            uint8_t dlen = (uint8_t)(((end - i) > pkt) ? pkt : (end - i - 1));
            const uint8_t *src = data + i;
            buf[0] = (uint8_t)seg;
            buf[1] = dlen;
            memcpy(&buf[2], src, dlen + 1);
            buf[dlen + 3] = cksum(src, dlen + 1);
            ret = fw_talk(d, buf, dlen + 4, rb, 1, 200, "fw");
            if (ret == LIBUSB_ERROR_TIMEOUT)
                ret = 0;
            if (ret != 0) {
                fprintf(stderr, "firmware packet @%u ret=%s ack=%02x\n",
                        i, usb_err(ret), rb[0]);
                return ret;
            }
            if (rb[0] && rb[0] != 0x88 && rb[0] != 0x77 && rb[0] != 0x44)
                fprintf(stderr, "firmware packet @%u odd ack=%02x\n", i, rb[0]);
            i = (uint16_t)(i + dlen + 1);
            if (seg == 1 && i < 500)
                usleep(500);
        }

        if (seg == 1) {
            uint8_t cmd81[] = {0x81,0x0b,0,0,0,0,0,0,0,0,0,0,0,0,0};
            talk(d, cmd81, sizeof(cmd81), rb, 1, 1500, "cmd81");
        }
    }
    return 0;
}

static int upload_firmware(struct dev *d, const char *path)
{
    uint8_t *fw = NULL, rb[8] = {0};
    size_t fw_size = 0;
    int ret;
    uint8_t cmd82[] = {
        0x82,0x23,0xe0,0x54,0x02,0x70,0x02,0xd3,
        0x22,0xc3,0x22,0xe4,0xf5,0x90,0xc2,0x88,
        0xd2,0xa8,0xd2,0xaf,0x22,0x7d,0xfc,0xe4,
        0xff,0x12,0x11,0xe8,0xc2,0x9c,0x22,0x90,
        0x90,0x21,0x74,0x08,0xf0,0x22,0x08
    };
    uint8_t cmd8a[] = {0x8a, 0x00};

    if (read_file(path, &fw, &fw_size) != 0)
        return -1;
    fprintf(stderr, "upload firmware %s (%zu bytes)\n", path, fw_size);
    ret = upload_fw_data(d, fw, (uint16_t)fw_size);
    free(fw);
    if (ret != 0)
        return ret;

    talk(d, cmd82, sizeof(cmd82), rb, 1, 1500, "cmd82");
    ret = talk(d, cmd8a, sizeof(cmd8a), rb, 5, 1500, "cmd8a");
    fprintf(stderr, "8a ack: %02x %02x %02x %02x %02x ret=%s\n",
            rb[0], rb[1], rb[2], rb[3], rb[4], usb_err(ret));
    msleep(300);
    return ret;
}

static int __attribute__((unused)) cmd09(struct dev *d)
{
    uint8_t w[] = {0x09, 0x00}, rb[5] = {0};
    int r = talk(d, w, 2, rb, sizeof(rb), 500, "cmd09");
    fprintf(stderr, "cmd09 ack=%02x ret=%s\n", rb[0], usb_err(r));
    return r;
}

static int __attribute__((unused)) cmd16(struct dev *d, uint8_t mode)
{
    uint8_t w[] = {0x16, 0x01, mode}, rb[5] = {0};
    int r = talk(d, w, sizeof(w), rb, 2, 1500, "cmd16");
    fprintf(stderr, "cmd16 ack=%02x ret=%s\n", rb[0], usb_err(r));
    return r;
}

static int cmd(struct dev *d, const uint8_t *w, int wlen, int rlen, const char *tag)
{
    uint8_t rb[16] = {0};
    int r = talk(d, w, wlen, rb, rlen, 1500, tag);
    fprintf(stderr, "%-18s ack=%02x %02x ret=%s\n", tag, rb[0], rb[1], usb_err(r));
    if (wlen == 5 && w[0] == 0x05 && w[2] == GL5_SEL)
        d->gl5_hint = w[4];
    return r;
}

static int read85(struct dev *d, uint8_t sel, uint8_t reg, uint8_t hint,
                  const char *tag, uint8_t *val)
{
    uint8_t w[] = {0x85, 0x02, sel, reg, hint};
    uint8_t rb[8] = {0};
    int r = talk(d, w, sizeof(w), rb, 2, 1500, tag);
    uint8_t v = (rb[0] == 0x55 || rb[0] == 0x88) ? rb[1] : rb[0];
    if (val)
        *val = v;
    fprintf(stderr, "%-18s 85 02 %02x %02x %02x -> %02x %02x pick=%02x ret=%s\n",
            tag, sel, reg, hint, rb[0], rb[1], v, usb_err(r));
    return r;
}

static int read84(struct dev *d, uint8_t sel, uint8_t reg, uint8_t suffix,
                  const char *tag, uint8_t *val)
{
    uint8_t w[] = {0x84, 0x03, sel, reg, suffix};
    uint8_t rb[8] = {0};
    int r = talk(d, w, sizeof(w), rb, 2, 1500, tag);
    uint8_t v = rb[0] == 0x55 ? rb[1] : rb[0];
    if (val)
        *val = v;
    fprintf(stderr, "%-18s 84 03 %02x %02x %02x -> %02x %02x pick=%02x ret=%s\n",
            tag, sel, reg, suffix, rb[0], rb[1], v, usb_err(r));
    return r;
}

static int gl5_w(struct dev *d, uint8_t reg, uint8_t val)
{
    uint8_t w[] = {0x05, 0x04, GL5_SEL, reg, val};
    int r = cmd(d, w, sizeof(w), 4, "gl5_w");
    d->gl5_hint = val;
    return r;
}

static void __attribute__((unused)) gl5_w3(struct dev *d, uint8_t reg, uint8_t val)
{
    for (int i = 0; i < 3; i++)
        gl5_w(d, reg, val);
}

static int __attribute__((unused)) gl5_alt_w(struct dev *d, uint8_t reg, uint8_t val)
{
    uint8_t w[] = {0x04, 0x03, GL5_ALT_SEL, reg, val};
    return cmd(d, w, sizeof(w), 4, "gl5_alt_w");
}

static int gl5_read(struct dev *d, uint8_t reg, uint8_t *val)
{
    return read85(d, GL5_SEL, reg, d->gl5_hint, "gl5_read", val);
}

static bool hw_locked(struct dev *d)
{
    uint8_t v4b = 0, va4 = 0;
    gl5_read(d, 0x4b, &v4b);
    gl5_read(d, 0xa4, &va4);
    return (v4b & 0x80) && (va4 & 0x01);
}

static int win3_probe_init(struct dev *d)
{
    uint8_t id = 0, reset = 0;
    static const uint8_t cmd16_0[] = {0x16,0x01,0x00};
    static const uint8_t w0200[] = {0x05,0x04,0x32,0x02,0x00};
    static const uint8_t w0201[] = {0x05,0x04,0x32,0x02,0x01};
    static const uint8_t w01e0[] = {0x05,0x04,0x32,0x01,0xe0};
    static const uint8_t w0160[] = {0x05,0x04,0x32,0x01,0x60};
    static const uint8_t w0d01[] = {0x04,0x03,0xc0,0x0d,0x01};
    static const uint8_t w0d02[] = {0x04,0x03,0xc0,0x0d,0x02};
    static const uint8_t w0d03[] = {0x04,0x03,0xc0,0x0d,0x03};
    static const uint8_t w0d04[] = {0x04,0x03,0xc0,0x0d,0x04};
    static const uint8_t w0d05[] = {0x04,0x03,0xc0,0x0d,0x05};
    static const uint8_t w0d00[] = {0x04,0x03,0xc0,0x0d,0x00};
    static const uint8_t wblob[] = {
        0x04,0x11,0xc0,0x00,0x27,0x18,0x00,0x00,0xf2,
        0x01,0x0a,0x08,0x02,0x54,0x73,0x75,0x00,0x00,0x00
    };
    static const uint8_t w079f[] = {0x05,0x04,0x32,0x07,0x9f};
    static const uint8_t w0900[] = {0x05,0x04,0x32,0x09,0x00};
    static const uint8_t w0a00[] = {0x05,0x04,0x32,0x0a,0x00};
    static const uint8_t w0b00[] = {0x05,0x04,0x32,0x0b,0x00};
    static const uint8_t w0c00[] = {0x05,0x04,0x32,0x0c,0x00};
    static const uint8_t w071c[] = {0x05,0x04,0x32,0x07,0x1c};

    fprintf(stderr, "=== 3csv strict probe init ===\n");
    read85(d, 0x32, 0x00, 0x00, "chip#1", &id);
    cmd(d, cmd16_0, sizeof(cmd16_0), 2, "cmd16");
    read85(d, 0x32, 0x00, 0x00, "chip#2", &id);
    read85(d, 0x32, 0x02, 0x00, "reset", &reset);
    cmd(d, w0200, sizeof(w0200), 1, "32.02=00");
    cmd(d, w0201, sizeof(w0201), 1, "32.02=01");
    msleep(12);
    cmd(d, w01e0, sizeof(w01e0), 1, "32.01=e0");
    cmd(d, w0d01, sizeof(w0d01), 1, "c0.0d=01");
    read84(d, 0xc0, 0x10, 0x01, "c0.10#1", NULL);
    cmd(d, w0d02, sizeof(w0d02), 1, "c0.0d=02");
    read84(d, 0xc0, 0x10, 0x01, "c0.10#2", NULL);
    cmd(d, w0d03, sizeof(w0d03), 1, "c0.0d=03");
    read84(d, 0xc0, 0x10, 0x01, "c0.10#3", NULL);
    cmd(d, w0d04, sizeof(w0d04), 1, "c0.0d=04");
    read84(d, 0xc0, 0x10, 0x01, "c0.10#4", NULL);
    cmd(d, w0d05, sizeof(w0d05), 1, "c0.0d=05");
    read84(d, 0xc0, 0x10, 0x01, "c0.10#5", NULL);
    cmd(d, w0d00, sizeof(w0d00), 1, "c0.0d=00");
    cmd(d, wblob, sizeof(wblob), 1, "c0 blob");
    cmd(d, w0160, sizeof(w0160), 1, "32.01=60");
    read85(d, 0x32, 0x07, 0x60, "32.07 read", NULL);
    cmd(d, w079f, sizeof(w079f), 1, "32.07=9f");
    cmd(d, w0900, sizeof(w0900), 1, "32.09=00");
    cmd(d, w0a00, sizeof(w0a00), 1, "32.0a=00");
    cmd(d, w0b00, sizeof(w0b00), 1, "32.0b=00");
    cmd(d, w0c00, sizeof(w0c00), 1, "32.0c=00");
    read85(d, 0x32, 0x07, 0x00, "32.07 verify", NULL);
    cmd(d, w071c, sizeof(w071c), 1, "32.07=1c");
    fprintf(stderr, "probe init done chip=%02x reset=%02x\n", id, reset);
    return 0;
}

static void gl5_init(struct dev *d)
{
    win3_probe_init(d);
}

static const struct c0_entry *find_c0(uint32_t freq_hz)
{
    const struct c0_entry *best = NULL;
    uint32_t best_diff = UINT32_MAX;
    for (size_t i = 0; i < sizeof(c0_table) / sizeof(c0_table[0]); i++) {
        uint32_t f = c0_table[i].freq_hz;
        uint32_t diff = f > freq_hz ? f - freq_hz : freq_hz - f;
        if (diff < best_diff) {
            best = &c0_table[i];
            best_diff = diff;
        }
    }
    return best;
}

static void ftm_post_c0(struct dev *d, uint32_t freq_hz)
{
    uint8_t val = 0;
    bool use_short_602 = freq_hz == 578000000U || freq_hz == 586000000U ||
                         freq_hz == 602000000U;
    static const uint8_t w0200[] = {0x05,0x04,0x32,0x02,0x00};
    static const uint8_t w0201[] = {0x05,0x04,0x32,0x02,0x01};
    static const uint8_t w0160[] = {0x05,0x04,0x32,0x01,0x60};
    static const uint8_t w0300[] = {0x05,0x04,0x32,0x03,0x00};
    static const uint8_t w7e01[] = {0x05,0x04,0x32,0x7e,0x01};
    static const uint8_t wc500[] = {0x05,0x04,0x36,0xc5,0x00};
    static const uint8_t w0402[] = {0x05,0x04,0x32,0x04,0x02};
    static const uint8_t w0400[] = {0x05,0x04,0x32,0x04,0x00};
    static const uint8_t w3701[] = {0x05,0x04,0x32,0x37,0x01};
    static const uint8_t w7e00[] = {0x05,0x04,0x32,0x7e,0x00};
    static const uint8_t wc506[] = {0x05,0x04,0x36,0xc5,0x06};
    static const uint8_t w7d71[] = {0x05,0x04,0x32,0x7d,0x71};

    fprintf(stderr, "=== Windows FTM post-C0 ===\n");
    cmd(d, w0160, sizeof(w0160), 1, "ftm 01=60");
    read85(d, 0x32, 0x03, 0x60, "ftm 03 rd", &val);
    cmd(d, w0300, sizeof(w0300), 1, "ftm 03=00");
    read85(d, 0x32, 0x7e, 0x00, "ftm 7e rd", &val);
    cmd(d, w7e01, sizeof(w7e01), 1, "ftm 7e=01");
    read85(d, 0x36, 0xc5, 0x01, "ftm 36.c5 rd", &val);
    cmd(d, wc500, sizeof(wc500), 1, "ftm 36.c5=00");
    read85(d, 0x32, 0x00, 0x00, "ftm chip#1", NULL);
    read85(d, 0x32, 0x02, 0x00, "ftm rst#1", NULL);
    cmd(d, w0200, sizeof(w0200), 1, "ftm 02=00");
    cmd(d, w0201, sizeof(w0201), 1, "ftm 02=01");
    read85(d, 0x32, 0x04, 0x01, "ftm 04 rd", NULL);
    read85(d, 0x32, 0x37, 0x01, "ftm 37 rd", NULL);
    cmd(d, w0402, sizeof(w0402), 1, "ftm 04=02");
    cmd(d, w3701, sizeof(w3701), 1, "ftm 37=01");
    read85(d, 0x32, 0x00, 0x01, "ftm chip#2", NULL);
    read85(d, 0x32, 0x02, 0x01, "ftm rst#2", NULL);
    cmd(d, w0200, sizeof(w0200), 1, "ftm 02=00");
    cmd(d, w0201, sizeof(w0201), 1, "ftm 02=01");
    if (use_short_602) {
        fprintf(stderr, "ftm path: 602 short lock path\n");
        msleep(37);
        read85(d, 0x32, 0x4b, 0x01, "ftm 4b#1", &val);
        msleep(32);
        read85(d, 0x32, 0x4b, 0x01, "ftm 4b#2", &val);
        msleep(21);
        read85(d, 0x32, 0xa4, 0x01, "ftm a4#1", &val);
        msleep(21);
        read85(d, 0x32, 0xa4, 0x01, "ftm a4#2", &val);
        msleep(22);
        read85(d, 0x32, 0xa4, 0x01, "ftm a4#3", &val);
    } else {
        fprintf(stderr, "ftm path: full 626-style lock path\n");
        for (int i = 0; i < 19; i++) {
            char tag[24];
            msleep(i == 0 ? 37 : 32);
            snprintf(tag, sizeof(tag), "ftm 4b pre#%02d", i + 1);
            read85(d, 0x32, 0x4b, 0x01, tag, &val);
        }
        read85(d, 0x32, 0x04, 0x01, "ftm 04 mid rd", NULL);
        read85(d, 0x32, 0x37, 0x01, "ftm 37 mid rd", NULL);
        cmd(d, w0400, sizeof(w0400), 1, "ftm 04=00");
        cmd(d, w3701, sizeof(w3701), 1, "ftm 37=01 mid");
        read85(d, 0x32, 0x00, 0x01, "ftm chip#3", NULL);
        read85(d, 0x32, 0x02, 0x01, "ftm rst#3", NULL);
        cmd(d, w0200, sizeof(w0200), 1, "ftm 02=00");
        cmd(d, w0201, sizeof(w0201), 1, "ftm 02=01");
        msleep(32);
        read85(d, 0x32, 0x4b, 0x01, "ftm 4b lock", &val);
        for (int i = 0; i < 4; i++) {
            char tag[24];
            msleep(i == 0 ? 1 : 22);
            snprintf(tag, sizeof(tag), "ftm a4#%d", i + 1);
            read85(d, 0x32, 0xa4, 0x01, tag, &val);
        }
    }
    read85(d, 0x32, 0xa2, 0x01, "ftm a2", &val);
    read85(d, 0x32, 0x7e, 0x01, "ftm 7e final", NULL);
    cmd(d, w7e00, sizeof(w7e00), 1, "ftm 7e=00");
    read85(d, 0x36, 0xc5, 0x00, "ftm 36.c5 fin", NULL);
    cmd(d, wc506, sizeof(wc506), 1, "ftm 36.c5=06");
    read85(d, 0x32, 0x00, 0x06, "ftm chip#4", NULL);
    read85(d, 0x32, 0x02, 0x06, "ftm rst#4", NULL);
    cmd(d, w0200, sizeof(w0200), 1, "ftm 02=00");
    cmd(d, w0201, sizeof(w0201), 1, "ftm 02=01");
    cmd(d, w7d71, sizeof(w7d71), 1, "ftm 7d=71");
    read85(d, 0x32, 0x00, 0x71, "ftm chip#5", NULL);
    read85(d, 0x32, 0x02, 0x71, "ftm rst#5", NULL);
    cmd(d, w0200, sizeof(w0200), 1, "ftm 02=00");
    cmd(d, w0201, sizeof(w0201), 1, "ftm 02=01");
}

static int tune_c0(struct dev *d, uint32_t freq_hz)
{
    const struct c0_entry *e = find_c0(freq_hz);
    uint8_t w0200[] = {0x05,0x04,0x32,0x02,0x00};
    uint8_t w0201[] = {0x05,0x04,0x32,0x02,0x01};
    uint8_t w01e0[] = {0x05,0x04,0x32,0x01,0xe0};
    uint8_t c0[9], wc00a[] = {0x04,0x03,0xc0,0x0a,0x73};
    uint8_t rd84[] = {0x84,0x03,0xc0,0x04,0x01};
    uint8_t rb[4] = {0};
    uint8_t wc004[5];

    if (!e)
        return -1;
    fprintf(stderr, "C0 tune %u Hz using table %u Hz\n", freq_hz, e->freq_hz);
    msleep(7547);
    read85(d, 0x32, 0x00, 0x1c, "pre-c0 chip", NULL);
    read85(d, 0x32, 0x02, 0x1c, "pre-c0 reset", NULL);
    cmd(d, w0200, sizeof(w0200), 1, "pre-c0 02=00");
    cmd(d, w0201, sizeof(w0201), 1, "pre-c0 02=01");
    cmd(d, w01e0, sizeof(w01e0), 1, "32.01=e0 pre-c0");
    c0[0] = 0x04;
    c0[1] = 0x07;
    memcpy(&c0[2], e->cmd, 7);
    cmd(d, c0, sizeof(c0), 1, "c0 tune");
    cmd(d, wc00a, sizeof(wc00a), 1, "c0.0a=73");
    talk(d, rd84, sizeof(rd84), rb, 2, 1500, "c0 TF read");
    fprintf(stderr, "c0 TF pre-read raw=%02x %02x table=%02x\n", rb[0], rb[1], e->cmd[6]);
    wc004[0] = 0x04;
    wc004[1] = 0x03;
    wc004[2] = 0xc0;
    wc004[3] = 0x04;
    wc004[4] = (uint8_t)(e->cmd[6] | 0x40);
    cmd(d, wc004, sizeof(wc004), 1, "c0.04=tf|40");
    ftm_post_c0(d, freq_hz);
    msleep(80);
    return 0;
}

static int csv_stream_handshake(struct dev *d)
{
    uint8_t all_pids[] = {0x03,0x06,0x00,0xff,0x01,0x1f,0x20,0x81};
    uint8_t stream_on[] = {0x06,0x00};
    uint8_t rb[8] = {0};
    int r;

    r = talk(d, all_pids, sizeof(all_pids), rb, 1, 1500, "all_pids#1");
    fprintf(stderr, "all_pids#1 ack=%02x ret=%s\n", rb[0], usb_err(r));
    memset(rb, 0, sizeof(rb));
    r = talk(d, all_pids, sizeof(all_pids), rb, 1, 1500, "all_pids#2");
    fprintf(stderr, "all_pids#2 ack=%02x ret=%s\n", rb[0], usb_err(r));
    memset(rb, 0, sizeof(rb));
    r = talk(d, stream_on, sizeof(stream_on), rb, 1, 1500, "06_00");
    fprintf(stderr, "06_00 ack=%02x ret=%s\n", rb[0], usb_err(r));
    return r;
}

static void read_lock_status(struct dev *d, const char *tag)
{
    uint8_t v4b = 0, va4 = 0, va2 = 0, v7d = 0;

    read85(d, 0x32, 0x4b, 0x01, "lock 4b", &v4b);
    read85(d, 0x32, 0xa4, 0x01, "lock a4", &va4);
    read85(d, 0x32, 0xa2, 0x01, "lock a2", &va2);
    read85(d, 0x32, 0x7d, 0x01, "lock 7d", &v7d);
    fprintf(stderr, "%s: 4b=%02x a4=%02x a2=%02x 7d=%02x locked=%s\n",
            tag, v4b, va4, va2, v7d,
            ((v4b & 0x80) && (va4 & 0x01)) ? "yes" : "no");
}

static void lock_test(struct dev *d, uint32_t freq_hz)
{
    gl5_init(d);
    tune_c0(d, freq_hz);
    read_lock_status(d, "lock test");
}

static void ts_cb(struct libusb_transfer *xfer)
{
    struct stream *s = xfer->user_data;

    if (xfer->status == LIBUSB_TRANSFER_COMPLETED && xfer->actual_length > 0) {
        fwrite(xfer->buffer, 1, (size_t)xfer->actual_length, s->out);
        s->bytes += (uint64_t)xfer->actual_length;
        for (int i = 0; i < xfer->actual_length; i += 188) {
            if (xfer->buffer[i] == 0x47)
                s->packets47++;
        }
    }

    if (!s->active || g_stop)
        return;
    int r = libusb_submit_transfer(xfer);
    if (r != 0)
        fprintf(stderr, "TS resubmit ret=%s\n", usb_err(r));
}

static int stream_start(struct stream *s, struct dev *d, const char *path, int seconds)
{
    memset(s, 0, sizeof(*s));
    s->d = d;
    s->seconds = seconds;
    s->out = !strcmp(path, "-") ? stdout : fopen(path, "wb");
    if (!s->out) {
        perror(path);
        return -1;
    }
    setvbuf(s->out, NULL, _IONBF, 0);
    gettimeofday(&s->deadline, NULL);
    s->deadline.tv_sec += seconds;
    s->active = 1;

    set_alt1(d);
    fprintf(stderr, "starting TS on ep=0x%02x for %d seconds\n", g_ts_ep, seconds);
    for (int i = 0; i < TS_XFER_COUNT; i++) {
        s->bufs[i] = malloc(TS_XFER_SIZE);
        s->xfers[i] = libusb_alloc_transfer(0);
        if (!s->bufs[i] || !s->xfers[i])
            return -1;
        libusb_fill_bulk_transfer(s->xfers[i], d->h, g_ts_ep, s->bufs[i],
                                  TS_XFER_SIZE, ts_cb, s, 1500);
        int r = libusb_submit_transfer(s->xfers[i]);
        if (r != 0)
            fprintf(stderr, "TS submit[%d] ret=%s\n", i, usb_err(r));
    }
    return 0;
}

static bool ts_confirm_sync(const uint8_t *buf, size_t len, size_t off)
{
    for (int i = 0; i < TS_SYNC_CONFIRM; i++) {
        size_t pos = off + (size_t)i * TS_PACKET_SIZE;
        if (pos >= len || buf[pos] != 0x47)
            return false;
    }
    return true;
}

static void ts_aligner_push(struct ts_aligner *a, FILE *out,
                            const uint8_t *data, size_t data_len)
{
    if (data_len > sizeof(a->buf)) {
        data += data_len - sizeof(a->buf);
        data_len = sizeof(a->buf);
    }
    if (a->len + data_len > sizeof(a->buf)) {
        size_t drop = a->len + data_len - sizeof(a->buf);
        memmove(a->buf, a->buf + drop, a->len - drop);
        a->len -= drop;
        a->dropped_bytes += drop;
        a->locked = false;
    }
    memcpy(a->buf + a->len, data, data_len);
    a->len += data_len;

    for (;;) {
        if (!a->locked) {
            size_t off;
            for (off = 0; off < a->len; off++) {
                if (a->buf[off] == 0x47 &&
                    ts_confirm_sync(a->buf, a->len, off))
                    break;
            }
            if (off == a->len) {
                size_t keep = TS_PACKET_SIZE * (TS_SYNC_CONFIRM - 1);
                if (a->len > keep) {
                    a->dropped_bytes += a->len - keep;
                    memmove(a->buf, a->buf + a->len - keep, keep);
                    a->len = keep;
                }
                return;
            }
            if (off > 0) {
                a->dropped_bytes += off;
                memmove(a->buf, a->buf + off, a->len - off);
                a->len -= off;
            }
            a->locked = true;
            a->relocks++;
        }

        while (a->len >= TS_PACKET_SIZE) {
            if (a->buf[0] != 0x47) {
                a->locked = false;
                break;
            }
            fwrite(a->buf, 1, TS_PACKET_SIZE, out);
            a->written_packets++;
            memmove(a->buf, a->buf + TS_PACKET_SIZE, a->len - TS_PACKET_SIZE);
            a->len -= TS_PACKET_SIZE;
        }
        if (a->locked)
            return;
    }
}

static int stream_sync(struct dev *d, const char *path, int seconds)
{
    FILE *out = !strcmp(path, "-") ? stdout : fopen(path, "wb");
    struct timeval start, now;
    struct ts_aligner aligner;
    uint8_t *buf;
    int ret = 0;

    if (!out) {
        perror(path);
        return -1;
    }
    setvbuf(out, NULL, _IONBF, 0);
    buf = malloc(TS_XFER_SIZE);
    if (!buf) {
        fclose(out);
        return -1;
    }
    memset(&aligner, 0, sizeof(aligner));

    set_alt1(d);
    msleep(320);
    gettimeofday(&start, NULL);
    while (!g_stop) {
        gettimeofday(&now, NULL);
        if (seconds > 0 && now.tv_sec - start.tv_sec >= seconds)
            break;
        memset(buf, 0, TS_XFER_SIZE);
        ret = bulk_xfer(d, g_ts_ep, buf, TS_XFER_SIZE, 1500, "ts sync");
        if (ret == 0) {
            ts_aligner_push(&aligner, out, buf, TS_XFER_SIZE);
            if (g_verbose > 0)
                fprintf(stderr, "ts sync packets=%" PRIu64 " dropped=%" PRIu64 " relocks=%" PRIu64 "\n",
                        aligner.written_packets, aligner.dropped_bytes,
                        aligner.relocks);
        } else if (ret != LIBUSB_ERROR_TIMEOUT) {
            fprintf(stderr, "ts sync ret=%s\n", usb_err(ret));
            break;
        }
    }

    fprintf(stderr, "ts aligned packets=%" PRIu64 " dropped=%" PRIu64 " relocks=%" PRIu64 "\n",
            aligner.written_packets, aligner.dropped_bytes, aligner.relocks);
    free(buf);
    if (out != stdout)
        fclose(out);
    return ret;
}

static void stream_loop(struct stream *s)
{
    struct timeval next_keepalive;
    bool keepalive_sent = false;

    gettimeofday(&next_keepalive, NULL);
    next_keepalive.tv_usec += 500000;
    if (next_keepalive.tv_usec >= 1000000) {
        next_keepalive.tv_sec++;
        next_keepalive.tv_usec -= 1000000;
    }
    while (!g_stop && s->active) {
        struct timeval now, tv = {0, 200000};
        int completed = 0;
        libusb_handle_events_timeout_completed(s->d->ctx, &tv, &completed);
        gettimeofday(&now, NULL);
        if (g_ts_keepalive &&
            (now.tv_sec > next_keepalive.tv_sec ||
             (now.tv_sec == next_keepalive.tv_sec &&
              now.tv_usec >= next_keepalive.tv_usec))) {
            if (!keepalive_sent) {
                uint8_t all_pids[] = {0x03,0x06,0x00,0xff,0x01,0x1f,0x20,0x81};
                uint8_t stream_on[] = {0x06,0x00};
                uint8_t rb[8] = {0};
                talk(s->d, all_pids, sizeof(all_pids), rb, 1, 1500, "ts keepalive#1");
                memset(rb, 0, sizeof(rb));
                talk(s->d, all_pids, sizeof(all_pids), rb, 1, 1500, "ts keepalive#2");
                memset(rb, 0, sizeof(rb));
                talk(s->d, stream_on, sizeof(stream_on), rb, 1, 1500, "ts keepalive_06");
                keepalive_sent = true;
            }
            gettimeofday(&next_keepalive, NULL);
            next_keepalive.tv_sec += 1;
        }
        if (s->seconds > 0 &&
            (now.tv_sec > s->deadline.tv_sec ||
             (now.tv_sec == s->deadline.tv_sec && now.tv_usec >= s->deadline.tv_usec)))
            break;
    }
    s->active = 0;
    for (int i = 0; i < TS_XFER_COUNT; i++) {
        if (s->xfers[i])
            libusb_cancel_transfer(s->xfers[i]);
    }
    for (int n = 0; n < 20; n++) {
        struct timeval tv = {0, 50000};
        libusb_handle_events_timeout_completed(s->d->ctx, &tv, NULL);
    }
    fprintf(stderr, "captured %" PRIu64 " bytes, sync-ish packets=%" PRIu64 "\n",
            s->bytes, s->packets47);
}

static void stream_close(struct stream *s)
{
    for (int i = 0; i < TS_XFER_COUNT; i++) {
        if (s->xfers[i])
            libusb_free_transfer(s->xfers[i]);
        free(s->bufs[i]);
    }
    if (s->out)
        if (s->out != stdout)
            fclose(s->out);
}

static void probe_device(void)
{
    libusb_context *ctx = NULL;
    libusb_device **list = NULL;
    ssize_t n;

    libusb_init(&ctx);
    n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        if (libusb_get_device_descriptor(list[i], &dd) == 0 &&
            dd.idVendor == LME_VID && dd.idProduct == LME_PID) {
            printf("found %04x:%04x bus=%u addr=%u bcd=%04x configs=%u\n",
                   dd.idVendor, dd.idProduct, libusb_get_bus_number(list[i]),
                   libusb_get_device_address(list[i]), dd.bcdDevice,
                   dd.bNumConfigurations);
        }
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage:\n"
        "  %s probe\n"
        "  %s upload -f firmware\n"
        "  %s init [--no-upload] [-f firmware]\n"
        "  %s lock [--no-upload] [-f firmware] -r hz\n"
        "  %s run [-f firmware] [--no-upload] [--no-keepalive] [--sync-ts] -r hz -o file.ts [-s seconds] [-e ep]\n"
        "    -s 0 means live until Ctrl-C\n"
        "\n"
        "default firmware: firmware/dvb-usb-lme2510c-dtmb-5300.fw\n",
        argv0, argv0, argv0, argv0, argv0);
}

int main(int argc, char **argv)
{
    const char *cmd_name, *fw_path = "firmware/dvb-usb-lme2510c-dtmb-5300.fw";
    const char *out_path = "/tmp/lme_dtmb.ts";
    uint32_t freq_hz = 602000000U;
    int seconds = 10;
    bool no_upload = false;
    struct dev d;
    int r;

    signal(SIGINT, on_sigint);
    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    cmd_name = argv[1];
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "-f") && i + 1 < argc)
            fw_path = argv[++i];
        else if (!strcmp(argv[i], "-r") && i + 1 < argc)
            freq_hz = (uint32_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "-o") && i + 1 < argc)
            out_path = argv[++i];
        else if (!strcmp(argv[i], "-s") && i + 1 < argc)
            seconds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-e") && i + 1 < argc)
            g_ts_ep = (uint8_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--no-upload"))
            no_upload = true;
        else if (!strcmp(argv[i], "--no-keepalive"))
            g_ts_keepalive = false;
        else if (!strcmp(argv[i], "--sync-ts"))
            g_ts_sync = true;
        else if (!strcmp(argv[i], "-q"))
            g_verbose = 0;
        else if (!strcmp(argv[i], "-vv"))
            g_verbose = 3;
        else {
            usage(argv[0]);
            return 2;
        }
    }

    if (!strcmp(cmd_name, "probe")) {
        probe_device();
        r = open_dev(&d);
        if (r == 0) {
            uint8_t rb[8] = {0};
            uint8_t ping[] = {0x81,0x0b,0,0,0,0,0,0,0,0,0,0,0,0,0};
            set_alt1(&d);
            talk(&d, ping, sizeof(ping), rb, 1, 500, "bridge ping");
            fprintf(stderr, "bridge ping ack=%02x\n", rb[0]);
            close_dev(&d);
        } else {
            fprintf(stderr, "open ret=%s\n", usb_err(r));
        }
        return 0;
    }

    r = open_dev(&d);
    if (r != 0) {
        fprintf(stderr, "open ret=%s\n", usb_err(r));
        return 1;
    }

    if (!strcmp(cmd_name, "upload")) {
        r = upload_firmware(&d, fw_path);
        close_dev(&d);
        return r == 0 ? 0 : 1;
    }

    if (!no_upload) {
        r = upload_firmware(&d, fw_path);
        close_dev(&d);
        if (r != 0 && r != LIBUSB_ERROR_NO_DEVICE && r != LIBUSB_ERROR_IO &&
            r != LIBUSB_ERROR_PIPE && r != LIBUSB_ERROR_TIMEOUT)
            return 1;

        fprintf(stderr, "waiting for post-upload device...\n");
        r = wait_reopen(&d, 20);
        if (r != 0) {
            fprintf(stderr, "reopen ret=%s\n", usb_err(r));
            return 1;
        }
    } else {
        r = set_alt1(&d);
        if (r != 0) {
            close_dev(&d);
            return 1;
        }
    }

    if (!strcmp(cmd_name, "init")) {
        gl5_init(&d);
        close_dev(&d);
        return 0;
    }

    if (!strcmp(cmd_name, "lock")) {
        lock_test(&d, freq_hz);
        close_dev(&d);
        return 0;
    }

    if (!strcmp(cmd_name, "run")) {
        struct stream s;
        gl5_init(&d);
        tune_c0(&d, freq_hz);
        csv_stream_handshake(&d);
        if (g_ts_sync) {
            r = stream_sync(&d, out_path, seconds);
        } else {
            if (stream_start(&s, &d, out_path, seconds) != 0) {
                close_dev(&d);
                return 1;
            }
            stream_loop(&s);
            stream_close(&s);
            r = 0;
        }
        close_dev(&d);
        return r == 0 ? 0 : 1;
    }

    usage(argv[0]);
    close_dev(&d);
    return 2;
}
