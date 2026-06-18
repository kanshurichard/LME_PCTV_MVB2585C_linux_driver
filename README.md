# dtmb-linux

LME2510C + LGS8GL5 + MAX2165 的 Linux 侧代码整理。

这个仓库只保留公开所需的最小文件：

- `kernel/`：内核模块版本的 DTMB 驱动
- `userspace/`：基于 `libusb` 的用户态控制与 TS 抓流工具

以下内容没有放进来：

- 本地部署脚本
- NAS/IP/账号/口令
- 抓包分析日志
- Docker / HTTP 转发层
- 私有固件文件

## 目录

### `kernel/`

- `lme2510_dtmb.c`
- `Makefile`

编译：

```bash
cd kernel
make
```

默认固件名：

```text
/lib/firmware/dvb-usb-lme2510c-dtmb.fw
```

### `userspace/`

- `lme2510_dtmb_libusb.c`
- `Makefile`
- `run_lme2510_libusb.sh`

编译：

```bash
cd userspace
make
```

运行前请自行准备固件：

```text
userspace/firmware/dvb-usb-lme2510c-dtmb-5300.fw
```

示例：

```bash
cd userspace
./run_lme2510_libusb.sh probe
./run_lme2510_libusb.sh run --sync-ts -r 602000000 -o /tmp/602.ts -s 10
```

## 说明

- 当前代码主要面向 `3344:1120`
- 内核模块与用户态工具是两条独立路线，可分别使用
- 用户态脚本默认使用自身目录，不依赖固定主机路径
