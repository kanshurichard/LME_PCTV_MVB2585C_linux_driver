# LME_PCTV_MVB2585C 第一波道DTMB电视棒 Linux 驱动

本驱动fork自：https://gitee.com/xuyizai

主要解决了原驱动严重卡顿和预置频点不全的问题。

已添加香港全部频点，目前在香港可以搜到全部dtmb频道（仅限kernel，userspace部分因用不到，暂时未改）。

经测试，在armbian下完全兼容tvheadend。

TODO：

- 将预置频点的模式改为所有频点自适应
- 优化userspace部分

# 以下为原作者内容：

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

仓库已包含当前使用的固件文件：

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
