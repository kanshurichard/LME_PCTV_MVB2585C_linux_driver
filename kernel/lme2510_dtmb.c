// SPDX-License-Identifier: GPL-2.0-only
/*
 * LME2510C + LGS8GL5 + MAX2165  DTMB USB 驱动
 *
 * 硬件拓扑:
 *   主机 <--[USB Bulk EP01/81]--> LME2510C(8051)
 *            |-- [gate=4, I2C] --> LGS8GL5 demod  addr=0x19 (alt=0x1B)
 *            |-- [USB C0 cmd ] --> MAX2165 tuner   addr=0x60 (桥内部)
 *   TS流: LME2510C --> EP 0x88 --> demux --> dvr0
 *
 * 已确认参数（来自实测+抓包）:
 *   VID:PID          = 3344:1120
 *   LGS8GL5 I2C      = 0x19,  alt = 0x1B
 *   MAX2165 I2C      = 0x60   (仅C0路径，不走直接I2C)
 *   TS EP            = 0x88 (602/6262.csv; alt1 bulk IN)
 *   固件             = dvb-usb-lme2510c-dtmb.fw
 *   GL5 reg[0x07]    = 0x1C   (零中频+ADC，Windows实测值)
 *   GL5 reg[09-0C]   = 0x00   (零中频时IF寄存器全0)
 *   GL5 锁定判据     = reg[0xA4] & 0x03 == 0x01
 *
 * 关键修正（相对初版）:
 *   1. reg[0x07]=0x1C 且三连发确保写入GL5
 *   2. IF寄存器09-0C全清零（零中频）
 *   3. 0xA4锁定判据修正为低2位==0x01
 *   4. 桥写关键寄存器后立即读回验证
 *   5. TS URB完整实现（start/stop_feed）
 *   6. double-disconnect防护
 *   7. set_fe防重入
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/firmware.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <media/dvb_frontend.h>
#include <media/dvbdev.h>
#include <media/dvb_demux.h>
#include <media/dmxdev.h>

/* ------------------------------------------------------------------ */
/* 常量                                                                */
/* ------------------------------------------------------------------ */

#define LME_VID           0x3344
#define LME_PID           0x1120
#define LME_FW_NAME       "dvb-usb-lme2510c-dtmb.fw"
#define LME_ROM_NAME      "dvb-usb-lme2510c-dtmb-rom.fw"

#define LME_EP_CMD_OUT    0x01   /* Bulk OUT: 控制/I2C命令 */
#define LME_EP_FW_OUT    0x01   /* Bulk OUT: firmware upload (Windows/mac use EP01) */
#define LME_EP_CMD_IN     0x81   /* Bulk IN:  命令应答     */
/* FTM ~254× 06 00 before first TS; ~50ms between bursts */
#define LME_TS_KEEPALIVE_MS    50
#define LME_TS_BRIDGE_ON_CNT   20   /* was 4; probe GL5 TS ramp-up time */
#define LME_TS_PRIME_ON_CNT     1   /* CSV: single 06 00 before TS URB completes */

#define LME_EP_TS_IN      0x88   /* Bulk IN: TS — 602.csv / 6262.csv */

#define LME_GATE_DEMOD    4      /* I2C gate → LGS8GL5    */
#define LME_GATE_TUNER    5      /* I2C gate → MAX2165     */

/* LGS8GL5 I2C地址（7-bit） */
#define GL5_ADDR          0x19
#define GL5_ALT_ADDR      0x1B   /* = GL5_ADDR + 2 */
#define GL5_DEMOD_SEL     0x32   /* vendorgl5 dtmb_demod_selector = addr<<1 */

/* MAX2165 I2C地址（7-bit，桥 gate5） */
#define MAX2165_ADDR      0x60

/* LGS8GL5 寄存器 */
#define GL5_R_RESET       0x02   /* bit0: 1=正常, 0=复位   */
#define GL5_R_03          0x03
#define GL5_R_04          0x04
#define GL5_R_07          0x07   /* 0x1C=零中频调谐, 0x01=MPEG TS 输出(1.ftm) */
#define GL5_R_07_ZIF      0x1C
#define GL5_R_07_TS_OUT   0x01
#define GL5_R_09          0x09   /* IF频率[3] 零中频时=0x00 */
#define GL5_R_0A          0x0A   /* IF频率[2]              */
#define GL5_R_0B          0x0B   /* IF频率[1]              */
#define GL5_R_0C          0x0C   /* IF频率[0]              */
#define GL5_R_37          0x37
#define GL5_R_STRENGTH    0x4B   /* bit7=有载波, [6:0]=强度 */
#define GL5_R_7D          0x7D   /* 锁定后写入A2值          */
#define GL5_R_7E          0x7E   /* 自动模式控制            */
#define GL5_R_A2          0xA2   /* 检测到的参数（锁定后读）*/
#define GL5_R_STATUS      0xA4   /* [1:0]: 01=LOCK          */

/* TS URB参数 */
#define TS_URB_COUNT      32     /* Increased for better buffering */
#define TS_URB_SIZE       (4096 * 4) /* 16KB per URB */
#define TS_PACKET_SIZE    188
#define TS_SYNC_CONFIRM   5
#define TS_ALIGN_BUF_SIZE (TS_URB_SIZE + TS_PACKET_SIZE * (TS_SYNC_CONFIRM + 2))

/* mac_driver ftm short-path timing for 578/586/602 */
#define FTM_SHORT_4B1_MS  37
#define FTM_SHORT_4B2_MS  32
#define FTM_SHORT_A4_1_MS 21
#define FTM_SHORT_A4_2_MS 21
#define FTM_SHORT_A4_3_MS 22

/* ------------------------------------------------------------------ */
/* 模块参数                                                            */
/* ------------------------------------------------------------------ */

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "调试级别 0=关 1=开 2=寄存器详细");

static int force_cold;
static bool fw_uploaded;
static unsigned long fw_upload_jiffies;
module_param(force_cold, int, 0644);
MODULE_PARM_DESC(force_cold, "Warm attach: force cold reset+firmware reload (default 0)");

static int i2c_gate = 4;
module_param(i2c_gate, int, 0644);
MODULE_PARM_DESC(i2c_gate, "GL5 demod I2C gate (default 4; gate 5 is MAX2165 tuner only)");

static int ts_probe_on_tune;
module_param(ts_probe_on_tune, int, 0644);
MODULE_PARM_DESC(ts_probe_on_tune, "Run post-lock TS step probe (debug)");

static int dtmb_ftm602_handshake = 0;  /* 0=off(default), 1=force report2 */
module_param(dtmb_ftm602_handshake, int, 0644);
MODULE_PARM_DESC(dtmb_ftm602_handshake, "report2 stream handshake (0=off, 1=force)");

/* GL5 alt reg 0xC2 MPEG TS mode (decimal): 40=0x28 default, 42=0x2A inv clk, 44=0x2C freerun, 46=0x2E both */
static int ts_c2_val = 0x28;
module_param(ts_c2_val, int, 0644);
MODULE_PARM_DESC(ts_c2_val, "GL5 reg C2 TS interface byte (0x28=parallel default)");

static int post_8a_delay_ms = 300;
module_param(post_8a_delay_ms, int, 0644);
MODULE_PARM_DESC(post_8a_delay_ms, "Extra delay after 0x8A before leaving probe for warm re-enumeration");

static int post_8a_usb_reset;
module_param(post_8a_usb_reset, int, 0644);
MODULE_PARM_DESC(post_8a_usb_reset, "Experimental: call usb_reset_device() after 0x8A before leaving probe");

/*
 * 冷启动上传完固件后的接力策略:
 *   0 = current behavior: leave probe, wait for warm re-enumeration
 *   1 = same-probe only: wait bridge ready, then jump straight into warm_init
 *   2 = hybrid: try same-probe first; on timeout fall back to re-enumeration
 */
static int post_fw_handover_mode = 2;
module_param(post_fw_handover_mode, int, 0644);
MODULE_PARM_DESC(post_fw_handover_mode, "Post-firmware handover mode: 0=wait warm re-enum, 1=same-probe, 2=hybrid (default)");

static int same_probe_wait_ms = 12000;
module_param(same_probe_wait_ms, int, 0644);
MODULE_PARM_DESC(same_probe_wait_ms, "same-probe bridge ready wait budget after 0x8A");

/*
 * 固件上传模式:
 *   0 = legacy/mac-linux segmented
 *   1 = strict Windows CSV simplified (81 -> 02 31[offset 500..])
 *   2 = annotated 602.md full path (01 31[offset 0..499] -> 81 -> 02 31[offset 500..])
 *
 * 默认切到 2，单独验证 annotated.md/tsv 所描述的完整路径。
 *
 * strict_win_csv_fw_upload 保留为兼容旧脚本:
 *   0 -> legacy
 *   1 -> strict simplified
 *  -1 -> 忽略兼容参数，仅看 fw_upload_mode
 */
static int fw_upload_mode = 2;
module_param(fw_upload_mode, int, 0644);
MODULE_PARM_DESC(fw_upload_mode, "Firmware upload mode: 0=legacy, 1=strict-win-csv, 2=annotated-602-md (default)");

static int strict_win_csv_fw_upload = -1;
module_param(strict_win_csv_fw_upload, int, 0644);
MODULE_PARM_DESC(strict_win_csv_fw_upload, "Deprecated compatibility override: 0=legacy, 1=strict-win-csv, -1=use fw_upload_mode");

/*
 * Experimental warm-tail patch:
 * runtime 0x12bd checks xdata 0xc00b bit1 and returns carry=ready.
 * When enabled, patch the final 3 firmware bytes so 0x12bd becomes:
 *   d3    setb C
 *   22    ret
 *   00    nop
 * This bypasses the ready gate while keeping the rest of the warm tail intact.
 */
static int fw_gate_bypass;
module_param(fw_gate_bypass, int, 0644);
MODULE_PARM_DESC(fw_gate_bypass, "Experimental: bypass warm runtime gate at rt 0x12bd (default 0)");

/* debug=1: log first 30 USB bridge ops during probe */
static atomic_t bridge_log_count;

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

#define dinfo(fmt, ...)  pr_info("lme_dtmb: " fmt, ##__VA_ARGS__)
#define derr(fmt, ...)   pr_err ("lme_dtmb: " fmt, ##__VA_ARGS__)
#define ddbg(fmt, ...)   do { if (debug)   pr_info("lme_dtmb: " fmt, ##__VA_ARGS__); } while (0)
#define ddbg2(fmt, ...)  do { if (debug>1) pr_info("lme_dtmb: " fmt, ##__VA_ARGS__); } while (0)

/* ------------------------------------------------------------------ */
/* 私有状态                                                            */
/* ------------------------------------------------------------------ */

struct lme_ts_urb {
    struct urb *urb;
    u8         *buf;
};

struct lme_state {
    struct usb_device  *udev;
    struct dvb_adapter  dvb_adap;
    struct dvb_demux    demux;
    struct dmxdev       dmxdev;
    struct dvb_frontend fe;
    struct dmx_frontend fe_hw;

    struct mutex        usb_mutex;     /* 保护usb_bulk_msg        */
    u8                  cmd_buf[64];   /* 收发复用缓冲             */

    /* 生命周期 */
    atomic_t            probe_ok;      /* probe完成标志            */
    atomic_t            disconnected;  /* 防double-disconnect      */

    /* 解调防重入 */
    atomic_t            demod_busy;
    enum fe_status      cached_fe_status; /* read_status when demod_busy=1 */

    /* TS流 */
    struct lme_ts_urb   ts_urbs[TS_URB_COUNT];
    atomic_t            ts_active;
    atomic_t            ts_probe_rx;     /* bytes during step-probe window */
    struct mutex        ts_mutex;
    spinlock_t          ts_align_lock;
    int                 feedcount;
    bool                ts_bridge_pending; /* lock+route done; need bridge before URBs */
    bool                gl5_ts_enabled;    /* enable_ts done in win_start_demod */
    bool                ts_stream_started; /* 06 00 already sent this tune */
    struct delayed_work ts_keepalive_work; /* periodic 06 00 while feed active */
    u8                  ts_align_buf[TS_ALIGN_BUF_SIZE];
    u8                  ts_align_out[TS_ALIGN_BUF_SIZE];
    size_t              ts_align_len;
    bool                ts_align_locked;
    u64                 ts_align_relocks;
    u64                 ts_align_dropped;
    ktime_t             ftm_c5_done;       /* for C5→first 06 00 / first IN timing */
    bool                ftm_c5_valid;
    bool                ts_first_in_logged;
    bool                i2c_active;         /* when false: only TS bridge cmds */

    /* vendorgl5 WinDetect state */
    u8                  win_mode;      /* 0=GL5, 1=G75 */
    u8                  win_demod_id;  /* reg 0x32 chip id */
    u8                  active_gate;   /* selected I2C gate (4 or 5) */
    u8                  gl5_read_hint; /* Windows 85-read fifth byte: last GL5 write value */
    bool                warm_attach; /* probe saw warm (fw already loaded) */
    bool                fw_done_in_probe; /* cold upload ok in this probe — skip string2 reload */
    u8                  ftm_last_4b;
    u8                  ftm_last_a4;
    u8                  ftm_last_a2;

    /* MAX2165 calibration (from ROM table) */
    u8  max_tf_ntch_low_cfg;
    u8  max_tf_ntch_hi_cfg;
    u8  max_tf_balun_low_ref;
    u8  max_tf_balun_hi_ref;
    u8  max_bb_filter_7mhz_cfg;
    u8  max_bb_filter_8mhz_cfg;
    u8  max_osc_clk;    /* MHz */

    /* C0 tuning data for current frequency */
    u8  c0_cmd[17];
    u32 tuned_freq_hz;
};

/* ------------------------------------------------------------------ */
/* C0 调谐频率表（来自 1(1).ftm Windows抓包，37个频点）               */
/* ------------------------------------------------------------------ */

struct c0_entry {
    u32 freq_hz;
    u8  cmd[17];
};

static const struct c0_entry c0_table[] = {
/* auto-generated from 1 (1).ftm */
    { 482000000U, { 0xC0,0x00,0x28,0x12,0xAA,0xAA,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    { 522000000U, { 0xC0,0x00,0x2B,0x18,0x00,0x00,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    { 538000000U, { 0xC0,0x00,0x2C,0x1D,0x55,0x55,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    /* HK working short-path alias (23.csv-derived payload, user keeps this as 586) */
    { 586000000U, { 0xC0,0x00,0x30,0x1D,0x55,0x55,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    /* HK ~602MHz */
    { 602000000U, { 0xC0,0x00,0x32,0x12,0xAA,0xAA,0xB6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } },
    /* HK ~626MHz */
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

/* 0x77 = bridge command ACK, never valid GL5/MAX2165 register data */
static bool gl5_reg_val_valid(u8 val)
{
    /* 0x00 is valid (e.g. a4 unlocked); only reject bridge ack / empty */
    return val != 0x77 && val != 0xff;
}

static bool gl5_chip_id_valid(u8 val)
{
    return val != 0x00 && val != 0x77 && val != 0xff;
}

static bool gl5_use_short_ftm_path(const struct lme_state *st)
{
    return true;
}

static bool gl5_prime_ts_inside_ftm(const struct lme_state *st)
{
    return !gl5_use_short_ftm_path(st);
}

static int win3_read85(struct lme_state *st, u8 sel, u8 reg, u8 hint,
                       const char *tag, u8 *val);

/*
 * mac sync-ts path:
 * - keep a rolling buffer
 * - look for 0x47 spaced every 188 bytes
 * - once locked, emit only aligned 188-byte packets
 */

static void lme_ts_align_reset(struct lme_state *st)
{
    unsigned long flags;

    spin_lock_irqsave(&st->ts_align_lock, flags);
    st->ts_align_len = 0;
    st->ts_align_locked = false;
    st->ts_align_relocks = 0;
    st->ts_align_dropped = 0;
    spin_unlock_irqrestore(&st->ts_align_lock, flags);
}

static bool lme_ts_confirm_sync(const u8 *buf, size_t len, size_t off)
{
    int i;

    for (i = 0; i < TS_SYNC_CONFIRM; i++) {
        size_t pos = off + (size_t)i * TS_PACKET_SIZE;

        if (pos >= len || buf[pos] != 0x47)
            return false;
    }
    return true;
}

static int lme_ts_aligner_push(struct lme_state *st, const u8 *data, size_t data_len)
{
    unsigned long flags;
    size_t feed_len = 0;

    if (data_len > TS_ALIGN_BUF_SIZE) {
        data += data_len - TS_ALIGN_BUF_SIZE;
        data_len = TS_ALIGN_BUF_SIZE;
    }

    spin_lock_irqsave(&st->ts_align_lock, flags);

    if (st->ts_align_len + data_len > TS_ALIGN_BUF_SIZE) {
        size_t drop = st->ts_align_len + data_len - TS_ALIGN_BUF_SIZE;

        memmove(st->ts_align_buf, st->ts_align_buf + drop,
                st->ts_align_len - drop);
        st->ts_align_len -= drop;
        st->ts_align_dropped += drop;
        st->ts_align_locked = false;
    }

    memcpy(st->ts_align_buf + st->ts_align_len, data, data_len);
    st->ts_align_len += data_len;

    for (;;) {
        size_t off, keep, pos;

        if (!st->ts_align_locked) {
            for (off = 0; off < st->ts_align_len; off++) {
                if (st->ts_align_buf[off] == 0x47 &&
                    lme_ts_confirm_sync(st->ts_align_buf,
                                        st->ts_align_len, off))
                    break;
            }

            if (off == st->ts_align_len) {
                keep = TS_PACKET_SIZE * (TS_SYNC_CONFIRM - 1);
                if (st->ts_align_len > keep) {
                    st->ts_align_dropped += st->ts_align_len - keep;
                    memmove(st->ts_align_buf,
                            st->ts_align_buf + st->ts_align_len - keep,
                            keep);
                    st->ts_align_len = keep;
                }
                break;
            }

            if (off > 0) {
                st->ts_align_dropped += off;
                memmove(st->ts_align_buf, st->ts_align_buf + off,
                        st->ts_align_len - off);
                st->ts_align_len -= off;
            }

            st->ts_align_locked = true;
            st->ts_align_relocks++;
        }

        pos = 0;
        while (pos + TS_PACKET_SIZE <= st->ts_align_len &&
               st->ts_align_buf[pos] == 0x47)
            pos += TS_PACKET_SIZE;

        if (pos > 0) {
            memcpy(st->ts_align_out + feed_len, st->ts_align_buf, pos);
            feed_len += pos;
            memmove(st->ts_align_buf, st->ts_align_buf + pos,
                    st->ts_align_len - pos);
            st->ts_align_len -= pos;
        }

        if (st->ts_align_len >= TS_PACKET_SIZE &&
            st->ts_align_buf[0] != 0x47) {
            st->ts_align_locked = false;
            continue;
        }
        break;
    }

    spin_unlock_irqrestore(&st->ts_align_lock, flags);

    if (feed_len > 0)
        dvb_dmx_swfilter_packets(&st->demux, st->ts_align_out,
                                 feed_len / TS_PACKET_SIZE);

    return (int)feed_len;
}

static int gl5_ftm_short_lock_probe(struct lme_state *st, u8 *val)
{
    msleep(FTM_SHORT_4B1_MS);
    win3_read85(st, 0x32, 0x4b, 0x01, "ftm 4b#1", val);
    msleep(FTM_SHORT_4B2_MS);
    win3_read85(st, 0x32, 0x4b, 0x01, "ftm 4b#2", val);
    msleep(FTM_SHORT_A4_1_MS);
    win3_read85(st, 0x32, 0xa4, 0x01, "ftm a4#1", val);
    msleep(FTM_SHORT_A4_2_MS);
    win3_read85(st, 0x32, 0xa4, 0x01, "ftm a4#2", val);
    msleep(FTM_SHORT_A4_3_MS);
    win3_read85(st, 0x32, 0xa4, 0x01, "ftm a4#3", val);
    return 3;
}

static u8 gl5_pick_reg_data(u8 rb[5])
{
    /* Gate/demod read: ACK in [0], register data in [1] */
    if (rb[0] == 0x55 || rb[0] == 0x88)
        return rb[1];
    if (gl5_chip_id_valid(rb[1]) || gl5_reg_val_valid(rb[1]))
        return rb[1];
    return 0;
}

/* ================================================================== */
/* USB 通信层                                                          */
/* ================================================================== */

static void lme_usb_log_op(const u8 *wbuf, int wlen, const u8 *rbuf, int rlen, int ret)
{
    int n, i;

    if (!debug)
        return;
    n = atomic_read(&bridge_log_count);
    if (n >= 30)
        return;
    atomic_inc(&bridge_log_count);

    pr_info("lme_dtmb: bridge[%02d] OUT(%d):", n, wlen);
    for (i = 0; i < wlen && i < 16; i++)
        pr_cont(" %02x", wbuf[i]);
    if (wlen > 16)
        pr_cont(" ...");
    pr_cont("\n");

    if (rlen > 0 && rbuf) {
        pr_info("lme_dtmb: bridge[%02d] IN(%d):", n, rlen);
        for (i = 0; i < rlen && i < 8; i++)
            pr_cont(" %02x", rbuf[i]);
        if (rlen > 8)
            pr_cont(" ...");
        pr_cont(" ret=%d\n", ret);
    } else {
        pr_info("lme_dtmb: bridge[%02d] IN(0) ret=%d\n", n, ret);
    }
}

static int lme_usb_talk(struct lme_state *st,
                        u8 *wbuf, int wlen,
                        u8 *rbuf, int rlen)
{
    int ret, actual;

    ret = mutex_lock_interruptible(&st->usb_mutex);
    if (ret)
        return -EAGAIN;

    memcpy(st->cmd_buf, wbuf, wlen);
    ret = usb_bulk_msg(st->udev,
                       usb_sndbulkpipe(st->udev, LME_EP_CMD_OUT),
                       st->cmd_buf, wlen, &actual, 1500);
    if (ret) { ddbg("bulk OUT ret=%d\n", ret); goto log_out; }

    if (rlen > 0) {
        if (rbuf)
            memset(rbuf, 0, rlen);
        memset(st->cmd_buf, 0, sizeof(st->cmd_buf));
        ret = usb_bulk_msg(st->udev,
                           usb_rcvbulkpipe(st->udev, LME_EP_CMD_IN),
                           st->cmd_buf, rlen, &actual, 1500);
        if (ret == -EOVERFLOW) {
            ddbg("bulk IN overflow exact-len actual=%d want=%d first=%02x %02x\n",
                 actual, rlen, st->cmd_buf[0], st->cmd_buf[1]);
            if (rbuf && actual > 0)
                memcpy(rbuf, st->cmd_buf, min(rlen, actual));
            ret = 0;
        }
        if (ret) { ddbg("bulk IN ret=%d\n", ret); goto log_out; }
        if (rbuf && actual > 0)
            memcpy(rbuf, st->cmd_buf, min(rlen, actual));
    }
log_out:
    lme_usb_log_op(wbuf, wlen, rbuf, rlen, ret);
    mutex_unlock(&st->usb_mutex);
    return ret;
}

/*
 * Fast exact-length command path for report2/echo experiments.
 * Unlike the generic bridge path, keep the IN length exact so we can tell
 * whether the bridge truly returned the echoed payload or only a 1-byte ACK.
 */
static int lme_usb_talk_fast_exact(struct lme_state *st,
                                   u8 *wbuf, int wlen,
                                   u8 *rbuf, int rlen)
{
    int ret, actual;

    ret = mutex_lock_interruptible(&st->usb_mutex);
    if (ret)
        return -EAGAIN;

    memcpy(st->cmd_buf, wbuf, wlen);
    ret = usb_bulk_msg(st->udev,
                       usb_sndbulkpipe(st->udev, LME_EP_CMD_OUT),
                       st->cmd_buf, wlen, &actual, 50);
    if (ret)
        goto out;

    if (rlen > 0) {
        if (rbuf)
            memset(rbuf, 0, rlen);
        memset(st->cmd_buf, 0, sizeof(st->cmd_buf));
        ret = usb_bulk_msg(st->udev,
                           usb_rcvbulkpipe(st->udev, LME_EP_CMD_IN),
                           st->cmd_buf, rlen, &actual, 50);
        if (ret == -EOVERFLOW) {
            ddbg("fast bulk IN overflow actual=%d first=%02x %02x\n",
                 actual, st->cmd_buf[0], st->cmd_buf[1]);
            if (rbuf && actual > 0)
                memcpy(rbuf, st->cmd_buf, min(rlen, actual));
            ret = 0;
        }
        if (!ret && rbuf && actual > 0)
            memcpy(rbuf, st->cmd_buf, min(rlen, actual));
    }
out:
    lme_usb_log_op(wbuf, wlen, rbuf, rlen, ret);
    mutex_unlock(&st->usb_mutex);
    return ret;
}

/*
 * Firmware upload path.
 *
 * Windows/mac cold upload uses the normal bulk OUT pipe on EP01 plus EP81 ACKs.
 * Keep the shorter firmware-stage timeouts, but avoid Linux-only clear_halt()
 * side effects during every packet.
 */
static int lme_fw_usb_talk(struct lme_state *st,
                        u8 *wbuf, int wlen,
                        u8 *rbuf, int rlen)
{
    int ret, actual;
    int rxlen;

    ret = mutex_lock_interruptible(&st->usb_mutex);
    if (ret)
        return -EAGAIN;

    memcpy(st->cmd_buf, wbuf, wlen);
    ret = usb_bulk_msg(st->udev,
                       usb_sndbulkpipe(st->udev, LME_EP_FW_OUT),
                       st->cmd_buf, wlen, &actual, 500);
    if (ret)
        goto out;

    if (rlen > 0) {
        if (rbuf)
            memset(rbuf, 0, rlen);
        /*
         * Bootloader ACKs are not always a literal 1-byte response on Linux.
         * The device may return a longer short packet (for example 0x88 plus
         * trailing status bytes). Reading only 1 byte here turns that into
         * -EOVERFLOW/-75, even though the OUT packet was accepted.
         */
        rxlen = max(rlen, (int)sizeof(st->cmd_buf));
        rxlen = min(rxlen, (int)sizeof(st->cmd_buf));
        memset(st->cmd_buf, 0, sizeof(st->cmd_buf));
        ret = usb_bulk_msg(st->udev,
                           usb_rcvbulkpipe(st->udev, LME_EP_CMD_IN),
                           st->cmd_buf, rxlen, &actual, 200);
        if (ret == -EOVERFLOW) {
            ddbg("fw bulk IN overflow actual=%d first=%02x %02x\n",
                 actual, st->cmd_buf[0], st->cmd_buf[1]);
            if (rbuf && actual > 0)
                memcpy(rbuf, st->cmd_buf, min(rlen, actual));
            ret = 0;
            goto out;
        }
        if (ret == -ETIMEDOUT || ret == -110) {
            ret = 0;
            goto out;
        }
        if (ret)
            goto out;
        if (rbuf && actual > 0)
            memcpy(rbuf, st->cmd_buf, min(rlen, actual));
    }
out:
    mutex_unlock(&st->usb_mutex);
    return ret;
}

/* vendorgl5 dtmb_bridge_cmd09 — flush 桥内部状态（最多 6 次） */
static int lme_cmd09(struct lme_state *st)
{
    u8 w[] = { 0x09, 0x00 };
    u8 r[5];
    int ret, i;

    for (i = 0; i < 6; i++) {
        ret = lme_usb_talk(st, w, 2, r, sizeof(r));
        if (ret == 0)
            break;
        msleep(80);
    }
    dinfo("cmd09 ack=0x%02x ret=%d tries=%d\n", r[0], ret, i + 1);
    return ret;
}

static int lme_cmd16(struct lme_state *st, u8 mode)
{
    u8 w[] = { 0x16, 0x01, mode };
    u8 r[5];
    int ret;

    ret = lme_usb_talk(st, w, 3, r, sizeof(r));
    dinfo("cmd16 mode=%u ack=0x%02x ret=%d%s\n", mode, r[0], ret,
          (r[0] != 0x77) ? " UNEXPECTED" : "");
    return ret;
}

/* vendorgl5 lme_coldreset: 0x0A → bridge drops to bootloader for fw reload */
static void lme_coldreset(struct lme_state *st)
{
    u8 data[] = { 0x0A };
    u8 rb[1];

    dinfo("FRM Firmware Cold Reset (0x0A)\n");
    lme_usb_talk(st, data, 1, rb, 1);
}

/*
 * Gate I2C read — vendorgl5 lme2510_i2c_xfer fallback:
 *   gate=4: 84 03 <sel> <reg> 01  → 2B IN
 *   gate=5: 85 02 <sel> <reg>     → 2B IN  (ob[1]=2, no read-len byte)
 */
static int gl5_i2c_gate_read(struct lme_state *st, u8 gate, u8 addr, u8 reg,
                             u8 rb[5], bool cmd09_first)
{
    u8 ob[5], ib[2];
    u8 val;
    int ret, wlen;

    if (cmd09_first)
        lme_cmd09(st);

    if (gate == 5) {
        ob[0] = LME_GATE_TUNER | 0x80;
        ob[1] = 2;
        ob[2] = addr << 1;
        ob[3] = reg;
        wlen = 4;
    } else {
        ob[0] = LME_GATE_DEMOD | 0x80;
        ob[1] = 3;
        ob[2] = addr << 1;
        ob[3] = reg;
        ob[4] = 1;
        wlen = 5;
    }
    memset(rb, 0, 5);
    memset(ib, 0, sizeof(ib));
    ret = lme_usb_talk(st, ob, wlen, ib, 2);
        if (ret == 0) {
        if (ib[0] == 0x55 || ib[0] == 0x88)
            val = ib[1];
        else if (ib[0] == 0x77)
            val = 0;  /* bridge cmd ack, not I2C data */
        else
            val = ib[0];
        rb[0] = ib[0];
        rb[1] = val;
        rb[2] = ib[1];
    }
    if (wlen > 4)
        dinfo("gl5_gate%u_r %02x %02x %02x %02x ib=%02x %02x pick=0x%02x ret=%d\n",
              gate, ob[0], ob[1], ob[2], ob[3], ib[0], ib[1], rb[1], ret);
    else
        dinfo("gl5_gate%u_r %02x %02x %02x ib=%02x %02x pick=0x%02x ret=%d\n",
              gate, ob[0], ob[1], ob[2], ib[0], ib[1], rb[1], ret);
    return ret;
}

static __maybe_unused int gl5_i2c_gate_write(struct lme_state *st, u8 gate, u8 addr,
                              u8 reg, u8 val)
{
    u8 ob[5], ib[4];
    u8 g = (gate == 5) ? LME_GATE_TUNER : LME_GATE_DEMOD;
    int ret;

    ob[0] = g;
    ob[1] = 3;
    ob[2] = addr << 1;
    ob[3] = reg;
    ob[4] = val;
    ret = lme_usb_talk(st, ob, 5, ib, 4);
    dinfo("gl5_gate%u_w %02x 03 %02x %02x %02x ack=0x%02x ret=%d\n",
          gate, g, ob[2], reg, val, ib[0], ret);
    return ret;
}

static void gl5_gate_probe_and_select(struct lme_state *st, const char *tag)
{
    static const u8 regs[] = { 0x32, GL5_R_07, GL5_R_STRENGTH, GL5_R_STATUS };
    u8 rb[5];
    int r;

    if (i2c_gate == 5) {
        dinfo("[%s] i2c_gate=5 is MAX2165 tuner gate; forcing GL5 demod gate=4\n", tag);
        st->active_gate = LME_GATE_DEMOD;
        return;
    }

    st->active_gate = LME_GATE_DEMOD;
    if (i2c_gate == 4) {
        dinfo("[%s] i2c_gate=4 (GL5 demod locked)\n", tag);
        return;
    }

    dinfo("=== gate4 probe [%s] warm=%d (gate5 skipped: MAX2165 only) ===\n",
          tag, st->warm_attach);
    for (r = 0; r < (int)ARRAY_SIZE(regs); r++) {
        gl5_i2c_gate_read(st, LME_GATE_DEMOD, GL5_ADDR, regs[r], rb, r == 0);
        dinfo("  gate4 reg[0x%02x]=0x%02x ib=%02x %02x%s\n",
              regs[r], rb[1], rb[0], rb[1],
              gl5_reg_val_valid(rb[1]) ? "" : " (invalid/ack)");
    }
}

/*
 * GL5 read — Windows 3.csv path:
 *   85 02 32 <reg> <hint> → 2B IN: 55 <value>
 *
 * The fifth byte tracks the most recent GL5 write value in the Windows trace.
 * Do not silently fall back to 84/04 here: 84 03 32 reg returns bridge ACK
 * (88 00) on the current Block 4 firmware and hides a broken read path.
 */
static int gl5_read_gate_raw(struct lme_state *st, u8 addr, u8 reg,
                             u8 rb[5], bool cmd09_first)
{
    return gl5_i2c_gate_read(st, st->active_gate, addr, reg, rb, cmd09_first);
}

static int gl5_read_raw(struct lme_state *st, u8 addr, u8 reg,
                        u8 rb[5], bool cmd09_first)
{
    u8 ob[5];
    int ret;

    if (cmd09_first)
        lme_cmd09(st);

    /* Windows 3.csv: 85 02 32 reg hint → 55 value */
    ob[0] = 0x85;
    ob[1] = 0x02;
    ob[2] = addr << 1;
    ob[3] = reg;
    ob[4] = st->gl5_read_hint;
    memset(rb, 0, 5);
    ret = lme_usb_talk(st, ob, sizeof(ob), rb, 2);
    dinfo("gl5_r85 85 02 %02x %02x %02x ib=%02x %02x %02x %02x %02x pick=0x%02x ret=%d\n",
          ob[2], reg, ob[4], rb[0], rb[1], rb[2], rb[3], rb[4],
          gl5_pick_reg_data(rb), ret);
    return ret;
}

/* Some old FTM traces sent cmd09 before reads; 3.csv warm init does not. */
static int gl5_read_win(struct lme_state *st, u8 reg, u8 *val)
{
    u8 rb[5];
    int ret;

    ret = gl5_read_raw(st, GL5_ADDR, reg, rb, false);
    if (ret == 0)
        *val = gl5_pick_reg_data(rb);
    dinfo("gl5_read reg=0x%02x ibuf=%02x %02x %02x %02x %02x pick=0x%02x\n",
          reg, rb[0], rb[1], rb[2], rb[3], rb[4], *val);
    return ret;
}

/* Windows 3.csv GL5 write: 05 04 32 reg val (5 bytes, NO trailing 01) */
static int gl5_write_w32(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[5], rb[4];
    int ret;

    ob[0] = 0x05;
    ob[1] = 0x04;
    ob[2] = GL5_DEMOD_SEL;
    ob[3] = reg;
    ob[4] = val;
    ret = lme_usb_talk(st, ob, 5, rb, 4);
    st->gl5_read_hint = val;
    ddbg2("gl5_w32 05 04 32 %02x %02x ack=0x%02x ret=%d\n",
          reg, val, rb[0], ret);
    return ret;
}

static void gl5_write_w32_triple(struct lme_state *st, u8 reg, u8 val)
{
    int i;

    for (i = 0; i < 3; i++)
        gl5_write_w32(st, reg, val);
}

static int gl5_write_win(struct lme_state *st, u8 reg, u8 val)
{
    int ret = gl5_write_w32(st, reg, val);

    dinfo("gl5_w 05 04 32 %02x %02x ret=%d\n", reg, val, ret);
    return ret;
}

static int gl5_write_alt_gate(struct lme_state *st, u8 reg, u8 val);
static void gl5_mpeg_ts_mode(struct lme_state *st);
static int gl5_enable_ts(struct lme_state *st);
static void gl5_enable_ts_locked(struct lme_state *st);
static int gl5_win_start_demod(struct lme_state *st);
static int max2165_write_reg(struct lme_state *st, u8 reg, u8 val);
static int max2165_read_reg(struct lme_state *st, u8 reg, u8 *val);
static int gl5_ts_enable_probe(struct lme_state *st);
static void gl5_write_win3(struct lme_state *st, u8 reg, u8 val);
static void gl5_ts_hw_regs_log(struct lme_state *st, const char *tag);
static void gl5_log_ts_regs(struct lme_state *st, const char *tag);
static void gl5_log_iface_regs(struct lme_state *st, const char *tag);
static void lme_ts_send_stream_on(struct lme_state *st);
static void lme_ts_bridge_set_alt(struct lme_state *st);
static int  __maybe_unused lme_ftm602_report2_handshake(struct lme_state *st, const char *ctx);
static void __maybe_unused lme_ts_send_bridge_byte(struct lme_state *st, u8 v);
static void lme_ts_bridge_stream_cmds(struct lme_state *st, const char *ctx);
static void lme_ts_bridge_all_pids(struct lme_state *st, const char *ctx);
static int lme_ts_start(struct lme_state *st);
static int lme_ts_prime_after_enable_ts(struct lme_state *st);
static void lme_ts_wait_demod_lock(struct lme_state *st);
static void lme_ts_bridge_restart(struct lme_state *st);
static bool lme_ts_alt1_active(struct lme_state *st);
static void lme_ts_keepalive_stop(struct lme_state *st);
static void lme_ts_keepalive_start(struct lme_state *st);
static void lme_ts_stream_on(struct lme_state *st);
static bool gl5_hw_locked(struct lme_state *st);
static bool lme_ts_should_use_report2(const struct lme_state *st, const char *ctx);
static int max2165_init(struct lme_state *st);
static void gl5_ftm_post_c0(struct lme_state *st);
static int win3_cmd(struct lme_state *st, const u8 *cmd, int len,
                    int exp, const char *tag);
static int win3_read85(struct lme_state *st, u8 sel, u8 reg, u8 hint,
                       const char *tag, u8 *val);
static int win3_read84(struct lme_state *st, u8 sel, u8 reg, u8 suffix,
                       const char *tag, u8 *val);

/* TS step-probe: first step index (0..6) that saw bulk IN bytes */
static int ts_probe_first_step = -1;

/* Step 1: full GL5 register dump for C0 boundary debug */
static void __maybe_unused gl5_dump_regs(struct lme_state *st, const char *tag)
{
    u8 r01, r03, r04, r07, r37, r7e, r4b;

    gl5_read_win(st, 0x01, &r01);
    gl5_read_win(st, GL5_R_03, &r03);
    gl5_read_win(st, GL5_R_04, &r04);
    gl5_read_win(st, GL5_R_07, &r07);
    gl5_read_win(st, GL5_R_37, &r37);
    gl5_read_win(st, GL5_R_7E, &r7e);
    gl5_read_win(st, GL5_R_STRENGTH, &r4b);
    dinfo("GL5_DUMP [%s]: 01=%02x 03=%02x 04=%02x 07=%02x 37=%02x 7e=%02x 4b=%02x\n",
          tag, r01, r03, r04, r07, r37, r7e, r4b);
}

/* Step 2: read 4b after each post-C0 write */
static void post_c0_read_4b(struct lme_state *st, int step, const char *action)
{
    u8 r4b = 0xff;

    gl5_read_win(st, GL5_R_STRENGTH, &r4b);
    dinfo("post_c0 step %d: %s → 4b=0x%02x\n", step, action, r4b);
}

/*
 * 偏移探测：同一 reg 连读 5 次，对比 ibuf[1] vs ibuf[2] 谁稳定
 * reg[07] 期望 ~0x54；reg[4b] 有 RF 时 bit7=1
 */
static void __maybe_unused gl5_offset_probe(struct lme_state *st, const char *tag, bool flush)
{
    static const struct {
        u8 reg;
        const char *name;
        const char *expect;
    } targets[] = {
        { GL5_R_07,       "07", "exp~0x54" },
        { GL5_R_STRENGTH, "4b", "exp bit7" },
        { 0x00,           "00", "chip-id?" },
    };
    int t, n;
    u8 rb[5];

    dinfo("=== offset_probe [%s] cmd09_before=%d ===\n", tag, flush);
    for (t = 0; t < (int)ARRAY_SIZE(targets); t++) {
        for (n = 0; n < 5; n++) {
            if (gl5_read_raw(st, GL5_ADDR, targets[t].reg, rb, flush))
                dinfo("  reg[0x%s] #%d READ_FAIL (%s)\n",
                      targets[t].name, n, targets[t].expect);
            else
                dinfo("  reg[0x%s] #%d ibuf=%02x %02x %02x %02x %02x"
                      " | b1=0x%02x b2=0x%02x (%s)\n",
                      targets[t].name, n,
                      rb[0], rb[1], rb[2], rb[3], rb[4],
                      rb[1], rb[2], targets[t].expect);
            udelay(500);
        }
    }
}

/* ================================================================== */
/* GL5 I2C 读写（经 LME2510C gate=4）                                 */
/* ================================================================== */

/*
 * GL5 I2C — DTMB 固件 0x05/0x85（非 lmedm04 gate）
 *   写: 05 addr<<1 reg val (4B)   读: 85 addr<<1 reg (3B) → rb[1]=数据
 */
static int gl5_write3(struct lme_state *st, u8 addr, u8 reg, u8 val)
{
    int i, ret = 0;

    (void)addr;
    for (i = 0; i < 3; i++) {
        ret = gl5_write_w32(st, reg, val);
        if (ret)
            break;
        udelay(200);
    }
    return ret;
}

static int gl5_write(struct lme_state *st, u8 reg, u8 val)
{
    return gl5_write_w32(st, reg, val);
}

/*
 * GL5 alt @ 0x1B — 两种 USB 路径（DTMB 固件均可能用到）：
 *   DTMB 05/85: selector 0x36 (Windows 注释: GL5 demod 0x05/0x85 sel 0x32/0x36)
 *   lmedm04 gate: 04/84 + selector 0x36 (vendorgl5 i2c fallback for addr 0x1B)
 */
        /* Windows FTM alt @ 0x1B: 05 04 36 <reg> <val> (no tail suffix) */

static int gl5_write_alt_05(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[5], rb[4];
    int ret;

    ob[0] = 0x05;
    ob[1] = 0x04;
    ob[2] = GL5_ALT_ADDR << 1;
    ob[3] = reg;
    ob[4] = val;
    ret = lme_usb_talk(st, ob, 5, rb, 4);
    dinfo("gl5_alt_w 05 04 36 %02x %02x ack=0x%02x ret=%d\n",
          reg, val, rb[0], ret);
    return ret;
}

/* Alt read: 85 02 36 <reg> (NOT 84 03 32 on demod 0x19) */
static int gl5_read_alt_05(struct lme_state *st, u8 reg, u8 *val)
{
    u8 ob[4], rb[5];
    int ret;

    lme_cmd09(st);
    ob[0] = 0x85;
    ob[1] = 0x02;
    ob[2] = GL5_ALT_ADDR << 1;
    ob[3] = reg;
    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk(st, ob, 4, rb, 5);
    if (ret == 0)
        *val = gl5_pick_reg_data(rb);
    dinfo("gl5_alt_r 85 02 36 %02x → 0x%02x ib=%02x %02x ret=%d\n",
          reg, *val, rb[0], rb[1], ret);
    return ret;
}

static int gl5_write_alt_gate(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[5], rb[4];
    int ret;

    ob[0] = LME_GATE_DEMOD;
    ob[1] = 3;
    ob[2] = GL5_ALT_ADDR << 1;
    ob[3] = reg;
    ob[4] = val;
    ret = lme_usb_talk(st, ob, 5, rb, 4);
    dinfo("gl5_alt_gate 04 03 36 %02x %02x ack=0x%02x ret=%d\n",
          reg, val, rb[0], ret);
    return ret;
}

/* vendorgl5: 04 03 36 C2 xx — parallel/serial + clock polarity (lgs8gxx TS_* bits in low 3) */
static void gl5_mpeg_ts_mode(struct lme_state *st)
{
    u8 vc2 = 0, c2 = (u8)(ts_c2_val & 0xff);

    gl5_write_alt_gate(st, 0xC2, c2);
    msleep(5);
    gl5_read_alt_05(st, 0xC2, &vc2);
    dinfo("GL5 C2: wrote 04 03 36 C2 %02x, read → C2=0x%02x 1F pending\n", c2, vc2);
}

/* CSV Seq 1061: 05 04 36 C5 06 — no tail suffix, no C5=01 follow-up */
static void gl5_ftm_ts_route_c5(struct lme_state *st)
{
    u8 vc2 = 0;
    u8 wc506[] = { 0x05, 0x04, 0x36, 0xc5, 0x06 };

    win3_cmd(st, wc506, sizeof(wc506), 1, "ftm C5=06");
    msleep(5);

    st->ftm_c5_done = ktime_get();
    st->ftm_c5_valid = true;

    gl5_read_alt_05(st, 0xC2, &vc2);
    dinfo("FTM C5 done: C2=0x%02x (05 04 36 C5 06, no tail)\n", vc2);
    dinfo("FTM TS route: C5=06 @ %lld ns\n", ktime_to_ns(st->ftm_c5_done));
}

static void gl5_ftm_soft_reset(struct lme_state *st)
{
    gl5_write_win(st, GL5_R_RESET, 0x00);
    msleep(5);
    gl5_write_win(st, GL5_R_RESET, 0x01);
    msleep(5);
}

static void gl5_ftm_log_4b_step(struct lme_state *st, const char *step)
{
    u8 v4b = 0, va4 = 0, v7e = 0, v7d = 0, v07 = 0;

    gl5_read_win(st, GL5_R_STRENGTH, &v4b);
    gl5_read_win(st, GL5_R_STATUS, &va4);
    gl5_read_win(st, GL5_R_7E, &v7e);
    gl5_read_win(st, GL5_R_7D, &v7d);
    gl5_read_win(st, GL5_R_07, &v07);
    dinfo("FTM post-lock after %s: 07=0x%02x 4b=0x%02x a4=0x%02x 7e=0x%02x 7d=0x%02x%s\n",
          step, v07, v4b, va4, v7e, v7d,
          ((v4b & 0x80) && (va4 & 0x01)) ? " LOCK" :
          ((v4b & 0x80) && v4b != 0xff) ? " carrier" : " NO-LOCK");
}

/*
 * 1(1).ftm lock→TS: 7e 00→01, 02×2, 7d 71→01, 02×2, C5. Never reg[07]=0x01.
 */
/*
 * CSV post-lock route (6262.csv Seq 1045→1109):
 *   A2=71 → 7E=00 → C5=06 → reset → 7D=71 → reset → all_pids → 06_00
 * No 7E=01, no 7D=01, no C5 triple — those kill lock.
 */
static void gl5_ftm_post_lock_route(struct lme_state *st)
{
    gl5_ftm_log_4b_step(st, "entry");

    /* 7E=00 — CSV Seq 1053: 05 04 32 7E 00 */
    gl5_write_win(st, GL5_R_7E, 0x00);
    msleep(5);
    gl5_ftm_log_4b_step(st, "7e=00");

    /* C5=06 — CSV Seq 1061: 05 04 36 C5 06 (5 bytes, no tail) */
    {
        u8 wc506[] = { 0x05, 0x04, 0x36, 0xc5, 0x06 };
        win3_cmd(st, wc506, sizeof(wc506), 1, "ftm C5=06");
    }
    gl5_ftm_log_4b_step(st, "C5=06");

    gl5_ftm_soft_reset(st);
    gl5_ftm_log_4b_step(st, "02 reset #1");

    /* 7D=71 — CSV Seq 1081: 05 04 32 7D 71 */
    gl5_write_win(st, GL5_R_7D, 0x71);
    msleep(5);
    gl5_ftm_log_4b_step(st, "7d=71");

    gl5_ftm_soft_reset(st);
    gl5_ftm_log_4b_step(st, "02 reset #2");

    gl5_log_ts_regs(st, "FTM post-C5 pre-bridge");
}

static __maybe_unused int gl5_write_alt_win(struct lme_state *st, u8 reg, u8 val)
{
    return gl5_write_alt_gate(st, reg, val);
}

static __maybe_unused void gl5_write_alt_block(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[5];

    ob[0] = LME_GATE_DEMOD;
    ob[1] = 3;
    ob[2] = GL5_ALT_ADDR << 1;
    ob[3] = reg;
    ob[4] = val;
    lme_usb_talk(st, ob, 5, NULL, 0);
}

/* 兼容旧名 */
static __maybe_unused int gl5_write_alt(struct lme_state *st, u8 reg, u8 val)
{
    return gl5_write_alt_gate(st, reg, val);
}

/* vendorgl5 i2c_xfer: write+read 合并 → 5B OUT + 2B IN */
static __maybe_unused int gl5_update_alt_gate(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[5], rb[2];
    u8 cur;
    int ret;

    ob[0] = LME_GATE_DEMOD | 0x80;
    ob[1] = 3;
    ob[2] = GL5_ALT_ADDR << 1;
    ob[3] = reg;
    ob[4] = 1;
    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk(st, ob, 5, rb, 2);
    cur = (rb[0] == 0x55) ? rb[1] : rb[0];
    dinfo("gl5_alt_rw gate 84 03 36 %02x 01 (5B out 2B in)"
          " rb=%02x %02x cur=0x%02x ret=%d\n",
          reg, rb[0], rb[1], cur, ret);

    return gl5_write_alt_gate(st, reg, val);
}

/* 05/85 update_alt: 85 02 36 reg → 05 04 36 reg val 01 */
static __maybe_unused int gl5_update_alt_05(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[4], rb[5];
    int ret;

    ob[0] = 0x85;
    ob[1] = 0x02;
    ob[2] = GL5_ALT_ADDR << 1;
    ob[3] = reg;
    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk(st, ob, 4, rb, 5);
    dinfo("gl5_alt_r 85 02 36 %02x ibuf=%02x %02x %02x %02x %02x ret=%d\n",
          reg, rb[0], rb[1], rb[2], rb[3], rb[4], ret);

    return gl5_write_alt_05(st, reg, val);
}

static int gl5_update_alt_reg(struct lme_state *st, u8 reg, u8 val)
{
    u8 rb[5], cur = 0;

    gl5_read_gate_raw(st, GL5_ALT_ADDR, reg, rb, true);
    cur = rb[1];
    dinfo("gl5_alt_rw gate 84 03 36 %02x 01 cur=0x%02x\n", reg, cur);
    return gl5_write_alt_gate(st, reg, val);
}

static void __maybe_unused gl5_log_ibuf_verdict(const char *tag, u8 reg, u8 rb[5])
{
    dinfo("GL5 %s reg[0x%02x] ibuf=%02x %02x %02x %02x %02x"
          " pick[1]=0x%02x pick[2]=0x%02x → ",
          tag, reg, rb[0], rb[1], rb[2], rb[3], rb[4], rb[1], rb[2]);
    if (rb[1] == 0x54)
        dinfo("ibuf[1]=0x54 GL5已通\n");
    else if (rb[2] == 0x54)
        dinfo("ibuf[2]=0x54 改pick=[2]\n");
    else if (!rb[1] && !rb[2] && !rb[3] && !rb[4])
        dinfo("全零 GL5仍未激活\n");
    else
        dinfo("数值待确认，重复读对比\n");
}

static int gl5_read(struct lme_state *st, u8 addr, u8 reg, u8 *val)
{
    u8 rb[5];
    int ret;

    ret = gl5_read_raw(st, addr, reg, rb, false);
    if (ret == 0)
        *val = gl5_pick_reg_data(rb);
    ddbg2("gl5_read addr=0x%02x reg=0x%02x ibuf=%02x %02x %02x %02x %02x"
          " b1=0x%02x b2=0x%02x pick=0x%02x ret=%d\n",
          addr, reg,
          rb[0], rb[1], rb[2], rb[3], rb[4],
          rb[1], rb[2], *val, ret);
    return ret;
}

/* 读后写（update_reg，对需要先读才能写的寄存器）*/
static __maybe_unused int gl5_update(struct lme_state *st, u8 reg, u8 val)
{
    u8 dummy;
    gl5_read(st, GL5_ADDR, reg, &dummy);
    return gl5_write(st, reg, val);
}

static __maybe_unused int gl5_update_alt(struct lme_state *st, u8 reg, u8 val)
{
    return gl5_update_alt_reg(st, reg, val);
}

/* Windows bridge init from 3.csv: BRIDGE_81 + BRIDGE_82.
 * Sent once after firmware upload to configure bridge for DTMB. */
static void __maybe_unused lme_bridge_windows_init(struct lme_state *st)
{
    u8 cmd81[] = {
        0x81, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    u8 cmd82[] = {
        0x82, 0x23, 0xe0, 0x54, 0x02, 0x70, 0x02, 0xd3,
        0x22, 0xc3, 0x22, 0xe4, 0xf5, 0x90, 0xc2, 0x88,
        0xd2, 0xa8, 0xd2, 0xaf, 0x22, 0x7d, 0xfc, 0xe4,
        0xff, 0x12, 0x11, 0xe8, 0xc2, 0x9c, 0x22, 0x90,
        0x90, 0x21, 0x74, 0x08, 0xf0, 0x22, 0x08
    };

    lme_usb_talk(st, cmd81, sizeof(cmd81), NULL, 0);
    msleep(20);
    lme_usb_talk(st, cmd82, sizeof(cmd82), NULL, 0);
    msleep(20);
    dinfo("bridge windows init done\n");
}

/* vendorgl5 dtmb_wait_usb_ready: 06 00 唤醒桥 */
static int __maybe_unused lme_wait_usb_ready(struct lme_state *st)
{
    u8 w[] = { 0x06, 0x00 };
    int elapsed = 0, ret;

    while (elapsed < 500) {
        ret = lme_usb_talk(st, w, 2, NULL, 0);
        if (ret == 0)
            return 0;
        msleep(50);
        elapsed += 50;
    }
    dinfo("wait_usb_ready timeout ret=%d\n", ret);
    return ret;
}

/* forward decls for WinInit/C0 helpers */
static void lme_bridge_flush(struct lme_state *st);
static void __maybe_unused max2165_c0_fixup(struct lme_state *st, const u8 cmd[17]);

/*
 * Windows demod read wrapper. No cmd09 before each read in the 3.csv path.
 */
static int gl5_demod_read(struct lme_state *st, u8 reg, u8 *val)
{
    u8 rb[5];
    int ret;

    ret = gl5_read_raw(st, GL5_ADDR, reg, rb, false);
    if (ret == 0)
        *val = gl5_pick_reg_data(rb);
    dinfo("gl5_demod_r reg=%02x ibuf=%02x %02x %02x %02x %02x val=0x%02x ret=%d\n",
          reg, rb[0], rb[1], rb[2], rb[3], rb[4], *val, ret);
    return ret;
}

/* vendorgl5 dtmb_windows_detect_and_prep */
static int gl5_win_detect_and_prep(struct lme_state *st)
{
    int ret, tries;
    u8 id = 0xff, reset = 0xff;

    dinfo("=== vendorgl5 WinDetect (warm=%d gate=%u) ===\n",
          st->warm_attach, st->active_gate);

    gl5_gate_probe_and_select(st, "WinDetect-pre");

    /*
     * Windows 3.csv after warm re-enumeration:
     *   85 02 32 00 00 -> 55 0e
     *   85 02 32 00 00 -> 55 0e
     *   85 02 32 02 00 -> 55 01
     * No cmd09 before this sequence; cmd09 makes Block 4 answer 77 00.
     */
    ret = -EIO;
    for (tries = 0; tries < 8; tries++) {
        if (!gl5_demod_read(st, 0x00, &id) && gl5_chip_id_valid(id)) {
            gl5_demod_read(st, 0x00, &id);
            gl5_demod_read(st, GL5_R_RESET, &reset);
            ret = 0;
            break;
        }
        msleep(80);
    }
    if (ret)
        dinfo("WinDetect read chip id reg00 failed after %d tries (warm=%d)\n",
              tries, st->warm_attach);

    st->win_demod_id = id;
    st->win_mode = (id == 0x0e) ? 0 : 1;
    /* LME2510C+GL5 DTMB stick: always GL5 path */
    st->win_mode = 0;

    dinfo("WinDetect chip_id(reg00)=0x%02x reset(reg02)=0x%02x warm=%d → force GL5 mode=%u\n",
          id, reset, st->warm_attach, st->win_mode);
    return 0;
}

/* vendorgl5 dtmb_win_write_e_mode: 05 04 32 01 val 01 */
static int gl5_win_write_e_mode(struct lme_state *st, u8 val)
{
    return gl5_write_w32(st, 0x01, val);
}

static int gl5_win_mode1_postcfg(struct lme_state *st)
{
    u8 rbuf[4];
    u8 seq1[] = { 0x05, 0x04, 0x01, 0x32, 0xe0 };
    u8 seq2[] = { 0x05, 0x04, 0x01, 0x32, 0x60 };
    int ret;

    ret = lme_usb_talk(st, seq1, sizeof(seq1), rbuf, sizeof(rbuf));
    dinfo("WinInit mode1_postcfg e0 ret=%d ack=0x%02x\n", ret, rbuf[0]);
    if (ret)
        return ret;
    ret = lme_usb_talk(st, seq2, sizeof(seq2), rbuf, sizeof(rbuf));
    dinfo("WinInit mode1_postcfg 60 ret=%d ack=0x%02x\n", ret, rbuf[0]);
    return ret;
}

/* vendorgl5 dtmb_win_mode_prepare */
static int gl5_win_mode_prepare(struct lme_state *st, const char *tag)
{
    u8 probe = 0xff;
    int ret;

    ret = lme_cmd16(st, st->win_mode);
    if (ret) {
        dinfo("%s cmd16 mode=%u FAILED (%d)\n", tag, st->win_mode, ret);
        return ret;
    }
    dinfo("%s cmd16 mode=%u ok\n", tag, st->win_mode);

    if (st->win_mode == 1) {
        ret = gl5_demod_read(st, 0x32, &probe);
        if (ret) {
            dinfo("%s mode1 prime FAILED (%d)\n", tag, ret);
            return ret;
        }
        dinfo("%s mode1 prime sel0x32=0x%02x\n", tag, probe);
    }

    ret = gl5_win_write_e_mode(st, 0xe0);
    if (ret) {
        dinfo("%s write E0 FAILED (%d)\n", tag, ret);
        return ret;
    }
    dinfo("%s write reg[01]=0xe0 ok\n", tag);
    return 0;
}

/* vendorgl5 dtmb_win_mode_finalize */
static int gl5_win_mode_finalize(struct lme_state *st, const char *tag)
{
    int ret;

    ret = gl5_win_write_e_mode(st, 0x60);
    if (ret) {
        dinfo("%s write 60 FAILED (%d)\n", tag, ret);
        return ret;
    }
    dinfo("%s write reg[01]=0x60 ok\n", tag);

    if (st->win_mode == 1) {
        ret = gl5_win_mode1_postcfg(st);
        if (ret) {
            dinfo("%s mode1 postcfg FAILED (%d)\n", tag, ret);
            return ret;
        }
        dinfo("%s mode1 postcfg ok\n", tag);
    }
    return 0;
}

/*
 * vendorgl5 probe WinInit (frontend_attach + tuner attach):
 *   detect(85 reg00/reg00/reg02, no cmd09) → prepare(cmd16,e0)
 *   → MAX2165 → finalize(60,postcfg)
 * C0 / soft-reset / 07=1c deferred to first tune (post_c0_sequence).
 */
static void __maybe_unused gl5_activate(struct lme_state *st)
{
    u8 v00, v07, v4b, va4, id_post = 0xff;
    int ret, tries;

    dinfo("=== vendorgl5 full WinInit (probe path warm=%d) ===\n",
          st->warm_attach);

    if (!st->active_gate)
        st->active_gate = 4;

    ret = gl5_win_detect_and_prep(st);
    if (ret)
        dinfo("WinDetect failed (%d), continuing with GL5 mode=0\n", ret);

    ret = gl5_win_mode_prepare(st, "WinInit");
    if (ret)
        derr("WinInit mode prepare failed (%d)\n", ret);

    max2165_init(st);

    ret = gl5_win_mode_finalize(st, "WinInit");
    if (ret)
        derr("WinInit mode finalize failed (%d)\n", ret);

    /*
     * Windows bridge uses 05 04 01 32 e0/60 as path select; vendorgl5 only
     * sends this for win_mode==1, but GL5 sticks may need it when detect fails.
     */
    ret = gl5_win_mode1_postcfg(st);
    dinfo("WinInit unconditional mode1_postcfg ret=%d\n", ret);

    /* Retry chip_id AFTER e0/60/postcfg (not only before cmd16) */
    for (tries = 0; tries < 8; tries++) {
        if (!gl5_demod_read(st, 0x00, &id_post) && gl5_chip_id_valid(id_post))
            break;
        msleep(80);
    }
    dinfo("WinDetect POST-postcfg chip_id(reg00)=0x%02x (pre-cmd16 was 0x%02x warm=%d)\n",
          id_post, st->win_demod_id, st->warm_attach);
    if (gl5_chip_id_valid(id_post))
        st->win_demod_id = id_post;

    gl5_gate_probe_and_select(st, "WinInit-post");

    gl5_demod_read(st, 0x00, &v00);
    gl5_demod_read(st, GL5_R_07, &v07);
    gl5_demod_read(st, GL5_R_STRENGTH, &v4b);
    gl5_demod_read(st, GL5_R_STATUS, &va4);
    dinfo("WinInit DONE gate=%u: chip_id(00)=0x%02x reg[07]=0x%02x "
          "4b=0x%02x a4=0x%02x mode=%u id=0x%02x warm=%d\n",
          st->active_gate, v00, v07, v4b, va4, st->win_mode,
          st->win_demod_id, st->warm_attach);

    if (debug > 1) {
        u8 grb[5];

        gl5_i2c_gate_read(st, LME_GATE_DEMOD, GL5_ADDR, 0x32, grb, true);
        gl5_i2c_gate_read(st, LME_GATE_DEMOD, GL5_ADDR, GL5_R_07, grb, false);
        gl5_i2c_gate_read(st, LME_GATE_DEMOD, GL5_ADDR, GL5_R_STRENGTH, grb, false);
        gl5_i2c_gate_read(st, LME_GATE_DEMOD, GL5_ADDR, GL5_R_STATUS, grb, false);
    }
}

/* 软复位 */
static void gl5_soft_reset(struct lme_state *st)
{
    u8 val;

    gl5_read_win(st, GL5_R_RESET, &val);
    gl5_write_win(st, GL5_R_RESET, val & ~0x01);
    gl5_write_win(st, GL5_R_RESET, val |  0x01);
    msleep(5);
}

/*
 * 写关键配置寄存器并读回验证
 * 返回0=写入正确，负数=USB错误，1=写入未生效
 */
static int __maybe_unused gl5_write_verify(struct lme_state *st, u8 reg, u8 val)
{
    u8 readback = 0xFF;
    int ret;

    ret = gl5_write3(st, GL5_ADDR, reg, val);
    if (ret) return ret;

    msleep(5);   /* 增加等待时间确保写入稳定 */
    ret = gl5_read(st, GL5_ADDR, reg, &readback);
    if (ret) {
        derr("gl5_verify: read reg[0x%02x] 失败 ret=%d\n", reg, ret);
        return ret;
    }

    /* 无论成功失败都打印，方便对比 */
    dinfo("gl5_verify: reg[0x%02x] 写0x%02x 读回0x%02x %s\n",
          reg, val, readback,
          (readback == val) ? "✓" : "✗ 未生效!");

    return (readback == val) ? 0 : 1;
}

static int win3_cmd(struct lme_state *st, const u8 *cmd, int len,
                    int rlen, const char *tag)
{
    u8 ob[32], rb[8];
    int ret;

    if (len > sizeof(ob) || rlen > sizeof(rb))
        return -EINVAL;

    memcpy(ob, cmd, len);
    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk(st, ob, len, rb, rlen);

    if (len == 5 && cmd[0] == 0x05 && cmd[2] == GL5_DEMOD_SEL)
        st->gl5_read_hint = cmd[4];

    dinfo("3csv %-18s ack=%02x %02x ret=%d\n", tag, rb[0], rb[1], ret);
    return ret;
}

static int win3_read85(struct lme_state *st, u8 sel, u8 reg, u8 hint,
                       const char *tag, u8 *val)
{
    u8 cmd[] = { 0x85, 0x02, sel, reg, hint };
    u8 rb[5];
    int ret;

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk(st, cmd, sizeof(cmd), rb, 2);
    if (val)
        *val = (rb[0] == 0x55) ? rb[1] : 0;
    dinfo("3csv %-18s 85 02 %02x %02x %02x -> %02x %02x ret=%d\n",
          tag, sel, reg, hint, rb[0], rb[1], ret);
    return ret;
}

static int win3_read84(struct lme_state *st, u8 sel, u8 reg, u8 suffix,
                       const char *tag, u8 *val)
{
    u8 cmd[] = { 0x84, 0x03, sel, reg, suffix };
    u8 rb[5];
    int ret;

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk(st, cmd, sizeof(cmd), rb, 2);
    if (val)
        *val = (rb[0] == 0x55) ? rb[1] : 0;
    dinfo("3csv %-18s 84 03 %02x %02x %02x -> %02x %02x ret=%d\n",
          tag, sel, reg, suffix, rb[0], rb[1], ret);
    return ret;
}

static void win3_probe_init(struct lme_state *st)
{
    u8 id = 0, reset = 0;
    static const u8 cmd16[] = { 0x16, 0x01, 0x00 };
    static const u8 w0200[] = { 0x05, 0x04, 0x32, 0x02, 0x00 };
    static const u8 w0201[] = { 0x05, 0x04, 0x32, 0x02, 0x01 };
    static const u8 w01e0[] = { 0x05, 0x04, 0x32, 0x01, 0xe0 };
    static const u8 w0160[] = { 0x05, 0x04, 0x32, 0x01, 0x60 };
    static const u8 w0d01[] = { 0x04, 0x03, 0xc0, 0x0d, 0x01 };
    static const u8 w0d02[] = { 0x04, 0x03, 0xc0, 0x0d, 0x02 };
    static const u8 w0d03[] = { 0x04, 0x03, 0xc0, 0x0d, 0x03 };
    static const u8 w0d04[] = { 0x04, 0x03, 0xc0, 0x0d, 0x04 };
    static const u8 w0d05[] = { 0x04, 0x03, 0xc0, 0x0d, 0x05 };
    static const u8 w0d00[] = { 0x04, 0x03, 0xc0, 0x0d, 0x00 };
    /* Single c0 blob matching Windows 6262.csv: 04 11 C0 00 <15 regs> */
    static const u8 wblob[] = {
        0x04, 0x11, 0xc0, 0x00,
        0x27, 0x18, 0x00, 0x00, 0xf2,  /* reg 00-04: NDIV/FRAC/TF */
        0x01, 0x0a, 0x08, 0x02, 0x54,  /* reg 05-09: LNA/PLL_CFG/TEST/SHDN/VCO */
        0x73, 0x75, 0x00, 0x00, 0x00   /* reg 0A-0E: BB/DCO/DAC/ROM/xxx */
    };
    static const u8 w079f[] = { 0x05, 0x04, 0x32, 0x07, 0x9f };
    static const u8 w0900[] = { 0x05, 0x04, 0x32, 0x09, 0x00 };
    static const u8 w0a00[] = { 0x05, 0x04, 0x32, 0x0a, 0x00 };
    static const u8 w0b00[] = { 0x05, 0x04, 0x32, 0x0b, 0x00 };
    static const u8 w0c00[] = { 0x05, 0x04, 0x32, 0x0c, 0x00 };
    static const u8 w071c[] = { 0x05, 0x04, 0x32, 0x07, 0x1c };

    dinfo("=== 3csv strict probe init ===\n");
    win3_read85(st, 0x32, 0x00, 0x00, "chip#1", &id);
    win3_cmd(st, cmd16, sizeof(cmd16), 2, "cmd16");
    win3_read85(st, 0x32, 0x00, 0x00, "chip#2", &id);
    win3_read85(st, 0x32, 0x02, 0x00, "reset", &reset);
    win3_cmd(st, w0200, sizeof(w0200), 1, "32.02=00");
    win3_cmd(st, w0201, sizeof(w0201), 1, "32.02=01");
    msleep(12);
    win3_cmd(st, w01e0, sizeof(w01e0), 1, "32.01=e0");
    win3_cmd(st, w0d01, sizeof(w0d01), 1, "c0.0d=01");
    win3_read84(st, 0xc0, 0x10, 0x01, "c0.10#1", NULL);
    win3_cmd(st, w0d02, sizeof(w0d02), 1, "c0.0d=02");
    win3_read84(st, 0xc0, 0x10, 0x01, "c0.10#2", NULL);
    win3_cmd(st, w0d03, sizeof(w0d03), 1, "c0.0d=03");
    win3_read84(st, 0xc0, 0x10, 0x01, "c0.10#3", NULL);
    win3_cmd(st, w0d04, sizeof(w0d04), 1, "c0.0d=04");
    win3_read84(st, 0xc0, 0x10, 0x01, "c0.10#4", NULL);
    win3_cmd(st, w0d05, sizeof(w0d05), 1, "c0.0d=05");
    win3_read84(st, 0xc0, 0x10, 0x01, "c0.10#5", NULL);
    win3_cmd(st, w0d00, sizeof(w0d00), 1, "c0.0d=00");
    win3_cmd(st, wblob, sizeof(wblob), 1, "c0 blob");

    win3_cmd(st, w0160, sizeof(w0160), 1, "32.01=60");
    win3_read85(st, 0x32, 0x07, 0x60, "32.07 read", NULL);
    win3_cmd(st, w079f, sizeof(w079f), 1, "32.07=9f");
    win3_cmd(st, w0900, sizeof(w0900), 1, "32.09=00");
    win3_cmd(st, w0a00, sizeof(w0a00), 1, "32.0a=00");
    win3_cmd(st, w0b00, sizeof(w0b00), 1, "32.0b=00");
    win3_cmd(st, w0c00, sizeof(w0c00), 1, "32.0c=00");
    win3_read85(st, 0x32, 0x07, 0x00, "32.07 verify", NULL);
    win3_cmd(st, w071c, sizeof(w071c), 1, "32.07=1c");

    /* Verify MAX2165 init registers via c0 blob */
    {
        u8 v00, v04, v05, v06, v08, v09;
        win3_read84(st, 0xc0, 0x00, 0x01, "c0.00 NDIV", &v00);
        win3_read84(st, 0xc0, 0x04, 0x01, "c0.04 TF", &v04);
        win3_read84(st, 0xc0, 0x05, 0x01, "c0.05 LNA", &v05);
        win3_read84(st, 0xc0, 0x06, 0x01, "c0.06 PLL", &v06);
        win3_read84(st, 0xc0, 0x08, 0x01, "c0.08 SHDN", &v08);
        win3_read84(st, 0xc0, 0x09, 0x01, "c0.09 VCO", &v09);
        dinfo("MAX2165: NDIV=0x%02x TF=0x%02x LNA=0x%02x PLL=0x%02x SHDN=0x%02x VCO=0x%02x\n",
              v00, v04, v05, v06, v08, v09);
    }

    st->win_demod_id = id;
    st->win_mode = 0;
    dinfo("3csv probe init done chip_id=0x%02x reset=0x%02x\n", id, reset);
}

/* ================================================================== */
/* LGS8GL5 初始化                                                      */
/* ================================================================== */

static int gl5_init(struct lme_state *st)
{
    u8 v07 = 0, v09, v4b, va4;

    dinfo("GL5 init 开始\n");

    win3_probe_init(st);

    gl5_read(st, GL5_ADDR, GL5_R_09,       &v09);
    gl5_read(st, GL5_ADDR, GL5_R_STRENGTH, &v4b);
    gl5_read(st, GL5_ADDR, GL5_R_STATUS,   &va4);
    dinfo("GL5 复位默认值: reg[07]=0x%02x reg[09]=0x%02x "
          "reg[4b]=0x%02x reg[a4]=0x%02x\n",
          v07, v09, v4b, va4);

    /* 再读一次确认 */
    gl5_read(st, GL5_ADDR, GL5_R_07, &v07);
    dinfo("GL5 init 完成: reg[07]=0x%02x (期望0x1c)\n", v07);

    return 0;
}

/* ================================================================== */
/* LGS8GL5 解调启动                                                    */
/* ================================================================== */

/*
 * vendorgl5 dtmb_win_gl5_start_demod: always arm demod→TS route (04/37/7e)
 * before lock poll; locked sticks still need 04=02 and 37=01.
 */
/* Locked post-C0: vendorgl5 route regs without soft-reset (keep 03=0x01 from post-C0) */
static void gl5_demod_route_locked(struct lme_state *st)
{
    u8 r03 = 0, r04 = 0, r37 = 0;

    gl5_read_win(st, GL5_R_03, &r03);
    if (r03 != 0x01)
        gl5_write_win3(st, GL5_R_03, 0x00);
    gl5_write_win3(st, GL5_R_7E, 0x01);
    gl5_write_alt_gate(st, 0xC5, 0x00);
    gl5_write_win3(st, GL5_R_04, 0x02);
    msleep(5);
    gl5_write_win3(st, GL5_R_37, 0x01);
    msleep(5);
    gl5_read_win(st, GL5_R_03, &r03);
    gl5_read_win(st, GL5_R_04, &r04);
    gl5_read_win(st, GL5_R_37, &r37);
    dinfo("demod route locked: 7e/C5/04/37 readback 03=0x%02x 04=0x%02x 37=0x%02x\n",
          r03, r04, r37);
}

/* demod→TS routing regs — always (locked or not); no soft reset */
static void gl5_demod_route_writes(struct lme_state *st)
{
    gl5_write_win3(st, GL5_R_03, 0x00);
    gl5_write_win3(st, GL5_R_7E, 0x01);
    gl5_write_alt_gate(st, 0xC5, 0x00);
    gl5_write_win3(st, GL5_R_07, 0x1c);
    gl5_write_win3(st, GL5_R_09, 0x00);
    gl5_write_win3(st, GL5_R_0A, 0x00);
    gl5_write_win3(st, GL5_R_0B, 0x00);
    gl5_write_win3(st, GL5_R_0C, 0x00);
    gl5_write_win3(st, GL5_R_04, 0x02);
    msleep(5);
    gl5_write_win3(st, GL5_R_37, 0x01);
    msleep(5);
}

/* vendorgl5 full start_demod when not yet locked (C2 + reset + route + reset) */
static void gl5_demod_route_arm(struct lme_state *st)
{
    gl5_mpeg_ts_mode(st);
    gl5_write_win3(st, GL5_R_RESET, 0x00);
    msleep(10);
    gl5_write_win3(st, GL5_R_RESET, 0x01);
    msleep(10);
    gl5_demod_route_writes(st);
    gl5_write_win3(st, GL5_R_RESET, 0x00);
    msleep(10);
    gl5_write_win3(st, GL5_R_RESET, 0x01);
    msleep(10);
    msleep(100);
    dinfo("demod route arm: full vendorgl5 (C2+reset+04/37+reset)\n");
}

static int gl5_win_start_demod(struct lme_state *st)
{
    u8 v4b = 0, va4 = 0, va2 = 0;
    int n;
    bool locked;

    msleep(200);
    locked = gl5_hw_locked(st);
    if (locked) {
        gl5_read_win(st, GL5_R_STRENGTH, &v4b);
        gl5_read_win(st, GL5_R_STATUS, &va4);
        dinfo("win_start_demod: pre-route LOCK 4b=0x%02x a4=0x%02x\n", v4b, va4);
        gl5_mpeg_ts_mode(st);
        gl5_demod_route_locked(st);
        gl5_read_win(st, GL5_R_STRENGTH, &v4b);
        gl5_read_win(st, GL5_R_STATUS, &va4);
        dinfo("win_start_demod: post-route 4b=0x%02x a4=0x%02x → enable_ts_locked\n",
              v4b, va4);
        gl5_enable_ts_locked(st);
        gl5_ts_hw_regs_log(st, "post-enable_ts_locked");
        st->gl5_ts_enabled = true;
        lme_ts_prime_after_enable_ts(st);
        goto stream_start;
    }

    gl5_demod_route_arm(st);

    for (n = 0; n < 200; n++) {
        gl5_read(st, GL5_ADDR, GL5_R_STRENGTH, &v4b);
        if ((v4b & 0x80) && v4b != 0xff)
            break;
        msleep(25);
    }
    if (!(v4b & 0x80)) {
        gl5_read(st, GL5_ADDR, GL5_R_STATUS, &va4);
        dinfo("win_start_demod: no carrier 4b=0x%02x a4=0x%02x\n", v4b, va4);
        return 0;
    }

    for (n = 0; n < 120; n++) {
        gl5_read(st, GL5_ADDR, GL5_R_STATUS, &va4);
        if (va4 & 0x01)
            break;
        msleep(25);
    }
    if (!(va4 & 0x01)) {
        gl5_read(st, GL5_ADDR, GL5_R_STRENGTH, &v4b);
        dinfo("win_start_demod: carrier ok, no lock 4b=0x%02x a4=0x%02x\n",
              v4b, va4);
        return 0;
    }

    gl5_read(st, GL5_ADDR, GL5_R_STRENGTH, &v4b);
    gl5_read(st, GL5_ADDR, GL5_R_STATUS, &va4);
    dinfo("win_start_demod: LOCK 4b=0x%02x a4=0x%02x\n", v4b, va4);

    gl5_read(st, GL5_ADDR, 0xa2, &va2);
    gl5_write_win3(st, GL5_R_7D, va2);
    msleep(5);
    gl5_write_win3(st, GL5_R_RESET, 0x00);
    msleep(10);
    gl5_write_win3(st, GL5_R_RESET, 0x01);
    msleep(10);

    gl5_enable_ts(st);
    st->gl5_ts_enabled = true;
    lme_ts_prime_after_enable_ts(st);

stream_start:
    if (ts_probe_on_tune)
        gl5_ts_enable_probe(st);

    if (st->feedcount > 0 && atomic_read(&st->ts_active)) {
        msleep(5);
        lme_ts_bridge_restart(st);
        gl5_ts_hw_regs_log(st, "post-bridge");
        st->ts_stream_started = true;
        st->ts_bridge_pending = false;
        dinfo("win_start_demod: stream_restart (URB up)\n");
    } else if (st->ts_stream_started) {
        st->ts_bridge_pending = false;
        dinfo("win_start_demod: TS primed (URB+06 in enable_ts window)\n");
    } else {
        st->ts_bridge_pending = !st->gl5_ts_enabled;
        dinfo("win_start_demod: %s (no feed yet)\n",
              st->gl5_ts_enabled ? "TS enabled, wait start_feed" :
              "bridge pending");
    }
    return 1;
}

static bool gl5_hw_locked(struct lme_state *st)
{
    u8 v4b = 0, va4 = 0;

    gl5_read_win(st, GL5_R_STRENGTH, &v4b);
    gl5_read_win(st, GL5_R_STATUS, &va4);

    return (v4b & 0x80) && v4b != 0xff && (va4 & 0x01);
}

/* Hypothesis A: log GL5 + alt MPEG regs at lock (lgs8gxx: TS mode=0xC2, not 0x1F) */
static void gl5_ts_hw_regs_log(struct lme_state *st, const char *tag)
{
    u8 v00 = 0, v06 = 0, v1f = 0, v20 = 0, vc2 = 0, v7d = 0, v7e = 0;

    gl5_read_win(st, 0x00, &v00);
    gl5_read_win(st, 0x06, &v06);
    gl5_read_win(st, 0x1F, &v1f);
    gl5_read_win(st, 0x20, &v20);
    gl5_read_win(st, GL5_R_7D, &v7d);
    gl5_read_win(st, GL5_R_7E, &v7e);
    gl5_read_alt_05(st, 0xC2, &vc2);
    dinfo("TS_hw_regs [%s]: 00=0x%02x 06=0x%02x 1F=0x%02x 20=0x%02x "
          "C2=0x%02x 7d=0x%02x 7e=0x%02x\n",
          tag, v00, v06, v1f, v20, vc2, v7d, v7e);
}

/* FTM post-lock GL5 route; demod may drop lock briefly during 02 reset */
static __maybe_unused int gl5_enable_ts_wait_lock(struct lme_state *st)
{
    int n;

    gl5_enable_ts(st);
    for (n = 0; n < 120; n++) {
        if (gl5_hw_locked(st)) {
            dinfo("GL5 TS route: lock restored after %d ms\n", n * 25);
            return 0;
        }
        msleep(25);
    }
    dinfo("GL5 TS route: lock NOT restored after enable_ts\n");
    return -ETIMEDOUT;
}

/* SET_INTERFACE only while no TS URBs are in flight (usbmon: else Bi:6 -108) */
static bool lme_ts_alt1_active(struct lme_state *st)
{
    struct usb_host_interface *alt;

    if (!st->udev->actconfig || !st->udev->actconfig->interface[0])
        return false;
    alt = st->udev->actconfig->interface[0]->cur_altsetting;
    return alt && alt->desc.bAlternateSetting == 1;
}

static const char *gl5_lock_hint(u8 v4b, u8 va4)
{
    if ((v4b & 0x80) && (va4 & 0x01))
        return " LOCK";
    if ((v4b & 0x80) && v4b != 0xff)
        return " carrier";
    return " NO-LOCK";
}

/* 4b/a4/7e/7d diagnostic (usbmon correlation) */
static void gl5_log_ts_regs(struct lme_state *st, const char *tag)
{
    u8 v4b = 0, va4 = 0, v7e = 0, v7d = 0, v07 = 0;

    gl5_read_win(st, GL5_R_07, &v07);
    gl5_read_win(st, GL5_R_STRENGTH, &v4b);
    gl5_read_win(st, GL5_R_STATUS, &va4);
    gl5_read_win(st, GL5_R_7E, &v7e);
    gl5_read_win(st, GL5_R_7D, &v7d);
    dinfo("GL5 [%s]: 07=0x%02x 4b=0x%02x a4=0x%02x 7e=0x%02x 7d=0x%02x%s\n",
          tag, v07, v4b, va4, v7e, v7d, gl5_lock_hint(v4b, va4));
}

/* reg[01]=TS 并口/时钟极性, reg[03]=路由; FTM post-C0 末值通常 01=0x01, 03=0x01 */
static void gl5_log_iface_regs(struct lme_state *st, const char *tag)
{
    u8 r01 = 0, r03 = 0, r07 = 0;

    gl5_read_win(st, 0x01, &r01);
    gl5_read_win(st, GL5_R_03, &r03);
    gl5_read_win(st, GL5_R_07, &r07);
    dinfo("GL5 iface [%s]: reg[01]=0x%02x reg[03]=0x%02x reg[07]=0x%02x "
          "(FTM ref post-C0: 01=0x01 03=0x01 07=0x1c)\n",
          tag, r01, r03, r07);
}

static void lme_ts_bridge_log_lock(struct lme_state *st, const char *ctx, const char *when)
{
    char tag[56];

    scnprintf(tag, sizeof(tag), "%s %s", ctx, when);
    gl5_log_ts_regs(st, tag);
}

static bool lme_ts_should_use_report2(const struct lme_state *st, const char *ctx)
{
    if (dtmb_ftm602_handshake > 0) {
        dinfo("TS %s: forcing report2 handshake via module param (A4=0x%02x A2=0x%02x 4B=0x%02x freq=%u)\n",
              ctx, st->ftm_last_a4, st->ftm_last_a2, st->ftm_last_4b,
              st->tuned_freq_hz);
        return true;
    }
    return false;
}

/*
 * CSV stream handshake (602.csv Seq 3095–3128 / 6262.csv Seq 1101–1112):
 *   OUT 03 06 00 FF 01 1F 20 81 → IN 88
 *   OUT 03 06 00 FF 01 1F 20 81 → IN 88
 *   OUT 06 00                  → IN 88
 */
static int lme_ts_csv_stream_handshake(struct lme_state *st, const char *ctx)
{
    u8 all_pids[] = { 0x03, 0x06, 0x00, 0xFF, 0x01, 0x1F, 0x20, 0x81 };
    u8 stream_on[] = { 0x06, 0x00 };
    u8 rb[8];
    int ret, step;

    for (step = 1; step <= 2; step++) {
        ret = lme_usb_talk(st, all_pids, sizeof(all_pids), rb, 1);
        dinfo("TS %s: all_pids#%d ack=%02x ret=%d\n", ctx, step,
              ret == 0 ? rb[0] : 0, ret);
    }

    ret = lme_usb_talk(st, stream_on, sizeof(stream_on), rb, 1);
    dinfo("TS %s: 06_00 ack=%02x ret=%d\n", ctx, ret == 0 ? rb[0] : 0, ret);
    return ret;
}

/* Legacy name — same as full CSV handshake */
static void lme_ts_bridge_all_pids(struct lme_state *st, const char *ctx)
{
    lme_ts_csv_stream_handshake(st, ctx);
}

static void __maybe_unused lme_ts_bridge_06_burst(struct lme_state *st)
{
    int i;
    ktime_t t0;

    for (i = 0; i < LME_TS_BRIDGE_ON_CNT; i++) {
        t0 = ktime_get();
        if (i == 0 && st->ftm_c5_valid) {
            s64 us = ktime_us_delta(t0, st->ftm_c5_done);

            dinfo("FTM timing: first 06 00 at +%lld us after C5 (FTM ref ~ms–s)\n", us);
        }
        lme_ts_send_stream_on(st);
        if (i < 5 || i == LME_TS_BRIDGE_ON_CNT - 1)
            dinfo("TS 06 00 burst[%d/%d] t+%lld us\n", i + 1,
                  LME_TS_BRIDGE_ON_CNT,
                  st->ftm_c5_valid ?
                  ktime_us_delta(ktime_get(), st->ftm_c5_done) : 0LL);
        msleep(LME_TS_KEEPALIVE_MS);
    }
    lme_ts_keepalive_start(st);
    dinfo("TS 06 00: sent %d × %dms, keepalive every %dms\n",
          LME_TS_BRIDGE_ON_CNT, LME_TS_KEEPALIVE_MS, LME_TS_KEEPALIVE_MS);
    gl5_log_ts_regs(st, "post-06-burst");
}

/*
 * Windows 6262.csv FTM post-C0 sequence.
 * Runs after lme_c0_tune has sent C0 + BASEBAND + TRACK_FILTER|0x40.
 * Exact replay of the USBlyzer capture: 01=60 → 03=00 → 7E=01 →
 * C5=00 → reset → 04=02/37=01 → reset → lock poll → post-lock → TS route.
 */
static void gl5_ftm_post_c0(struct lme_state *st)
{
    u8 val = 0;
    bool use_short_path;
    int n;
    static const u8 w0200[] = { 0x05, 0x04, 0x32, 0x02, 0x00 };
    static const u8 w0201[] = { 0x05, 0x04, 0x32, 0x02, 0x01 };
    static const u8 w0160[] = { 0x05, 0x04, 0x32, 0x01, 0x60 };
    static const u8 w0300[] = { 0x05, 0x04, 0x32, 0x03, 0x00 };
    static const u8 w7e01[] = { 0x05, 0x04, 0x32, 0x7e, 0x01 };
    static const u8 wc500[] = { 0x05, 0x04, 0x36, 0xc5, 0x00 };
    static const u8 w0402[] = { 0x05, 0x04, 0x32, 0x04, 0x02 };
    static const u8 w3701[] = { 0x05, 0x04, 0x32, 0x37, 0x01 };
    static const u8 w0400[] = { 0x05, 0x04, 0x32, 0x04, 0x00 };
    static const u8 w7e00[] = { 0x05, 0x04, 0x32, 0x7e, 0x00 };
    static const u8 wc506[] = { 0x05, 0x04, 0x36, 0xc5, 0x06 };
    static const u8 w7d71[] = { 0x05, 0x04, 0x32, 0x7d, 0x71 };

    dinfo("=== Windows FTM post-C0 ===\n");
    use_short_path = gl5_use_short_ftm_path(st);
    st->ftm_last_4b = 0;
    st->ftm_last_a4 = 0;
    st->ftm_last_a2 = 0;

    /* Step 5-7: 01=60, read 03, 03=00 */
    win3_cmd(st, w0160, sizeof(w0160), 1, "ftm 01=60");
    win3_read85(st, 0x32, 0x03, 0x60, "ftm 03 rd", &val);
    win3_cmd(st, w0300, sizeof(w0300), 1, "ftm 03=00");

    /* Step 8-9: read 7E, 7E=01 */
    win3_read85(st, 0x32, 0x7e, 0x00, "ftm 7e rd", &val);
    win3_cmd(st, w7e01, sizeof(w7e01), 1, "ftm 7e=01");

    /* Step 10-11: read gate-6 C5, C5=00 */
    win3_read85(st, 0x36, 0xc5, 0x01, "ftm 36.c5 rd", &val);
    win3_cmd(st, wc500, sizeof(wc500), 1, "ftm 36.c5=00");

    /* Step 12-15: read chip/reset, GL5 reset */
    win3_read85(st, 0x32, 0x00, 0x00, "ftm chip#1", NULL);
    win3_read85(st, 0x32, 0x02, 0x00, "ftm rst#1", NULL);
    win3_cmd(st, w0200, sizeof(w0200), 1, "ftm 02=00");
    win3_cmd(st, w0201, sizeof(w0201), 1, "ftm 02=01");

    /* Step 16-19: read 04/37, write 04=02, 37=01 */
    win3_read85(st, 0x32, 0x04, 0x01, "ftm 04 rd", NULL);
    win3_read85(st, 0x32, 0x37, 0x01, "ftm 37 rd", NULL);
    win3_cmd(st, w0402, sizeof(w0402), 1, "ftm 04=02");
    win3_cmd(st, w3701, sizeof(w3701), 1, "ftm 37=01");

    /* Step 20-23: read chip/reset, GL5 reset */
    win3_read85(st, 0x32, 0x00, 0x01, "ftm chip#2", NULL);
    win3_read85(st, 0x32, 0x02, 0x01, "ftm rst#2", NULL);
    win3_cmd(st, w0200, sizeof(w0200), 1, "ftm 02=00");
    win3_cmd(st, w0201, sizeof(w0201), 1, "ftm 02=01");

    if (use_short_path) {
        dinfo("FTM path: short lock path for %u Hz\n", st->tuned_freq_hz);
        n = gl5_ftm_short_lock_probe(st, &val);
    } else {
        dinfo("FTM path: full 626-style lock path for %u Hz\n", st->tuned_freq_hz);
        for (n = 0; n < 25; n++) {
            win3_read85(st, 0x32, 0x4b, 0x01, "ftm poll 4b", &val);
            if (val & 0x80)
                break;
            msleep(32);
        }
        st->ftm_last_4b = val;
        dinfo("FTM 4b poll: 0x%02x after %d tries\n", val, n + 1);

        win3_read85(st, 0x32, 0x04, 0x01, "ftm 04 post", NULL);
        win3_read85(st, 0x32, 0x37, 0x01, "ftm 37 post", NULL);
        win3_cmd(st, w0400, sizeof(w0400), 1, "ftm 04=00");
        win3_cmd(st, w3701, sizeof(w3701), 1, "ftm 37=01");

        win3_read85(st, 0x32, 0x00, 0x01, "ftm chip#3", NULL);
        win3_read85(st, 0x32, 0x02, 0x01, "ftm rst#3", NULL);
        win3_cmd(st, w0200, sizeof(w0200), 1, "ftm 02=00");
        win3_cmd(st, w0201, sizeof(w0201), 1, "ftm 02=01");

        win3_read85(st, 0x32, 0x4b, 0x01, "ftm 4b lock", &val);
        st->ftm_last_4b = val;
        for (n = 0; n < 4; n++) {
            win3_read85(st, 0x32, 0xa4, 0x01, "ftm poll a4", &val);
            if (val & 0x01)
                break;
            msleep(20);
        }
    }
    st->ftm_last_a4 = val;
    dinfo("FTM A4 stage final: 0x%02x after %d tries\n", val, n + 1);

    /* Step 32-34: read A2, read 7E, 7E=00 */
    usleep_range(1000, 2000);
    win3_read85(st, 0x32, 0xa2, 0x01, "ftm a2", &val);
    st->ftm_last_a2 = val;
    dinfo("FTM A2: 0x%02x\n", val);
    win3_read85(st, 0x32, 0x7e, 0x01, "ftm 7e final", NULL);
    win3_cmd(st, w7e00, sizeof(w7e00), 1, "ftm 7e=00");

    /* Step 35-36: read gate-6 C5, C5=06 */
    win3_read85(st, 0x36, 0xc5, 0x00, "ftm 36.c5 fin", NULL);
    win3_cmd(st, wc506, sizeof(wc506), 1, "ftm 36.c5=06");

    /* Step 37: read chip/reset, GL5 reset */
    win3_read85(st, 0x32, 0x00, 0x06, "ftm chip#4", NULL);
    win3_read85(st, 0x32, 0x02, 0x06, "ftm rst#4", NULL);
    win3_cmd(st, w0200, sizeof(w0200), 1, "ftm 02=00");
    win3_cmd(st, w0201, sizeof(w0201), 1, "ftm 02=01");

    /* Step 38-39: 7D=71, read chip/reset, GL5 reset */
    win3_cmd(st, w7d71, sizeof(w7d71), 1, "ftm 7d=71");
    win3_read85(st, 0x32, 0x00, 0x71, "ftm chip#5", NULL);
    win3_read85(st, 0x32, 0x02, 0x71, "ftm rst#5", NULL);
    win3_cmd(st, w0200, sizeof(w0200), 1, "ftm 02=00");
    win3_cmd(st, w0201, sizeof(w0201), 1, "ftm 02=01");

    st->gl5_ts_enabled = true;
    st->ts_bridge_pending = true;
    st->ts_stream_started = false;

    /*
     * The confirmed macOS 602 success route keeps short-path FTM separate
     * from the later stream handshake. Preserve in-window TS prime only for
     * the 626-style full path; let short-path frequencies wait for start_feed.
     */
    if (gl5_prime_ts_inside_ftm(st)) {
        if (lme_ts_prime_after_enable_ts(st) == 0)
            dinfo("FTM post-C0: TS prime moved into tune window\n");
        else
            derr("FTM post-C0: TS prime in tune window failed\n");
    } else {
        st->ts_stream_started = false;
        st->ts_bridge_pending = true;
        dinfo("FTM post-C0: short-path freq, defer all_pids/06_00 to start_feed\n");
    }

    dinfo("=== FTM post-C0 done ===\n");
}

/* CSV TS bridge (602.csv / 6262.csv): all_pids×2 + 06_00, each ack IN 88 */
static void lme_ts_bridge_stream_cmds(struct lme_state *st, const char *ctx)
{
    if (lme_ts_should_use_report2(st, ctx)) {
        int ret;
        ret = lme_ftm602_report2_handshake(st, ctx);
        dinfo("TS %s: report2 handshake ret=%d\n", ctx, ret);
        st->ts_stream_started = (ret == 0);
        st->ts_bridge_pending = false;
        dinfo("TS %s: done, expect TS on EP 0x%02x\n", ctx, LME_EP_TS_IN);
        return;
    }
    lme_ts_csv_stream_handshake(st, ctx);

    st->ts_stream_started = true;
    st->ts_bridge_pending = false;
    dinfo("TS %s: done, expect TS on EP 0x%02x\n", ctx, LME_EP_TS_IN);
}

/* Alt 1 + clear_halt only — before URB submit (never set_interface after URBs) */
static void lme_ts_bridge_set_alt(struct lme_state *st)
{
    int ret;

    if (!lme_ts_alt1_active(st)) {
        ret = usb_set_interface(st->udev, 0, 1);
        dinfo("TS bridge: set_interface(0,1) ret=%d (before URBs)\n", ret);
    } else {
        dinfo("TS bridge: already alt 1\n");
    }
}

static void __maybe_unused lme_ts_wait_demod_lock(struct lme_state *st)
{
    int n;

    for (n = 0; n < 80; n++) {
        u8 v4b = 0;

        if (gl5_hw_locked(st))
            break;
        gl5_read_win(st, GL5_R_STRENGTH, &v4b);
        if ((v4b & 0x80) && v4b != 0xff)
            break;
        msleep(25);
    }
}

/*
 * vendorgl5: URBs must be pending before all_pids / 06 00.
 * Caller already ran set_interface + lme_ts_start().
 */

/*
 * vendorgl5: enable_ts_output → short settle → all_pids + 06.
 * 602 short path in the Windows trace enters TS prime about 12ms after
 * the 7D=71 / reset tail, while the 626-style full path tolerates a wider
 * settle window. Keep the shorter window only for the short-path trio.
 */
static int lme_ts_prime_after_enable_ts(struct lme_state *st)
{
    int ret, i;
    ktime_t t0;
    unsigned int prime_delay_ms;

    if (st->ts_stream_started) {
        dinfo("TS prime: already started\n");
        return 0;
    }

    t0 = ktime_get();
    lme_ts_bridge_set_alt(st);
    ret = lme_ts_start(st);
    if (ret) {
        derr("TS prime: lme_ts_start %d\n", ret);
        return ret;
    }

    if (lme_ts_should_use_report2(st, "prime_after_enable_ts")) {
        ret = lme_ftm602_report2_handshake(st, "prime_after_enable_ts");
        lme_ts_keepalive_start(st);
        st->ts_stream_started = (ret == 0);
        st->ts_bridge_pending = false;
        dinfo("TS prime: report2 handshake ret=%d within %lld ms of bridge start\n",
              ret, ktime_ms_delta(ktime_get(), t0));
        return ret;
    }

    prime_delay_ms = gl5_use_short_ftm_path(st) ? 12 : 50;
    msleep(prime_delay_ms);
    lme_ts_csv_stream_handshake(st, "prime_after_enable_ts");

    for (i = 0; i < LME_TS_PRIME_ON_CNT; i++) {
        if (i == 0 && st->ftm_c5_valid)
            dinfo("TS prime: csv handshake +%lld us after C5\n",
                  ktime_us_delta(ktime_get(), st->ftm_c5_done));
    }

    lme_ts_keepalive_start(st);
    st->ts_stream_started = true;
    st->ts_bridge_pending = false;

    dinfo("TS prime: delay=%ums, URB+all_pids+06×%d within %lld ms of bridge start\n",
          prime_delay_ms, LME_TS_PRIME_ON_CNT,
          (s64)ktime_to_ms(ktime_sub(ktime_get(), t0)));

    return 0;
}

/* ================================================================== */
/* TS step-probe + 流使能（锁定后）                                    */
/* ================================================================== */

static void lme_ts_urb_complete(struct urb *urb);

/*
 * Count TS bytes during probe: prefer URB rx counter (sync bulk conflicts
 * with active URBs on EP 0x88). Fall back to one sync read if no URBs.
 */
static int ts_probe_bytes(struct lme_state *st)
{
    u8 buf[512];
    int ret, actual = 0, before, after;

    before = atomic_read(&st->ts_probe_rx);
    msleep(150);
    after = atomic_read(&st->ts_probe_rx);
    if (after > before)
        return after - before;

    if (atomic_read(&st->ts_active))
        return 0;

    ret = mutex_lock_interruptible(&st->usb_mutex);
    if (ret)
        return 0;

    ret = usb_bulk_msg(st->udev,
                       usb_rcvbulkpipe(st->udev, LME_EP_TS_IN),
                       buf, sizeof(buf), &actual, 200);
    mutex_unlock(&st->usb_mutex);

    if (ret == 0 && actual > 0)
        return actual;
    return 0;
}

/* Start TS URBs + all_pids without 06 00 (for per-step GL5 probe) */
static int ts_probe_urb_start(struct lme_state *st)
{
    u8 all_pids[] = { 0x03, 0x06, 0x00, 0xFF, 0x01, 0x1F, 0x20, 0x81 };
    u8 rb[2];
    int i, ret;

    if (atomic_read(&st->ts_active))
        return 0;

    ret = usb_set_interface(st->udev, 0, 1);
    if (ret)
        dinfo("TS_probe: set_interface(0,1) ret=%d\n", ret);

    mutex_lock(&st->ts_mutex);
    for (i = 0; i < TS_URB_COUNT; i++) {
        if (!st->ts_urbs[i].urb)
            continue;
        st->ts_urbs[i].buf = usb_alloc_coherent(st->udev, TS_URB_SIZE,
                                                 GFP_KERNEL,
                                                 &st->ts_urbs[i].urb->transfer_dma);
        if (!st->ts_urbs[i].buf) {
            ret = -ENOMEM;
            goto err;
        }
        usb_fill_bulk_urb(st->ts_urbs[i].urb, st->udev,
                          usb_rcvbulkpipe(st->udev, LME_EP_TS_IN),
                          st->ts_urbs[i].buf, TS_URB_SIZE,
                          lme_ts_urb_complete, st);
        st->ts_urbs[i].urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    }

    atomic_set(&st->ts_active, 1);
    for (i = 0; i < TS_URB_COUNT; i++) {
        if (!st->ts_urbs[i].urb)
            continue;
        ret = usb_submit_urb(st->ts_urbs[i].urb, GFP_KERNEL);
        if (ret)
            goto err_submit;
    }
    mutex_unlock(&st->ts_mutex);

    /* URBs must be pending before all_pids / 06 00 (vendorgl5 + 1(1).ftm) */
    lme_usb_talk(st, all_pids, sizeof(all_pids), rb, 1);
    msleep(20);
    return 0;

err_submit:
    atomic_set(&st->ts_active, 0);
err:
    for (i = 0; i < TS_URB_COUNT; i++) {
        if (st->ts_urbs[i].buf) {
            usb_free_coherent(st->udev, TS_URB_SIZE,
                              st->ts_urbs[i].buf,
                              st->ts_urbs[i].urb->transfer_dma);
            st->ts_urbs[i].buf = NULL;
        }
    }
    mutex_unlock(&st->ts_mutex);
    return ret;
}

static void ts_probe_log_step(int step, const char *desc, int bytes)
{
    dinfo("TS_probe step %d %s: TS_bytes=%d\n", step, desc, bytes);
    if (bytes > 0 && ts_probe_first_step < 0)
        ts_probe_first_step = step;
}

/*
 * After (a4 & 0x03)==0x01: run user's 6 GL5/USB steps, probe TS after each.
 * Returns first step index with TS_bytes > 0, or -1 if none.
 */
static int gl5_ts_enable_probe(struct lme_state *st)
{
    u8 a2 = 0, v7e = 0;
    u8 stream_on[] = { 0x06, 0x00 };
    u8 rb[2];
    int bytes;

    ts_probe_first_step = -1;
    atomic_set(&st->ts_probe_rx, 0);
    dinfo("TS_probe: begin (post-lock, before TS seq)\n");

    if (!atomic_read(&st->ts_active))
        ts_probe_urb_start(st);
    else
        dinfo("TS_probe: URBs active (feedcount=%d)\n", st->feedcount);

    bytes = ts_probe_bytes(st);
    ts_probe_log_step(0, "baseline (after lock, before TS seq)", bytes);

    gl5_read_win(st, GL5_R_A2, &a2);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(1, "read a2", bytes);
    dinfo("TS_probe step 1 read a2=0x%02x: TS_bytes=%d\n", a2, bytes);

    gl5_write_win(st, GL5_R_7D, a2);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(2, "write 7d=a2", bytes);
    dinfo("TS_probe step 2 write 7d=0x%02x: TS_bytes=%d\n", a2, bytes);

    gl5_read_win(st, GL5_R_7E, &v7e);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(3, "read 7e", bytes);
    dinfo("TS_probe step 3 read 7e=0x%02x: TS_bytes=%d\n", v7e, bytes);

    gl5_write_win(st, GL5_R_7E, v7e);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(4, "write 7e", bytes);
    dinfo("TS_probe step 4 write 7e=0x%02x: TS_bytes=%d\n", v7e, bytes);

    gl5_soft_reset(st);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(5, "soft_reset", bytes);

    lme_usb_talk(st, stream_on, sizeof(stream_on), rb, 1);
    msleep(50);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(6, "cmd 06 00", bytes);

    /* FTM extras: 7e=00 before 01, 7d=71+01, reset, 06 00 x4 */
    gl5_write_win(st, GL5_R_7E, 0x00);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(7, "FTM write 7e=0x00", bytes);

    gl5_write_win(st, GL5_R_7E, 0x01);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(8, "FTM write 7e=0x01", bytes);

    gl5_ftm_ts_route_c5(st);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(9, "FTM C5=06 (5-byte, no tail)", bytes);

    gl5_soft_reset(st);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(10, "FTM soft_reset #1", bytes);

    gl5_write_win(st, GL5_R_7D, 0x71);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(11, "FTM write 7d=0x71", bytes);

    gl5_soft_reset(st);
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(12, "FTM soft_reset #2", bytes);

    {
        int i;
        for (i = 0; i < 4; i++) {
            lme_usb_talk(st, stream_on, sizeof(stream_on), rb, 1);
            msleep(15);
        }
    }
    bytes = ts_probe_bytes(st);
    ts_probe_log_step(14, "cmd 06 00 x4", bytes);

    atomic_set(&st->ts_probe_rx, 0);

    if (ts_probe_first_step >= 0)
        dinfo("TS_probe: FIRST TS at step %d\n", ts_probe_first_step);
    else
        dinfo("TS_probe: no TS bytes on any step (0-14)\n");

    return ts_probe_first_step;
}

/*
 * vendorgl5 dtmb_win_enable_ts_output (after 04/37 already set):
 * 7e 0→1 edge latches TS path, then 7d + 02 resets; log 4b after each step.
 */
static void gl5_enable_ts_locked(struct lme_state *st)
{
    gl5_ftm_post_lock_route(st);
    gl5_ftm_log_4b_step(st, "enable_ts_locked done");
}

static int gl5_enable_ts(struct lme_state *st)
{
    /*
     * 1(1).ftm post-lock: 7e/7d/02/C5 — reg[07] stays 0x1C (never 0x01 after lock).
     * Soft reset after 7e/7d applies TS route; does not re-search IF.
     */
    if (gl5_hw_locked(st))
        dinfo("GL5 enable_ts: locked — full FTM post-lock route\n");
    else
        dinfo("GL5 enable_ts: not locked — FTM post-lock anyway\n");

    gl5_ftm_post_lock_route(st);
    gl5_ts_hw_regs_log(st, "post-enable_ts");
    return 0;
}

/* ================================================================== */
/* C0 调谐命令                                                         */
/* ================================================================== */

static __maybe_unused void lme_bridge_flush(struct lme_state *st)
{
    int i;

    for (i = 0; i < 4; i++)
        lme_cmd09(st);
    msleep(20);
}

static int max2165_write_reg(struct lme_state *st, u8 reg, u8 val)
{
    u8 ob[5], ib[4];
    /* NO cmd09 before MAX2165 I2C — matches Windows capture */
    ob[0] = 0x04;
    ob[1] = 0x03;
    ob[2] = MAX2165_ADDR << 1;
    ob[3] = reg;
    ob[4] = val;
    return lme_usb_talk(st, ob, 5, ib, 4);
}

static int max2165_read_reg(struct lme_state *st, u8 reg, u8 *val)
{
    u8 ob[5], ib[4];
    int ret;
    ob[0] = 0x84;
    ob[1] = 0x03;
    ob[2] = MAX2165_ADDR << 1;
    ob[3] = reg;
    ob[4] = 0x01;
    ret = lme_usb_talk(st, ob, 5, ib, 4);
    if (ret == 0)
        *val = ib[1];
    dinfo("max2165_r reg=0x%02x rsp=%02x %02x %02x %02x val=0x%02x\n",
          reg, ib[0], ib[1], ib[2], ib[3], *val);
    return ret;
}

/* MAX2165 register map (from kernel max2165_priv.h) */
#define MAX_REG_NDIV_INT        0x00
#define MAX_REG_NDIV_FRAC2      0x01
#define MAX_REG_NDIV_FRAC1      0x02
#define MAX_REG_NDIV_FRAC0      0x03
#define MAX_REG_TRACK_FILTER    0x04
#define MAX_REG_LNA             0x05
#define MAX_REG_PLL_CFG         0x06
#define MAX_REG_TEST            0x07
#define MAX_REG_SHUTDOWN        0x08
#define MAX_REG_VCO_CTRL        0x09
#define MAX_REG_BASEBAND_CTRL   0x0A
#define MAX_REG_DC_OFFSET_CTRL  0x0B
#define MAX_REG_DC_OFFSET_DAC   0x0C
#define MAX_REG_ROM_TABLE_ADDR  0x0D
#define MAX_REG_ROM_TABLE_DATA  0x10
#define MAX_REG_STATUS          0x11
#define MAX_REG_AUTOTUNE        0x12

static int max2165_mask_write(struct lme_state *st, u8 reg, u8 mask, u8 val)
{
    u8 v;
    int ret;
    ret = max2165_read_reg(st, reg, &v);
    if (ret) return ret;
    v &= ~mask;
    v |= (val & mask);
    return max2165_write_reg(st, reg, v);
}

static __maybe_unused void max2165_set_osc(struct lme_state *st, u8 osc_mhz)
{
    u8 v;
    v = osc_mhz / 2;
    if (v == 2) v = 0x7; else v -= 8;
    max2165_mask_write(st, MAX_REG_PLL_CFG, 0x07, v);
}

static void max2165_read_rom_table(struct lme_state *st)
{
    u8 dat[3]; int i;
    for (i = 0; i < 3; i++) {
        max2165_write_reg(st, MAX_REG_ROM_TABLE_ADDR, i + 1);
        max2165_read_reg(st, MAX_REG_ROM_TABLE_DATA, &dat[i]);
    }
    st->max_tf_ntch_low_cfg  = dat[0] >> 4;
    st->max_tf_ntch_hi_cfg   = dat[0] & 0x0F;
    st->max_tf_balun_low_ref = dat[1] & 0x0F;
    st->max_tf_balun_hi_ref  = dat[1] >> 4;
    st->max_bb_filter_7mhz_cfg = dat[2] & 0x0F;
    st->max_bb_filter_8mhz_cfg = dat[2] >> 4;
    dinfo("MAX2165 ROM: ntch_l=%x ntch_h=%x balun_l=%x balun_h=%x bb7=%x bb8=%x\n",
          st->max_tf_ntch_low_cfg, st->max_tf_ntch_hi_cfg,
          st->max_tf_balun_low_ref, st->max_tf_balun_hi_ref,
          st->max_bb_filter_7mhz_cfg, st->max_bb_filter_8mhz_cfg);
}

static void __maybe_unused max2165_set_bandwidth(struct lme_state *st, u32 bw_hz)
{
    u8 val = (bw_hz == 8000000) ? st->max_bb_filter_8mhz_cfg
                                : st->max_bb_filter_7mhz_cfg;
    max2165_mask_write(st, MAX_REG_BASEBAND_CTRL, 0xF0, val << 4);
}

static int max2165_init(struct lme_state *st)
{
    st->max_osc_clk = 12;  /* 12 MHz per vendorgl5 */

    /*
     * Blob write in win3_probe_init already sets all MAX2165 registers
     * with Windows-matching values: PLL_CFG=0x0A, SHDN=0x02, VCO=0x54,
     * BB=0x73, DCO=0x75, DAC=0x00.  Do NOT overwrite any of them.
     *
     * set_osc / set_bandwidth use mask_write which reads back 0x00
     * (MAX2165 control regs are write-only via gate-4), so RMW corrupts.
     * Skip both — the blob values are already correct.
     */

    /* ROM table read (needed to populate filter/balun fields for later) */
    max2165_read_rom_table(st);

    dinfo("MAX2165 init done (blob-only, osc=%uMHz)\n", st->max_osc_clk);
    return 0;
}
static void __maybe_unused lme_pre_c0_gl5(struct lme_state *st)
{
    /* vendorgl5 cmdc0 + FTM: 02 reset → 01=e0 → 01=0x01 (C0 前释放 Windows 模式) */
    gl5_write_win3(st, GL5_R_RESET, 0x00);
    msleep(5);
    gl5_write_win3(st, GL5_R_RESET, 0x01);
    msleep(5);
    gl5_write_win3(st, 0x01, 0xe0);
    msleep(10);
    gl5_write_win3(st, 0x01, 0x01);
    msleep(5);
    gl5_log_iface_regs(st, "pre-C0 done");
}

/* vendorgl5: C0 只写 NDIV_INT，需补 FRAC/TF 并读 PLL */
static void __maybe_unused max2165_set_rf(struct lme_state *st, u32 freq_hz)
{
    u32 quotient, fraction, remainder;
    u32 freq_khz = freq_hz / 1000;
    u32 osc_khz = st->max_osc_clk * 1000;
    u8 tf, tf_ntch;
    u32 t;
    int i;

    /* Compute NDIV and 20-bit fraction */
    quotient = freq_khz / osc_khz;
    remainder = freq_khz - quotient * osc_khz;
    fraction = 0;
    for (i = 0; i < 20; i++) {
        remainder <<= 1;
        fraction <<= 1;
        if (remainder >= osc_khz) {
            fraction |= 1;
            remainder -= osc_khz;
        }
    }

    max2165_write_reg(st, MAX_REG_NDIV_INT, (u8)quotient);
    max2165_mask_write(st, MAX_REG_NDIV_FRAC2, 0x0F, (u8)(fraction >> 16));
    max2165_write_reg(st, MAX_REG_NDIV_FRAC1, (u8)(fraction >> 8));
    max2165_write_reg(st, MAX_REG_NDIV_FRAC0, (u8)fraction);

    /* Track filter */
    tf_ntch = (freq_hz < 725000000U) ? st->max_tf_ntch_low_cfg
                                     : st->max_tf_ntch_hi_cfg;
    t = st->max_tf_balun_low_ref;
    t += (st->max_tf_balun_hi_ref - st->max_tf_balun_low_ref)
         * (freq_khz - 470000) / (780000 - 470000);
    tf = (u8)t | (tf_ntch << 4);
    max2165_write_reg(st, MAX_REG_TRACK_FILTER, tf);

    dinfo("MAX2165 set_rf %uHz: ndiv=%u frac=0x%05x tf=0x%02x\n",
          freq_hz, quotient, fraction, tf);
}

static void __maybe_unused max2165_c0_fixup(struct lme_state *st, const u8 cmd[17])
{
    u8 stat, autotune, vrf;

    dinfo("MAX2165 fixup: NDIV=0x%02x FRAC2=0x%02x FRAC1=0x%02x FRAC0=0x%02x TF=0x%02x\n",
          cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);

    /* Windows uses gate-4 I2C (no cmd09) + C0 for MAX2165.
     * Write NDIV/FRAC/TF via gate-4, C0 handles the rest. */
    max2165_write_reg(st, MAX_REG_NDIV_INT, cmd[2]);
    max2165_write_reg(st, MAX_REG_NDIV_FRAC2, cmd[3]);
    max2165_write_reg(st, MAX_REG_NDIV_FRAC1, cmd[4]);
    max2165_write_reg(st, MAX_REG_NDIV_FRAC0, cmd[5]);
    max2165_write_reg(st, MAX_REG_TRACK_FILTER, cmd[6]);

    /* Verify writes took effect */
    max2165_read_reg(st, MAX_REG_NDIV_INT, &vrf);
    dinfo("  verify NDIV readback=0x%02x (expect 0x%02x)\n", vrf, cmd[2]);

    msleep(200);
    max2165_read_reg(st, MAX_REG_STATUS, &stat);
    max2165_read_reg(st, MAX_REG_AUTOTUNE, &autotune);
    dinfo("MAX2165 stat=0x%02x auto=0x%02x pll=%d vco_ok=%d vco_act=%d vco=%d sub=%d\n",
          stat, autotune,
          (stat >> 4) & 1, (stat >> 6) & 1, (stat >> 5) & 1,
          autotune >> 6, (autotune >> 3) & 7);
}

static void gl5_write_win3(struct lme_state *st, u8 reg, u8 val)
{
    gl5_write_w32_triple(st, reg, val);
}

/*
 * vendorgl5 dtmb_win_post_c0_sequence + start_demod
 * new.ftm: 60→01→03→cmd16→07=1c→7e→cmd09×4→re-07→alt C5→lock poll
 */
static int __maybe_unused lme_post_c0_sequence(struct lme_state *st)
{
    u8 v07, v4b, va4, v7d;
    int step = 0;
    bool skip_zif = gl5_hw_locked(st);

    gl5_write_win3(st, 0x01, 0x60);
    post_c0_read_4b(st, ++step, "write 01=0x60 (GL5 reg 0x01 via 05 04 32)");
    gl5_log_iface_regs(st, "after 01=0x60 write");
    msleep(5);
    gl5_write_win3(st, 0x01, 0x01);
    post_c0_read_4b(st, ++step, "write 01=0x01");
    gl5_log_iface_regs(st, "after 01=0x01 write");
    msleep(20);
    gl5_write_win3(st, GL5_R_03, 0x00);
    post_c0_read_4b(st, ++step, "write 03=0x00");
    msleep(10);
    gl5_write_win3(st, GL5_R_03, 0x01);
    post_c0_read_4b(st, ++step, "write 03=0x01");
    gl5_log_iface_regs(st, "after 03=0x01 write");
    msleep(10);
    lme_cmd16(st, 0);
    post_c0_read_4b(st, ++step, "cmd16 mode=0");
    msleep(10);

    if (!skip_zif) {
        gl5_write_win3(st, GL5_R_07, 0x1c);
        gl5_write_win3(st, GL5_R_09, 0x00);
        gl5_write_win3(st, GL5_R_0A, 0x00);
        gl5_write_win3(st, GL5_R_0B, 0x00);
        gl5_write_win3(st, GL5_R_0C, 0x00);
        gl5_read_win(st, GL5_R_07, &v07);
        post_c0_read_4b(st, ++step, "write 07=0x1c (+09-0c=0)");
        dinfo("post_c0: wrote 07=1c read=0x%02x\n", v07);
    } else {
        dinfo("post_c0: skip 07=1c/09-0c (already locked)\n");
    }
    msleep(10);

    gl5_write_win3(st, GL5_R_7E, 0x01);
    post_c0_read_4b(st, ++step, "write 7e=0x01");
    msleep(10);

    if (!skip_zif && !gl5_hw_locked(st)) {
        gl5_write_win3(st, GL5_R_07, 0x1c);
        gl5_write_win3(st, GL5_R_09, 0x00);
        gl5_write_win3(st, GL5_R_0A, 0x00);
        gl5_write_win3(st, GL5_R_0B, 0x00);
        gl5_write_win3(st, GL5_R_0C, 0x00);
        gl5_read_win(st, GL5_R_07, &v07);
        post_c0_read_4b(st, ++step, "re-write 07=0x1c (+09-0c=0)");
        dinfo("post_c0: after cmd09 re-07=1c read=0x%02x\n", v07);
    } else if (gl5_hw_locked(st)) {
        gl5_read_win(st, GL5_R_STRENGTH, &v4b);
        dinfo("post_c0: skip re-07=1c (locked 4b=0x%02x)\n", v4b);
    }

    gl5_write_alt_05(st, 0xC5, 0x00);
    post_c0_read_4b(st, ++step, "alt C5=0x00 (05 04 36 C5 00 01)");
    msleep(500);  /* Windows waits longer after C0 before start_demod */
    post_c0_read_4b(st, ++step, "before start_demod");
    gl5_win_start_demod(st);

    gl5_read_win(st, GL5_R_STRENGTH, &v4b);
    gl5_read_win(st, GL5_R_STATUS, &va4);
    gl5_read_win(st, GL5_R_7D, &v7d);
    dinfo("post_c0 done: 4b=0x%02x a4=0x%02x 7d=0x%02x\n", v4b, va4, v7d);
    gl5_log_iface_regs(st, "post_c0 done (compare FTM)");

    gl5_gate_probe_and_select(st, "post-tune");
    return 0;
}

static int lme_c0_tune(struct lme_state *st, u32 freq_hz)
{
    u8 cmd[17];
    int i;
    u32 best_freq = 0;
    int best_idx  = -1;
    u32 best_diff = U32_MAX;
    static const u8 w0200[] = { 0x05, 0x04, 0x32, 0x02, 0x00 };
    static const u8 w0201[] = { 0x05, 0x04, 0x32, 0x02, 0x01 };
    static const u8 w01e0[] = { 0x05, 0x04, 0x32, 0x01, 0xe0 };

    for (i = 0; i < ARRAY_SIZE(c0_table); i++) {
        u32 diff = (c0_table[i].freq_hz > freq_hz) ?
                   (c0_table[i].freq_hz - freq_hz) :
                   (freq_hz - c0_table[i].freq_hz);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx  = i;
            best_freq = c0_table[i].freq_hz;
        }
    }

    if (best_idx < 0) {
        derr("C0查表失败 freq=%u\n", freq_hz);
        return -EINVAL;
    }

    memcpy(cmd, c0_table[best_idx].cmd, 17);

    if (best_diff == 0) {
        dinfo("C0调谐 %uHz: 精确命中表项\n", freq_hz);
    } else {
        /* 未精确命中表项时，计算 NDIV 与 FRAC */
        u32 freq_khz = freq_hz / 1000;
        u32 osc_khz = 12000; /* 晶振 12 MHz */
        u32 quotient = freq_khz / osc_khz;
        u32 remainder = freq_khz - quotient * osc_khz;
        u32 fraction = 0;
        int j;

        for (j = 0; j < 20; j++) {
            remainder <<= 1;
            fraction <<= 1;
            if (remainder >= osc_khz) {
                fraction |= 1;
                remainder -= osc_khz;
            }
        }

        /* 覆盖 NDIV 与 FRAC 寄存器 */
        cmd[2] = (u8)quotient;
        cmd[3] = (u8)(fraction >> 16) & 0x0F;
        cmd[4] = (u8)(fraction >> 8);
        cmd[5] = (u8)fraction;
    }

    /* 【关键修复】强制纠正 UHF 频段（>=470MHz）的射频开关与跟踪滤波器参数 */
    if (freq_hz >= 470000000U && cmd[6] != 0xB6) {
        dinfo("C0调谐 %uHz: 修正 Track Filter 参数 (0x%02x -> 0xB6)\n", freq_hz, cmd[6]);
        cmd[6] = 0xB6;
    }

    /* Save for gl5_ftm_post_c0 to build frequency-specific C0 command */
    memcpy(st->c0_cmd, cmd, 17);
    st->tuned_freq_hz = freq_hz;

    /*
     * 602new.csv confirms Windows replays a pre-C0 settle block immediately
     * before the frequency write, not just once during earlier init:
     *   85 02 32 00 1C
     *   85 02 32 02 1C
     *   05 04 32 02 00
     *   05 04 32 02 01
     *   05 04 32 01 E0
     */
    win3_read85(st, 0x32, 0x00, 0x1c, "pre-c0 chip", NULL);
    win3_read85(st, 0x32, 0x02, 0x1c, "pre-c0 reset", NULL);
    win3_cmd(st, w0200, sizeof(w0200), 1, "pre-c0 02=00");
    win3_cmd(st, w0201, sizeof(w0201), 1, "pre-c0 02=01");
    win3_cmd(st, w01e0, sizeof(w01e0), 1, "32.01=e0 pre-c0");

    /* C0 tune: Windows 6262.csv Seq 0821: 04 07 C0 00 <NDIV> <FRAC2> <FRAC1> <FRAC0> <TF> */
    {
        u8 c0[9];
        c0[0] = 0x04; c0[1] = 0x07;
        memcpy(&c0[2], cmd, 7);
        win3_cmd(st, c0, sizeof(c0), 1, "c0 tune");
    }
    /* BASEBAND: Windows Seq 0825: 04 03 C0 0A 73 */
    {
        u8 wc00a[] = { 0x04, 0x03, 0xc0, 0x0a, 0x73 };
        win3_cmd(st, wc00a, sizeof(wc00a), 1, "c0.0a=73");
    }
    /* Windows Seq 0829-0833 reads TF first, then writes TF|0x40.
     * Keep the read for bridge timing/debug, but use c0_table because
     * gate-4 readback is unreliable on the Linux firmware path. */
    {
        u8 wc004[] = { 0x04, 0x03, 0xc0, 0x04, cmd[6] | 0x40 };
        u8 tf_rd = 0;
        int ret;

        ret = win3_read84(st, 0xc0, 0x04, 0x01, "c0.04 rd", &tf_rd);
        dinfo("c0 TF pre-read raw=55 %02x ret=%d (c0_table=0x%02x)\n",
              tf_rd, ret, cmd[6]);
        win3_cmd(st, wc004, sizeof(wc004), 1, "c0.04=tf|40");
        dinfo("c0 TF write=0x%02x (from c0_table[6]=0x%02x)\n",
              cmd[6] | 0x40, cmd[6]);
    }

    /* Windows FTM post-C0 sequence: 01=60 → ... → lock poll → TS route */
    gl5_ftm_post_c0(st);

    /*
     * mac path leaves an additional settle window after ftm_post_c0() before
     * frontend status sampling. Keep that quiet period so short-path lock can
     * stabilize before userspace starts polling immediately.
     */
    msleep(80);

    /* gl5_ftm_post_c0 now completes the 7D=71 -> reset -> TS prime window. */

    return 0;
}

/* ================================================================== */
/* TS URB                                                              */
/* ================================================================== */

static void lme_ts_urb_complete(struct urb *urb);

static void lme_ts_urb_complete(struct urb *urb)
{
    struct lme_state *st = urb->context;
    int ret;

    if (!atomic_read(&st->ts_active))
        return;

    if (urb->status == 0 && urb->actual_length > 0) {
        if (!st->ts_first_in_logged) {
            st->ts_first_in_logged = true;
            if (st->ftm_c5_valid)
                dinfo("TS first EP88 IN: %d bytes, +%lld us after C5\n",
                      urb->actual_length,
                      ktime_us_delta(ktime_get(), st->ftm_c5_done));
            else
                dinfo("TS first EP88 IN: %d bytes\n", urb->actual_length);
            
            /* Stop periodic 06 00 keepalive once stream starts to avoid starving EP88 */
            cancel_delayed_work(&st->ts_keepalive_work);
        }
        atomic_add(urb->actual_length, &st->ts_probe_rx);
        {
            u8 *b = urb->transfer_buffer;
            int aligned;
            int syncs = 0;
            for (int j = 0; j < urb->actual_length; j++)
                if (b[j] == 0x47) syncs++;
            /* 
             * u16 pid = (b[0] == 0x47) ? ((b[1] & 0x1F) << 8) | b[2] : 0xFFFF;
             * dinfo("TS: %dB first=0x%02x pid=0x%04x syncs=%d feeds=%d\n",
             *       urb->actual_length, b[0], pid, syncs,
             *       list_empty(&st->demux.feed_list) ? 0 : 1);
             */

            /* hexdump first 64 bytes of first 3 URBs for format analysis */
            {
                static atomic_t hexdump_cnt;
                int n = atomic_inc_return(&hexdump_cnt);
                if (n <= 3) {
                    char hex[200];
                    int pos = 0;
                    for (int j = 0; j < min(urb->actual_length, 64); j++) {
                        pos += snprintf(hex + pos, sizeof(hex) - pos,
                                        "%02x ", b[j]);
                        if ((j & 15) == 15) pos += snprintf(hex + pos, sizeof(hex) - pos, "\n");
                    }
                    if ((urb->actual_length & 15) || urb->actual_length > 16)
                        pos += snprintf(hex + pos, sizeof(hex) - pos, "\n");
                    dinfo("TS hexdump #%d (%dB):\n%s\n", n, urb->actual_length, hex);
                }
            }

            aligned = lme_ts_aligner_push(st, urb->transfer_buffer,
                                          urb->actual_length);
            if (aligned > 0) {
                static atomic_t ts_aligned_log_cnt;

                if (atomic_inc_return(&ts_aligned_log_cnt) <= 12) {
                    dinfo("TS aligned: %dB (%d packets)\n",
                          aligned, aligned / TS_PACKET_SIZE);
                }
            }
        }
    } else if (urb->status == 0 && urb->actual_length == 0) {
        static atomic_t ts_urb_zlen;
        if (atomic_inc_return(&ts_urb_zlen) <= 6)
            dinfo("TS URB: 0-byte completion\n");
    } else if (urb->status != -ENOENT &&
               urb->status != -ECONNRESET &&
               urb->status != -ESHUTDOWN) {
        dinfo("TS URB: status=%d len=%d\n", urb->status, urb->actual_length);
    }

    /* 重提交 */
    if (atomic_read(&st->ts_active)) {
        ret = usb_submit_urb(urb, GFP_ATOMIC);
        if (ret)
            ddbg("TS URB 重提交失败: %d\n", ret);
    }
}

static int lme_ts_start(struct lme_state *st)
{
    int i, ret;

    mutex_lock(&st->ts_mutex);

    if (atomic_read(&st->ts_active)) {
        mutex_unlock(&st->ts_mutex);
        return 0;
    }

    lme_ts_align_reset(st);
    st->ts_first_in_logged = false;

    /* set_interface is done in lme_ts_bridge_set_alt() before this call */

    /* 分配URB和缓冲 */
    for (i = 0; i < TS_URB_COUNT; i++) {
        st->ts_urbs[i].buf = usb_alloc_coherent(st->udev, TS_URB_SIZE,
                                                 GFP_KERNEL,
                                                 &st->ts_urbs[i].urb->transfer_dma);
        if (!st->ts_urbs[i].buf) {
            ret = -ENOMEM;
            goto err_alloc;
        }
        usb_fill_bulk_urb(st->ts_urbs[i].urb, st->udev,
                          usb_rcvbulkpipe(st->udev, LME_EP_TS_IN),
                          st->ts_urbs[i].buf, TS_URB_SIZE,
                          lme_ts_urb_complete, st);
        st->ts_urbs[i].urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    }

    atomic_set(&st->ts_active, 1);

    for (i = 0; i < TS_URB_COUNT; i++) {
        ret = usb_submit_urb(st->ts_urbs[i].urb, GFP_KERNEL);
        if (ret) {
            derr("TS URB[%d] 提交失败: %d\n", i, ret);
            atomic_set(&st->ts_active, 0);
            goto err_submit;
        }
    }

    dinfo("TS URBs pending (EP=0x%02x, URB×%d)\n",
          LME_EP_TS_IN, TS_URB_COUNT);

    mutex_unlock(&st->ts_mutex);
    return 0;

err_submit:
    for (i = 0; i < TS_URB_COUNT; i++)
        usb_kill_urb(st->ts_urbs[i].urb);
err_alloc:
    for (i = 0; i < TS_URB_COUNT; i++) {
        if (st->ts_urbs[i].buf)
            usb_free_coherent(st->udev, TS_URB_SIZE,
                              st->ts_urbs[i].buf,
                              st->ts_urbs[i].urb->transfer_dma);
    }
    mutex_unlock(&st->ts_mutex);
    return ret;
}

/* usb_set_interface() unlinks in-flight TS URBs; resubmit while buffers stay */
static void __maybe_unused lme_ts_send_bridge_byte(struct lme_state *st, u8 v)
{
    u8 ob[1] = { v };
    lme_usb_talk(st, ob, 1, NULL, 0);
}

static __maybe_unused void lme_ts_resubmit_pending(struct lme_state *st)
{
    int i, ret, ok = 0;

    mutex_lock(&st->ts_mutex);
    if (!atomic_read(&st->ts_active))
        goto out;

    for (i = 0; i < TS_URB_COUNT; i++) {
        if (!st->ts_urbs[i].buf || !st->ts_urbs[i].urb)
            continue;
        usb_kill_urb(st->ts_urbs[i].urb);
    }

    usb_clear_halt(st->udev, usb_rcvbulkpipe(st->udev, LME_EP_TS_IN));

    for (i = 0; i < TS_URB_COUNT; i++) {
        if (!st->ts_urbs[i].buf)
            continue;
        ret = usb_submit_urb(st->ts_urbs[i].urb, GFP_KERNEL);
        if (ret)
            dinfo("TS URB[%d] 重挂失败: %d\n", i, ret);
        else
            ok++;
    }
    dinfo("TS URBs re-pending after set_interface (%d/%d)\n", ok, TS_URB_COUNT);
out:
    mutex_unlock(&st->ts_mutex);
}

static void lme_ts_send_stream_on(struct lme_state *st)
{
    u8 stream_on[] = { 0x06, 0x00 };
    u8 drain[64];

    /* Must read response to prevent EP 0x81 IN overflow.
     * Use 64-byte drain buffer — firmware may return large payloads
     * during TS streaming, and unread responses accumulate. */
    lme_usb_talk(st, stream_on, sizeof(stream_on), drain, sizeof(drain));
}

static void lme_ts_keepalive_fn(struct work_struct *work)
{
    struct lme_state *st = container_of(work, struct lme_state,
                                        ts_keepalive_work.work);

    if (!atomic_read(&st->ts_active) || st->feedcount <= 0)
        return;

    lme_ts_send_stream_on(st);
    schedule_delayed_work(&st->ts_keepalive_work,
                          msecs_to_jiffies(LME_TS_KEEPALIVE_MS));
}

static void lme_ts_keepalive_stop(struct lme_state *st)
{
    cancel_delayed_work_sync(&st->ts_keepalive_work);
}

static void lme_ts_keepalive_start(struct lme_state *st)
{
    cancel_delayed_work_sync(&st->ts_keepalive_work);
    schedule_delayed_work(&st->ts_keepalive_work, 0);
}

/*
 * Retune with URBs up: same arm as prepare except NO set_interface.
 * Order: FTM post-lock (if not yet) → clear_halt → all_pids → clear_halt → 06 00×N.
 */
static void lme_ts_bridge_restart(struct lme_state *st)
{
    if (st->feedcount <= 0 || !atomic_read(&st->ts_active)) {
        dinfo("TS bridge_restart: skip (feed=%d URBs=%d)\n",
              st->feedcount, atomic_read(&st->ts_active));
        return;
    }

    lme_ts_bridge_log_lock(st, "bridge_restart", "entry");

    /* post-C0 already did lock route. Don't re-route — resets kill FEC lock.
     * Just send bridge commands directly. */
    lme_ts_bridge_stream_cmds(st, "bridge_restart");
    lme_ts_bridge_log_lock(st, "bridge_restart", "after 06 burst");
}

/* 1(1).ftm: initial 06 00 burst then same keepalive timer */
static __maybe_unused void lme_ts_stream_on(struct lme_state *st)
{
    int i;

    for (i = 0; i < LME_TS_BRIDGE_ON_CNT; i++) {
        lme_ts_send_stream_on(st);
        msleep(LME_TS_KEEPALIVE_MS);
    }
    lme_ts_keepalive_start(st);
    dinfo("TS stream ON (06 00 x%d + keepalive %dms)\n",
          LME_TS_BRIDGE_ON_CNT, LME_TS_KEEPALIVE_MS);
}

static void lme_ts_stop(struct lme_state *st)
{
    u8 clear[] = { 0x03, 0x02, 0x20, 0xA0 };
    u8 rb[2];
    int i;

    lme_ts_keepalive_stop(st);

    mutex_lock(&st->ts_mutex);

    if (!atomic_read(&st->ts_active)) {
        mutex_unlock(&st->ts_mutex);
        return;
    }

    atomic_set(&st->ts_active, 0);

    for (i = 0; i < TS_URB_COUNT; i++)
        usb_kill_urb(st->ts_urbs[i].urb);

    for (i = 0; i < TS_URB_COUNT; i++) {
        if (st->ts_urbs[i].buf)
            usb_free_coherent(st->udev, TS_URB_SIZE,
                              st->ts_urbs[i].buf,
                              st->ts_urbs[i].urb->transfer_dma);
        st->ts_urbs[i].buf = NULL;
    }

    lme_usb_talk(st, clear, sizeof(clear), rb, 1);
    st->ts_stream_started = false; /* next start_feed re-primes bridge */
    lme_ts_align_reset(st);
    dinfo("TS stream stopped\n");

    mutex_unlock(&st->ts_mutex);
}

/* ================================================================== */
/* DVB demux feed 回调                                                 */
/* ================================================================== */

static int lme_start_feed(struct dvb_demux_feed *feed)
{
    struct lme_state *st = feed->demux->priv;
    int ret = 0, wait_ms = 0;

    /* dvbv5-zap -r opens dvr before set_frontend; wait before claiming feed */
    while (atomic_read(&st->demod_busy) && wait_ms < 35000) {
        msleep(10);
        wait_ms += 10;
    }
    if (wait_ms)
        dinfo("demux start_feed: waited %d ms for tune\n", wait_ms);

    if (st->feedcount++ == 0) {
        /* Windows 3.csv: set alt, bridge cmds with ACK, then submit TS IN URBs. */
        lme_ts_bridge_set_alt(st);

        if (!st->ts_stream_started) {
            lme_ts_bridge_stream_cmds(st, "start_feed");
        } else {
            dinfo("demux start_feed: bridge already primed in win_start_demod\n");
            if (!atomic_read(&st->ts_active)) {
                lme_ts_bridge_set_alt(st);
                ret = lme_ts_start(st);
                if (ret)
                    st->feedcount = 0;
            }
        }

        if (!atomic_read(&st->ts_active)) {
            dinfo("demux start_feed: lme_ts_start (URBs after bridge cmds)\n");
            ret = lme_ts_start(st);
            if (ret) {
                st->feedcount = 0;
                return ret;
            }
        }

        /* keepalive keeps firmware streaming; without it TS stops after ~16KB */
        lme_ts_keepalive_start(st);
        dinfo("demux start_feed: URBs active, keepalive running\n");
    }
    return ret;
}

static int lme_stop_feed(struct dvb_demux_feed *feed)
{
    struct lme_state *st = feed->demux->priv;

    if (st->feedcount > 0 && --st->feedcount == 0)
        lme_ts_stop(st);
    return 0;
}

/* ================================================================== */
/* DVB frontend ops                                                    */
/* ================================================================== */

static int dtmb_fe_init(struct dvb_frontend *fe)
{
    struct lme_state *st = fe->demodulator_priv;
    /*
     * dvbv5-zap calls init before every tune; full gl5_init resets the
     * demod and breaks an already-running TS feed.
     */
    atomic_set(&st->demod_busy, 0);
    if (atomic_read(&st->probe_ok))
        return 0;
    return gl5_init(st);
}

static int dtmb_set_frontend(struct dvb_frontend *fe)
{
    struct lme_state *st = fe->demodulator_priv;
    struct dtv_frontend_properties *p = &fe->dtv_property_cache;
    int ret;

    /* 防重入：避免dvbv5-zap多次调用打断lock流程 */
    if (atomic_cmpxchg(&st->demod_busy, 0, 1) != 0) {
        dinfo("set_fe: 解调器忙，跳过 (freq=%u)\n", p->frequency);
        return 0;
    }

    dinfo("set_frontend: %u Hz\n", p->frequency);

    st->ts_bridge_pending = false;
    st->ts_stream_started = false;
    st->gl5_ts_enabled = false;
    st->ftm_c5_valid = false;
    st->ts_first_in_logged = false;
    st->cached_fe_status = 0;

    /*
     * Do NOT lme_ts_stop() here: dvbv5-zap -r opens dvr before set_fe, so stop
     * would usb_kill_urb() all EP 0x88 IN. Retune with URBs up; stream cmds
     * in start_feed after lme_ts_start().
     */

    ret = lme_c0_tune(st, p->frequency);
    if (ret)
        dinfo("C0/post_c0 序列失败: %d (继续尝试TS)\n", ret);

    /* TS mode and TS enable are handled by start_feed → lme_ts_start.
     * FTM post-C0 + lock poll already done in lme_c0_tune → gl5_ftm_post_c0.
     * Do NOT enable TS here — it would conflict with the bridge stream cmds. */

    /* URB + bridge only from start_feed (after demod_busy clears) */
    ret = 0;

    atomic_set(&st->demod_busy, 0);
    return ret;
}

static int dtmb_read_status(struct dvb_frontend *fe, enum fe_status *s)
{
    struct lme_state *st = fe->demodulator_priv;
    u8 strength = 0, status = 0;

    *s = 0;

    /* During set_frontend (FTM sequence with resets), 4b/a4 values
     * flicker.  Return cached status so the DVB core doesn't see a
     * transient lock, start the demux, then lose lock on the next
     * reset. */
    if (atomic_read(&st->demod_busy) || atomic_read(&st->ts_active)) {
        *s = st->cached_fe_status;
        /* If streaming, ensure lock bits remain set so dvb core doesn't stop */
        if (atomic_read(&st->ts_active))
            *s |= FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK | FE_HAS_CARRIER | FE_HAS_SIGNAL;
        return 0;
    }

    u8 v07 = 0;

    gl5_read(st, GL5_ADDR, GL5_R_STRENGTH, &strength);
    gl5_read(st, GL5_ADDR, GL5_R_STATUS,   &status);
    gl5_read_win(st, GL5_R_07, &v07);

    if (strength & 0x7F)           *s |= FE_HAS_SIGNAL;
    if (strength & 0x80)           *s |= FE_HAS_CARRIER;
    if ((status  & 0x03) == 0x01)  *s |= FE_HAS_VITERBI |
                                         FE_HAS_SYNC    |
                                         FE_HAS_LOCK;
    /* TS pin: a4 may be 0x00 while MPEG is routed.
     * reg[07] stays 0x1C (GL5_R_07_ZIF) after lock, never 0x01. */
    else if ((v07 == GL5_R_07_ZIF || v07 == GL5_R_07_TS_OUT)
             && (strength & 0x80) && strength != 0xff)
        *s |= FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;

    st->cached_fe_status = *s;
    ddbg("read_status: 4b=0x%02x a4=0x%02x → 0x%02x\n",
         strength, status, *s);
    return 0;
}

static int dtmb_read_signal_strength(struct dvb_frontend *fe, u16 *v)
{
    struct lme_state *st = fe->demodulator_priv;
    u8 strength = 0;
    
    if (atomic_read(&st->ts_active)) {
        *v = 0xFFFF; /* Fake max strength while streaming to avoid I2C stalls */
        return 0;
    }
    
    gl5_read(st, GL5_ADDR, GL5_R_STRENGTH, &strength);
    *v = (strength & 0x7F) << 9;
    return 0;
}

static int dtmb_read_snr(struct dvb_frontend *fe, u16 *v)
{
    return dtmb_read_signal_strength(fe, v);
}

static int dtmb_read_ber(struct dvb_frontend *fe, u32 *v) { *v=0; return 0; }
static int dtmb_read_ucblocks(struct dvb_frontend *fe, u32 *v) { *v=0; return 0; }
static int dtmb_get_frontend(struct dvb_frontend *fe,
                             struct dtv_frontend_properties *p)
{
    memcpy(p, &fe->dtv_property_cache, sizeof(*p));
    return 0;
}

static void dtmb_fe_release(struct dvb_frontend *fe) {}

/*
 * TVHeadend on the current FlyDVR stack only stays stable if this frontend
 * advertises SYS_DVBT. The protocol and RF path are DTMB, but the exported
 * delivery-system shell must remain DVB-T compatible for now.
 */
static const struct dvb_frontend_ops dtmb_fe_ops = {
    .delsys = { SYS_DVBT },
    .info = {
        .name               = "LME2510C+LGS8GL5 DTMB",
        .frequency_min_hz   = 474000000,
        .frequency_max_hz   = 858000000,
        .frequency_stepsize_hz = 10000,
        .caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
                FE_CAN_TRANSMISSION_MODE_AUTO |
                FE_CAN_BANDWIDTH_AUTO |
                FE_CAN_GUARD_INTERVAL_AUTO,
    },
    .release              = dtmb_fe_release,
    .get_frontend         = dtmb_get_frontend,
    .init                 = dtmb_fe_init,
    .set_frontend         = dtmb_set_frontend,
    .read_status          = dtmb_read_status,
    .read_signal_strength = dtmb_read_signal_strength,
    .read_snr             = dtmb_read_snr,
    .read_ber             = dtmb_read_ber,
    .read_ucblocks        = dtmb_read_ucblocks,
};

/* ================================================================== */
/* 固件下载                                                            */
/* ================================================================== */

static u8 __maybe_unused lme_cksum(const u8 *d, int len)
{
    u8 s = 0; int i;
    for (i = 0; i < len; i++) s += d[i];
    return s;
}

static int lme_upload_fw_segment(struct lme_state *st, const u8 *data,
                                 u16 start, u16 end, u8 seg, bool pace_seg1)
{
    u8 *buf;
    u8 rb[1];
    int ret = 0, i;
    bool soft_ok;
    const u8 PKT = 0x31;

    dinfo("fw upload segment: %02x 31 offset=%u..%u (%u bytes)%s\n",
          seg, start, end ? (u16)(end - 1) : end, end - start,
          pace_seg1 ? " paced" : "");

    buf = kzalloc(128, GFP_KERNEL);
    if (!buf) return -ENOMEM;

    for (i = start; i < end; ) {
        u8 dlen = ((end - i) > PKT) ? PKT : (u8)(end - i - 1);
        const u8 *src = data + i;

        buf[0] = seg;
        buf[1] = dlen;
        memcpy(&buf[2], src, dlen + 1);
        buf[dlen + 3] = lme_cksum(src, dlen + 1);

        soft_ok = false;
        ret = lme_fw_usb_talk(st, buf, dlen + 4, rb, 1);
        /* ROM bootloader may not ACK every packet; timeout is OK if OUT succeeded.
         * Only fail on hard USB errors (not timeout -110 or bad ack from bridge). */
        if (ret == -110 || ret == -ETIMEDOUT || ret == -EOVERFLOW) {
            /* OUT was accepted, continue. Assume segment processed OK. */
            ret = 0;
            soft_ok = true;
        } else if (ret || (!soft_ok && rb[0] != 0x88 && rb[0] != 0x77 &&
                           rb[0] != 0x44 && rb[0] != 0x00)) {
            derr("固件上传失败 seg=%u @%d ret=%d ack=0x%02x\n",
                 seg, i, ret, rb[0]);
            kfree(buf);
            return -EIO;
        }
        i += dlen + 1;
        if (pace_seg1 && i < end)
            udelay(500);
    }

    kfree(buf);
    return 0;
}

static int lme_upload_fw_data_legacy(struct lme_state *st, const u8 *data, u16 size)
{
    u8 rb[1];
    u8 cmd81[] = { 0x81, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int ret;

    ret = lme_upload_fw_segment(st, data, 0, 500, 0x01, true);
    if (ret)
        return ret;

    /* Legacy mac/Linux route: segment1 first, then BRIDGE_81, then segment2. */
    dinfo("fw upload control: 81 0b before 02 31 main code stream\n");
    lme_usb_talk(st, cmd81, sizeof(cmd81), rb, 1);
    return lme_upload_fw_segment(st, data, 500, size, 0x02, false);
}

static int lme_upload_fw_data_annotated_602_md(struct lme_state *st,
                                               const u8 *data, u16 size)
{
    if (size <= 500) {
        derr("固件大小异常 %u，无法走 annotated 602.md 上传\n", size);
        return -EINVAL;
    }

    /*
     * 602_annotated.md / .tsv:
     *   01 31 descriptor preload
     *   81 0B setup block
     *   02 31 code stream
     *   82 23 tail
     *   8A 00 launch
     *
     * 在线路级别上，这与旧的 legacy segmented upload 相同，
     * 这里单独命名出来，是为了明确区分它和“strict CSV 简化版”。
     */
    return lme_upload_fw_data_legacy(st, data, size);
}

static int lme_upload_fw_data_strict_win_csv(struct lme_state *st,
                                             const u8 *data, u16 size)
{
    u8 rb[1];
    u8 cmd81[] = { 0x81, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    if (size <= 500) {
        derr("固件大小异常 %u，无法走 strict win csv 上传\n", size);
        return -EINVAL;
    }

    /* Windows 602 capture starts with BRIDGE_81, then immediately 02 31 from fw offset 500. */
    dinfo("fw upload control: 81 0b before strict 02 31 offset 500..\n");
    lme_usb_talk(st, cmd81, sizeof(cmd81), rb, 1);
    return lme_upload_fw_segment(st, data, 500, size, 0x02, false);
}

static int lme_pick_fw_upload_mode(void)
{
    if (strict_win_csv_fw_upload == 0 || strict_win_csv_fw_upload == 1)
        return strict_win_csv_fw_upload;

    switch (fw_upload_mode) {
    case 0:
    case 1:
    case 2:
        return fw_upload_mode;
    default:
        derr("未知 fw_upload_mode=%d，回退到 annotated 602.md 路径\n",
             fw_upload_mode);
        return 2;
    }
}

static int lme_upload_fw_data(struct lme_state *st, const u8 *data, u16 size)
{
    switch (lme_pick_fw_upload_mode()) {
    case 1:
        dinfo("firmware upload path: strict Windows CSV (81 -> 02 31 offset 500..)\n");
        return lme_upload_fw_data_strict_win_csv(st, data, size);
    case 2:
        dinfo("firmware upload path: annotated 602.md full path (01 31 -> 81 -> 02 31)\n");
        return lme_upload_fw_data_annotated_602_md(st, data, size);
    case 0:
    default:
        dinfo("firmware upload path: legacy mac/linux segmented upload\n");
        return lme_upload_fw_data_legacy(st, data, size);
    }

}

/*
 * 5300 runtime image starts at file offset 0x01f4.
 * The rt 0x12bd gate entry lands at the last 3 bytes of the firmware blob:
 *   file offset = 0x01f4 + 0x12bd = 0x14b1
 * The remaining bytes of that helper come from the 0x82/0x23 tail payload.
 */
static void lme_patch_fw_gate_bypass(u8 *data, size_t size)
{
    size_t gate_off = 0x14b1;

    if (!fw_gate_bypass)
        return;

    if (size < gate_off + 3) {
        derr("fw_gate_bypass: firmware too short (%zu)\n", size);
        return;
    }

    dinfo("fw_gate_bypass: patch rt 0x12bd @ file 0x%zx -> setb C; ret\n",
          gate_off);
    data[gate_off + 0] = 0xd3; /* setb C */
    data[gate_off + 1] = 0x22; /* ret */
    data[gate_off + 2] = 0x00; /* nop */
}

static int lme_download_fw(struct lme_state *st)
{
    const struct firmware *fw;
    u8 *fw_data;
    u8 buf[2];
    u8 rb[5];
    int ret;

    ret = request_firmware(&fw, LME_FW_NAME, &st->udev->dev);
    if (ret) { derr("找不到固件 %s\n", LME_FW_NAME); return ret; }

    dinfo("上传固件 %s (%zu bytes)\n", LME_FW_NAME, fw->size);
    fw_data = kmemdup(fw->data, fw->size, GFP_KERNEL);
    if (!fw_data) {
        release_firmware(fw);
        return -ENOMEM;
    }
    lme_patch_fw_gate_bypass(fw_data, fw->size);
    ret = lme_upload_fw_data(st, fw_data, (u16)fw->size);
    kfree(fw_data);
    release_firmware(fw);
    if (ret) { derr("固件上传失败\n"); return ret; }

    /* Windows 3.csv: BRIDGE_82 right before 0x8A */
    {
        u8 cmd82[] = { 0x82, 0x23, 0xe0, 0x54, 0x02, 0x70, 0x02, 0xd3,
                       0x22, 0xc3, 0x22, 0xe4, 0xf5, 0x90, 0xc2, 0x88,
                       0xd2, 0xa8, 0xd2, 0xaf, 0x22, 0x7d, 0xfc, 0xe4,
                       0xff, 0x12, 0x11, 0xe8, 0xc2, 0x9c, 0x22, 0x90,
                       0x90, 0x21, 0x74, 0x08, 0xf0, 0x22, 0x08 };
        lme_usb_talk(st, cmd82, sizeof(cmd82), rb, 1);
    }

    buf[0] = 0x8A;
    buf[1] = 0x00;
    lme_usb_talk(st, buf, 2, rb, 5);
    msleep(300);  /* Windows: ~257ms before first I2C */
    dinfo("固件上传完成 (0x8A 00)\n");

    return 0;
}

#define LME_BRIDGE_POLL_MS   20000
#define LME_BRIDGE_RETRY_MS  15000

enum lme_string2_state {
    LME_STR2_ERR = -1,
    LME_STR2_OTHER = 0,
    LME_STR2_DEFG,
    LME_STR2_GGGG,
};

static enum lme_string2_state lme_get_string2_state(struct lme_state *st,
                                                    u8 *raw, int *raw_len,
                                                    int *ret_out)
{
    u8 buf[8] = {0};
    int ret, n;

    ret = usb_control_msg(st->udev, usb_rcvctrlpipe(st->udev, 0),
                          USB_REQ_GET_DESCRIPTOR,
                          USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                          (USB_DT_STRING << 8) | 2, 0x0409,
                          buf, sizeof(buf), 1000);
    n = ret > 0 ? min(ret, (int)sizeof(buf)) : 0;

    if (raw && n > 0)
        memcpy(raw, buf, n);
    if (raw_len)
        *raw_len = n;
    if (ret_out)
        *ret_out = ret;

    if (ret < 6 || buf[0] != 0x06 || buf[1] != 0x03)
        return ret < 0 ? LME_STR2_ERR : LME_STR2_OTHER;

    if (buf[2] == 0x44 && buf[3] == 0x45 &&
        buf[4] == 0x46 && buf[5] == 0x47)
        return LME_STR2_DEFG;

    if (buf[2] == 0x47 && buf[3] == 0x47 &&
        buf[4] == 0x47 && buf[5] == 0x47)
        return LME_STR2_GGGG;

    return LME_STR2_OTHER;
}

/*
 * Experimental same-probe readiness check kept for diagnostics.
 * On the Feiniu host we have observed that 0x8A often causes a real USB
 * disconnect, so the default cold path now waits for warm re-enumeration
 * instead of relying on this helper.
 */
static bool lme_wait_bridge_ready(struct lme_state *st, int ms_total)
{
    int waited = 0, ret = -ETIMEDOUT, tries;
    u8 id = 0, reset = 0;

    msleep(500);
    waited += 500;
    tries = ms_total > waited ? (ms_total - waited) / 100 : 0;

    for (int i = 0; i < tries; i++, waited += 100) {
        ret = gl5_demod_read(st, 0x00, &id);
        if (ret == 0 && gl5_chip_id_valid(id)) {
            gl5_demod_read(st, GL5_R_RESET, &reset);
            dinfo("post-8A bridge ready @%dms chip_id=0x%02x reset=0x%02x\n",
                  waited, id, reset);
            return true;
        }
        if ((i % 5) == 0)
            dinfo("post-8A gl5[%d]: ret=%d chip_id=0x%02x\n",
                  i, ret, id);
        msleep(100);
    }

    derr("post-8A bridge not ready after %dms (last ret=%d chip_id=0x%02x)\n",
         ms_total, ret, id);
    return false;
}

static bool lme_warm_string2_prime(struct lme_state *st)
{
    u8 raw[8];
    int i, ret, n;
    bool ready = false;

    for (i = 0; i < 2; i++) {
        memset(raw, 0, sizeof(raw));
        lme_get_string2_state(st, raw, &n, &ret);
        dinfo("warm string2[%d]: ret=%d raw=%*phN\n", i, ret, n, raw);
        if (n >= 6 &&
            raw[0] == 0x06 && raw[1] == 0x03 &&
            raw[2] == 0x47 && raw[3] == 0x47 &&
            raw[4] == 0x47 && raw[5] == 0x47)
            ready = true;
        msleep(50);
    }
    return ready;
}

static bool lme_try_same_probe_handover(struct lme_state *st)
{
    if (same_probe_wait_ms <= 0) {
        derr("same-probe disabled by same_probe_wait_ms=%d\n", same_probe_wait_ms);
        return false;
    }

    dinfo("post-8A same-probe wait start budget=%dms\n", same_probe_wait_ms);
    return lme_wait_bridge_ready(st, same_probe_wait_ms);
}

/* ================================================================== */
/* USB probe / disconnect                                               */
/* ================================================================== */

static int lme_probe(struct usb_interface *intf,
                     const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(intf);
    struct lme_state  *st;
    int  i, ret;
    bool warm;

    dinfo("probe: %04x:%04x\n",
          le16_to_cpu(udev->descriptor.idVendor),
          le16_to_cpu(udev->descriptor.idProduct));

    st = kzalloc(sizeof(*st), GFP_KERNEL);
    if (!st) return -ENOMEM;

    st->udev = udev;
    mutex_init(&st->usb_mutex);
    mutex_init(&st->ts_mutex);
    spin_lock_init(&st->ts_align_lock);
    atomic_set(&st->probe_ok,    0);
    atomic_set(&st->disconnected, 0);
    atomic_set(&st->demod_busy,  0);
    atomic_set(&st->ts_active,   0);
    atomic_set(&st->ts_probe_rx, 0);
    st->active_gate = 4;
    st->warm_attach = false;
    lme_ts_align_reset(st);
    INIT_DELAYED_WORK(&st->ts_keepalive_work, lme_ts_keepalive_fn);

    /* 预分配URB对象（不分配buffer，buffer在start时分配）*/
    for (i = 0; i < TS_URB_COUNT; i++) {
        st->ts_urbs[i].urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!st->ts_urbs[i].urb) {
            ret = -ENOMEM; goto err_urb;
        }
    }

    usb_set_intfdata(intf, st);

    /*
     * 判断cold/warm:
     * Windows 3.csv 在 first GL5 read 前只做 Select Configuration，
     * 不做 SetInterface(alt 1)。不要用 usb_set_interface() 探测
     * warm，否则会改变桥状态并让 85-read 退化成 77 ACK。
     */
    warm = (le16_to_cpu(udev->descriptor.bcdDevice) == 0x0010);

    if (warm && force_cold && !fw_uploaded) {
        dinfo("warm attach (bcd=0x%04x): force cold reset + firmware upload\n",
              le16_to_cpu(udev->descriptor.bcdDevice));
        lme_coldreset(st);
        msleep(800);
        usb_set_interface(udev, 0, 1);
        msleep(50);
        ret = lme_download_fw(st);
        if (ret)
            goto err_urb;
        dinfo("固件上传完成，等待 warm 重枚举\n");
        st->warm_attach = true;
        st->fw_done_in_probe = true;
        fw_uploaded = true;
        fw_upload_jiffies = jiffies;
        atomic_set(&bridge_log_count, 0);
        goto post_fw_handover;
    }

    if (warm && !force_cold) {
        /*
         * Warm attach: probe string descriptor #2 twice first.
         * Keep force_cold=0 semantics: no firmware re-upload on warm.
         */
        if (lme_warm_string2_prime(st)) {
            dinfo("bridge alive (GGGG)，跳过固件上传\n");
            st->warm_attach = true;
            atomic_set(&bridge_log_count, 0);
            goto warm_init;
        }

        derr("warm attach: string2 not ready, force_cold=0 so skip firmware reupload\n");
        ret = -EAGAIN;
        goto err_urb;
    }

    /* Cold ROM: alt 1 required for bulk firmware path. */
    if (!warm) {
        ret = usb_set_interface(udev, 0, 1);
        dinfo("cold/upload alt1 select ret=%d\n", ret);
        msleep(50);
    }

    dinfo("bridge 无 GGGG，上传固件\n");
    ret = lme_download_fw(st);
    if (ret)
        goto err_urb;
    st->fw_done_in_probe = true;
    fw_uploaded = true;
    fw_upload_jiffies = jiffies;
    st->warm_attach = true;
    atomic_set(&bridge_log_count, 0);
    if (post_8a_delay_ms > 0) {
        dinfo("post-8A extra delay %dms before wait_warm_reenum\n",
              post_8a_delay_ms);
        msleep(post_8a_delay_ms);
    }
    if (post_8a_usb_reset) {
        int reset_ret;

        dinfo("post-8A usb_reset_device experiment start\n");
        reset_ret = usb_reset_device(st->udev);
        dinfo("post-8A usb_reset_device experiment ret=%d\n", reset_ret);
    }

post_fw_handover:
    switch (post_fw_handover_mode) {
    case 1:
        dinfo("post-fw handover mode=1 (same-probe only)\n");
        if (lme_try_same_probe_handover(st))
            goto warm_init;
        derr("same-probe handover failed; mode=1 so abort current probe\n");
        ret = -EIO;
        goto err_urb;
    case 2:
        dinfo("post-fw handover mode=2 (hybrid same-probe -> re-enum)\n");
        if (lme_try_same_probe_handover(st))
            goto warm_init;
        dinfo("same-probe handover failed; falling back to warm re-enumeration\n");
        goto wait_warm_reenum;
    case 0:
    default:
        dinfo("固件上传完成，退出当前probe并等待warm重枚举\n");
        goto wait_warm_reenum;
    }

wait_warm_reenum:
    ret = -ENODEV;
    goto err_urb;

warm_init:
    if (!st->fw_done_in_probe && !lme_warm_string2_prime(st)) {
        derr("warm init: bridge 无 GGGG 且本 probe 未上传固件\n");
        ret = -EIO;
        goto err_urb;
    }
    ret = usb_set_interface(udev, 0, 1);
    if (st->fw_done_in_probe)
        dinfo("warm bulk alt1 reselect ret=%d (current-probe firmware upload path)\n", ret);
    else
        dinfo("warm bulk alt1 select ret=%d\n", ret);
    msleep(500);  /* post-firmware boot settle */
    msleep(200);  /* 等USB稳定，避免MAX2165 I2C风暴 */

    /* 注册DVB adapter */
    ret = dvb_register_adapter(&st->dvb_adap,
                                "LME2510C DTMB Feiniu",
                                THIS_MODULE,
                                &udev->dev,
                                adapter_nr);
    if (ret < 0) { derr("dvb_register_adapter失败\n"); goto err_urb; }

    /* 初始化frontend（内嵌在state里，不单独malloc）*/
    memcpy(&st->fe.ops, &dtmb_fe_ops, sizeof(dtmb_fe_ops));
    st->fe.demodulator_priv = st;
    st->fe.dvb = &st->dvb_adap;

    /* 初始化 dtv_property_cache，避免 TVHeadend 收到零值后崩溃 */
    {
        struct dtv_frontend_properties *c = &st->fe.dtv_property_cache;
        c->delivery_system = SYS_DVBT;
        c->frequency = 474000000;
        c->bandwidth_hz = 8000000;
        c->inversion = INVERSION_AUTO;
        c->modulation = QAM_AUTO;
        c->fec_inner = FEC_AUTO;
        c->transmission_mode = TRANSMISSION_MODE_AUTO;
        c->guard_interval = GUARD_INTERVAL_AUTO;
        c->hierarchy = HIERARCHY_NONE;
    }

    /* 初始化demux（必须在register_frontend之前）*/
    st->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING;
    st->demux.priv      = st;
    st->demux.feednum   = 256;
    st->demux.filternum = 256;
    st->demux.start_feed = lme_start_feed;
    st->demux.stop_feed  = lme_stop_feed;

    ret = dvb_dmx_init(&st->demux);
    if (ret) { derr("dvb_dmx_init失败: %d\n", ret); goto err_adap; }

    st->dmxdev.demux = &st->demux.dmx;
    st->dmxdev.filternum = st->demux.filternum;
    ret = dvb_dmxdev_init(&st->dmxdev, &st->dvb_adap);
    if (ret) { derr("dvb_dmxdev_init失败: %d\n", ret); goto err_dmx; }

    /* 注册frontend */
    ret = dvb_register_frontend(&st->dvb_adap, &st->fe);
    if (ret) { derr("dvb_register_frontend失败: %d\n", ret); goto err_dmxdev; }

    st->fe_hw.source = DMX_FRONTEND_0;
    ret = st->demux.dmx.add_frontend(&st->demux.dmx, &st->fe_hw);
    if (ret) { derr("add_frontend失败: %d\n", ret); goto err_fe; }
    ret = st->demux.dmx.connect_frontend(&st->demux.dmx, &st->fe_hw);
    if (ret) { derr("connect_frontend失败: %d\n", ret); goto err_fe_conn; }

    /* GL5上电初始化 */
    ret = gl5_init(st);
    if (ret)
        derr("GL5 init失败（非致命）: %d\n", ret);

    /* probe完成，之后disconnect才做完整清理 */
    atomic_set(&st->probe_ok, 1);

    dinfo("初始化完成 → /dev/dvb/adapter%d\n", st->dvb_adap.num);
    return 0;

err_fe_conn:
    st->demux.dmx.remove_frontend(&st->demux.dmx, &st->fe_hw);
err_fe:
    dvb_unregister_frontend(&st->fe);
err_dmxdev:
    dvb_dmxdev_release(&st->dmxdev);
err_dmx:
    dvb_dmx_release(&st->demux);
err_adap:
    dvb_unregister_adapter(&st->dvb_adap);
err_urb:
    usb_set_intfdata(intf, NULL);
    for (i = 0; i < TS_URB_COUNT; i++)
        if (st->ts_urbs[i].urb)
            usb_free_urb(st->ts_urbs[i].urb);
    kfree(st);
    return ret;
}

static void lme_disconnect(struct usb_interface *intf)
{
    struct lme_state *st = usb_get_intfdata(intf);
    int i;

    if (!st) return;

    /* 防double-disconnect */
    if (atomic_xchg(&st->disconnected, 1) != 0) {
        dinfo("disconnect: 已处理，跳过\n");
        return;
    }

    if (atomic_read(&st->probe_ok) == 0) {
        /* probe未完成（固件上传阶段的正常重枚举）*/
        dinfo("disconnect: probe未完成，仅释放URB\n");
        for (i = 0; i < TS_URB_COUNT; i++)
            if (st->ts_urbs[i].urb)
                usb_free_urb(st->ts_urbs[i].urb);
        kfree(st);
        usb_set_intfdata(intf, NULL);
        return;
    }

    dinfo("disconnect: 开始清理\n");

    lme_ts_stop(st);

    st->demux.dmx.disconnect_frontend(&st->demux.dmx);
    st->demux.dmx.remove_frontend(&st->demux.dmx, &st->fe_hw);
    dvb_unregister_frontend(&st->fe);
    dvb_dmxdev_release(&st->dmxdev);
    dvb_dmx_release(&st->demux);
    dvb_unregister_adapter(&st->dvb_adap);

    for (i = 0; i < TS_URB_COUNT; i++)
        if (st->ts_urbs[i].urb)
            usb_free_urb(st->ts_urbs[i].urb);

    kfree(st);
    usb_set_intfdata(intf, NULL);
    dinfo("disconnect: 完成\n");
}

/* ================================================================== */
/* 模块注册                                                            */
/* ================================================================== */


/*
 * report2 / FTM602 exact USB bidirectional handshake (from FULL_ANALYSIS_REPORT):
 *   #27 OUT all_pids (03 06 00 FF 01 1F 20 81)
 *   #28 IN  77              (bridge acks)
 *   #29 OUT 77              (echo)
 *   #30 IN  all_pids echo   (bridge echoes route)
 *   #31 OUT all_pids        (RESEND - this was missing!)
 *   #32 IN  88
 *   #33 OUT 88              (echo)
 *   #34 IN  06 00           (bridge sends stream_on!)
 *   #35 OUT 06 00           (echo)
 *   #36 IN  88
 *   #37 OUT 88              (final - TS on EP 0x86)
 *
 * Entire handshake from #27 to #38 in 602.ftm: ~543 microseconds.
 * No msleep() or cmd09 during the handshake - they break the fast protocol.
 */
static int lme_ftm602_report2_handshake(struct lme_state *st, const char *ctx)
{
    u8 all_pids[] = { 0x03, 0x06, 0x00, 0xFF, 0x01, 0x1F, 0x20, 0x81 };
    u8 stream_on[] = { 0x06, 0x00 };
    u8 ack77[] = { 0x77 };
    u8 ack88[] = { 0x88 };
    u8 rb[8] = { 0 };
    int ret, first_ret = 0;

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk_fast_exact(st, all_pids, sizeof(all_pids), rb, 1);
    if (ret && !first_ret)
        first_ret = ret;
    dinfo("TS %s: report2 #27/#28 all_pids -> %02x ret=%d\n",
          ctx, rb[0], ret);

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk_fast_exact(st, ack77, sizeof(ack77), rb, sizeof(all_pids));
    if (ret && !first_ret)
        first_ret = ret;
    dinfo("TS %s: report2 #29/#30 77 -> %*phN ret=%d\n",
          ctx, (int)sizeof(all_pids), rb, ret);

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk_fast_exact(st, all_pids, sizeof(all_pids), rb, 1);
    if (ret && !first_ret)
        first_ret = ret;
    dinfo("TS %s: report2 #31/#32 all_pids -> %02x ret=%d\n",
          ctx, rb[0], ret);

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk_fast_exact(st, ack88, sizeof(ack88), rb, sizeof(stream_on));
    if (ret && !first_ret)
        first_ret = ret;
    dinfo("TS %s: report2 #33/#34 88 -> %*phN ret=%d\n",
          ctx, (int)sizeof(stream_on), rb, ret);

    memset(rb, 0, sizeof(rb));
    ret = lme_usb_talk_fast_exact(st, stream_on, sizeof(stream_on), rb, 1);
    if (ret && !first_ret)
        first_ret = ret;
    dinfo("TS %s: report2 #35/#36 06 00 -> %02x ret=%d\n",
          ctx, rb[0], ret);

    ret = lme_usb_talk_fast_exact(st, ack88, sizeof(ack88), NULL, 0);
    if (ret && !first_ret)
        first_ret = ret;
    dinfo("TS %s: report2 #37 final 88 ret=%d\n", ctx, ret);

    return first_ret;
}
static const struct usb_device_id lme_id_table[] = {
    { USB_DEVICE(LME_VID, LME_PID) },
    {}
};
MODULE_DEVICE_TABLE(usb, lme_id_table);

static struct usb_driver lme_driver = {
    .name       = "lme2510_dtmb",
    .probe      = lme_probe,
    .disconnect = lme_disconnect,
    .id_table   = lme_id_table,
};

module_usb_driver(lme_driver);

MODULE_DESCRIPTION("LME2510C+LGS8GL5+MAX2165 DTMB USB Driver");
MODULE_AUTHOR("逆向工程整理");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(LME_FW_NAME);
MODULE_FIRMWARE(LME_ROM_NAME);
