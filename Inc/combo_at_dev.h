/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     port         first version: 安信可 combo 模组(WiFi+BLE) AT 设备类
 *
 * 安信可 "Combo 模组通用指令 V4.18P_3.8.0" 的 RT-Thread at_device 对接层。
 * 手册：applications/combo模组通用指令_v4.18p_3.8.0.pdf（Ai-WB2 / BW16 / PB / TB 系列通用）
 *
 * 三个文件分工：
 *   combo_at_dev.h/.c   设备类（WiFi + BLE），注册进 at_device 框架
 *   at_socket_combo.c   AT socket 协议实现（可选，见下面 AT_USING_SOCKET）
 *
 * ============================== 开关与依赖 ==============================
 * 本文件默认**完全不参与编译**（applications/SConscript 是 glob *.c），
 * 只有定义 AT_DEVICE_USING_COMBO 后才会激活。激活时需要同时满足：
 *   RT_USING_AT + AT_USING_CLIENT    AT 客户端框架
 *   RT_USING_NETDEV                  at_device 的 class 模型依赖 netdev
 *   AT_USING_SOCKET（可选）          需要 socket 时才开，会 select RT_USING_SAL
 * 缺 RT_USING_NETDEV 会直接 #error 提示，不会静默出错。
 *
 * ============================== 模组侧前提 ==============================
 * 1. 串口：AT 指令串口默认 115200 8N1，与下载串口复用；启动信息以 ready 结尾
 * 2. 指令以 \r\n 结束；成功回 \r\nOK\r\n，失败回 \r\n+<CMD>:<errno>\r\nERROR\r\n
 * 3. URC 需要掩码打开（AT+SYSMSG=<mode>,<mask>,<save>），默认 0xFFFFFFFF 全开：
 *      bit0 WIFI_DISCONNECT   bit1 WIFI_CONNECT   bit15 WIFI_GOT_IP
 *      bit4 SocketDown        bit6 SocketDissconnect（固件拼写如此）
 *      bit12 BLE_DISCONNECT   bit13 BLE_CONNECTED  bit14 +DATA
 *    本驱动启动时会确认一次掩码，被关掉会导致收不到数据/事件。
 * 4. socket 收数据用**主动模式**（AT+SOCKETRECVCFG=1），数据直接搭在 URC 上：
 *      +EVENT:SocketDown,<ConID>,<length>,<data>
 *    被动模式(默认)只报长度、数据缓存在模组 RAM 链表里，需要 AT+SOCKETREAD 逐包取，
 *    本驱动不用（会在 socket 初始化时切到主动模式）。
 *
 * ============================== 用法 ==============================
 *   // 1) 只开设备类：在 rtconfig.h 加
 *   //    #define AT_DEVICE_USING_COMBO
 *   //    并把 COMBO_SAMPLE_* 宏按板子改掉（也可在 rtconfig.h 里覆盖）
 *   // 2) 上电自动注册（combo_at_dev.c 末尾 INIT_APP_EXPORT），或自己调：
 *   struct at_device_combo combo;
 *   combo_at_dev_init(&combo, "combo0", "uart2", ssid, pwd);   // 填 device/client 名
 *   combo_at_dev_register(&combo);
 *
 *   // 3) 之后有两种用法（可同时用）：
 *   //    a. netdev（需 RT_USING_NETDEV）：ifconfig / ping / 走 SAL 的 socket
 *   //    b. 直接调下面的 combo_* API（不需要 netdev 也能读 WiFi 信息）
 */

#ifndef __COMBO_AT_DEV_H__
#define __COMBO_AT_DEV_H__

#include <stdlib.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 0. 编译开关 / 依赖检查
 * ========================================================================== */
#ifdef AT_DEVICE_USING_COMBO

#ifndef RT_USING_AT
#error combo_at_dev: 需要 RT_USING_AT
#endif
#ifndef AT_USING_CLIENT
#error combo_at_dev: 需要 AT_USING_CLIENT
#endif
#ifndef RT_USING_NETDEV
#error combo_at_dev: at_device 的 class 模型依赖 netdev, 请打开 RT_USING_NETDEV (components/net/netdev/Kconfig)
#endif

#include <at_device.h>

/* AT 设备类 ID：接在包的最后一个 AT_DEVICE_CLASS_GD32VW553(0x18) 之后。
 * 包内 inc/at_device.h 里没有这一项，所以在本文件本地定义。 */
#define AT_DEVICE_CLASS_COMBO             0x19U

/* combo 模组能同时存在的 socket 连接数上限（AT+SOCKET 的 ConID 由我们指定） */
#define AT_DEVICE_COMBO_SOCKETS_NUM       5

/* ==========================================================================
 * 1. 配置宏（都可用 rtconfig.h / Kconfig 覆盖）
 * ========================================================================== */
#ifndef COMBO_SAMPLE_DEVICE_NAME
#define COMBO_SAMPLE_DEVICE_NAME          "combo0"
#endif

/** AT 指令串口设备名（bSP 里注册的 uart 名字，例如 "uart2"） */
#ifndef COMBO_SAMPLE_CLIENT_NAME
#define COMBO_SAMPLE_CLIENT_NAME          "uart2"
#endif

/**
 * AT 行缓冲大小（接收、发送各一份）。
 * socket 收数据时**整包都要塞进这一行**，所以下固件包这种大数据必须放大：
 *   至少要 ≥ 模组单包最大长度 + 前缀长度 + 2。
 * 默认 4096 能覆盖常见 TCP 段（1460）两倍多。
 * ⚠️ 太小会触发 at_client 的 "read line failed ... out of buffer size" 并**丢包**
 *    （而且是静默丢整包，后续字节还会被当成新行解析 → 数据流错位）。
 */
#ifndef COMBO_SAMPLE_RECV_BUFF_LEN
#define COMBO_SAMPLE_RECV_BUFF_LEN        (4096)
#endif

/**
 * 模组使能脚（手册 1.3：芯片使能管脚，**高电平有效**）。
 * -1（默认）= 不控制，由硬件常置有效；>=0 = 由驱动拉高/拉低。
 *
 * 引脚**不在 Kconfig 里配**（那边只能填裸数字序号，容易填错），
 * 在代码里用 GET_PIN(端口, 序号) 定义即可，且要在 #include "combo_at_dev.h" 之前：
 *     #define COMBO_SAMPLE_POWER_PIN    GET_PIN(C, 6)
 */
#ifndef COMBO_SAMPLE_POWER_PIN
#define COMBO_SAMPLE_POWER_PIN            (-1)
#endif

/** 使能脚有效电平。本板高电平有效 -> PIN_HIGH；低有效改 PIN_LOW */
#ifndef COMBO_POWER_ON_LEVEL
#define COMBO_POWER_ON_LEVEL              (PIN_HIGH)
#endif

/** 拉高使能脚后，留出模组启动到打出 "ready" 的时间 */
#ifndef COMBO_POWER_ON_DELAY_MS
#define COMBO_POWER_ON_DELAY_MS           (1000)
#endif
/** 拉低使能脚后，等电源真正掉下去的时间 */
#ifndef COMBO_POWER_OFF_DELAY_MS
#define COMBO_POWER_OFF_DELAY_MS          (300)
#endif
/** 1 = 每次初始化先断电再上电（硬复位，状态最干净）；0 = 直接拉高 */
#ifndef COMBO_POWER_CYCLE
#define COMBO_POWER_CYCLE                 (1)
#endif
/** 等 "ready" 失败后断电重来的次数。只有配了使能脚(POWER_PIN>=0)才会重试 */
#ifndef COMBO_INIT_RETRY_NUM
#define COMBO_INIT_RETRY_NUM              (2)
#endif

/**
 * 模组电源状态指示脚：**可选**，没接就保持 -1。
 * 它和使能脚是**各自独立**判断的 —— 只接使能脚、不接状态脚是完全正常的用法。
 * 同样用 GET_PIN 在代码里定义： #define COMBO_SAMPLE_STATUS_PIN  GET_PIN(D, 3)
 */
#ifndef COMBO_SAMPLE_STATUS_PIN
#define COMBO_SAMPLE_STATUS_PIN           (-1)
#endif

/* ---- 手册 2.10 AT+SLEEP=<mode>[,<wakeup source>,<param1>,<param2>] ----
 * mode        0/1 浅睡眠(上电不自动/自动)  2 深度睡眠  3 普通模式
 * wake source 0 定时器唤醒  2 GPIO 唤醒
 * param1      source=0 时的定时 ms / source=2 时的 GPIO 序号
 * param2      source=2 时：0 低电平 1 高电平 2 下降沿 3 上升沿 4 双边沿
 *
 * Ai-WB2 注意事项（手册 2.10 注释）：
 *   1. 只支持 mode 2/3，默认 3（普通模式）—— 浅睡眠 0/1 是 PB/TB 系列才有
 *   2. GPIO 唤醒**只能用 IO7**，而 IO7 就是模组的 RX，所以官方推荐
 *      AT+SLEEP=2,2,7,0（深度睡眠 + IO7 低电平唤醒）
 *      "低电平唤醒"和"串口发任意数据唤醒"其实是同一件事：
 *      串口数据的起始位就会把 RX 拉低
 *   3. 深度睡眠被唤醒后模组会重启，重新打印一遍启动信息
 */
#ifndef COMBO_SLEEP_MODE
#define COMBO_SLEEP_MODE                  (2)     /* 深度睡眠 */
#endif
#ifndef COMBO_SLEEP_WAKE_SOURCE
#define COMBO_SLEEP_WAKE_SOURCE           (2)     /* GPIO 唤醒 */
#endif
#ifndef COMBO_SLEEP_WAKE_PIN
#define COMBO_SLEEP_WAKE_PIN              (7)     /* Ai-WB2 只有 IO7(=RX) */
#endif
#ifndef COMBO_SLEEP_WAKE_LEVEL
#define COMBO_SLEEP_WAKE_LEVEL            (0)     /* 0 = 低电平唤醒 */
#endif
/** AT+SLEEP 回 OK 只表示"已受理"，再给模组一点时间真正睡下去 */
#ifndef COMBO_SLEEP_ENTER_DELAY_MS
#define COMBO_SLEEP_ENTER_DELAY_MS        (100)
#endif
/** 发完唤醒信号后，等模组反应的时间 */
#ifndef COMBO_WAKEUP_DELAY_MS
#define COMBO_WAKEUP_DELAY_MS             (200)
#endif

/* ---- 唤醒脚（可选）：combo_wakeup() 优先用它，没配才退回"串口发数据" ----
 * 别把两侧的"脚号"搞混：
 *   COMBO_SLEEP_WAKE_PIN  = **模组侧**的 IO 序号，Ai-WB2 固定是 7
 *   COMBO_SAMPLE_WAKEUP_PIN = **MCU 侧**那根去拉唤醒网络的 GPIO 编号（-1 = 没接）
 * 两者可以指向同一根网络（板上把 MCU 某个 GPIO 接到模组的 IO7/RX），
 * 也可以就是同一根 UART 线（那样只能用串口方式，把本项留 -1）。
 *
 * 同样不在 Kconfig 里配，用 GET_PIN(端口, 序号) 在代码里定义：
 *     #define COMBO_SAMPLE_WAKEUP_PIN   GET_PIN(A, 1)
 */
#ifndef COMBO_SAMPLE_WAKEUP_PIN
#define COMBO_SAMPLE_WAKEUP_PIN           (-1)
#endif
/** 唤醒脚的"有效"电平。Ai-WB2 是低电平唤醒 -> PIN_LOW */
#ifndef COMBO_WAKEUP_PIN_LEVEL
#define COMBO_WAKEUP_PIN_LEVEL            (PIN_LOW)
#endif
/** 唤醒脚保持有效电平的时间，之后恢复到无效电平（不要一直拉着） */
#ifndef COMBO_WAKEUP_PIN_PULSE_MS
#define COMBO_WAKEUP_PIN_PULSE_MS         (50)
#endif

/** 上电自动连接 AP 的账号（空串表示不自动连，只初始化模组） */
#ifndef COMBO_SAMPLE_WIFI_SSID
#define COMBO_SAMPLE_WIFI_SSID            "aithinker"
#endif
#ifndef COMBO_SAMPLE_WIFI_PASSWORD
#define COMBO_SAMPLE_WIFI_PASSWORD        "12345678"
#endif

/** BLE 默认设备名（备注：本模组仅允许在蓝牙关闭状态下改名字） */
#ifndef COMBO_SAMPLE_BLE_NAME
#define COMBO_SAMPLE_BLE_NAME             "ai-thinker"
#endif
/** BLE 默认工作模式：0 从机  1 主机  2 iBeacon  9 关闭 */
#ifndef COMBO_BLE_MODE
#define COMBO_BLE_MODE                    (0)
#endif

/* ---- 初始化阶段要不要顺带把 WiFi / BLE 配起来 ----
 * 默认都是 0 = **完全不碰**。init 只做设备本身的最小初始化：
 *     上电 -> 等 ready -> ATE0 -> 确认 URC 掩码
 * WiFi / BLE 的配置与连接由使用方按需调 combo_wifi_start() / combo_ble_start()。
 * 这样"只用蓝牙"、"只用 WiFi"、"两者共存"三种用法互不干扰
 * （只用蓝牙时不会去动 WiFi 模式，反之亦然）。
 *
 * 设成 1 则恢复"初始化时自动配"的老行为：
 *   COMBO_INIT_WIFI_AUTO=1 -> 只要有 wifi_ssid 就 AT+WMODE + 连 AP + 等 IP
 *   COMBO_INIT_BLE_SETUP=1 -> 设 BLE 名字 + 关掉"连接后自动进透传"
 */
#ifndef COMBO_INIT_WIFI_AUTO
#define COMBO_INIT_WIFI_AUTO              (0)
#endif
#ifndef COMBO_INIT_BLE_SETUP
#define COMBO_INIT_BLE_SETUP              (0)
#endif

/* ---- 改 WiFi/BLE 参数要不要写进模组 flash ----
 * 这两条 AT 指令都有 saveFlash 参数。默认 0 = 只本次生效、不写 flash，
 * 否则每次上电（含断电重试）都会重写一遍 flash。
 * 要持久化就自己显式调一次 combo_wifi_set_mode(1, 1)。
 */
#ifndef COMBO_WIFI_MODE_SAVE
#define COMBO_WIFI_MODE_SAVE              (0)
#endif
#ifndef COMBO_BLE_CFG_SAVE
#define COMBO_BLE_CFG_SAVE                (0)
#endif

/* ==========================================================================
 * 2. 设备实例（字段布局与 at_device_a9g / at_device_esp8266 保持一致）
 * ========================================================================== */
struct at_device_combo
{
    char  *device_name;        /* netdev / at_device 名字            */
    char  *client_name;        /* AT 客户端名（就是串口设备名）      */
    int    power_pin;          /* 使能脚，-1 不用                    */
    int    power_status_pin;   /* 电源状态指示脚，-1 不用            */
    int    wakeup_pin;         /* 唤醒脚(MCU 侧 GPIO)，-1 = 没有，退回串口方式 */
    size_t recv_line_num;      /* AT 行缓冲大小                      */

    char  *wifi_ssid;          /* 上电要连的 AP，NULL/空串则跳过     */
    char  *wifi_password;

    struct at_device device;
    void  *user_data;
};

/* ==========================================================================
 * 3. WiFi 信息（AT+STAINFO? 的解析结果）
 * ========================================================================== */
struct combo_wifi_info
{
    int  status;               /* 0 未连 1 连接中 2 已连未拿到 IP 3 已连且有 IP 4 失败 */
    char ssid[36];
    char password[68];
    char bssid[20];            /* 小写带冒号 */
    char security[24];         /* Open / WPA2 AES / ... */
    char mac[20];              /* 模组 STA MAC，小写带冒号 */
    int  channel;
    char ip[16];
    char gateway[16];
};

/* ==========================================================================
 * 4. 注册
 * ========================================================================== */

/**
 * @brief 填默认值（device_name/client_name 必须传，其余可留 NULL/0 用默认）
 * @note  这是"照 at_sample_a9g 风格"的辅助函数；也可以自己静态初始化结构体。
 */
void combo_at_dev_init(struct at_device_combo *combo,
                       const char *device_name, const char *client_name,
                       const char *ssid, const char *password);

/**
 * @brief 把 combo 模组注册进 at_device 框架（内部会创建 AT 客户端、加 netdev）
 * @return RT_EOK 成功，其它为错误码
 */
int combo_at_dev_register(struct at_device_combo *combo);

/** 取设备类对象（一般不用管，INIT_DEVICE_EXPORT 已自动注册） */
int combo_at_device_class_register(void);

/* ==========================================================================
 * 4.1 电源 / 低功耗（手册 1.3 使能脚 + 2.10 AT+SLEEP）
 * --------------------------------------------------------------------------
 * 这几个都需要 COMBO_SAMPLE_POWER_PIN >= 0（使能脚真的接到 MCU 上）；
 * 没配使能脚时 power on/off 返回 -RT_ENOSYS，sleep/wakeup 仍然可用。
 * ========================================================================== */

/**
 * @brief 完整重新上电：断电(可选) -> 拉高使能脚 -> 等 "ready"
 *        -> ATE0 -> 确认 URC 掩码 -> 按 wifi_ssid 重连 AP -> BLE 设置
 * @return RT_EOK 成功；-RT_ENOSYS 没配使能脚；-RT_ETIMEOUT 模组没应答
 * @note   打开 AT_DEVICE_COMBO_INIT_ASYN 时这里是异步的，返回只代表线程已启动
 */
int combo_power_on(void);

/**
 * @brief 断电：拉低使能脚 + netdev link down + is_init = RT_FALSE
 * @return RT_EOK 成功；-RT_ENOSYS 没配使能脚
 */
int combo_power_off(void);

/**
 * @brief 手册 2.10 AT+SLEEP=<mode>[,<wakeup source>,<param1>,<param2>]
 * @param mode        0/1 浅睡眠  2 深度睡眠  3 普通模式（Ai-WB2 只有 2/3）
 * @param wake_source 0 定时器唤醒  2 GPIO 唤醒
 * @param param1      source=0 定时 ms / source=2 GPIO 序号
 * @param param2      source=2 唤醒电平 0 低 1 高 2 下降 3 上升 4 双边
 * @note  Ai-WB2 用 combo_sleep(2, 2, 7, 0)；深度睡眠被唤醒后模组会重启
 */
int combo_sleep(int mode, int wake_source, int param1, int param2);

/**
 * @brief 唤醒：先产生唤醒信号，再探测模组是否回来
 *        唤醒信号优先用**专用唤醒脚**（COMBO_SAMPLE_WAKEUP_PIN >= 0）；
 *        没配脚就退回"往串口发任意数据"（起始位把 IO7/RX 拉低）。
 *        之后先按"浅睡眠未重启"探测 AT，不行再按"深度睡眠已重启"等 ready，
 *        最后补 ATE0 + URC 掩码。
 * @return RT_EOK 成功；-RT_ETIMEOUT 模组没应答
 */
int combo_wakeup(void);

/* ==========================================================================
 * 5. WiFi API（都是一次性 AT 指令，不依赖 netdev，任何线程可调）
 * ========================================================================== */

/**
 * @brief 按需启动 WiFi STA 并连接 AP（**不写 flash**）= AT+WMODE=1,0 + AT+WJAP
 * @note  连上并拿到 IP 后模组会发 +EVENT:WIFI_GOT_IP，驱动内部的"网络监视"
 *        线程收到后会查 AT+STAINFO? 并把 netdev 拉起来（IP/网关/掩码/DNS/
 *        link up），所以这里**不需要**自己轮询等 IP。
 *        （URC 回调跑在 AT 客户端接收线程里，不能在里面发 AT 指令，
 *          所以查询必须异步做 —— 这就是那个线程存在的原因。）
 * @return RT_EOK 表示模组已受理 AT+WJAP；真正联网成功看 netdev 状态
 */
int combo_wifi_start(const char *ssid, const char *password);

/** AT+WMODE=<MODE>,<save>: 0 关闭 1 STA 2 AP 3 AP+STA */
int combo_wifi_set_mode(int mode, int save_flash);

/** AT+WJAP=<ssid>,<pwd> —— 连接 AP（注意：本模组参数**不加引号**） */
int combo_wifi_join(const char *ssid, const char *password);

/** AT+WDISCONNECT —— 断开 WiFi（内部会先关 WiFi 再按原模式重启） */
int combo_wifi_disconnect(void);

/** AT+STAINFO? —— 查询连接信息（推荐，不像 AT+WJAP? 会被逗号搞乱） */
int combo_wifi_get_info(struct combo_wifi_info *info);

/** AT+WRSSI —— 已连接 AP 的信号强度，单位 dBm */
int combo_wifi_get_rssi(int *rssi);

/** AT+WAUTOCONN=<status>[,ssid,pwd] —— 上电自动重连；ssid/pwd 传 NULL 表示"用上次成功的" */
int combo_wifi_set_auto_conn(int enable, const char *ssid, const char *password);

/** AT+WSCAN? —— 扫描 AP，结果回调给 cb（可能多行） */
int combo_wifi_scan(void (*cb)(const char *line, void *arg), void *arg);

/** AT+WCONFIG=<status>[,name] —— 手机配网: 0 关 1 WiFi配网 2 蓝牙配网 3 AirKiss 9 blufi 10 WPS */
int combo_wifi_smartconfig(int status, const char *name);

/** AT+PING=<addr>[,<count>] —— count 0 表示用模组默认(3) */
int combo_ping(const char *host, int count, int *avg_ms);

/** AT+WDOMAIN=<server name> —— 域名解析；ip 缓冲至少 16 字节。
 *  socket 层（at_socket_combo.c）与 netdev ping 都用它 */
int combo_domain_resolve(const char *name, char ip[16]);

/* ==========================================================================
 * 6. BLE API
 * --------------------------------------------------------------------------
 * 手册注意点（已在实现里按注释处理）：
 *   - 改名字/UUID/配对码/MTU/功率/广播数据/连接间隔 都必须先"蓝牙关闭"
 *     （AT+BLEMODE=9）后再改，改完再设回原模式。
 *   - 连接成功后模组默认会自动进入透传模式（所以 AT 指令会失效），
 *     用 AT+TRANSENTER=<autoEntry>,<saveFlash> 关掉自动透传。
 * ========================================================================== */

/** AT+BLEMODE=<mode> / AT+BLEMODE?: 0 从机 1 主机 2 iBeacon 9 关闭 */
/* ==========================================================================
 * 6. BLE API（手册 5.x）
 * ========================================================================== */

/**
 * @brief 按需启动 BLE：关掉"连接后自动进透传" + 设名字 + 设工作模式
 * @param name 设备名，NULL/空串则不设
 * @param mode 0 从机  1 主机  2 iBeacon  9 关闭；<0 表示不改模式
 * @note  改名字要求蓝牙处于关闭状态，combo_ble_set_name() 内部会自动处理
 * @return RT_EOK 全部成功；-RT_ERROR 至少有一项失败（日志里有具体哪项）
 */
int combo_ble_start(const char *name, int mode);

int combo_ble_set_mode(int mode);
int combo_ble_get_mode(int *mode);

/** AT+BLENAME: 仅蓝牙关闭时可设；get_name 内部会切到关闭态再切回来 */
int combo_ble_set_name(const char *name);
int combo_ble_get_name(char *name, size_t len);

/** AT+BLEMAC: 小写无分隔，如 94c960123456；重启后生效 */
int combo_ble_get_mac(char *mac, size_t len);
int combo_ble_set_mac(const char *mac);

/** AT+BLEADVDATA=<hex字符串>：最大 32 字节（64 个 hex 字符） */
int combo_ble_set_adv_data(const char *hex_data);

/** AT+BLEADVEN=<status> / AT+BLEADVEN? */
int combo_ble_adv_enable(int enable);
int combo_ble_get_adv_enable(int *enabled);

/** AT+BLEADVINTV=<intv>：单位 0.625ms，取值 160~16384 */
int combo_ble_set_adv_interval(int intv);

/**
 * @brief 手册 5.2.3 AT+BLECONINTV=<min>,<max>,<latency>,<timeout>
 *        **BLE 连接状态下省电的正道**（拉长连接间隔，从机可以少醒几次）
 * @param min_interval 最小连接间隔 6~3200，实际 = ×1.25ms（要求 7.5ms~4s）
 * @param max_interval 最大连接间隔 6~3200，实际 = ×1.25ms（要求 7.5ms~4s）
 * @param latency      slave latency，从机可跳过几次连接事件，0~499
 * @param timeout      监督超时 10~3200，实际 = ×10ms（100ms~32s），
 *                     且必须满足：timeout×10 > (1+latency)×max_interval×1.25
 * @note  仅允许在蓝牙关闭状态下设置 —— 函数内部自动关蓝牙、设完再还原
 * @note  例：combo_ble_set_conn_interval(800, 800, 0, 600) → 1s 间隔 / 6s 超时
 * @return RT_EOK；-RT_EINVAL 参数越界或不满足上面的规范约束；-RT_ERROR 指令失败
 */
int combo_ble_set_conn_interval(int min_interval, int max_interval, int latency, int timeout);

/**
 * @brief 查询 BLE 连接间隔（手册 5.2.3 响应 +BLECONINTV:<min>,<max>,<lat>,<timeout>）
 * @param 四个输出参数都能传 NULL
 */
int combo_ble_get_conn_interval(int *min_interval, int *max_interval, int *latency, int *timeout);

/** AT+BLEAUTH=<pind>：6 位数字，或 "DISENABLE" 关闭配对码 */
int combo_ble_set_pin(const char *pin);

/** AT+BLEMTU=<mtu>：23~250 */
int combo_ble_set_mtu(int mtu);

/** AT+BLESTATE?：0 未连接 1 已连接 */
int combo_ble_get_state(int *connected);

/** AT+BLEDISCON */
int combo_ble_disconnect(void);

/** AT+BLESEND=<len>,<data>（单行，注意指令总长限制 1023 字节） */
int combo_ble_send(const void *data, rt_size_t len);

/** AT+BLESENDRAW=<len> + 等到 '>' 再发原始数据（二进制安全，推荐） */
int combo_ble_send_raw(const void *data, rt_size_t len);

/** AT+TRANSENTER=<autoEntry>,<saveFlash>：连接后是否自动进透传（建议关掉，便于 AT 控制） */
int combo_ble_trans_enter_config(int auto_entry, int save_flash);

/** AT+BLESCAN（主机模式）：扫描结果通过 combo_ble_set_scan_cb 回调 */
int combo_ble_scan(void);

/** AT+BLECONNECT=<MAC>（主机模式单次连接，不重连） */
int combo_ble_connect(const char *mac);

/** AT+BLEAUTOCON=<MAC>,<UUID>,<save>：MAC/UUID 任一即可，另一个传 "FALSE" */
int combo_ble_auto_connect(const char *mac, const char *uuid_suffix, int save_flash);

/** AT+BLEDISAUTOCON */
int combo_ble_disable_auto_connect(void);

/** AT+BLERFPWR=<power>：整数或 "MAX"/"MIN"（需蓝牙关闭状态） */
int combo_ble_set_rf_power(const char *power);

/* --- 事件/数据回调（都在 AT 客户端接收线程上下文里被调用，别做耗时操作） --- */

/** 收到蓝牙透传数据（URC +DATA:<len>,<data>） */
void combo_ble_set_recv_cb(void (*cb)(const char *data, rt_size_t len, void *arg), void *arg);

/** 连接状态变化（URC +EVENT:BLE_CONNECTED / +EVENT:BLE_DISCONNECT），connected: 1/0 */
void combo_ble_set_event_cb(void (*cb)(int connected, void *arg), void *arg);

/** 主机扫描结果（AT+BLESCAN 输出的 "MAC:<mac>" 行） */
void combo_ble_set_scan_cb(void (*cb)(const char *mac, const char *name, int rssi, void *arg), void *arg);

/* ==========================================================================
 * 7. 逃生口
 * ========================================================================== */

/** 取 AT 客户端（可拿去自己发指令） */
struct at_client *combo_at_get_client(void);

/** 执行任意 AT 指令（printf 风格），resp 内部管理，返回 RT_EOK / 负错误码 */
int combo_at_exec(const char *cmd_fmt, ...);

/* ==========================================================================
 * 8. socket（实现在 at_socket_combo.c，仅 AT_USING_SOCKET 时编译）
 * ========================================================================== */
#ifdef AT_USING_SOCKET

/** 设备类注册时调用：填 socket_ops / socket_num */
int combo_socket_class_register(struct at_device_class *class);

/** 设备 init 时调用：把模组切到"主动上报数据"模式 */
int combo_socket_init(struct at_device *device);

#endif /* AT_USING_SOCKET */

#ifdef __cplusplus
}
#endif

#endif /* AT_DEVICE_USING_COMBO */

#endif /* __COMBO_AT_DEV_H__ */
