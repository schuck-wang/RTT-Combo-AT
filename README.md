# combo_at —— 安信可 Combo 模组（WiFi + BLE）at_device 驱动

把安信可 **Combo 模组**接到 RT-Thread 的 `at_device` 框架上，支持：

- **WiFi STA/AP**：连接、扫描、RSSI、自动重连、查询连接信息
- **BSD socket**：WiFi 通了之后直接用 `socket()/connect()/send()/recv()`
  （走 SAL → at_socket → 模组的 `AT+SOCKET*` 指令）
- **BLE**：从机广播 / 主机扫描连接 / 透传收发数据
- **电源与低功耗**：使能脚上电、深度睡眠、串口或专用脚唤醒


| 项目     | 说明                                                                  |
| -------- | --------------------------------------------------------------------- |
| 指令手册 | 安信可《Combo 模组通用指令》**V4.18P_3.8.0**（见 `docs/`）            |
| 实测模组 | **Ai-WB2** 系列                                                       |
| 依赖     | RT-Thread 5.x、`at_device`(latest)、`RT_USING_NETDEV`、`RT_USING_SAL` |
| 对应工程 | RD-P006(spec) / STM32H750 ART-Pi                                      |

> 模组的 AT 口固定 **115200 8N1**，指令以 `\r\n` 结尾。

---

## 1. 目录结构

```
combo_at-latest/
├── Inc/combo_at_dev.h        配置宏、struct at_device_combo、全部对外 API
├── Src/combo_at_dev.c        设备类：netdev ops / WiFi / BLE / 电源低功耗 / URC / MSH
├── Src/at_socket_combo.c     socket 层：AT+SOCKET* → at_socket_ops（AT_USING_SOCKET 才编）
├── examples/at_sample_combo.c 示例：静态实例 + 自动注册 + MSH 演示
└── SConscript                构建门控
```

`SConscript` 的门控关系：

```python
DefineGroup('combo_at', src, depend=['AT_DEVICE_USING_COMBO'])   # 整个组
# AT_USING_SOCKET 没开 -> 剔除 Src/at_socket_combo.c
# AT_DEVICE_COMBO_SAMPLE 开了 -> 追加 examples/at_sample_combo.c
```

---

## 2. 依赖与开关（menuconfig / rtconfig.h）

```
AT_DEVICE_USING_COMBO       = y    # 驱动总开关（会自动 select RT_USING_AT / AT_USING_CLIENT / RT_USING_NETDEV）
AT_DEVICE_COMBO_SOCKET      = y    # 编译 socket 层（会自动 select AT_USING_SOCKET）
AT_DEVICE_COMBO_SAMPLE      = y    # 编译 examples/at_sample_combo.c，不需要就关掉
AT_DEVICE_COMBO_INIT_ASYN   = n    # 初始化放独立线程；配了使能脚建议设 y（见第 7 节）
```

默认的串口 / SSID 等在这里填：

```
COMBO_SAMPLE_DEVICE_NAME    = "combo0"       # netdev 名字
COMBO_SAMPLE_CLIENT_NAME    = "uart2"        # AT 串口设备名
COMBO_SAMPLE_RECV_BUFF_LEN  = 4096           # AT 行缓冲，必须 ≥ 一个完整的 socket 包
COMBO_SAMPLE_WIFI_SSID      = "aithinker"    # COMBO_INIT_WIFI_AUTO=1 时用它去连 AP
COMBO_SAMPLE_WIFI_PASSWORD  = "12345678"
COMBO_SAMPLE_BLE_NAME       = "ai-thinker"   # COMBO_INIT_BLE_SETUP=1 时设成 BLE 名字
COMBO_BLE_MODE              = 0              # BLE 默认模式 0 从机 1 主机 2 iBeacon 9 关
```

> ⚠️ `COMBO_SAMPLE_RECV_BUFF_LEN` 不能小气。socket 数据是搭在
> `+EVENT:SocketDown,<ConID>,<len>,<data>` 这一行里过来的，行解析器
> （`at_recv_readline()`）一旦装不下整行，会 `LOG_E("read line failed ... out of buffer size")`
> 后**把整行丢掉**，剩下的字节还会被当成新行解析 → 数据静默丢失 + 流错位。
> 所以它至少要能装下**一个完整的 socket 包**（默认 4096）。

**只能改在头文件里的（不在 Kconfig，因为不常用）**：

```c
COMBO_INIT_WIFI_AUTO   0    /* 1 = 初始化时自动连 wifi_ssid */
COMBO_INIT_BLE_SETUP   0    /* 1 = 初始化时设 BLE 名字 + 关自动透传 */
COMBO_WIFI_MODE_SAVE   0    /* 1 = AT+WMODE=1,1 存 flash */
COMBO_BLE_CFG_SAVE     0    /* 1 = AT+TRANSENTER 存 flash */
```

**使能脚 / 状态脚 / 唤醒脚不在 Kconfig 里配** —— 那边只能填 GPIO 的裸数字序号，容易填错。
改用代码里的 `GET_PIN(端口, 序号)`，见下一节。

---

## 3. 硬件接线


| 模组侧      | MCU 侧     | 必需 | 说明                                                     |
| ----------- | ---------- | ---- | -------------------------------------------------------- |
| TX          | `uart2` RX | ✅   | AT 口，115200 8N1                                        |
| RX          | `uart2` TX | ✅   |                                                          |
| **EN/使能** | 任意 GPIO  | 建议 | 手册 1.3：**高电平有效**。接上后驱动能做硬复位、下电省电 |
| GND         | GND        | ✅   | 共地                                                     |

可选：

- **电源状态脚**：模组给的"已上电"指示 → `COMBO_SAMPLE_STATUS_PIN`
- **唤醒脚**：一根**独立** GPIO（不要和 UART TX 同一根）接到模组的 IO7/RX 网络 → `COMBO_SAMPLE_WAKEUP_PIN`

---

## 4. 初始化

### 4.1 引脚怎么配：`GET_PIN`

引脚类宏都是 `#ifndef` 包着的，**在 `#include "combo_at_dev.h"` 之前** 定义即可
（`GET_PIN` 来自 STM32 BSP 的 `drivers/drv_gpio.h`）：

```c
#include <rtthread.h>
#include <drv_gpio.h>          /* GET_PIN() */

#define COMBO_SAMPLE_POWER_PIN    GET_PIN(C, 6)   /* 使能脚：PC6，高电平有效 */
// #define COMBO_SAMPLE_WAKEUP_PIN   GET_PIN(A, 1)   /* 唤醒脚：PA1（没接就别写） */
// #define COMBO_SAMPLE_STATUS_PIN   GET_PIN(D, 3)   /* 电源状态脚（没接就别写） */

#include "combo_at_dev.h"
```

不定义 = 头文件默认 `(-1)` = 不控制。

> ⚠️ 别把 `rtconfig.h` 里再定义一遍同名宏，否则和你代码里的 `#define` 撞成宏重定义
> （这也是这些引脚项从 Kconfig 里移除的原因）。

也可以在运行时直接赋值，同样生效：

```c
combo0.power_pin = GET_PIN(C, 6);
```

### 4.2 方式 A：用示例（最省事）

打开 `AT_DEVICE_COMBO_SAMPLE`，示例里**静态实例 + `INIT_APP_EXPORT` 自动注册**：

```c
static struct at_device_combo combo0 =
{
    .device_name      = COMBO_SAMPLE_DEVICE_NAME,
    .client_name      = COMBO_SAMPLE_CLIENT_NAME,
    .power_pin        = COMBO_SAMPLE_POWER_PIN,
    .power_status_pin = COMBO_SAMPLE_STATUS_PIN,
    .wakeup_pin       = COMBO_SAMPLE_WAKEUP_PIN,
    .recv_line_num    = COMBO_SAMPLE_RECV_BUFF_LEN,
    .wifi_ssid        = COMBO_SAMPLE_WIFI_SSID,
    .wifi_password    = COMBO_SAMPLE_WIFI_PASSWORD,
};

static int combo_device_register(void)
{
    /* 只是存函数指针；AT 客户端要到 at_device_register() 里才创建 */
    combo_ble_set_recv_cb(combo_sample_ble_recv, RT_NULL);
    combo_ble_set_event_cb(combo_sample_ble_event, RT_NULL);

    return combo_at_dev_register(&combo0);
}
INIT_APP_EXPORT(combo_device_register);
```

### 4.3 方式 B：自己在应用里注册

```c
#include "combo_at_dev.h"

static struct at_device_combo combo;

static int my_app_init(void)
{
    combo_at_dev_init(&combo, "combo0", "uart2", "my_ssid", "my_pwd");
    combo.power_pin  = GET_PIN(C, 6);      /* 需要就补 */
    combo.wakeup_pin = -1;

    return combo_at_dev_register(&combo);
}
```

`combo_at_dev_register()` 内部会把 `AT_DEVICE_CLASS_COMBO`(0x19) 的设备类实例注册进
`at_device` 框架；设备类本身由 `INIT_DEVICE_EXPORT` 自动注册，不用管。

### 4.4 初始化流程做了什么

**初始化只做「设备本身」必须的事，WiFi / BLE 一概不碰**：

```
[使能脚拉高（必要时先断电，COMBO_POWER_CYCLE）]
  ↓ 等模组打印启动信息，最后一行是 "ready"   ← 顺序不能反：没使能就永远等不到
[ATE0]                    关回显
[AT+SYSMSG?]              查 URC 掩码，缺 socket/BLE 位就补一次（不写 flash）
[AT+GMR]                  打印版本
  ↓
device->is_init = RT_TRUE
```

WiFi / BLE 的配置与连接由你**按需**调 `combo_wifi_start()` / `combo_ble_start()`：


| 你的用法     | 要做什么                                                                |
| ------------ | ----------------------------------------------------------------------- |
| 只用**蓝牙** | init 什么都不用配，用的时候`combo_ble_start("name", 0)`                 |
| 只用**WiFi** | init 什么都不用配，用的时候`combo_wifi_start("ssid", "pwd")`            |
| **两者共存** | 两个都调，互不干扰（BLE 改参数时内部临时`BLEMODE=9` 再还原，不碰 WiFi） |

> 想恢复“初始化时自动配”的老行为：`COMBO_INIT_WIFI_AUTO=1`（有 `wifi_ssid` 就自动连）
> 和 / 或 `COMBO_INIT_BLE_SETUP=1`（设 BLE 名字 + 关自动透传）。

**默认是同步初始化**（在 `INIT_APP_EXPORT` 阶段跑）。配了使能脚 + 重试时最坏会阻塞
约 `2 × (0.3+1+10) ≈ 22.6 s`（只有模组真的没应答才会走到重试），
**建议同时打开 `AT_DEVICE_COMBO_INIT_ASYN=y`**。

---

## 5. WiFi

### 5.1 连接 / 查询

最常用的一步到位：

```c
combo_wifi_start("my_ssid", "my_pwd");
/* = AT+WMODE=1,0 + AT+WJAP（默认不写 flash，见 COMBO_WIFI_MODE_SAVE）
 * 连上并拿到 IP 后模组会发 +EVENT:WIFI_GOT_IP，驱动内部的"网络监视"线程
 * 会自动查 AT+STAINFO? 并把 netdev 拉起来（IP/网关/掩码/DNS/link up），
 * 所以**不需要**自己轮询等 IP。 */
```

分开调也行：

```c
combo_wifi_set_mode(1, 0);              /* AT+WMODE=1,0  STA 模式（write_flash=0） */
combo_wifi_join("my_ssid", "my_pwd");   /* AT+WJAP；注意本模组参数**不加引号** */

struct combo_wifi_info info;
combo_wifi_get_info(&info);             /* AT+STAINFO? -> status/ssid/ip/gw/mac/ch/sec */
/*  status: 0 未连  1 连接中  2 已连未拿 IP  3 已连且有 IP  4 失败 */

int rssi;
combo_wifi_get_rssi(&rssi);             /* AT+WRSSI，单位 dBm */

combo_wifi_disconnect();                /* AT+WDISCONNECT */
```

连上并拿到 IP 的状态用 `info.status == 3` 判断。

> ⚠️ **为什么 netdev 的更新要绕一个线程**：`+EVENT:WIFI_GOT_IP` 的 URC 回调是由
> `at_client.c` 的 `client_parser()` 在 **AT 客户端接收线程** 里原地调用的，在里面调
> `at_obj_exec_cmd()` 会等一个只有该线程才能发出的响应通知 → **必然超时**，
> 还会把接收线程堵住好几秒。所以 URC 只置事件，`AT+STAINFO?` 查询交给
> `combo_evt_thread_entry()` 那个小线程做（平时阻塞在事件上，不耗 CPU）。
> **你自己写 URC 处理时也要守这条规矩。**

### 5.2 扫描

```c
static void on_ap(const char *line, void *arg) { rt_kprintf("  %s\n", line); }

combo_wifi_scan(on_ap, RT_NULL);        /* AT+WSCAN?，结果逐行回调 */
```

### 5.3 上电自动重连

```c
combo_wifi_set_auto_conn(1, "my_ssid", "my_pwd");  /* AT+WAUTOCONN=1,ssid,pwd */
combo_wifi_set_auto_conn(1, RT_NULL, RT_NULL);     /* 用"上次连成功的 AP" */
```

深度睡眠唤醒后模组会重启并重连，靠的就是这个（下次初始化也会再 `WJAP` 一次兜底）。

### 5.4 手机配网

```c
combo_wifi_smartconfig(1, RT_NULL);   /* 1 WiFi配网 2 蓝牙配网 3 AirKiss 9 blufi 10 WPS */
```

### 5.5 用 BSD socket

WiFi 通了之后 netdev 就是正常网卡，直接用 POSIX socket（`AT_USING_SOCKET` 打开时）：

```c
int fd = socket(AF_INET, SOCK_STREAM, 0);
struct sockaddr_in addr = {0};
addr.sin_family = AF_INET;
addr.sin_port   = htons(8080);
addr.sin_addr.s_addr = inet_addr("192.168.1.100");

connect(fd, (struct sockaddr *)&addr, sizeof(addr));
send(fd, "hello", 5, 0);
int n = recv(fd, buf, sizeof(buf), 0);
closesocket(fd);
```

也可以走 netdev 的 `ping` / `netstat`：

```
msh > ping www.baidu.com
msh > netstat
```

---

## 6. 蓝牙（BLE）

手册 5.x 里**大量参数"仅允许在蓝牙关闭状态下设置"**（名字/UUID/配对码/MTU/发射功率/
广播数据/连接间隔）。驱动内部用 `combo_ble_enter_setting()/exit_setting()` 包了一层：
自动 `AT+BLEMODE=9` 关蓝牙 → 设置 → 还原原来的模式。所以直接调 API 就行，不用自己关。
一步启动：

```c
combo_ble_start("my-device", 0);
/* = 关掉"连接后自动进透传"(AT+TRANSENTER=0,0) + AT+BLENAME + AT+BLEMODE=0
 * name 传 NULL 就不改名字；mode < 0 就不改模式。
 * 需要细调再往下看 6.1 / 6.2 的单项 API。 */
```

### 6.1 从机模式（默认，`BLEMODE=0`）

```c
combo_ble_set_mode(0);                       /* 从机 */
combo_ble_set_name("my-device");             /* AT+BLENAME */
combo_ble_set_adv_data("...hex...");         /* AT+BLEADVDATA，十六进制字符串 */
combo_ble_adv_enable(1);                     /* AT+BLEADVEN，从机模式下才有效，默认开 */
combo_ble_set_adv_interval(1600);            /* AT+BLEADVINTV，160~16384，实际 = ×0.625ms */
combo_ble_set_pin("123456");                 /* AT+BLEAUTH 配对码，DISENABLE 关掉 */

char name[64], mac[32];
combo_ble_get_name(name, sizeof(name));
combo_ble_get_mac(mac, sizeof(mac));

int connected = 0;
combo_ble_get_state(&connected);             /* AT+BLESTATE? */
combo_ble_disconnect();                      /* AT+BLEDISCON */
```

手机连上/断开通过事件回调通知（URC `+EVENT:BLE_CONNECTED` / `+EVENT:BLE_DISCONNECT`）：

```c
static void on_ble_event(int connected, void *arg)
{
    LOG_I("ble %s", connected ? "connected" : "disconnected");
}
combo_ble_set_event_cb(on_ble_event, RT_NULL);
```

### 6.2 主机模式（`BLEMODE=1`）

```c
combo_ble_set_mode(1);

static void on_scan(const char *mac, const char *name, int rssi, void *arg)
{
    LOG_I("scan: %s  %s  %d dBm", mac, name, rssi);
}
combo_ble_set_scan_cb(on_scan, RT_NULL);
combo_ble_scan();                            /* AT+BLESCAN，结果走上面的回调 */

combo_ble_connect("aa:bb:cc:dd:ee:ff");      /* AT+BLECONNECT */
combo_ble_auto_connect("aa:bb:...", RT_NULL, 1);  /* AT+BLEAUTOCON，MAC/UUID 二选一 */
combo_ble_disable_auto_connect();            /* AT+BLEDISAUTOCON */
```

> `AT+BLESCAN` 的输出是 `name:<name>` / `MAC:<mac>` / `rssi:<rssi>` **三行一组**，
> 驱动的 URC 表把它们拼好后再回调，所以回调里能同时拿到三者。

### 6.3 收发数据

收（从机被手机写入，或主机收到 notify）：

```c
static void on_ble_recv(const char *data, rt_size_t len, void *arg)
{
    /* 在 AT 客户端接收线程里执行，别做耗时操作，丢队列/线程里处理 */
    LOG_HEX("ble", 16, data, len);
}
combo_ble_set_recv_cb(on_ble_recv, RT_NULL);
```

数据来源是 URC `+DATA:<len>,<data>`，**回调里的 `data` 不是 `\0` 结尾的**，长度看 `len`。

发：

```c
combo_ble_send("hello", 5);          /* AT+BLESEND=<len>,<data>，单行文本，≤128 字节 */
combo_ble_send_raw(buf, len);        /* AT+BLESENDRAW=<len> -> 等 '>' -> 原始字节，二进制安全 */
```

二进制 / 含逗号的数据用 `combo_ble_send_raw()`。

### 6.4 透传

`AT+TRANSENTER` 控制"BLE 连接后是否自动进透传"。**进透传后 AT 指令会失效**，所以驱动
初始化里已经设成 `0`（不自动进）。要临时进透传：

```
msh > combo_at AT+SOCKETTT     # 模组透传（socket 用）
msh > combo_at AT+BLEMODE?     # 随便发条指令看看是否还在 AT 模式
```

退出透传发 `+++`。

---

## 7. 低功耗

### 7.1 `AT+SLEEP` 参数（手册 2.10）

```
AT+SLEEP=<mode>[,<wakeup source>,<param1>,<param2>]
```


| 参数            | 取值                                                                      |
| --------------- | ------------------------------------------------------------------------- |
| `mode`          | `0` 浅睡眠(上电不自动) `1` 浅睡眠(上电自动) **`2` 深度睡眠** `3` 普通模式 |
| `wakeup source` | `0` 定时器唤醒  `2` GPIO 唤醒（仅 mode=0/1/2 有效）                       |
| `param1`        | source=0 时是定时 ms；source=2 时是 **GPIO 序号**                         |
| `param2`        | source=2 时的唤醒电平：`0` 低 `1` 高 `2` 下降沿 `3` 上升沿 `4` 双边沿     |

**唤醒方式（手册备注）：向串口发送任意数据即可唤醒模组。**

### 7.2 Ai-WB2 的限制（这条一定要知道）

手册 2.10 注释原文：

> PB 系列默认模式为 3，普通模式；TB 系列默认模式为 3，普通模式；
> **Ai-WB2 系列支持 mode 2/3，默认为 3**。
> 注意：Ai-WB2 系列的 GPIO 唤醒配置时 param1 指定的引脚是没有映射的，也就是芯片实际的
> 引脚，且**仅支持 IO7 引脚**（IO7 是 RX 引脚，所以串口唤醒我们一般设置 IO7 低电平环境
> 即可，也就是 `AT+SLEEP=2,2,7,0`）。

推论：

1. **Ai-WB2 没有浅睡眠**（mode 0/1），只有深度睡眠(2) / 普通(3)
2. 深度睡眠会掉电 RAM → **被唤醒后模组是从复位重新启动的**，会重新打印一遍启动信息
   （结尾 `ready`）
3. GPIO 唤醒只能用 **IO7**，而 IO7 就是模组的 **RX** —— 所以"IO7 低电平唤醒"和
   "串口发任意数据唤醒"**在电气上是同一件事**，区别只是由谁去拉低那根线

### 7.3 怎么进睡眠

```c
/* 完整参数形式，手册怎么写就怎么传 */
combo_sleep(2, 2, 7, 0);     /* Ai-WB2 推荐值：深度睡眠 + IO7 低电平唤醒 */

/* 用头文件里的默认值（Kconfig 之外的调参宏，可自行改） */
combo_sleep(COMBO_SLEEP_MODE, COMBO_SLEEP_WAKE_SOURCE,
            COMBO_SLEEP_WAKE_PIN, COMBO_SLEEP_WAKE_LEVEL);

combo_sleep(3, 0, 0, 0);     /* 回普通模式 */
```

也可以走 `at_device` 框架的标准控制命令。⚠️ 注意 **`at_device` 不会注册 `rt_device`**，
所以 `rt_device_find("combo0")` 是找不到它的，要用 `at_device_get_by_name()`：

```c
#include <at_device.h>

struct at_device *dev = at_device_get_by_name(AT_DEVICE_NAMETYPE_NETDEV, "combo0");
if (dev != RT_NULL)
{
    at_device_control(dev, AT_DEVICE_CTRL_POWER_OFF, RT_NULL);
    at_device_control(dev, AT_DEVICE_CTRL_POWER_ON,  RT_NULL);
    at_device_control(dev, AT_DEVICE_CTRL_SLEEP,     RT_NULL);  /* 走 COMBO_SLEEP_* 默认参数 */
    at_device_control(dev, AT_DEVICE_CTRL_LOW_POWER, RT_NULL);
    at_device_control(dev, AT_DEVICE_CTRL_WAKEUP,    RT_NULL);
}
```

`AT+SLEEP` 回 `OK` **只表示"已受理"**，驱动里再 `mdelay(100)` 给模组一点时间真正睡下去。
睡下去之后串口就不通了（发什么都不会有回应，除了把模组唤醒）。

相关调参宏（都在 `combo_at_dev.h`，都是 `#ifndef`）：

```c
COMBO_SLEEP_MODE             2      /* 深度睡眠 */
COMBO_SLEEP_WAKE_SOURCE      2      /* GPIO 唤醒 */
COMBO_SLEEP_WAKE_PIN         7      /* 模组侧 IO7（Ai-WB2 固定 7） */
COMBO_SLEEP_WAKE_LEVEL       0      /* 低电平唤醒 */
COMBO_SLEEP_ENTER_DELAY_MS   100    /* 回 OK 后再等多久 */
COMBO_WAKEUP_DELAY_MS        200    /* 发完唤醒信号后等模组反应 */
```

### 7.4 怎么唤醒

```c
combo_wakeup();
```

唤醒信号**优先用专用唤醒脚**，没配就退回串口：


| `COMBO_SAMPLE_WAKEUP_PIN` | 唤醒动作                                                                                                 |
| ------------------------- | -------------------------------------------------------------------------------------------------------- |
| `>= 0`                    | 把该 GPIO 拉成`COMBO_WAKEUP_PIN_LEVEL`（默认低），保持 `COMBO_WAKEUP_PIN_PULSE_MS`(50ms)，再恢复无效电平 |
| `-1`（默认）              | 往串口发 2 字节`\r\n`，借用起始位把 IO7/RX 拉低                                                          |

之后 `combo_wakeup()` 会自动分辨两种情况：

- **浅睡眠被唤醒** → 模组没重启 → 用 `AT` 探测到应答就结束
- **深度睡眠被唤醒** → 模组已重启 → `at_client_obj_wait_connect()` 等 `ready`，
  再补一次 `ATE0` + URC 掩码

> ⚠️ 唤醒脚只适合**独立的一根 GPIO**。如果那根网络就是 MCU 的 UART TX，不能用 GPIO 方式
> （引脚被 UART 外设占着），把 `COMBO_SAMPLE_WAKEUP_PIN` 留 `-1` 走串口方式即可。

### 7.5 蓝牙连上之后还能进低功耗吗？

**能进，但连接一定会断。分两种情况说：**


| 场景                               | 能不能省电          | 结论 / 做法                                                                                   |
| ---------------------------------- | ------------------- | --------------------------------------------------------------------------------------------- |
| BLE 已连接 +**深度睡眠**         | ✅ 省电             | 连接**必断**。深度睡眠会掉电 RAM，唤醒后模组重启，BLE 链路和对端都会掉。要恢复得重新广播/重连 |
| BLE 已连接 +**想保持连接同时省电** | ❌**Ai-WB2 做不到** | 浅睡眠(0/1)才能保持无线链路，但**手册明说 Ai-WB2 只支持 mode 2/3**，没有浅睡眠可用            |
| BLE 从机广播中（**未连接**）       | ✅                  | 拉长广播间隔`AT+BLEADVINTV`（160~16384，×0.625ms）；不用蓝牙就直接 `BLEMODE=9` 关掉          |
| 只连 WiFi / 什么都没连             | ✅                  | 直接`combo_sleep(2,2,7,0)`                                                                    |

**连接期间想省电，只能靠 BLE 自身的参数，不能用 `AT+SLEEP`：**

1. **拉长连接间隔**（最有效）：`AT+BLECONINTV=<min>,<max>,<latency>,<timeout>`
   - `min/max` 取值 6~3200，实际时间 = 值 × **1.25ms**（默认 PB `6,12,0,200`，TB `8,8,99,400`）
   - `latency` = slave latency，从机可以跳过几个连接事件不响应（默认 0）
   - `timeout` 单位 ×10ms，范围 10~3200（100ms~32s）
   - **必须满足（手册 5.2.3 原文）**：`timeout×10 > (1+latency) × max_interval × 1.25`（ms）
     注：蓝牙规范的要求更严（右边还要 ×2），按规范来更保险；驱动按手册这条拦参数
   - 例：`AT+BLECONINTV=800,800,0,600` → 1 s 间隔、6 s 超时（满足约束）
   - 例：`AT+BLECONINTV=800,800,4,2000` → 1 s 间隔 + 跳过 4 次、20 s 超时
2. **降低发射功率**：`AT+BLERFPWR=MIN` / `-20` / `MAX`（对应 `combo_ble_set_rf_power()`）
3. MCU 自己睡：模组保持连接，由模组侧负责链路维持

`AT+BLECONINTV` 现在有现成 API（内部自动关蓝牙 → 设置 → 还原模式，不写 flash）：

```c
combo_ble_set_conn_interval(800, 800, 0, 600);   /* 1s 间隔 / 6s 超时 */

int mn, mx, lat, tmo;
combo_ble_get_conn_interval(&mn, &mx, &lat, &tmo);
```

函数会先校验参数范围 + 上面那条约束，不满足直接返回 `-RT_EINVAL`，
不用等模组回参数错误。MSH 也有一份：

```
msh > combo_ble connintv                       # 查询
msh > combo_ble connintv 800 800 0 600         # 设置
```

`AT+BLERFPWR` 同样**仅允许在蓝牙关闭状态下设置**，`combo_ble_set_rf_power()` 已自动处理。

另外手册 2.15 有 `AT+TICKLESS`（浅睡眠监听 AP beacon 的间隔，注释里写"已适配
**BL616/618** 系列"），对 Ai-WB2 不一定适用，本驱动也没封装；要试可以
`msh > combo_at AT+TICKLESS=10`。

### 7.6 用使能脚省电（比睡眠更彻底）

使能脚拉低 = 模组彻底断电，比任何睡眠模式都省。代价是唤醒后要整套重新初始化（几秒）。

```c
combo_power_off();     /* 拉低使能脚 + netdev link down + is_init = FALSE */
combo_power_on();      /* 断电->上电->等 ready->ATE0/SYSMSG->连 AP->BLE 设置 */
```

需要 `COMBO_SAMPLE_POWER_PIN >= 0`，否则返回 `-RT_ENOSYS`。

---

## 8. MSH 命令速查

驱动自带（`combo_at_dev.c`）：


| 命令                                 | 作用                                       |
| ------------------------------------ | ------------------------------------------ |
| `combo_info`                         | 打印 WiFi 状态 + RSSI + BLE 模式/连接/MAC  |
| `combo_join <ssid> [pwd]`            | 连 AP                                      |
| `combo_scan`                         | 扫描 AP 列表                               |
| `combo_ping <host> [count]`          | `AT+PING`                                  |
| `combo_power on|off`                 | 使能脚上电/断电（需配`POWER_PIN`）         |
| `combo_sleep [mode] [src] [p1] [p2]` | 进睡眠，不带参数用`2,2,7,0`                |
| `combo_wakeup`                       | 唤醒                                       |
| `combo_ble <sub>`                    | `mode|name|mac|adv|connintv|state|discon|scan|send` |
| `combo_sock_test <host> <port> <path>` | HTTP 拉一个文件，打印正文字节数 + CRC32（验证二进制链路，不写 flash） |
| `combo_at <AT cmd...>`               | 直接发原始 AT 指令，例如`combo_at AT+GMR`  |

示例自带（`at_sample_combo.c`，`AT_DEVICE_COMBO_SAMPLE=y` 时）：

```
at_combo_sample                                  # 不带参数 = dump 状态
at_combo_sample join <ssid> [pwd]                # 内部走 combo_wifi_start()
at_combo_sample auto <0|1> [ssid] [pwd]
at_combo_sample ble [name] [mode]                # 内部走 combo_ble_start()
at_combo_sample ping <host>
at_combo_sample disconn
at_combo_sample adv [0|1]
at_combo_sample tcp <host> <port>                # 走 socket 的 TCP 收发演示
```

**最常用的排障入口就是 `combo_at`** —— 任何手册上的指令都能直接发，不用等驱动封装。

---

## 9. 排查


| 现象                                               | 检查点                                                                                                                                    |
| -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| 一直`wait "ready" timeout`                         | 串口接线/波特率（115200 8N1）；使能脚是否为高、是否配了正确的`POWER_PIN`                                                                  |
| 收不到 socket / BLE 数据，也不报错                 | **URC 掩码被关了**。驱动启动时会查 `AT+SYSMSG?` 并自动补 `AT+SYSMSG=1,FFFFFFFF,0`，若日志里看到 `URC mask ... rewrite` 说明确实被关过     |
| 收到`+EVENT:WIFI_GOT_IP` 但 `ifconfig` 还是没有 IP | 看日志有没有`wifi up: ssid=...`；没有就是 `AT+STAINFO?` 还没就绪（监视线程会重试 5×200ms）。再没有就查 `AT+STAINFO?` 手工返回值          |
| 自己写的 URC 处理里发 AT 指令总是超时              | **URC 回调就跑在 AT 客户端接收线程里**，不能在里面调 `at_obj_exec_cmd()`。改成置事件 + 用线程/工作队列去发（参照 `urc_wifi_got_ip_func`） |
| AT 指令突然全部失效                                | 是不是进了透传模式（`AT+TRANSENTER` / `+++`）                                                                                             |
| BLE 参数设置失败                                   | 那些参数要求**蓝牙关闭状态**，直接调驱动 API 即可（内部会关），手发 `combo_at` 时记得先 `combo_ble mode 9`                                |
| `combo_ble_get_state()` 解析不出                   | 手册响应是`+ BLESTATE:<status>`（`+` 后有空格），驱动已按此匹配                                                                           |
| `AT+WRSSI` 解析不出                                | 手册响应是`+WRSSI :<rssi>`（冒号前有空格），驱动已按此匹配                                                                                |
| socket 收数据收不到                                | 驱动初始化已设`AT+SOCKETRECVCFG=1`（主动模式，数据直接搭在 URC 里）。若模组被复位回默认值 0，就会变成"只报长度不报数据"                   |
| 日志出现`read line failed ... out of buffer size(%d)` | 单个 socket 包比`COMBO_SAMPLE_RECV_BUFF_LEN`还大 → 整包被丢弃且流错位。把它调到 ≥ 最大单包（默认 4096）                                |
| 日志出现`short read %d/%d bytes`                   | 载荷没凑齐，一般是对端提前断开/超时。用`combo_sock_test`对比字节数与 CRC32                                                                            |
| 下载的文件校验不上                                   | 先用`combo_sock_test`跑同一条 URL，字节数和 CRC32 与 PC 端一致了再怀疑 flash 写入部分                                                                 |

日志开关：`AT_DEVICE_USING_COMBO` 打开后，把 `at_log.h` 的等级调到 `LOG_LVL_DBG`
能看到每条 AT 指令（`LOG_TAG` 分别是 `combo.dev` / `combo.skt`）。

---

## 10. 已知限制 / 未实现

- **未实现监听 / TCP Server**：`AT_USING_SOCKET_SERVER` 打开会**编译期 `#error`** 拦住
  （`+EVENT:SocketSeed` 产生的新 ConID 没处理，开了会让 `at_listen` 空指针）
- **未封装 `AT+TICKLESS`**（手册标注适配 BL616/618）
- **未封装** GPS / host name 相关的 `AT_DEVICE_CTRL_*` 命令（本模组也没有）
- Ai-WB2 上 **`LOW_POWER` 与 `SLEEP` 是同一个动作**（都走 `COMBO_SLEEP_*`），因为没有浅睡眠
- socket 收数据是"把数据搭在 URC 行里"的，**二进制安全接收已支持**（行内部分走 URC，超出部分用 `at_client_obj_recv()` 裸读补齐），
  但要求 `COMBO_SAMPLE_RECV_BUFF_LEN` ≥ 一个完整 socket 包（详见第 11 节）
- 深度睡眠唤醒后模组会重启，**上一次的 socket 连接全部失效**，需要重建

---

## 11. 从网站拉固件包（HTTP OTA）

### 结论

**普通 socket 就够用，`AT+SOCKETTT` / `AT+SOCKETAUTOTT` 两个都不要用。**

驱动已支持**二进制安全接收**（见 11.1），推荐直接用 RT-Thread 的 `ota_downloader`
软件包走 HTTP 把文件下到 `download` 分区，再复位让 bootloader 搬运。

### 11.1 为什么原来"下不了二进制"，现在能了

数据是搭在 `+EVENT:SocketDown,<ConID>,<len>,<data>` **这一行**里传过来的，
而 `at_client` 的行解析器（`at_recv_readline()`）**一遇到 `\r\n` 就断行** ——
固件包里几乎必然含 `0x0D 0x0A`，于是：

| 阶段 | 老行为 | 现在 |
|---|---|---|
| 载荷里没有 CRLF | `avail == len+2`，正常投递 | **完全一样，无变化** |
| 载荷里有 CRLF | 行被切断，`len` 被夹到 `avail`，**只投递一个碎片**，其余字节被当成新行解析 → 数据静默损坏 + 流错位 | 行内部分先投递，剩下用 `at_client_obj_recv()` **裸读补齐**，最后再吃掉模块行尾的 CRLF → **完整** |

关键点：HTTP 响应头**本身就是 CRLF 拼的**，所以**文本才是最早受害的那一类**，
这次改动对它同样是修复而不是回归。

三个"不会出问题"的理由：

- **不会误配 URC**：`get_urc_obj()` 要求缓冲区**开头**是某个前缀，而缓冲区开头是
  `+EVENT:SocketDown`，两张 URC 表里没有第二个条目跟它共享前缀
- **不会读过头**：每次裸读上限是 `min(剩余长度, 256)`，绝不会跨界读到下一个包
- **不干扰接收线程**：`at_client_obj_recv()` 结束后会把 `RT_SERIAL_CTRL_SET_RX_TIMEOUT`
  恢复成 `RT_WAITING_NO`，这正是 `at_client_init()` 初始设的值，等于没动过

> 注意：`at_client_obj_recv()` 的官方注释写着"只能在 URC 回调里用" —— 我们正是这么用的。

### 11.2 先验证链路：`combo_sock_test`

只有真机能证明这套补齐逻辑对。用它从服务器拉一个文件，**只算字节数 + CRC32，不写 flash**，
跟 PC 端比一下就知道了：

```
msh > combo_sock_test 192.168.1.10 80 /fw.bin
GET http://192.168.1.10:80/fw.bin ...
  ... 65536 bytes
  ... 131072 bytes
--- response header ---
HTTP/1.1 200 OK
Content-Length: 139264
...
------------------------
Content-Length = 139264, received = 139264 -> MATCH
body: 139264 bytes, CRC32 = 3F2A8C11
```

PC 端对照（和固件里是同一个多项式）：

```bash
python -c "import zlib,sys;print('%08X'%zlib.crc32(open(sys.argv[1],'rb').read()))" fw.bin
```

挑测试文件时**最好选一个含大量 `0x0D 0x0A` 的**（任意 `.bin` / `.elf` 都行），
纯文本文件反而不容易暴露问题。

如果日志里出现 `read line failed ... out of buffer size(%d)`，
说明单个 socket 包比 `COMBO_SAMPLE_RECV_BUFF_LEN` 还大 → 把它再调大。

### 11.3 正式下载：用 `ota_downloader`

`PKG_USING_OTA_DOWNLOADER` + `PKG_USING_HTTP_OTA`（走 `webclient`，底层就是 SAL socket），
**`PKG_USING_YMODEM_OTA` 关掉**（那是走串口的，用不上）。

```
PKG_USING_OTA_DOWNLOADER = y
PKG_USING_HTTP_OTA       = y     # 会自动 select PKG_USING_WEBCLIENT
PKG_USING_YMODEM_OTA     = n
```

用法：

```
msh > at_combo_sample join MyAP 12345678     # 先把 WiFi 连上
msh > http_ota http://192.168.1.10/fw.bin    # 下到 download 分区
msh > reboot
```

**为什么这样做不会把自己写坏**（这点得先想清楚）：

| | 位置 | 说明 |
|---|---|---|
| app 运行的 flash | `norflash1` W25Q256，**QSPI**，内存映射在 `0x90000000` | 代码在这里 XIP 执行 |
| `download` 分区所在 | `norflash0`，**SPI2** | FAL 分区表：`download`(0, 2MB) / `easyflash` / `filesystem` |

两块是**物理上不同的芯片**，所以运行时写 `download` 分区不会覆盖正在执行的代码，
复位后由 bootloader 把 `download` 搬到 `app` 分区。

`ota_downloader` 的 HTTP 分支里分区名是**写死 `download`** 的，正好对上本工程的分区表，
不用额外改配置。

### 附：为什么透传（`AT+SOCKETTT` / `AT+SOCKETAUTOTT`）仍然不推荐

`AT+SOCKETTT`（手册 4.2.11）进透传后 UART 变成纯数据管道：

- **AT 指令全部失效**，只能发 `+++` 退出
- `at_client` 的接收线程会把二进制当 AT 行解析：`recv_line_buf`（本工程 4096 字节）很快溢满
- URC 机制失效 → **BLE 事件、socket 断开通知全丢**
- 退出靠 `+++`，**数据里恰好出现 `+++` 就会误退出**（经典的透传转义问题）

`AT+SOCKETAUTOTT`（手册 4.2.12）更极端：配置一次后**一连上就自动进透传**，模组被锁在
透传态，BLE/AT 功能等于全废 —— 它只适合"把模组当无线串口用"这种单一用途的产品。

### 那还有别的办法吗

**A. 更新【模组自己】的固件** → 用手册 2.14 的 `AT+OTA`，MCU 完全不参与数据传输：

```
msh > combo_at AT+OTA=1,your.server.com,80,/fw/combo.bin
#               Mode=1 HTTP / 2 HTTPS
```

升级是异步的：回 `OK` 只表示"任务启动成功"，成功后会重启模组并切到新固件。

**B. 只想快速验一下、不想装软件包** → 直接 `combo_sock_test`，或者自己用
`socket()/recv()` 收 —— 走的就是 SAL，收到的是**已经过 11.1 补齐的完整字节流**。

**C. 老方案的残留选项** → 让服务器把固件按 **hex / base64 文本**提供。
二进制安全接收做好之后这条路**已经没必要了**（数据量还要膨胀 2 倍 / 1.33 倍），
列在这里只是备个案。
