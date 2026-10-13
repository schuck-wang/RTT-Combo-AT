/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-11-23     luliang     first version
 */

#include <stdlib.h>
#include <string.h>

#include <rtthread.h>

#include "combo_at_dev.h"

#ifdef AT_DEVICE_USING_COMBO

#define LOG_TAG                        "at.sample.combo"
#include <at_log.h>

#ifdef AT_USING_SOCKET
#include <sys/socket.h>
#endif

/* ==========================================================================
 * 1. AT 设备实例 + 注册
 * --------------------------------------------------------------------------
 * 与 at_device 包的 samples/at_sample_esp8266.c 一个路子：
 *     静态实例 -> INIT_APP_EXPORT(注册)
 * 名字/串口/SSID 全取自 combo_at_dev.h 的 COMBO_SAMPLE_xxx 宏；要改就在
 * rtconfig.h 里覆盖同名宏（头文件里那些宏都是 #ifndef 包着的）。
 * ========================================================================== */
static struct at_device_combo combo0 =
{
    .device_name      = COMBO_SAMPLE_DEVICE_NAME,
    .client_name      = COMBO_SAMPLE_CLIENT_NAME,
    .power_pin        = COMBO_SAMPLE_POWER_PIN,
    .power_status_pin = COMBO_SAMPLE_STATUS_PIN,
    .recv_line_num    = COMBO_SAMPLE_RECV_BUFF_LEN,
    .wifi_ssid        = COMBO_SAMPLE_WIFI_SSID,
    .wifi_password    = COMBO_SAMPLE_WIFI_PASSWORD,
};

/* ==========================================================================
 * 2. BLE 数据 / 事件回调
 * --------------------------------------------------------------------------
 * 对应手册 1.5 的 URC：+DATA:<len>,<data> / +EVENT:BLE_CONNECTED /
 * +EVENT:BLE_DISCONNECT。回调在 AT 客户端接收线程里执行，别在里面做耗时操作
 * （要处理数据就丢到队列/线程里）。
 * ========================================================================== */
static void combo_sample_ble_recv(const char *data, rt_size_t len, void *arg)
{
    RT_UNUSED(arg);
    LOG_I("ble recv %d byte: %.*s", (int)len, (int)len, data);
}

static void combo_sample_ble_event(int connected, void *arg)
{
    RT_UNUSED(arg);
    LOG_I("ble %s", connected ? "connected" : "disconnected");
}

static int combo_device_register(void)
{
    /* 这里只是存函数指针，AT 客户端要到 at_device_register() 里才创建 */
    combo_ble_set_recv_cb(combo_sample_ble_recv, RT_NULL);
    combo_ble_set_event_cb(combo_sample_ble_event, RT_NULL);

    return combo_at_dev_register(&combo0);
}
INIT_APP_EXPORT(combo_device_register);

/* ==========================================================================
 * 3. MSH 演示命令（日志风格参考 samples/at_sample_client.c）
 * ========================================================================== */
static void combo_sample_dump(void)
{
    struct combo_wifi_info info;
    char ble_mac[32] = {0};
    int  rssi = 0, ble_state = 0, ble_mode = 0;

    if (combo_at_get_client() == RT_NULL)
    {
        rt_kprintf("combo: AT client not ready (check uart \"%s\")\n", COMBO_SAMPLE_CLIENT_NAME);
        return;
    }

    /* 手册 4.1.7 AT+STAINFO? */
    if (combo_wifi_get_info(&info) == RT_EOK)
    {
        rt_kprintf("wifi : status=%d ssid=%s ip=%s gw=%s mac=%s ch=%d sec=%s\n",
                   info.status, info.ssid, info.ip, info.gateway, info.mac,
                   info.channel, info.security);
    }
    else
    {
        rt_kprintf("wifi : AT+STAINFO? failed\n");
    }

    /* 手册 4.1.18 AT+WRSSI */
    if (combo_wifi_get_rssi(&rssi) == RT_EOK)
    {
        rt_kprintf("rssi : %d dBm\n", rssi);
    }

    /* 手册 5.1.2 / 5.1.4 / 5.1.1 */
    if (combo_ble_get_mode(&ble_mode) == RT_EOK &&
        combo_ble_get_state(&ble_state) == RT_EOK &&
        combo_ble_get_mac(ble_mac, sizeof(ble_mac)) == RT_EOK)
    {
        rt_kprintf("ble  : mode=%d connected=%d mac=%s\n", ble_mode, ble_state, ble_mac);
    }
}

#ifdef AT_USING_SOCKET
/* 手册 4.2.x：WiFi 通了之后就能直接用 BSD socket（走 SAL -> at_socket_combo.c） */
static void combo_sample_tcp(const char *host, int port)
{
    struct sockaddr_in addr;
    char ip[16] = {0};
    char buf[128];
    int  fd, n;

    if (combo_domain_resolve(host, ip) != RT_EOK)
    {
        rt_kprintf("dns \"%s\" failed\n", host);
        return;
    }
    rt_kprintf("tcp %s(%s):%d ...\n", host, ip, port);

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        rt_kprintf("socket() failed: %d\n", fd);
        return;
    }

    rt_memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    /* 自己换字节序，免得依赖 htons 的可用性 */
    addr.sin_port        = (rt_uint16_t)(((port & 0xFF) << 8) | ((port >> 8) & 0xFF));
    addr.sin_addr.s_addr = inet_addr(ip);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        rt_kprintf("connect() failed\n");
        closesocket(fd);
        return;
    }

    n = send(fd, "hello combo\r\n", 13, 0);
    rt_kprintf("send %d byte\n", n);

    n = recv(fd, buf, sizeof(buf) - 1, 0);
    if (n > 0)
    {
        buf[n] = '\0';
        rt_kprintf("recv %d byte: %s\n", n, buf);
    }
    else
    {
        rt_kprintf("recv %d (timeout or closed by peer)\n", n);
    }

    closesocket(fd);
}
#endif /* AT_USING_SOCKET */

static void at_combo_sample(int argc, char **argv)
{
    if (argc == 1)
    {
        combo_sample_dump();
        return;
    }

    if (rt_strcmp(argv[1], "join") == 0)
    {
        if (argc < 3)
        {
            rt_kprintf("usage: at_combo_sample join <ssid> [password]\n");
            return;
        }
        rt_kprintf("AT+WJAP: %s (wait for +EVENT:WIFI_GOT_IP)\n",
                   (combo_wifi_join(argv[2], (argc >= 4) ? argv[3] : "") == RT_EOK) ? "OK" : "FAIL");
    }
    else if (rt_strcmp(argv[1], "auto") == 0 && argc >= 3)
    {
        /* 手册 4.1.9 AT+WAUTOCONN：上电自动重连 */
        rt_kprintf("auto conn %s: %s\n", argv[2],
                   (combo_wifi_set_auto_conn(atoi(argv[2]),
                                             (argc >= 4) ? argv[3] : RT_NULL,
                                             (argc >= 5) ? argv[4] : RT_NULL) == RT_EOK) ? "OK" : "FAIL");
    }
    else if (rt_strcmp(argv[1], "ping") == 0 && argc >= 3)
    {
        int avg = -1;
        rt_kprintf("ping %s: %s (avg %d ms)\n", argv[2],
                   (combo_ping(argv[2], 0, &avg) == RT_EOK) ? "OK" : "FAIL", avg);
    }
    else if (rt_strcmp(argv[1], "disconn") == 0)
    {
        rt_kprintf("AT+WDISCONNECT: %s\n", (combo_wifi_disconnect() == RT_EOK) ? "OK" : "FAIL");
    }
    else if (rt_strcmp(argv[1], "adv") == 0)
    {
        /* 手册 5.2.7：仅从机模式下可设，默认开启 */
        rt_kprintf("ble adv %s: %s\n", (argc >= 3) ? argv[2] : "1",
                   (combo_ble_adv_enable((argc >= 3) ? atoi(argv[2]) : 1) == RT_EOK) ? "OK" : "FAIL");
    }
#ifdef AT_USING_SOCKET
    else if (rt_strcmp(argv[1], "tcp") == 0 && argc >= 4)
    {
        combo_sample_tcp(argv[2], atoi(argv[3]));
    }
#endif
    else
    {
        rt_kprintf("usage: at_combo_sample [dump]\n"
                   "       at_combo_sample join <ssid> [pwd]\n"
                   "       at_combo_sample auto <0|1> [ssid] [pwd]\n"
                   "       at_combo_sample ping <host>\n"
                   "       at_combo_sample disconn\n"
                   "       at_combo_sample adv [0|1]\n"
#ifdef AT_USING_SOCKET
                   "       at_combo_sample tcp <host> <port>\n"
#endif
                   );
    }
}
MSH_CMD_EXPORT_ALIAS(at_combo_sample, at_combo_sample, combo module sample: wifi + socket + ble);

#endif /* AT_DEVICE_USING_COMBO */


