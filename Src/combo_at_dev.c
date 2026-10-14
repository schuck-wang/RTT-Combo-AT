/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     port         first version: 安信可 combo 模组(WiFi+BLE) at_device 类
 *
 * 安信可 "Combo 模组通用指令 V4.18P_3.8.0" 的设备类实现。
 * 手册：applications/combo模组通用指令_v4.18p_3.8.0.pdf
 *
 * 手册章节 -> 本文件对应实现：
 *   2.x   基础指令        -> combo_get_version() / combo_reset()
 *   4.1.x WiFi 指令       -> combo_netdev_* / combo_wifi_*()
 *   5.x   BLE 指令        -> combo_ble_*()
 *   1.5   URC / 1.6 错误码 -> urc_func 们 / urc_table[]
 *   socket 部分           -> at_socket_combo.c
 *
 * 关于 URC 掩码（手册 2.7 AT+SYSMSG）：
 *   模组默认 0xFFFFFFFF 全开，但用户可能改过。本驱动初始化时**先查询**，
 *   只有在 bit4(SocketDown)/bit6(SocketDissconnect)/bit13(BLE_CONNECTED)/
 *   bit14(+DATA) 被关掉时才写回一次（saveFlash=0，不动 flash）。
 *   掩码被关掉会表现为"收不到数据也收不到事件"，很难查。
 *
 * 关于响应解析：全部走 AT 框架的 at_resp_get_line / at_resp_parse_line_args_by_kw，
 * 不自己拼字符串，免得踩模组响应格式的坑。
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "combo_at_dev.h"

#ifdef AT_DEVICE_USING_COMBO

#define LOG_TAG                        "combo.dev"
#include <at_log.h>

#include <at.h>
#include <netdev.h>

/* ==========================================================================
 * 参数
 * ========================================================================== */
#define COMBO_WAIT_CONNECT_TIME        (10000)      /* 等 ready 最长时间 ms     */
#define COMBO_INIT_THREAD_STACK_SIZE   (2048)
#define COMBO_INIT_THREAD_PRIORITY     (RT_THREAD_PRIORITY_MAX / 2)
#define COMBO_RESP_TIMEOUT             (5 * RT_TICK_PER_SECOND)
#define COMBO_CMD_MAX_LEN              (256)
#define COMBO_RESP_BUF_SIZE            (512)
#define COMBO_IP_LEN                   (16)
#define COMBO_MAC_LEN                  (16)

#define COMBO_DEFAULT_MTU              (1500)
#define COMBO_HWADDR_LEN               (6)

/* 取"有效电平"的反相电平（PIN_HIGH <-> PIN_LOW） */
#define COMBO_PIN_INACTIVE(level)      (((level) == PIN_HIGH) ? PIN_LOW : PIN_HIGH)

/* 手册 1.5 的 URC 掩码位（AT+SYSMSG 的 mask） */
#define COMBO_SYSMSG_BIT_SOCKET_DOWN   (1u << 4)
#define COMBO_SYSMSG_BIT_SOCKET_DISC   (1u << 6)
#define COMBO_SYSMSG_BIT_BLE_CONNECTED (1u << 13)
#define COMBO_SYSMSG_BIT_BLE_DATA      (1u << 14)
#define COMBO_SYSMSG_WANT              (COMBO_SYSMSG_BIT_SOCKET_DOWN  | \
                                        COMBO_SYSMSG_BIT_SOCKET_DISC  | \
                                        COMBO_SYSMSG_BIT_BLE_CONNECTED | \
                                        COMBO_SYSMSG_BIT_BLE_DATA)

/* ==========================================================================
 * 状态
 * ========================================================================== */
static struct at_client *combo_client = RT_NULL;
static struct at_device *combo_device = RT_NULL;
static rt_bool_t         combo_link_up = RT_FALSE;

/* BLE / 扫描 回调：都在 AT 客户端接收线程上下文里被调用 */
static void (*combo_ble_recv_cb)(const char *data, rt_size_t len, void *arg) = RT_NULL;
static void  *combo_ble_recv_arg = RT_NULL;
static void (*combo_ble_event_cb)(int connected, void *arg) = RT_NULL;
static void  *combo_ble_event_arg = RT_NULL;
static void (*combo_ble_scan_cb)(const char *mac, const char *name, int rssi, void *arg) = RT_NULL;
static void  *combo_ble_scan_arg = RT_NULL;

/* AT+BLESCAN 的输出是 name/MAC/rssi 三行一组，这里暂存前两行 */
static char   combo_scan_name[32] = {0};
static int    combo_scan_rssi = 0;

/* ---- 网络事件（异步）----
 * URC 回调是由 at_client.c 的 client_parser() 在 **AT 客户端接收线程**里
 * 原地调用的，而 at_obj_exec_cmd() 要等一个只有该线程（走完响应解析后）才会
 * rt_event_send 的通知 —— 在 URC 回调里发 AT 指令必然超时，还会把接收线程
 * 堵住。所以 URC 只置事件，真正的 AT 查询交给下面这个线程做。 */
#define COMBO_EVT_WIFI_GOT_IP         (1u << 0)
#define COMBO_EVT_THREAD_STACK_SIZE   (1024)
#define COMBO_EVT_THREAD_PRIORITY     (RT_THREAD_PRIORITY_MAX / 2)

static struct rt_event  combo_evt;
static rt_thread_t      combo_evt_tid   = RT_NULL;
static rt_bool_t        combo_evt_ready = RT_FALSE;

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/** 取（缓存）AT 客户端 */
struct at_client *combo_at_get_client(void)
{
    if (combo_client == RT_NULL && combo_device != RT_NULL)
    {
        combo_client = combo_device->client;
    }
    return combo_client;
}

/** 取 AT 客户端锁（RT-Thread mutex 可重入，可安全包住 at_obj_exec_cmd） */
static rt_mutex_t combo_get_lock(void)
{
    if (combo_device == RT_NULL)
    {
        return RT_NULL;
    }
    return at_device_get_client_lock(combo_device);
}

int combo_at_exec(const char *cmd_fmt, ...)
{
    char cmd[COMBO_CMD_MAX_LEN];
    at_response_t resp = RT_NULL;
    rt_err_t result;
    va_list args;

    if (combo_at_get_client() == RT_NULL || cmd_fmt == RT_NULL)
    {
        return -RT_ERROR;
    }

    va_start(args, cmd_fmt);
    rt_vsnprintf(cmd, sizeof(cmd), cmd_fmt, args);
    va_end(args);

    resp = at_create_resp(COMBO_RESP_BUF_SIZE, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        LOG_E("no memory for response create.");
        return -RT_ENOMEM;
    }

    result = at_obj_exec_cmd(combo_at_get_client(), resp, "%s", cmd);
    at_delete_resp(resp);

    if (result != RT_EOK)
    {
        LOG_D("cmd \"%s\" failed.", cmd);
    }

    return (result == RT_EOK) ? RT_EOK : -RT_ERROR;
}

/** 手册 2.8 AT+GMR：响应 at version:/sdk version:/firmware version: 三行 */
static rt_err_t combo_get_version(char *at_ver, rt_size_t at_ver_len)
{
    at_response_t resp = RT_NULL;
    const char *line;
    rt_err_t result = -RT_ERROR;
    rt_size_t i;

    if (combo_at_get_client() == RT_NULL)
    {
        return -RT_ERROR;
    }

    resp = at_create_resp(COMBO_RESP_BUF_SIZE, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+GMR") == RT_EOK)
    {
        result = RT_EOK;
        for (i = 1; i <= resp->line_counts; i++)
        {
            line = at_resp_get_line(resp, i);
            if (line == RT_NULL)
            {
                continue;
            }
            LOG_I("%s", line);
            if (at_ver != RT_NULL && at_ver_len > 0 && rt_strstr(line, "at version:") == line)
            {
                rt_strncpy(at_ver, line + rt_strlen("at version:"), at_ver_len - 1);
                at_ver[at_ver_len - 1] = '\0';
            }
        }
    }

    at_delete_resp(resp);
    return result;
}

/* ==========================================================================
 * 手册 2.7 AT+SYSMSG：确认 URC 掩码
 * ========================================================================== */
static void combo_sysmsg_ensure(void)
{
    at_response_t resp = RT_NULL;
    const char *line;
    rt_uint32_t mask = 0;
    rt_bool_t got_mask = RT_FALSE;
    rt_size_t i;

    if (combo_at_get_client() == RT_NULL)
    {
        return;
    }

    resp = at_create_resp(COMBO_RESP_BUF_SIZE, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+SYSMSG?") != RT_EOK)
    {
        LOG_W("query AT+SYSMSG? failed, URC mask unknown.");
        goto __exit;
    }

    /* 响应形如： +SYSMSG:\r\n1,FFFFFFFF\r\nOK */
    for (i = 1; i <= resp->line_counts; i++)
    {
        line = at_resp_get_line(resp, i);
        if (line == RT_NULL)
        {
            continue;
        }
        if (rt_sscanf(line, "%*d,%x", &mask) == 1)
        {
            got_mask = RT_TRUE;
            break;
        }
    }
    if (got_mask == RT_FALSE)
    {
        LOG_W("parse AT+SYSMSG? result failed.");
        goto __exit;
    }

    if ((mask & COMBO_SYSMSG_WANT) != COMBO_SYSMSG_WANT)
    {
        LOG_W("URC mask 0x%08X misses socket/BLE bits, rewrite as 0xFFFFFFFF.",
              (unsigned)mask);
        /* saveFlash=0：本次生效即可，不写 flash */
        if (combo_at_exec("AT+SYSMSG=1,FFFFFFFF,0") != RT_EOK)
        {
            LOG_E("set AT+SYSMSG failed, socket/BLE data may not be reported!");
        }
    }
    else
    {
        LOG_I("URC mask = 0x%08X (socket/BLE bits on)", (unsigned)mask);
    }

__exit:
    at_delete_resp(resp);
}

/* ==========================================================================
 * 手册 4.1.7 AT+STAINFO? ：查询 WiFi 连接信息
 * 响应：
 *   +STAINFO:<status>
 *   SSID:<ssid>
 *   Password:<pwd>
 *   <bssid>,<Security>,<MAC>,<ch>,<IP>,<gateway>
 *   OK
 * ========================================================================== */
static rt_err_t combo_read_stainfo(struct combo_wifi_info *info)
{
    at_response_t resp = RT_NULL;
    const char *line;
    rt_err_t result = -RT_ERROR;
    rt_size_t i;

    RT_ASSERT(info);

    rt_memset(info, 0, sizeof(struct combo_wifi_info));

    if (combo_at_get_client() == RT_NULL)
    {
        return -RT_ERROR;
    }

    resp = at_create_resp(COMBO_RESP_BUF_SIZE, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+STAINFO?") != RT_EOK)
    {
        goto __exit;
    }

    for (i = 1; i <= resp->line_counts; i++)
    {
        line = at_resp_get_line(resp, i);
        if (line == RT_NULL)
        {
            continue;
        }

        if (rt_strncmp(line, "+STAINFO:", 9) == 0)
        {
            rt_sscanf(line + 9, "%d", &info->status);
        }
        else if (rt_strncmp(line, "SSID:", 5) == 0)
        {
            rt_strncpy(info->ssid, line + 5, sizeof(info->ssid) - 1);
        }
        else if (rt_strncmp(line, "Password:", 9) == 0)
        {
            rt_strncpy(info->password, line + 9, sizeof(info->password) - 1);
        }
        else if (rt_sscanf(line, "%15[^,],%23[^,],%15[^,],%d,%15[^,],%15[^,]",
                           info->bssid, info->security, info->mac,
                           &info->channel, info->ip, info->gateway) == 6)
        {
            result = RT_EOK;
        }
    }

__exit:
    at_delete_resp(resp);
    return result;
}

/** 把 WiFi 信息回填到 netdev（IP/掩码/网关/MAC + link up） */
static void combo_netdev_update(struct netdev *netdev, const struct combo_wifi_info *info)
{
    ip_addr_t addr;

    if (netdev == RT_NULL || info == RT_NULL)
    {
        return;
    }

    if (info->status != 3 || info->ip[0] == '\0')
    {
        return;
    }

    if (inet_aton(info->ip, &addr))
    {
        netdev_low_level_set_ipaddr(netdev, &addr);
    }
    if (inet_aton(info->gateway, &addr))
    {
        netdev_low_level_set_gw(netdev, &addr);
    }
    /* 本模组的 AT+STAINFO? 不返回掩码（只有 AT+WSDHCP? 静态模式才给），
     * 这里按最常见的 /24 填；静态 IP 场景请自行调 combo_wifi_set_static_ip() */
    if (inet_aton("255.255.255.0", &addr))
    {
        netdev_low_level_set_netmask(netdev, &addr);
    }

    if (info->mac[0] != '\0')
    {
        rt_uint32_t mac[COMBO_HWADDR_LEN] = {0};
        int n;

        if (rt_sscanf(info->mac, "%x:%x:%x:%x:%x:%x",
                      &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6)
        {
            for (n = 0; n < COMBO_HWADDR_LEN && n < netdev->hwaddr_len; n++)
            {
                netdev->hwaddr[n] = (rt_uint8_t)mac[n];
            }
        }
    }

    combo_link_up = RT_TRUE;
    netdev_low_level_set_link_status(netdev, RT_TRUE);
    netdev_low_level_set_status(netdev, RT_TRUE);
    netdev_low_level_set_internet_status(netdev, RT_TRUE);
}

/** 手册 4.2.15 AT+WDNS? -> +WDNS:<IP1>,<IP2> */
static void combo_netdev_update_dns(struct netdev *netdev)
{
    at_response_t resp = RT_NULL;
    char dns1[COMBO_IP_LEN] = {0};
    char dns2[COMBO_IP_LEN] = {0};
    ip_addr_t addr;

    if (netdev == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return;
    }

    resp = at_create_resp(256, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+WDNS?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+WDNS:", "+WDNS:%15[^,],%15[^\r]", dns1, dns2) > 0)
    {
        if (dns1[0] != '\0' && inet_aton(dns1, &addr))
        {
            netdev_low_level_set_dns_server(netdev, 0, &addr);
        }
        if (dns2[0] != '\0' && inet_aton(dns2, &addr))
        {
            netdev_low_level_set_dns_server(netdev, 1, &addr);
        }
    }

    at_delete_resp(resp);
}

/* ==========================================================================
 * netdev ops
 * ========================================================================== */
static int combo_net_init(struct at_device *device);

static int combo_netdev_set_up(struct netdev *netdev)
{
    RT_ASSERT(netdev);

    if (netdev->flags & NETDEV_FLAG_UP)
    {
        return RT_EOK;
    }

    netdev->flags |= NETDEV_FLAG_UP;
    netdev_low_level_set_status(netdev, RT_TRUE);
    return RT_EOK;
}

static int combo_netdev_set_down(struct netdev *netdev)
{
    RT_ASSERT(netdev);

    if (!(netdev->flags & NETDEV_FLAG_UP))
    {
        return RT_EOK;
    }

    netdev->flags &= ~NETDEV_FLAG_UP;
    netdev_low_level_set_status(netdev, RT_FALSE);
    netdev_low_level_set_link_status(netdev, RT_FALSE);

    /* 手册 4.1.2：AT+WDISCONNECT 会先关 WiFi 再按原模式重启 */
    combo_wifi_disconnect();
    combo_link_up = RT_FALSE;

    return RT_EOK;
}

/** 手册 4.1.5 AT+WSDHCP=<MODE>[,<IP>,<MASK>,<GATEWAY>]：静态 IP */
static int combo_netdev_set_addr_info(struct netdev *netdev, ip_addr_t *ip_addr,
                                      ip_addr_t *netmask, ip_addr_t *gw)
{
    char ip_str[COMBO_IP_LEN]   = {0};
    char mask_str[COMBO_IP_LEN] = {0};
    char gw_str[COMBO_IP_LEN]   = {0};

    RT_ASSERT(netdev);
    RT_ASSERT(ip_addr || netmask || gw);

    if (ip_addr != RT_NULL)
    {
        rt_strncpy(ip_str, inet_ntoa(*ip_addr), sizeof(ip_str) - 1);
    }
    else
    {
        rt_strncpy(ip_str, inet_ntoa(netdev->ip_addr), sizeof(ip_str) - 1);
    }

    if (gw != RT_NULL)
    {
        rt_strncpy(gw_str, inet_ntoa(*gw), sizeof(gw_str) - 1);
    }
    else
    {
        rt_strncpy(gw_str, inet_ntoa(netdev->gw), sizeof(gw_str) - 1);
    }

    if (netmask != RT_NULL)
    {
        rt_strncpy(mask_str, inet_ntoa(*netmask), sizeof(mask_str) - 1);
    }
    else
    {
        rt_strncpy(mask_str, inet_ntoa(netdev->netmask), sizeof(mask_str) - 1);
    }

    if (ip_str[0] == '\0' || mask_str[0] == '\0' || gw_str[0] == '\0')
    {
        return -RT_EINVAL;
    }

    /* MODE=0：禁用 DHCP 用静态 IP，必须把 IP/MASK/GATEWAY 一起给 */
    if (combo_at_exec("AT+WSDHCP=0,%s,%s,%s", ip_str, mask_str, gw_str) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (ip_addr != RT_NULL)
    {
        netdev_low_level_set_ipaddr(netdev, ip_addr);
    }
    if (gw != RT_NULL)
    {
        netdev_low_level_set_gw(netdev, gw);
    }
    if (netmask != RT_NULL)
    {
        netdev_low_level_set_netmask(netdev, netmask);
    }
    netdev_low_level_set_dhcp_status(netdev, RT_FALSE);

    return RT_EOK;
}

/** 手册 4.2.15 AT+WDNS=<IP1>,<IP2>：两路要一起写 */
static int combo_netdev_set_dns_server(struct netdev *netdev, rt_uint8_t dns_num, ip_addr_t *dns_server)
{
    char dns_str[COMBO_IP_LEN] = {0};
    char keep1[COMBO_IP_LEN] = {0};
    char keep2[COMBO_IP_LEN] = {0};
    at_response_t resp = RT_NULL;
    int ret;

    RT_ASSERT(netdev);
    RT_ASSERT(dns_server);

    rt_strncpy(dns_str, inet_ntoa(*dns_server), sizeof(dns_str) - 1);
    if (dns_str[0] == '\0')
    {
        return -RT_EINVAL;
    }

    /* 先把当前两个 DNS 查出来，保留另一路 */
    resp = at_create_resp(256, 0, COMBO_RESP_TIMEOUT);
    if (resp != RT_NULL)
    {
        if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+WDNS?") == RT_EOK)
        {
            at_resp_parse_line_args_by_kw(resp, "+WDNS:", "+WDNS:%15[^,],%15[^\r]", keep1, keep2);
        }
        at_delete_resp(resp);
    }

    if (dns_num == 0)
    {
        ret = combo_at_exec("AT+WDNS=%s,%s", dns_str, (keep2[0] != '\0') ? keep2 : dns_str);
    }
    else
    {
        ret = combo_at_exec("AT+WDNS=%s,%s", (keep1[0] != '\0') ? keep1 : dns_str, dns_str);
    }

    if (ret != RT_EOK)
    {
        return -RT_ERROR;
    }

    netdev_low_level_set_dns_server(netdev, dns_num, dns_server);
    return RT_EOK;
}

static int combo_netdev_set_dhcp(struct netdev *netdev, rt_bool_t is_enabled)
{
    RT_ASSERT(netdev);

    /* 手册 4.1.5：MODE=1 用 DHCP 获取 IP；关 DHCP 必须先给静态 IP */
    if (is_enabled == RT_TRUE)
    {
        if (combo_at_exec("AT+WSDHCP=1") != RT_EOK)
        {
            return -RT_ERROR;
        }
    }
    else
    {
        LOG_W("disable DHCP needs static addr: call set_addr_info (ifconfig set) first.");
        return -RT_EOK;
    }

    netdev_low_level_set_dhcp_status(netdev, is_enabled);
    return RT_EOK;
}

/** 手册 4.1.13 AT+PING=<addr>[,<count>] -> +PING:<time> / +PING:TIMEOUT */
static int combo_netdev_ping(struct netdev *netdev, const char *host, size_t data_len,
                             rt_uint32_t timeout, struct netdev_ping_resp *ping_resp,
                             rt_bool_t isbind)
{
    char ip[COMBO_IP_LEN] = {0};
    int  avg_ms = -1;

    RT_ASSERT(netdev);
    RT_ASSERT(host);
    RT_ASSERT(ping_resp);

    RT_UNUSED(timeout);
    RT_UNUSED(isbind);

    if (combo_domain_resolve(host, ip) != RT_EOK)
    {
        return -RT_ERROR;
    }
    inet_aton(ip, &ping_resp->ip_addr);
    ping_resp->data_len = (rt_uint16_t)data_len;
    ping_resp->ttl      = 255;

    if (combo_ping(host, 0, &avg_ms) != RT_EOK || avg_ms < 0)
    {
        return -RT_ERROR;
    }

    ping_resp->ticks = (rt_uint32_t)((rt_uint64_t)avg_ms * RT_TICK_PER_SECOND / 1000);
    return RT_EOK;
}

/** 手册 4.2.3 AT+SOCKET?
 *  <ConID>,<type>,<status>,<remote host>,<remote port>,<local port>,<server ConID> */
static void combo_netdev_netstat(struct netdev *netdev)
{
    at_response_t resp = RT_NULL;
    const char *line;
    rt_size_t i;

    RT_ASSERT(netdev);

    resp = at_create_resp(COMBO_RESP_BUF_SIZE, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+SOCKET?") == RT_EOK)
    {
        rt_kprintf("socket list (ConID,type,status,remoteHost,remotePort,localPort,serverConID):\n");
        for (i = 1; i <= resp->line_counts; i++)
        {
            line = at_resp_get_line(resp, i);
            if (line != RT_NULL && line[0] != '\0')
            {
                rt_kprintf("  %s\n", line);
            }
        }
    }

    at_delete_resp(resp);
}

static const struct netdev_ops combo_netdev_ops =
{
    .set_up         = combo_netdev_set_up,
    .set_down       = combo_netdev_set_down,
    .set_addr_info  = combo_netdev_set_addr_info,
    .set_dns_server = combo_netdev_set_dns_server,
    .set_dhcp       = combo_netdev_set_dhcp,
#ifdef RT_USING_FINSH
    .ping           = combo_netdev_ping,
    .netstat        = combo_netdev_netstat,
#endif
};

static struct netdev *combo_netdev_add(const char *netdev_name)
{
    struct netdev *netdev = RT_NULL;

    RT_ASSERT(netdev_name);

    netdev = netdev_get_by_name(netdev_name);
    if (netdev != RT_NULL)
    {
        return netdev;
    }

    netdev = (struct netdev *)rt_calloc(1, sizeof(struct netdev));
    if (netdev == RT_NULL)
    {
        LOG_E("no memory for netdev(%s) create.", netdev_name);
        return RT_NULL;
    }

    netdev->mtu        = COMBO_DEFAULT_MTU;
    netdev->ops        = &combo_netdev_ops;
    netdev->hwaddr_len = COMBO_HWADDR_LEN;

#ifdef SAL_USING_AT
    {
        extern int sal_at_netdev_set_pf_info(struct netdev *netdev);
        /* 交给 SAL：之后 socket()/connect() 走 at_socket_combo.c 的实现 */
        sal_at_netdev_set_pf_info(netdev);
    }
#endif

    netdev_register(netdev, netdev_name, RT_NULL);

    return netdev;
}

/* ==========================================================================
 * 电源 / 低功耗（手册 1.3 使能脚 + 2.10 AT+SLEEP）
 * ========================================================================== */

/** 取组合设备对象 */
static struct at_device_combo *combo_get_combo(void)
{
    if (combo_device == RT_NULL)
    {
        return RT_NULL;
    }
    return (struct at_device_combo *)combo_device->user_data;
}

/** 只操作使能脚电平：on=1 拉有效电平，on=0 拉无效电平。没配脚就是空操作 */
static void combo_power_pin_write(int on)
{
    struct at_device_combo *combo = combo_get_combo();

    if (combo == RT_NULL || combo->power_pin < 0)
    {
        return;
    }

    rt_pin_mode(combo->power_pin, PIN_MODE_OUTPUT);
    rt_pin_write(combo->power_pin,
                 on ? COMBO_POWER_ON_LEVEL : COMBO_PIN_INACTIVE(COMBO_POWER_ON_LEVEL));
}

/**
 * 为"等 ready"做准备：先断电(可选)再上电，然后留出模组启动的时间。
 * 没配使能脚时直接返回（上电由硬件负责），这样对现有工程零影响。
 */
static void combo_power_cycle(void)
{
    struct at_device_combo *combo = combo_get_combo();

    if (combo == RT_NULL || combo->power_pin < 0)
    {
        return;
    }

#if COMBO_POWER_CYCLE
    combo_power_pin_write(0);
    rt_thread_mdelay(COMBO_POWER_OFF_DELAY_MS);
#endif
    combo_power_pin_write(1);
    rt_thread_mdelay(COMBO_POWER_ON_DELAY_MS);
}

/**
 * 上电 + 等 "ready"，失败就断电重来。
 *
 * !!! 顺序很关键 !!!
 * 手册 1.3 的使能脚是"高电平有效"：模组没被使能时不会打印启动信息，
 * 所以必须先上电再 at_client_obj_wait_connect()，反了就永远等不到 ready。
 *
 * @return RT_EOK 收到 ready；-RT_ETIMEOUT 重试用完还没应答
 */
static rt_err_t combo_power_up_and_wait(void)
{
    struct at_device_combo *combo = combo_get_combo();
    int retry, i;

    /* 没配使能脚就只能等一次：断不了电，重试没意义 */
    retry = (combo != RT_NULL && combo->power_pin >= 0) ? COMBO_INIT_RETRY_NUM : 1;

    for (i = 0; i < retry; i++)
    {
        combo_power_cycle();

        if (at_client_obj_wait_connect(combo_device->client, COMBO_WAIT_CONNECT_TIME) == RT_EOK)
        {
            if (i > 0)
            {
                LOG_I("module \"ready\" after %d power cycle(s).", i + 1);
            }
            return RT_EOK;
        }

        /* 先断电，下一轮循环开头再上电（不要在这里上电，否则会少掉一个断电间隔） */
        combo_power_pin_write(0);
        rt_thread_mdelay(COMBO_POWER_OFF_DELAY_MS);

        LOG_W("wait \"ready\" timeout(%d ms), retry %d/%d",
              COMBO_WAIT_CONNECT_TIME, i + 1, retry);
    }

    return -RT_ETIMEOUT;
}

/** AT 层通用初始化：关回显 + 确认 URC 掩码 + 打印版本 */
static void combo_at_link_setup(void)
{
    char at_ver[32] = {0};

    /* 手册 2.6 ATE0：关回显，否则 AT 客户端会把回显行当成数据 */
    combo_at_exec("ATE0");

    /* URC 掩码：收数据/事件的开关 */
    combo_sysmsg_ensure();

    if (combo_get_version(at_ver, sizeof(at_ver)) == RT_EOK && at_ver[0] != '\0')
    {
        LOG_I("combo AT version: %s", at_ver);
    }
}

/**
 * 产生唤醒信号：优先用专用唤醒脚，没配就退回"往串口发任意数据"。
 *
 * 为什么两种方式都行：手册 2.10 注明 Ai-WB2 的 GPIO 唤醒只能用 IO7，
 * 而 IO7 就是模组的 RX —— 串口数据的起始位本来就会把 RX 拉低，所以
 * "IO7 低电平唤醒"和"串口发数据唤醒"在电气上是同一件事，
 * 区别只是由谁去拉低：
 *   有专用脚 -> 直接拉 GPIO，时序可控，不往串口塞多余字节
 *   没专用脚 -> 串口发 2 字节，借用起始位
 */
static void combo_wakeup_signal(struct at_client *client)
{
    struct at_device_combo *combo = combo_get_combo();

    if (combo != RT_NULL && combo->wakeup_pin >= 0)
    {
        rt_pin_mode(combo->wakeup_pin, PIN_MODE_OUTPUT);
        rt_pin_write(combo->wakeup_pin, COMBO_WAKEUP_PIN_LEVEL);
        rt_thread_mdelay(COMBO_WAKEUP_PIN_PULSE_MS);
        rt_pin_write(combo->wakeup_pin, COMBO_PIN_INACTIVE(COMBO_WAKEUP_PIN_LEVEL));

        LOG_I("wakeup: drive pin %d to %d for %d ms.",
              combo->wakeup_pin, COMBO_WAKEUP_PIN_LEVEL, COMBO_WAKEUP_PIN_PULSE_MS);
    }
    else
    {
        rt_mutex_t lock = combo_get_lock();

        /* 手册 2.10 备注：向串口发送任意数据唤醒模组 */
        if (lock != RT_NULL)
        {
            rt_mutex_take(lock, RT_WAITING_FOREVER);
        }
        at_client_obj_send(client, "\r\n", 2);
        if (lock != RT_NULL)
        {
            rt_mutex_release(lock);
        }

        LOG_I("wakeup: send uart data (start bit pulls IO7/RX low).");
    }
}

int combo_power_on(void)
{
    struct at_device_combo *combo = combo_get_combo();

    if (combo_device == RT_NULL || combo == RT_NULL)
    {
        return -RT_ERROR;
    }
    if (combo->power_pin < 0)
    {
        LOG_W("no enable pin(COMBO_SAMPLE_POWER_PIN=%d), cannot power on in software.",
              combo->power_pin);
        return -RT_ENOSYS;
    }

    /* 直接复用完整初始化流程：上电 -> 等 ready -> ATE0/SYSMSG -> 连 AP -> BLE 设置 */
    combo_device->is_init = RT_FALSE;
    return combo_net_init(combo_device);
}

int combo_power_off(void)
{
    struct at_device_combo *combo = combo_get_combo();

    if (combo_device == RT_NULL || combo == RT_NULL)
    {
        return -RT_ERROR;
    }
    if (combo->power_pin < 0)
    {
        LOG_W("no enable pin(COMBO_SAMPLE_POWER_PIN=%d), cannot power off in software.",
              combo->power_pin);
        return -RT_ENOSYS;
    }

    combo_power_pin_write(0);
    rt_thread_mdelay(COMBO_POWER_OFF_DELAY_MS);

    combo_link_up = RT_FALSE;
    if (combo_device->netdev != RT_NULL)
    {
        netdev_low_level_set_link_status(combo_device->netdev, RT_FALSE);
        netdev_low_level_set_status(combo_device->netdev, RT_FALSE);
    }
    combo_device->is_init = RT_FALSE;

    LOG_I("combo module powered off.");
    return RT_EOK;
}

int combo_sleep(int mode, int wake_source, int param1, int param2)
{
    int ret;

    if (combo_at_get_client() == RT_NULL)
    {
        return -RT_ERROR;
    }
    if (mode < 0 || mode > 3)
    {
        return -RT_EINVAL;
    }

    if (mode == 3)
    {
        /* 普通模式：不带唤醒源参数 */
        ret = combo_at_exec("AT+SLEEP=3");
    }
    else if (wake_source == 2)
    {
        /* GPIO 唤醒：要引脚序号 + 唤醒电平 */
        ret = combo_at_exec("AT+SLEEP=%d,2,%d,%d", mode, param1, param2);
    }
    else
    {
        /* 定时器唤醒：只带定时 ms */
        ret = combo_at_exec("AT+SLEEP=%d,%d,%d", mode, wake_source, param1);
    }

    if (ret != RT_EOK)
    {
        return ret;
    }

    /* 回 OK 只表示"已受理"，再等一下让模组真的睡下去（睡下去后串口就不通了） */
    rt_thread_mdelay(COMBO_SLEEP_ENTER_DELAY_MS);

    combo_link_up = RT_FALSE;
    if (combo_device != RT_NULL && combo_device->netdev != RT_NULL)
    {
        netdev_low_level_set_link_status(combo_device->netdev, RT_FALSE);
    }

    LOG_I("combo enter sleep: mode=%d source=%d p1=%d p2=%d", mode, wake_source, param1, param2);
    return RT_EOK;
}

int combo_wakeup(void)
{
    struct at_client *client = combo_at_get_client();
    int i;

    if (client == RT_NULL)
    {
        return -RT_ERROR;
    }

    combo_wakeup_signal(client);

    rt_thread_mdelay(COMBO_WAKEUP_DELAY_MS);

    /* 情况 A：浅睡眠被唤醒 —— 模组没有重启，AT 直接就该应答 */
    for (i = 0; i < 3; i++)
    {
        if (combo_at_exec("AT") == RT_EOK)
        {
            combo_at_link_setup();
            LOG_I("combo woke up (light sleep, no reboot).");
            return RT_EOK;
        }
        rt_thread_mdelay(300);
    }

    /* 情况 B：深度睡眠被唤醒 —— 模组会重启并重新打印启动信息，等 ready */
    if (at_client_obj_wait_connect(client, COMBO_WAIT_CONNECT_TIME) != RT_EOK)
    {
        LOG_E("wakeup failed: module still no response.");
        return -RT_ETIMEOUT;
    }

    combo_at_link_setup();
    if (combo_device != RT_NULL)
    {
        combo_device->is_init = RT_TRUE;
    }

    LOG_I("combo woke up (deep sleep, module rebooted).");
    return RT_EOK;
}

/* ==========================================================================
 * 网络事件（URC -> 线程）
 * --------------------------------------------------------------------------
 * 为什么要有这么个线程：
 *   URC 回调是由 at_client.c 的 client_parser() 在 **AT 客户端接收线程**里
 *   原地调用的，而 at_obj_exec_cmd() 要等一个由该线程（走完响应解析后）才会
 *   rt_event_send 的通知 —— 在 URC 里发 AT 指令必然超时，而且会把接收线程
 *   堵住好几秒。所以 URC 只 rt_event_send，AT 查询全部交给本线程。
 * ========================================================================== */

/** 轮询 AT+STAINFO? 直到拿到 IP，然后回填 netdev。拿不到就超时返回 */
static rt_err_t combo_wifi_wait_and_update(int max_try, int interval_ms)
{
    struct combo_wifi_info info;
    int i;

    rt_memset(&info, 0, sizeof(info));

    if (combo_device == RT_NULL || combo_device->netdev == RT_NULL)
    {
        return -RT_ERROR;
    }

    for (i = 0; i < max_try; i++)
    {
        /* status==3 表示"已连上且拿到 IP" */
        if (combo_read_stainfo(&info) == RT_EOK && info.status == 3 && info.ip[0] != '\0')
        {
            LOG_I("wifi up: ssid=%s ch=%d ip=%s gw=%s",
                  info.ssid, info.channel, info.ip, info.gateway);
            combo_netdev_update(combo_device->netdev, &info);
            combo_netdev_update_dns(combo_device->netdev);
            return RT_EOK;
        }
        rt_thread_mdelay(interval_ms);
    }

    LOG_W("wifi got-IP poll timeout (%d tries): status=%d ip=\"%s\"",
          max_try, info.status, info.ip);
    return -RT_ETIMEOUT;
}

static void combo_evt_thread_entry(void *parameter)
{
    rt_uint32_t evt;

    RT_UNUSED(parameter);

    while (1)
    {
        if (rt_event_recv(&combo_evt, 0xFFFFFFFFu,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &evt) != RT_EOK)
        {
            continue;
        }

        if (evt & COMBO_EVT_WIFI_GOT_IP)
        {
            /* 刚拿到 IP 的那一瞬 AT+STAINFO? 可能还没更新完，给几次机会 */
            combo_wifi_wait_and_update(5, 200);
        }
    }
}

/** 建事件 + 起监视线程（只做一次） */
static void combo_net_evt_init(void)
{
    if (combo_evt_ready)
    {
        return;
    }

    rt_event_init(&combo_evt, "combo_evt", RT_IPC_FLAG_FIFO);

    combo_evt_tid = rt_thread_create("combo_netmon", combo_evt_thread_entry, RT_NULL,
                                     COMBO_EVT_THREAD_STACK_SIZE,
                                     COMBO_EVT_THREAD_PRIORITY, 20);
    if (combo_evt_tid == RT_NULL)
    {
        LOG_E("create combo net monitor thread failed, WIFI_GOT_IP will not update netdev.");
        return;
    }

    rt_thread_startup(combo_evt_tid);
    combo_evt_ready = RT_TRUE;
}

#if COMBO_INIT_WIFI_AUTO
/**
 * 初始化阶段自动连 WiFi（只有 COMBO_INIT_WIFI_AUTO=1 时才编译进来）。
 * 默认是关的：init 不碰 WiFi，用的时候调 combo_wifi_start()。
 */
static void combo_wifi_auto_connect(struct at_device *device)
{
    struct at_device_combo *combo = (struct at_device_combo *)device->user_data;
    int i;

    if (combo->wifi_ssid == RT_NULL || combo->wifi_ssid[0] == '\0')
    {
        LOG_I("no wifi ssid configured, skip join (call combo_wifi_start later).");
        return;
    }

    /* 手册 4.1.1：AT+WMODE=1,<save> -> STA 模式 */
    if (combo_wifi_set_mode(1, COMBO_WIFI_MODE_SAVE) != RT_EOK)
    {
        LOG_W("set STA mode failed.");
    }

    for (i = 0; i < 3; i++)
    {
        if (combo_wifi_join(combo->wifi_ssid, combo->wifi_password) == RT_EOK)
        {
            break;
        }
        LOG_W("join \"%s\" failed, retry %d/3", combo->wifi_ssid, i + 1);
        rt_thread_mdelay(2000);
    }

    /* 等 DHCP 拿 IP 并回填 netdev（最多 30*500ms） */
    combo_wifi_wait_and_update(30, 500);
}
#endif /* COMBO_INIT_WIFI_AUTO */

/* ==========================================================================
 * 初始化线程
 * ========================================================================== */
static void combo_init_thread_entry(void *parameter)
{
    struct at_device *device = (struct at_device *)parameter;
    struct at_device_combo *combo = (struct at_device_combo *)device->user_data;

    /* 使能脚先就位（手册 1.3：芯片使能管脚，高电平有效）。
     * power_pin 与 power_status_pin 各自独立判断 —— 只接使能脚、不接状态脚
     * 是完全正常的用法，不能因为 status_pin 是 -1 就把使能也一起跳过。 */
    if (combo->power_pin >= 0)
    {
        rt_pin_mode(combo->power_pin, PIN_MODE_OUTPUT);
    }
    if (combo->power_status_pin >= 0)
    {
        rt_pin_mode(combo->power_status_pin, PIN_MODE_INPUT);
    }

    /* 唤醒脚：有就配成输出，并先置成"无效"电平，
     * 别在上电阶段就把模组按在唤醒态（而深度睡眠时又恰恰需要它拉低） */
    if (combo->wakeup_pin >= 0)
    {
        rt_pin_mode(combo->wakeup_pin, PIN_MODE_OUTPUT);
        rt_pin_write(combo->wakeup_pin, COMBO_PIN_INACTIVE(COMBO_WAKEUP_PIN_LEVEL));
    }

    /* 先上电（必要时断电重来），再等启动信息（手册 1.2，最后一行是 ready）。
     * 顺序不能反：模组没使能的话永远等不到 ready。 */
    if (combo_power_up_and_wait() != RT_EOK)
    {
        LOG_E("combo module no response(%d ms), check uart wiring/baudrate(115200 8N1) "
              "and the enable pin.", COMBO_WAIT_CONNECT_TIME);
        return;
    }

    /* ATE0 + URC 掩码 + 打印版本 */
    combo_at_link_setup();

    /* ---- 可选的 WiFi 自动连接（默认关，见 COMBO_INIT_WIFI_AUTO）---- */
#if COMBO_INIT_WIFI_AUTO
    combo_wifi_auto_connect(device);
#else
    LOG_I("wifi auto-connect off (COMBO_INIT_WIFI_AUTO=0), "
          "call combo_wifi_start(ssid, pwd) when needed.");
#endif

    /* ---- 可选的 BLE 配置（默认关，见 COMBO_INIT_BLE_SETUP）---- */
#if COMBO_INIT_BLE_SETUP
    combo_ble_start(COMBO_SAMPLE_BLE_NAME, COMBO_BLE_MODE);
#else
    LOG_I("BLE setup off (COMBO_INIT_BLE_SETUP=0), "
          "call combo_ble_start(name, mode) when needed.");
#endif

    device->is_init = RT_TRUE;
    LOG_I("combo device(%s) init done.", device->name);
}

static int combo_net_init(struct at_device *device)
{
#ifdef AT_DEVICE_COMBO_INIT_ASYN
    rt_thread_t tid;

    tid = rt_thread_create("combo_net", combo_init_thread_entry, (void *)device,
                           COMBO_INIT_THREAD_STACK_SIZE, COMBO_INIT_THREAD_PRIORITY, 20);
    if (tid == RT_NULL)
    {
        LOG_E("create combo init thread failed.");
        return -RT_ERROR;
    }
    rt_thread_startup(tid);
#else
    combo_init_thread_entry(device);
#endif

    return RT_EOK;
}

/* ==========================================================================
 * URC（手册 1.5）
 * ========================================================================== */
static void urc_ready_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    LOG_I("module ready: %.*s", (int)size, data);

    /* 模组复位/上电：清状态，等 WIFI_GOT_IP 再拉起 netdev */
    combo_link_up = RT_FALSE;
    if (combo_device != RT_NULL && combo_device->netdev != RT_NULL)
    {
        netdev_low_level_set_link_status(combo_device->netdev, RT_FALSE);
    }
}

static void urc_wifi_got_ip_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    RT_UNUSED(data);
    RT_UNUSED(size);

    LOG_I("URC: WIFI_GOT_IP");

    /* !!! 这里绝对不能发 AT 指令 !!!
     * 本回调由 client_parser() 在 AT 客户端接收线程里原地调用，在里面
     * at_obj_exec_cmd() 会等一个只有该线程才能发出的响应通知 -> 必然超时，
     * 而且会把接收线程堵住 resp->timeout 那么久。
     * 所以只置事件，AT+STAINFO? 查询交给 combo_evt_thread_entry()。 */
    if (combo_evt_ready)
    {
        rt_event_send(&combo_evt, COMBO_EVT_WIFI_GOT_IP);
    }
}

static void urc_wifi_connect_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    RT_UNUSED(data);
    RT_UNUSED(size);
    LOG_I("URC: WIFI_CONNECT");
}

static void urc_wifi_disconnect_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    RT_UNUSED(data);
    RT_UNUSED(size);
    LOG_W("URC: WIFI_DISCONNECT");

    combo_link_up = RT_FALSE;
    if (combo_device != RT_NULL && combo_device->netdev != RT_NULL)
    {
        netdev_low_level_set_link_status(combo_device->netdev, RT_FALSE);
    }
}

static void urc_wifi_scan_done_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    LOG_I("URC: WIFI_SCAN_DONE %.*s", (int)size, data);
}

/** 一个函数同时处理 +EVENT:BLE_CONNECTED / +EVENT:BLE_DISCONNECT */
static void urc_ble_event_func(struct at_client *client, const char *data, rt_size_t size)
{
    rt_bool_t connected;

    RT_UNUSED(client);
    RT_UNUSED(size);

    connected = (rt_strstr(data, "DISCONNECT") == RT_NULL) ? RT_TRUE : RT_FALSE;
    LOG_I("URC: BLE_%s", connected ? "CONNECTED" : "DISCONNECT");

    if (combo_ble_event_cb != RT_NULL)
    {
        combo_ble_event_cb(connected ? 1 : 0, combo_ble_event_arg);
    }
}

/** 手册 5.2.1 URC: +DATA:<len>,<data> —— 主机模式收到蓝牙透传数据 */
static void urc_ble_data_func(struct at_client *client, const char *data, rt_size_t size)
{
    const char *p;
    int len;

    RT_UNUSED(client);

    p = data;
    if (rt_strncmp(p, "+DATA:", 6) != 0)
    {
        return;
    }
    p += 6;

    len = atoi(p);
    p = strchr(p, ',');
    if (p == RT_NULL || len <= 0)
    {
        return;
    }
    p++;

    {
        rt_size_t avail = (size > (rt_size_t)(p - data)) ? (size - (rt_size_t)(p - data)) : 0;
        if ((rt_size_t)len > avail)
        {
            len = (int)avail;
        }
    }

    if (combo_ble_recv_cb != RT_NULL && len > 0)
    {
        combo_ble_recv_cb(p, (rt_size_t)len, combo_ble_recv_arg);
    }
}

/** 手册 5.3.1：扫描输出 name:<name> / MAC:<mac> / rssi:<rssi> 三行一组 */
static void urc_ble_scan_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    RT_UNUSED(size);

    if (rt_strncmp(data, "name:", 5) == 0)
    {
        rt_strncpy(combo_scan_name, data + 5, sizeof(combo_scan_name) - 1);
        combo_scan_name[sizeof(combo_scan_name) - 1] = '\0';
    }
    else if (rt_strncmp(data, "rssi:", 5) == 0)
    {
        combo_scan_rssi = atoi(data + 5);
    }
    else if (rt_strncmp(data, "MAC:", 4) == 0)
    {
        if (combo_ble_scan_cb != RT_NULL)
        {
            combo_ble_scan_cb(data + 4, combo_scan_name, combo_scan_rssi, combo_ble_scan_arg);
        }
        combo_scan_name[0] = '\0';
        combo_scan_rssi = 0;
    }
}

static void urc_err_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    LOG_W("module error: %.*s", (int)size, data);
}

static const struct at_urc urc_table[] =
{
    /* 注意：SocketDissconnect 是固件里的拼写（三个 s），照抄。
     * socket 相关的 URC 在 at_socket_combo.c 里单独挂表。 */
    { "ready",                  "\r\n", urc_ready_func            },
    { "+EVENT:WIFI_GOT_IP",     "\r\n", urc_wifi_got_ip_func      },
    { "+EVENT:WIFI_CONNECT",    "\r\n", urc_wifi_connect_func     },
    { "+EVENT:WIFI_DISCONNECT", "\r\n", urc_wifi_disconnect_func  },
    { "+EVENT:WIFI_SCAN_DONE",  "\r\n", urc_wifi_scan_done_func   },
    { "+EVENT:BLE_CONNECTED",   "\r\n", urc_ble_event_func        },
    { "+EVENT:BLE_DISCONNECT",  "\r\n", urc_ble_event_func        },
    { "+DATA:",                 "\r\n", urc_ble_data_func         },
    { "Devices Found:",         "\r\n", urc_ble_scan_func         },
    { "name:",                  "\r\n", urc_ble_scan_func         },
    { "MAC:",                   "\r\n", urc_ble_scan_func         },
    { "rssi:",                  "\r\n", urc_ble_scan_func         },
    { "ERROR",                  "\r\n", urc_err_func              },
};

/* ==========================================================================
 * device ops
 * ========================================================================== */
static int combo_init(struct at_device *device)
{
    struct at_device_combo *combo = (struct at_device_combo *)device->user_data;

    RT_ASSERT(combo);

    /* AT 客户端：名字就是串口设备名 */
#if RT_VER_NUM >= 0x50100
    at_client_init(combo->client_name, combo->recv_line_num, combo->recv_line_num);
#else
    at_client_init(combo->client_name, combo->recv_line_num);
#endif

    device->client = at_client_get(combo->client_name);
    if (device->client == RT_NULL)
    {
        LOG_E("get AT client(%s) failed, check the uart device name.", combo->client_name);
        return -RT_ERROR;
    }

    combo_device = device;
    combo_client = device->client;
    combo_link_up = RT_FALSE;

    /* 先把"网络事件"通路建好，URC 随时可能来 */
    combo_net_evt_init();

    at_obj_set_urc_table(device->client, urc_table, sizeof(urc_table) / sizeof(urc_table[0]));

#ifdef AT_USING_SOCKET
    combo_socket_init(device);
#endif

    device->netdev = combo_netdev_add(combo->device_name);
    if (device->netdev == RT_NULL)
    {
        LOG_E("add netdev(%s) failed.", combo->device_name);
        return -RT_ERROR;
    }

    return combo_net_init(device);
}

static int combo_deinit(struct at_device *device)
{
    if (device->netdev != RT_NULL)
    {
        return combo_netdev_set_down(device->netdev);
    }
    return RT_EOK;
}

/** 手册 2.3 AT+RST：软重启，之后重新打印启动信息 */
static int combo_reset(struct at_device *device)
{
    struct at_client *client = device->client;

    if (combo_at_exec("AT+RST") != RT_EOK)
    {
        return -RT_ERROR;
    }
    rt_thread_mdelay(1000);

    device->is_init = RT_FALSE;
    if (at_client_obj_wait_connect(client, COMBO_WAIT_CONNECT_TIME) != RT_EOK)
    {
        return -RT_ETIMEOUT;
    }

    combo_net_init(device);
    device->is_init = RT_TRUE;

    return RT_EOK;
}

/** AT_DEVICE_CTRL_SET_WIFI_INFO：换 AP（手册 4.1.6） */
static int combo_wifi_info_set(struct at_device *device, struct at_device_ssid_pwd *info)
{
    struct combo_wifi_info wifi;

    if (info == RT_NULL || info->ssid == RT_NULL)
    {
        LOG_E("input wifi ssid error.");
        return -RT_EINVAL;
    }

    if (combo_wifi_join(info->ssid, (info->password != RT_NULL) ? info->password : "") != RT_EOK)
    {
        LOG_E("join \"%s\" failed.", info->ssid);
        return -RT_ERROR;
    }

    rt_thread_mdelay(3000);
    if (combo_read_stainfo(&wifi) == RT_EOK && device->netdev != RT_NULL)
    {
        combo_netdev_update(device->netdev, &wifi);
        combo_netdev_update_dns(device->netdev);
    }

    return RT_EOK;
}

static int combo_control(struct at_device *device, int cmd, void *arg)
{
    int result = -RT_ERROR;

    RT_ASSERT(device);

    switch (cmd)
    {
    case AT_DEVICE_CTRL_RESET:
        result = combo_reset(device);
        break;

    case AT_DEVICE_CTRL_SET_WIFI_INFO:
        result = combo_wifi_info_set(device, (struct at_device_ssid_pwd *)arg);
        break;

    case AT_DEVICE_CTRL_GET_SIGNAL:
        {
            int rssi = 0;
            result = combo_wifi_get_rssi(&rssi);
            if (result == RT_EOK && arg != RT_NULL)
            {
                *(rt_int32_t *)arg = rssi;      /* 单位 dBm */
            }
        }
        break;

    case AT_DEVICE_CTRL_GET_VER:
        {
            char ver[32] = {0};
            result = combo_get_version(ver, sizeof(ver));
            if (result == RT_EOK && arg != RT_NULL)
            {
                rt_strncpy((char *)arg, ver, 31);
            }
        }
        break;

    case AT_DEVICE_CTRL_NET_CONN:
        result = combo_wifi_set_mode(1, 0);
        break;

    case AT_DEVICE_CTRL_NET_DISCONN:
        result = combo_wifi_disconnect();
        break;

    case AT_DEVICE_CTRL_POWER_ON:
        result = combo_power_on();
        break;

    case AT_DEVICE_CTRL_POWER_OFF:
        result = combo_power_off();
        break;

    case AT_DEVICE_CTRL_LOW_POWER:
    case AT_DEVICE_CTRL_SLEEP:
        /* 手册 2.10：Ai-WB2 只有 mode 2(深度睡眠)/3(普通)，浅睡眠 0/1 是
         * PB/TB 系列才支持。两个命令都按 COMBO_SLEEP_* 走；要浅睡眠就改宏
         * 或直接调 combo_sleep(0/1, 2, 7, 0)。 */
        result = combo_sleep(COMBO_SLEEP_MODE, COMBO_SLEEP_WAKE_SOURCE,
                             COMBO_SLEEP_WAKE_PIN, COMBO_SLEEP_WAKE_LEVEL);
        break;

    case AT_DEVICE_CTRL_WAKEUP:
        result = combo_wakeup();
        break;

    case AT_DEVICE_CTRL_GET_GPS:
    case AT_DEVICE_CTRL_SET_HOST_NAME:
        LOG_W("not support the control cmd(%d).", cmd);
        break;

    default:
        LOG_E("input error control cmd(%d).", cmd);
        break;
    }

    return result;
}

static const struct at_device_ops combo_device_ops =
{
    .init    = combo_init,
    .deinit  = combo_deinit,
    .control = combo_control,
};

int combo_at_device_class_register(void)
{
    struct at_device_class *class = RT_NULL;

    class = (struct at_device_class *)rt_calloc(1, sizeof(struct at_device_class));
    if (class == RT_NULL)
    {
        LOG_E("no memory for combo device class create.");
        return -RT_ENOMEM;
    }

#ifdef AT_USING_SOCKET
    combo_socket_class_register(class);
#endif
    class->device_ops = &combo_device_ops;

    return at_device_class_register(class, AT_DEVICE_CLASS_COMBO);
}
INIT_DEVICE_EXPORT(combo_at_device_class_register);

/* ==========================================================================
 * 注册 API
 * ========================================================================== */
void combo_at_dev_init(struct at_device_combo *combo,
                       const char *device_name, const char *client_name,
                       const char *ssid, const char *password)
{
    if (combo == RT_NULL)
    {
        return;
    }

    rt_memset(combo, 0, sizeof(struct at_device_combo));

    combo->device_name      = (char *)((device_name != RT_NULL) ? device_name : COMBO_SAMPLE_DEVICE_NAME);
    combo->client_name      = (char *)((client_name != RT_NULL) ? client_name : COMBO_SAMPLE_CLIENT_NAME);
    combo->power_pin        = COMBO_SAMPLE_POWER_PIN;
    combo->power_status_pin = COMBO_SAMPLE_STATUS_PIN;
    combo->wakeup_pin       = COMBO_SAMPLE_WAKEUP_PIN;
    combo->recv_line_num    = COMBO_SAMPLE_RECV_BUFF_LEN;
    combo->wifi_ssid        = (char *)((ssid != RT_NULL) ? ssid : COMBO_SAMPLE_WIFI_SSID);
    combo->wifi_password    = (char *)((password != RT_NULL) ? password : COMBO_SAMPLE_WIFI_PASSWORD);
}

int combo_at_dev_register(struct at_device_combo *combo)
{
    if (combo == RT_NULL || combo->device_name == RT_NULL || combo->client_name == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (combo->recv_line_num == 0)
    {
        combo->recv_line_num = COMBO_SAMPLE_RECV_BUFF_LEN;
    }

    return at_device_register(&(combo->device),
                              combo->device_name,
                              combo->client_name,
                              AT_DEVICE_CLASS_COMBO,
                              (void *)combo);
}

/* ==========================================================================
 * WiFi API
 * ========================================================================== */
int combo_wifi_start(const char *ssid, const char *password)
{
    if (ssid == RT_NULL || ssid[0] == '\0')
    {
        return -RT_EINVAL;
    }

    /* 手册 4.1.1：先切到 STA 模式（默认不写 flash，见 COMBO_WIFI_MODE_SAVE） */
    if (combo_wifi_set_mode(1, COMBO_WIFI_MODE_SAVE) != RT_EOK)
    {
        LOG_W("set STA mode failed, try AT+WJAP anyway.");
    }

    /* 手册 4.1.6：AT+WJAP=<ssid>,<pwd>
     * 连上并拿到 IP 后模组会发 +EVENT:WIFI_GOT_IP，由监视线程拉起 netdev，
     * 所以这里不用轮询等 IP。 */
    return combo_wifi_join(ssid, password);
}

int combo_wifi_set_mode(int mode, int save_flash)
{
    if (mode < 0 || mode > 3)
    {
        return -RT_EINVAL;
    }
    /* 手册 4.1.1 注释：save_flash=1 时不支持 MODE=0 */
    if (save_flash && mode == 0)
    {
        return -RT_EINVAL;
    }

    return combo_at_exec("AT+WMODE=%d,%d", mode, save_flash ? 1 : 0);
}

int combo_wifi_join(const char *ssid, const char *password)
{
    if (ssid == RT_NULL || ssid[0] == '\0')
    {
        return -RT_EINVAL;
    }

    /* 手册 4.1.6：AT+WJAP=<ssid>,<pwd> —— 参数**不加引号** */
    return combo_at_exec("AT+WJAP=%s,%s", ssid, (password != RT_NULL) ? password : "");
}

int combo_wifi_disconnect(void)
{
    return combo_at_exec("AT+WDISCONNECT");
}

int combo_wifi_get_info(struct combo_wifi_info *info)
{
    return combo_read_stainfo(info);
}

int combo_wifi_get_rssi(int *rssi)
{
    at_response_t resp = RT_NULL;
    int value = 0;
    int result = -RT_ERROR;

    if (rssi == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(128, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /* 手册 4.1.18 响应 "+WRSSI :<rssi>"（冒号前有一个空格） */
    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+WRSSI") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+WRSSI", "%*[^:]:%d", &value) > 0)
    {
        *rssi  = value;
        result = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_wifi_set_auto_conn(int enable, const char *ssid, const char *password)
{
    if (ssid != RT_NULL && ssid[0] != '\0')
    {
        return combo_at_exec("AT+WAUTOCONN=%d,%s,%s", enable ? 1 : 0, ssid,
                             (password != RT_NULL) ? password : "");
    }

    /* 手册 4.1.9：不带 ssid 表示"把上次连接成功的 AP 设为上电自动连接" */
    return combo_at_exec("AT+WAUTOCONN=%d", enable ? 1 : 0);
}

int combo_wifi_scan(void (*cb)(const char *line, void *arg), void *arg)
{
    at_response_t resp = RT_NULL;
    const char *line;
    rt_size_t i;

    if (combo_at_get_client() == RT_NULL)
    {
        return -RT_ERROR;
    }

    resp = at_create_resp(1024, 0, 20 * RT_TICK_PER_SECOND);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+WSCAN?") != RT_EOK)
    {
        at_delete_resp(resp);
        return -RT_ERROR;
    }

    for (i = 1; i <= resp->line_counts; i++)
    {
        line = at_resp_get_line(resp, i);
        if (line != RT_NULL && line[0] != '\0' && cb != RT_NULL)
        {
            cb(line, arg);
        }
    }

    at_delete_resp(resp);
    return RT_EOK;
}

int combo_wifi_smartconfig(int status, const char *name)
{
    if (name != RT_NULL && name[0] != '\0')
    {
        return combo_at_exec("AT+WCONFIG=%d,%s", status, name);
    }

    return combo_at_exec("AT+WCONFIG=%d", status);
}

int combo_ping(const char *host, int count, int *avg_ms)
{
    at_response_t resp = RT_NULL;
    int value = 0;
    rt_err_t result;

    if (host == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(256, 0, 30 * RT_TICK_PER_SECOND);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /* 手册 4.1.13：成功回 +PING:<time>，失败回 +PING:TIMEOUT 且 ERROR */
    if (count > 0)
    {
        result = at_obj_exec_cmd(combo_at_get_client(), resp, "AT+PING=%s,%d", host, count);
    }
    else
    {
        result = at_obj_exec_cmd(combo_at_get_client(), resp, "AT+PING=%s", host);
    }

    if (result == RT_EOK && at_resp_parse_line_args_by_kw(resp, "+PING:", "+PING:%d", &value) > 0)
    {
        if (avg_ms != RT_NULL)
        {
            *avg_ms = value;
        }
        result = RT_EOK;
    }
    else
    {
        result = -RT_ERROR;
    }

    at_delete_resp(resp);
    return (int)result;
}

/** 手册 4.2.14 AT+WDOMAIN=<server name> -> +WDOMAIN:<IP> */
int combo_domain_resolve(const char *name, char ip[COMBO_IP_LEN])
{
    at_response_t resp = RT_NULL;
    char recv_ip[COMBO_IP_LEN] = {0};
    int a, b, c, d;
    int result = -RT_ERROR;

    if (name == RT_NULL || ip == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* 已经是点分 IP 就不用查了 */
    if (rt_sscanf(name, "%d.%d.%d.%d", &a, &b, &c, &d) == 4)
    {
        rt_strncpy(ip, name, COMBO_IP_LEN - 1);
        ip[COMBO_IP_LEN - 1] = '\0';
        return RT_EOK;
    }

    resp = at_create_resp(256, 0, 20 * RT_TICK_PER_SECOND);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+WDOMAIN=%s", name) == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+WDOMAIN:", "+WDOMAIN:%15[^\r]", recv_ip) > 0)
    {
        rt_strncpy(ip, recv_ip, COMBO_IP_LEN - 1);
        ip[COMBO_IP_LEN - 1] = '\0';
        result = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

/* ==========================================================================
 * BLE
 * --------------------------------------------------------------------------
 * 手册 5.x 多处标注"仅允许在蓝牙关闭状态下设置"（名字/UUID/配对码/MTU/
 * 发射功率/广播数据/连接间隔）。所以统一用下面这两个函数包一下：
 * 记下当前模式 -> AT+BLEMODE=9 -> 设置 -> 还原模式。
 * ========================================================================== */
static int combo_ble_enter_setting(int *saved_mode)
{
    if (saved_mode == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *saved_mode = 0;
    if (combo_ble_get_mode(saved_mode) != RT_EOK)
    {
        *saved_mode = 0;
    }

    if (*saved_mode != 9)
    {
        if (combo_at_exec("AT+BLEMODE=9") != RT_EOK)
        {
            return -RT_ERROR;
        }
        rt_thread_mdelay(200);      /* 等蓝牙真正关掉 */
    }

    return RT_EOK;
}

static void combo_ble_exit_setting(int saved_mode)
{
    if (saved_mode >= 0 && saved_mode <= 2)
    {
        combo_at_exec("AT+BLEMODE=%d", saved_mode);
    }
}

int combo_ble_start(const char *name, int mode)
{
    int result = RT_EOK;

    /* 手册 5.1.12：BLE 连接后自动进透传会让 AT 指令失效 -> 关掉
     * （默认不写 flash，见 COMBO_BLE_CFG_SAVE） */
    if (combo_ble_trans_enter_config(0, COMBO_BLE_CFG_SAVE) != RT_EOK)
    {
        LOG_D("set BLE auto-transparent off failed (old firmware?).");
    }

    /* 手册 5.1.1：改名字要求蓝牙关闭，combo_ble_set_name() 内部会自动处理 */
    if (name != RT_NULL && name[0] != '\0' && combo_ble_set_name(name) != RT_EOK)
    {
        LOG_W("set BLE name \"%s\" failed.", name);
        result = -RT_ERROR;
    }

    /* 手册 5.1.2：模式 0 从机 1 主机 2 iBeacon 9 关闭；mode<0 表示不动 */
    if (mode >= 0 && combo_ble_set_mode(mode) != RT_EOK)
    {
        LOG_W("set BLE mode %d failed.", mode);
        result = -RT_ERROR;
    }

    return result;
}

int combo_ble_set_mode(int mode)
{
    if (mode != 0 && mode != 1 && mode != 2 && mode != 9)
    {
        return -RT_EINVAL;
    }

    return combo_at_exec("AT+BLEMODE=%d", mode);
}

int combo_ble_get_mode(int *mode)
{
    at_response_t resp = RT_NULL;
    int value = 0;
    int result = -RT_ERROR;

    if (mode == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(128, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /* 手册 5.1.2 响应 "+BLEMODE :<mode>"（冒号前有空格） */
    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLEMODE?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+BLEMODE", "%*[^:]:%d", &value) > 0)
    {
        *mode  = value;
        result = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_ble_set_name(const char *name)
{
    int saved, result;

    if (name == RT_NULL || name[0] == '\0')
    {
        return -RT_EINVAL;
    }

    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLENAME=%s", name);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_get_name(char *name, rt_size_t len)
{
    at_response_t resp = RT_NULL;
    char value[64] = {0};
    int result = -RT_ERROR;

    if (name == RT_NULL || len == 0 || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(256, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLENAME?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+BLENAME", "%*[^:]:%63[^\r]", value) > 0)
    {
        rt_strncpy(name, value, len - 1);
        name[len - 1] = '\0';
        result = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_ble_get_mac(char *mac, rt_size_t len)
{
    at_response_t resp = RT_NULL;
    char value[32] = {0};
    int result = -RT_ERROR;

    if (mac == RT_NULL || len == 0 || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(128, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLEMAC?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+BLEMAC:", "+BLEMAC:%31[^\r]", value) > 0)
    {
        rt_strncpy(mac, value, len - 1);
        mac[len - 1] = '\0';
        result = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_ble_set_mac(const char *mac)
{
    int saved, result;

    if (mac == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLEMAC=%s", mac);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_set_adv_data(const char *hex_data)
{
    int saved, result;

    if (hex_data == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLEADVDATA=%s", hex_data);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_adv_enable(int enable)
{
    /* 手册 5.2.7：仅从机模式下可设 */
    return combo_at_exec("AT+BLEADVEN=%d", enable ? 1 : 0);
}

int combo_ble_get_adv_enable(int *enabled)
{
    at_response_t resp = RT_NULL;
    int value = 0;
    int result = -RT_ERROR;

    if (enabled == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(128, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLEADVEN?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+BLEADVEN:", "+BLEADVEN:%d", &value) > 0)
    {
        *enabled = value ? 1 : 0;
        result   = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_ble_set_adv_interval(int intv)
{
    int saved, result;

    /* 手册 5.2.5：取值 160~16384，实际间隔 = intv * 0.625ms */
    if (intv < 160 || intv > 16384)
    {
        return -RT_EINVAL;
    }
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLEADVINTV=%d", intv);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_set_conn_interval(int min_interval, int max_interval, int latency, int timeout)
{
    int saved, result;

    /* 手册 5.2.3 的参数范围 */
    if (min_interval < 6 || min_interval > 3200 ||
        max_interval < 6 || max_interval > 3200 ||
        latency < 0 || latency > 499 ||
        timeout < 10 || timeout > 3200)
    {
        LOG_E("invalid conn interval: min=%d max=%d latency=%d timeout=%d",
              min_interval, max_interval, latency, timeout);
        return -RT_EINVAL;
    }
    if (min_interval > max_interval)
    {
        LOG_E("min_interval(%d) must not exceed max_interval(%d).", min_interval, max_interval);
        return -RT_EINVAL;
    }

    /* 手册的规范约束： Timeout*10 > (1+Latency)*max_interval*1.25   （单位都是 ms）
     * 两边同乘 4 换算成整数，避免用浮点：
     *     timeout*40 > (1+latency)*max_interval*5
     * 不满足的话模组会回参数错误，这里先拦下来，免得白跑一趟 AT。 */
    if ((1 + latency) * max_interval * 5 >= timeout * 40)
    {
        long need_ms = ((long)(1 + latency) * max_interval * 125) / 100;

        LOG_E("timeout(%dms) too small: must be > (1+latency)*max_interval*1.25 = %ldms.",
              timeout * 10, need_ms);
        return -RT_EINVAL;
    }

    /* 手册 5.2.3：仅允许在蓝牙关闭状态下设置 */
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLECONINTV=%d,%d,%d,%d",
                           min_interval, max_interval, latency, timeout);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_get_conn_interval(int *min_interval, int *max_interval, int *latency, int *timeout)
{
    at_response_t resp = RT_NULL;
    int mn = 0, mx = 0, lat = 0, tmo = 0;
    int result = -RT_ERROR;

    if (combo_at_get_client() == RT_NULL)
    {
        return -RT_ERROR;
    }

    resp = at_create_resp(128, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /* 手册 5.2.3 响应 "+BLECONINTV:<min>,< max>,<latency>,< timeout>"（逗号后有空格，%d 会自己跳过） */
    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLECONINTV?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "+BLECONINTV",
                                      "+BLECONINTV:%d,%d,%d,%d",
                                      &mn, &mx, &lat, &tmo) > 0)
    {
        if (min_interval != RT_NULL) *min_interval = mn;
        if (max_interval != RT_NULL) *max_interval = mx;
        if (latency != RT_NULL) *latency = lat;
        if (timeout != RT_NULL) *timeout = tmo;
        result = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_ble_set_pin(const char *pin)
{
    int saved, result;

    if (pin == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLEAUTH=%s", pin);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_set_mtu(int mtu)
{
    int saved, result;

    /* 手册 5.1.6：23~250 */
    if (mtu < 23 || mtu > 250)
    {
        return -RT_EINVAL;
    }
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLEMTU=%d", mtu);
    combo_ble_exit_setting(saved);

    return result;
}

int combo_ble_get_state(int *connected)
{
    at_response_t resp = RT_NULL;
    int value = 0;
    int result = -RT_ERROR;

    if (connected == RT_NULL || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(128, 0, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /* 手册 5.1.4 响应 "+ BLESTATE:<status>"（+ 后面有个空格） */
    if (at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLESTATE?") == RT_EOK &&
        at_resp_parse_line_args_by_kw(resp, "BLESTATE", "%*[^:]:%d", &value) > 0)
    {
        *connected = value ? 1 : 0;
        result     = RT_EOK;
    }

    at_delete_resp(resp);
    return result;
}

int combo_ble_disconnect(void)
{
    return combo_at_exec("AT+BLEDISCON");
}

int combo_ble_send(const void *data, rt_size_t len)
{
    if (data == RT_NULL || len == 0 || len > COMBO_CMD_MAX_LEN / 2)
    {
        return -RT_EINVAL;
    }

    /* 手册 5.1.7：AT+BLESEND=<len>,<data>（单行，指令总长限制 1023 字节）。
     * 数据里含逗号/不可见字符请改用 combo_ble_send_raw() */
    return combo_at_exec("AT+BLESEND=%d,%.*s", (int)len, (int)len, (const char *)data);
}

int combo_ble_send_raw(const void *data, rt_size_t len)
{
    at_response_t resp = RT_NULL;
    rt_mutex_t lock = combo_get_lock();
    rt_err_t result;

    if (data == RT_NULL || len == 0 || combo_at_get_client() == RT_NULL)
    {
        return -RT_EINVAL;
    }

    resp = at_create_resp(128, 1, COMBO_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (lock != RT_NULL)
    {
        rt_mutex_take(lock, RT_WAITING_FOREVER);
    }

    /* 手册 5.1.8：AT+BLESENDRAW=<len> -> 等 '>' -> 直接发原始数据（二进制安全） */
    at_obj_set_end_sign(combo_at_get_client(), '>');
    result = at_obj_exec_cmd(combo_at_get_client(), resp, "AT+BLESENDRAW=%d", (int)len);
    if (result == RT_EOK)
    {
        result = at_client_obj_send(combo_at_get_client(), (const char *)data, len);
    }
    at_obj_set_end_sign(combo_at_get_client(), 0);

    if (lock != RT_NULL)
    {
        rt_mutex_release(lock);
    }

    at_delete_resp(resp);

    return (result == RT_EOK) ? RT_EOK : -RT_ERROR;
}

int combo_ble_trans_enter_config(int auto_entry, int save_flash)
{
    /* 手册 5.1.12：连接后是否自动进透传。AT 控制场景建议 0 */
    return combo_at_exec("AT+TRANSENTER=%d,%d", auto_entry ? 1 : 0, save_flash ? 1 : 0);
}

int combo_ble_scan(void)
{
    return combo_at_exec("AT+BLESCAN");
}

int combo_ble_connect(const char *mac)
{
    if (mac == RT_NULL)
    {
        return -RT_EINVAL;
    }

    return combo_at_exec("AT+BLECONNECT=%s", mac);
}

int combo_ble_auto_connect(const char *mac, const char *uuid_suffix, int save_flash)
{
    /* 手册 5.3.3：MAC/UUID 任一即可，不需要的填 FALSE */
    return combo_at_exec("AT+BLEAUTOCON=%s,%s,%d",
                         (mac != RT_NULL) ? mac : "FALSE",
                         (uuid_suffix != RT_NULL) ? uuid_suffix : "FALSE",
                         save_flash ? 1 : 0);
}

int combo_ble_disable_auto_connect(void)
{
    return combo_at_exec("AT+BLEDISAUTOCON");
}

int combo_ble_set_rf_power(const char *power)
{
    int saved, result;

    if (power == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (combo_ble_enter_setting(&saved) != RT_EOK)
    {
        return -RT_ERROR;
    }
    result = combo_at_exec("AT+BLERFPWR=%s", power);
    combo_ble_exit_setting(saved);

    return result;
}

void combo_ble_set_recv_cb(void (*cb)(const char *data, rt_size_t len, void *arg), void *arg)
{
    combo_ble_recv_cb  = cb;
    combo_ble_recv_arg = arg;
}

void combo_ble_set_event_cb(void (*cb)(int connected, void *arg), void *arg)
{
    combo_ble_event_cb  = cb;
    combo_ble_event_arg = arg;
}

void combo_ble_set_scan_cb(void (*cb)(const char *mac, const char *name, int rssi, void *arg), void *arg)
{
    combo_ble_scan_cb  = cb;
    combo_ble_scan_arg = arg;
}

/* ==========================================================================
 * MSH 调试命令
 * ========================================================================== */
#ifdef FINSH_USING_MSH

static void combo_scan_print(const char *line, void *arg)
{
    RT_UNUSED(arg);
    rt_kprintf("  %s\n", line);
}

static void combo_info(int argc, char **argv)
{
    struct combo_wifi_info info;
    char ble_mac[32] = {0};
    int  rssi = 0, ble_state = 0, ble_mode = 0;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    if (combo_at_get_client() == RT_NULL)
    {
        rt_kprintf("combo: not registered (check AT_DEVICE_USING_COMBO)\n");
        return;
    }

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

    if (combo_wifi_get_rssi(&rssi) == RT_EOK)
    {
        rt_kprintf("rssi : %d dBm\n", rssi);
    }

    if (combo_ble_get_mode(&ble_mode) == RT_EOK &&
        combo_ble_get_state(&ble_state) == RT_EOK &&
        combo_ble_get_mac(ble_mac, sizeof(ble_mac)) == RT_EOK)
    {
        rt_kprintf("ble  : mode=%d connected=%d mac=%s\n", ble_mode, ble_state, ble_mac);
    }
}
MSH_CMD_EXPORT_ALIAS(combo_info, combo_info, combo module: wifi + ble status);

static void combo_join(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: combo_join <ssid> [password]\n");
        return;
    }

    if (combo_wifi_join(argv[1], (argc >= 3) ? argv[2] : "") == RT_EOK)
    {
        rt_kprintf("AT+WJAP accepted, wait for +EVENT:WIFI_GOT_IP\n");
    }
    else
    {
        rt_kprintf("AT+WJAP failed, see errno in log (66 wifi connect fail / 4 param err)\n");
    }
}
MSH_CMD_EXPORT_ALIAS(combo_join, combo_join, combo module: join AP);

static void combo_scan(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("wifi scan:\n");
    combo_wifi_scan(combo_scan_print, RT_NULL);
}
MSH_CMD_EXPORT_ALIAS(combo_scan, combo_scan, combo module: scan AP list);

static void combo_power_msh(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: combo_power on|off   (needs COMBO_SAMPLE_POWER_PIN >= 0)\n");
        return;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        rt_kprintf("power on : %s\n", (combo_power_on() == RT_EOK) ? "OK" : "FAIL");
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        rt_kprintf("power off: %s\n", (combo_power_off() == RT_EOK) ? "OK" : "FAIL");
    }
    else
    {
        rt_kprintf("unknown subcmd \"%s\"\n", argv[1]);
    }
}
MSH_CMD_EXPORT_ALIAS(combo_power_msh, combo_power, combo module: enable pin on/off);

static void combo_sleep_msh(int argc, char **argv)
{
    int mode = COMBO_SLEEP_MODE;
    int src  = COMBO_SLEEP_WAKE_SOURCE;
    int p1   = COMBO_SLEEP_WAKE_PIN;
    int p2   = COMBO_SLEEP_WAKE_LEVEL;

    if (argc >= 2) mode = atoi(argv[1]);
    if (argc >= 3) src  = atoi(argv[2]);
    if (argc >= 4) p1   = atoi(argv[3]);
    if (argc >= 5) p2   = atoi(argv[4]);

    rt_kprintf("AT+SLEEP=%d,%d,%d,%d -> %s\n", mode, src, p1, p2,
               (combo_sleep(mode, src, p1, p2) == RT_EOK) ? "OK" : "ERROR");
    rt_kprintf("note: Ai-WB2 only supports mode 2/3; wake by sending anything on uart,\n"
               "      because IO7(=RX) low level is the only GPIO wake source.\n");
}
MSH_CMD_EXPORT_ALIAS(combo_sleep_msh, combo_sleep, combo module: sleep (mode,src,p1,p2));

static void combo_wakeup_msh(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("wakeup: %s\n", (combo_wakeup() == RT_EOK) ? "OK" : "FAIL");
}
MSH_CMD_EXPORT_ALIAS(combo_wakeup_msh, combo_wakeup, combo module: wake up from sleep);

static void combo_ping_msh(int argc, char **argv)
{
    int avg = -1;

    if (argc < 2)
    {
        rt_kprintf("usage: combo_ping <host> [count]\n");
        return;
    }

    if (combo_ping(argv[1], (argc >= 3) ? atoi(argv[2]) : 0, &avg) == RT_EOK)
    {
        rt_kprintf("ping %s: avg %d ms\n", argv[1], avg);
    }
    else
    {
        rt_kprintf("ping %s failed (timeout?)\n", argv[1]);
    }
}
MSH_CMD_EXPORT_ALIAS(combo_ping_msh, combo_ping, combo module: ping host);

static void combo_ble_msh(int argc, char **argv)
{
    char buf[64] = {0};
    int  value = 0;

    if (argc < 2)
    {
        rt_kprintf("usage:\n"
                   "  combo_ble mode [0|1|2|9]   work mode (0 slave,1 master,2 ibeacon,9 off)\n"
                   "  combo_ble name [str]       device name (no arg = query)\n"
                   "  combo_ble mac              ble mac\n"
                   "  combo_ble adv [0|1]        adv enable (no arg = query)\n"
                   "  combo_ble connintv [min max latency timeout]  conn interval (×1.25ms / ×10ms)\n"
                   "  combo_ble state            connect state\n"
                   "  combo_ble discon           disconnect\n"
                   "  combo_ble scan             scan (master mode)\n"
                   "  combo_ble send <str>       send text over ble transparent channel\n");
        return;
    }

    if (rt_strcmp(argv[1], "mode") == 0)
    {
        if (argc >= 3)
        {
            rt_kprintf("set ble mode %s: %s\n", argv[2],
                       (combo_ble_set_mode(atoi(argv[2])) == RT_EOK) ? "OK" : "FAIL");
        }
        else if (combo_ble_get_mode(&value) == RT_EOK)
        {
            rt_kprintf("ble mode = %d\n", value);
        }
    }
    else if (rt_strcmp(argv[1], "name") == 0)
    {
        if (argc >= 3)
        {
            rt_kprintf("set ble name \"%s\": %s\n", argv[2],
                       (combo_ble_set_name(argv[2]) == RT_EOK) ? "OK" : "FAIL");
        }
        else if (combo_ble_get_name(buf, sizeof(buf)) == RT_EOK)
        {
            rt_kprintf("ble name = %s\n", buf);
        }
    }
    else if (rt_strcmp(argv[1], "mac") == 0)
    {
        if (combo_ble_get_mac(buf, sizeof(buf)) == RT_EOK)
        {
            rt_kprintf("ble mac = %s\n", buf);
        }
    }
    else if (rt_strcmp(argv[1], "adv") == 0)
    {
        if (argc >= 3)
        {
            rt_kprintf("set adv %s: %s\n", argv[2],
                       (combo_ble_adv_enable(atoi(argv[2])) == RT_EOK) ? "OK" : "FAIL");
        }
        else if (combo_ble_get_adv_enable(&value) == RT_EOK)
        {
            rt_kprintf("adv enable = %d\n", value);
        }
    }
    else if (rt_strcmp(argv[1], "connintv") == 0)
    {
        if (argc >= 6)
        {
            rt_kprintf("set conn interval(min,max,latency,timeout)=%s,%s,%s,%s: %s\n",
                       argv[2], argv[3], argv[4], argv[5],
                       (combo_ble_set_conn_interval(atoi(argv[2]), atoi(argv[3]),
                                                    atoi(argv[4]), atoi(argv[5])) == RT_EOK)
                       ? "OK" : "FAIL");
        }
        else
        {
            int mn = 0, mx = 0, lat = 0, tmo = 0;

            if (combo_ble_get_conn_interval(&mn, &mx, &lat, &tmo) == RT_EOK)
            {
                rt_kprintf("conn interval: min=%d max=%d latency=%d timeout=%d\n",
                           mn, mx, lat, tmo);
            }
            else
            {
                rt_kprintf("query conn interval failed\n");
            }
        }
    }
    else if (rt_strcmp(argv[1], "state") == 0)
    {
        if (combo_ble_get_state(&value) == RT_EOK)
        {
            rt_kprintf("ble connected = %d\n", value);
        }
    }
    else if (rt_strcmp(argv[1], "discon") == 0)
    {
        rt_kprintf("disconnect: %s\n", (combo_ble_disconnect() == RT_EOK) ? "OK" : "FAIL");
    }
    else if (rt_strcmp(argv[1], "scan") == 0)
    {
        rt_kprintf("scan: %s (watch the MAC: lines)\n", (combo_ble_scan() == RT_EOK) ? "OK" : "FAIL");
    }
    else if (rt_strcmp(argv[1], "send") == 0 && argc >= 3)
    {
        rt_kprintf("send: %s\n", (combo_ble_send(argv[2], rt_strlen(argv[2])) == RT_EOK) ? "OK" : "FAIL");
    }
    else
    {
        rt_kprintf("unknown subcmd \"%s\"\n", argv[1]);
    }
}
MSH_CMD_EXPORT_ALIAS(combo_ble_msh, combo_ble, combo module: ble control);

static void combo_at_msh(int argc, char **argv)
{
    char cmd[COMBO_CMD_MAX_LEN] = {0};
    int i;

    if (argc < 2)
    {
        rt_kprintf("usage: combo_at <AT cmd...>   eg: combo_at AT+GMR\n");
        return;
    }

    for (i = 1; i < argc; i++)
    {
        rt_size_t used = rt_strlen(cmd);

        if (used >= sizeof(cmd) - 1)
        {
            break;
        }
        rt_snprintf(cmd + used, sizeof(cmd) - used, "%s%s",
                    argv[i], (i != argc - 1) ? " " : "");
    }

    rt_kprintf("%s -> %s\n", cmd, (combo_at_exec("%s", cmd) == RT_EOK) ? "OK" : "ERROR");
}
MSH_CMD_EXPORT_ALIAS(combo_at_msh, combo_at, combo module: send raw AT command);

#endif /* FINSH_USING_MSH */
#endif /* AT_DEVICE_USING_COMBO */
