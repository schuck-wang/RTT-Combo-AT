/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     port         first version: 安信可 combo 模组 socket 协议实现
 *
 * 安信可 "Combo 模组通用指令 V4.18P_3.8.0" 第 4.2 节（SOCKET 指令）实现。
 * 手册：applications/combo模组通用指令_v4.18p_3.8.0.pdf
 *
 * ============================== 设计要点 ==============================
 * 1. ConID 由我们指定
 *    手册 4.2.3：AT+SOCKET=<type>[,<remote host>],<port>[,<keep alive>,<conID>]
 *    最后一个参数可以指定新连接的 ConID。这里固定用
 *        ConID = 框架槽位(socket->user_data) + 1
 *    这样 ConID 与框架的 socket 一一对应，URC 里带回的 ConID 直接能查到 socket，
 *    不需要额外的映射表（+1 是为了避开 0 —— 槽位 0 的 user_data 是 NULL）。
 *
 * 2. 收数据用"主动模式"
 *    手册 4.2.10 AT+SOCKETRECVCFG=<mode>：
 *      0（默认/被动）：URC 只报长度 +EVENT:SocketDown,<ConID>,<length>，
 *                     数据缓存在模组 RAM 链表里，要 AT+SOCKETREAD 逐包取；
 *                     不读就会一直占内存，内存不足时新数据被丢弃。
 *      1（主动）    ：URC 直接带数据 +EVENT:SocketDown,<ConID>,<length>,<data>
 *    本驱动在 combo_socket_init() 里切到 1，配合框架的事件回调
 *    （at_set_event_cb 注册进来的 at_recv_notice_cb）把数据推给上层，
 *    不需要额外轮询线程。
 *    注意：模组把数据当字符串搭在 URC 行里，所以**二进制数据里若含 \r\n
 *    会被行解析截断**。要传二进制请自备转义，或改用 AT+SOCKETTT 透传模式
 *    （combo_at_exec("AT+SOCKETTT")，退出透传发 "+++"）。
 *
 * 3. 发送用长度模式（二进制安全）
 *    手册 4.2.5 AT+SOCKETSEND=<ConID>,<length> -> 回 '>' -> 再发 length 字节原始数据。
 *    单片上限按手册取 1024（超过模组自己分包），这里也按 1024 分片循环发。
 *
 * 4. URC 表是**追加**注册的
 *    at_obj_set_urc_table() 内部维护 client->urc_table[] 数组，多次调用是追加，
 *    所以 combo_at_dev.c 的设备 URC 和这里的 socket URC 互不覆盖。
 * ========================================================================== */

#include <string.h>
#include <stdio.h>

#include "combo_at_dev.h"

#if defined(AT_DEVICE_USING_COMBO) && defined(AT_USING_SOCKET)

#define LOG_TAG                        "combo.skt"
#include <at_log.h>

#include <at.h>

/* 未实现 server/监听模式：手册里 TCPServer/UDPServer 还需要处理
 * +EVENT:SocketSeed 产生的新 ConID，本文件没有做。开着会让 at_listen 空指针。 */
#ifdef AT_USING_SOCKET_SERVER
#error combo socket: 未实现 AT_USING_SOCKET_SERVER(监听/TCPSeed)，请关闭它，或自行补 combo_socket_listen
#endif

/* 手册 4.2.3 / 4.2.4 的 socket type 编号 */
#define COMBO_SOCKET_TYPE_UDP_SERVER   (1)
#define COMBO_SOCKET_TYPE_UDP_CLIENT   (2)
#define COMBO_SOCKET_TYPE_TCP_SERVER   (3)
#define COMBO_SOCKET_TYPE_TCP_CLIENT   (4)

/* 手册 4.2.5：单包上限，超过模组会自行分包 */
#define COMBO_SOCKET_SEND_MAX_SIZE     (1024)

#define COMBO_SOCKET_RESP_TIMEOUT      (10 * RT_TICK_PER_SECOND)

/* ConID = 框架槽位 + 1 */
#define COMBO_SOCKET_CONID(sock)       ((int)((sock)->user_data) + 1)

/* 框架通过 at_set_event_cb 把它的回调塞进来（at_recv_notice_cb / at_closed_notice_cb） */
static at_evt_cb_t at_evt_cb_set[] =
{
    [AT_SOCKET_EVT_RECV]   = RT_NULL,
    [AT_SOCKET_EVT_CLOSED] = RT_NULL,
};

/* ==========================================================================
 * 工具
 * ========================================================================== */

/** 由 AT 客户端找到 combo 设备对象 */
static struct at_device *combo_socket_get_device(struct at_client *client)
{
    char *client_name;

    if (client == RT_NULL || client->device == RT_NULL)
    {
        return RT_NULL;
    }

    client_name = client->device->parent.name;
    return at_device_get_by_name(AT_DEVICE_NAMETYPE_CLIENT, client_name);
}

/** 按 ConID 找到框架的 socket 对象
 *  magic 是框架在分配/释放时维护的，用它判断槽位是否在用，
 *  这样槽位 0（user_data == NULL）也能正确命中 */
static struct at_socket *combo_socket_get_by_conid(struct at_client *client, int conid)
{
    struct at_device *device;
    struct at_socket *sock;
    int i;

    if (conid <= 0)
    {
        return RT_NULL;
    }

    device = combo_socket_get_device(client);
    if (device == RT_NULL || device->class == RT_NULL || device->sockets == RT_NULL)
    {
        return RT_NULL;
    }

    for (i = 0; i < device->class->socket_num; i++)
    {
        sock = &device->sockets[i];
        if (sock->magic == AT_SOCKET_MAGIC && COMBO_SOCKET_CONID(sock) == conid)
        {
            return sock;
        }
    }

    return RT_NULL;
}

/* ==========================================================================
 * URC（手册 1.5 的 SOCKET 事件）
 * ========================================================================== */

/** +EVENT:SocketDown,<ConID>,<length>[,<data>] —— 收到 socket 数据 */
static void urc_socket_down_func(struct at_client *client, const char *data, rt_size_t size)
{
    const char *p;
    struct at_socket *sock;
    int conid, len;

    /* 跳过 "+EVENT:SocketDown" 到第一个逗号 */
    p = strchr(data, ',');
    if (p == RT_NULL)
    {
        return;
    }
    p++;
    conid = atoi(p);

    p = strchr(p, ',');
    if (p == RT_NULL)
    {
        return;
    }
    p++;
    len = atoi(p);

    sock = combo_socket_get_by_conid(client, conid);
    if (sock == RT_NULL)
    {
        LOG_D("SocketDown for unknown ConID=%d", conid);
        return;
    }

    /* 数据段：主动模式才有第二个逗号 */
    p = strchr(p, ',');
    if (p == RT_NULL || len <= 0)
    {
        LOG_W("SocketDown conid=%d len=%d without payload -- AT+SOCKETRECVCFG=1 not effective?",
              conid, len);
        return;
    }
    p++;

    {
        rt_size_t off   = (rt_size_t)(p - data);
        rt_size_t avail = (size > off) ? (size - off) : 0;

        if ((rt_size_t)len > avail)
        {
            LOG_W("SocketDown conid=%d len=%d but only %d bytes in URC line.", conid, len, (int)avail);
            len = (int)avail;
        }
    }

    if (len > 0 && at_evt_cb_set[AT_SOCKET_EVT_RECV] != RT_NULL)
    {
        at_evt_cb_set[AT_SOCKET_EVT_RECV](sock, AT_SOCKET_EVT_RECV, p, (size_t)len);
    }
}

/** +EVENT:SocketDissconnect,<ConID> —— 连接断开（固件拼写如此） */
static void urc_socket_close_func(struct at_client *client, const char *data, rt_size_t size)
{
    const char *p;
    struct at_socket *sock;

    RT_UNUSED(size);

    p = strchr(data, ',');
    if (p == RT_NULL)
    {
        return;
    }

    sock = combo_socket_get_by_conid(client, atoi(p + 1));
    if (sock == RT_NULL)
    {
        return;
    }

    LOG_W("socket(ConID=%d) disconnected by peer.", COMBO_SOCKET_CONID(sock));
    sock->state = AT_SOCKET_CLOSED;

    if (at_evt_cb_set[AT_SOCKET_EVT_CLOSED] != RT_NULL)
    {
        at_evt_cb_set[AT_SOCKET_EVT_CLOSED](sock, AT_SOCKET_EVT_CLOSED, RT_NULL, 0);
    }
}

/** +EVENT:SocketReconnect,<ConID> / +EVENT:SocketAutoDel,<ConID>：只记录 */
static void urc_socket_notice_func(struct at_client *client, const char *data, rt_size_t size)
{
    RT_UNUSED(client);
    LOG_I("socket urc: %.*s", (int)size, data);
}

static const struct at_urc urc_table[] =
{
    { "+EVENT:SocketDown",        "\r\n", urc_socket_down_func   },
    { "+EVENT:SocketDissconnect", "\r\n", urc_socket_close_func  },
    { "+EVENT:SocketReconnect",   "\r\n", urc_socket_notice_func },
    { "+EVENT:SocketAutoDel",     "\r\n", urc_socket_notice_func },
};

/* ==========================================================================
 * at_socket_ops
 * ========================================================================== */

/** 手册 4.2.9 AT+SOCKETDEL=<ConID> */
static int combo_socket_close(struct at_socket *socket)
{
    at_response_t resp = RT_NULL;
    struct at_device *device = (struct at_device *)socket->device;
    int result = -RT_ERROR;

    resp = at_create_resp(128, 0, COMBO_SOCKET_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(device->client, resp, "AT+SOCKETDEL=%d", COMBO_SOCKET_CONID(socket)) == RT_EOK)
    {
        result = RT_EOK;
    }
    else
    {
        LOG_W("delete ConID=%d failed.", COMBO_SOCKET_CONID(socket));
    }

    at_delete_resp(resp);
    return (result == RT_EOK) ? 0 : result;
}

/** 手册 4.2.3 AT+SOCKET=<type>,<remote host>,<port>,<keep alive>,<conID>
 *  响应：connect success ConID=<ConID> + OK */
static int combo_socket_connect(struct at_socket *socket, char *ip, int32_t port,
                                enum at_socket_type type, rt_bool_t is_client)
{
    at_response_t resp = RT_NULL;
    struct at_device *device = (struct at_device *)socket->device;
    const char *line;
    int type_id;
    int conid = COMBO_SOCKET_CONID(socket);
    int result = -RT_ERROR;

    RT_ASSERT(ip);
    RT_ASSERT(port >= 0);

    if (!is_client)
    {
        /* 监听模式未实现，见文件头 */
        LOG_E("combo socket: server(listen) mode not supported.");
        return -RT_ERROR;
    }

    switch (type)
    {
    case AT_SOCKET_TCP:
        type_id = COMBO_SOCKET_TYPE_TCP_CLIENT;
        break;

    case AT_SOCKET_UDP:
        type_id = COMBO_SOCKET_TYPE_UDP_CLIENT;
        break;

    default:
        LOG_E("not supported connect type %d.", type);
        return -RT_ERROR;
    }

    resp = at_create_resp(256, 0, COMBO_SOCKET_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /* ip 是域名也支持（手册：remote host 可以是域名或 IP）；
     * keep alive 传 0（预留未实现），conID 用我们算好的值 */
    if (at_obj_exec_cmd(device->client, resp, "AT+SOCKET=%d,%s,%d,0,%d",
                        type_id, ip, port, conid) != RT_EOK)
    {
        LOG_E("create socket(type=%d, %s:%d, conid=%d) failed.", type_id, ip, port, conid);
        goto __exit;
    }

    /* 校验一下模组给的 ConID 是否与我们要的一致（不一致会影响 URC 匹配） */
    line = at_resp_get_line_by_kw(resp, "connect success ConID=");
    if (line != RT_NULL)
    {
        int got = -1;
        if (rt_sscanf(line, "connect success ConID=%d", &got) == 1 && got != conid)
        {
            LOG_W("module assigned ConID=%d, but we requested %d -- URC may mismatch.", got, conid);
        }
    }

    socket->state = AT_SOCKET_CONNECT;
    result = RT_EOK;

__exit:
    at_delete_resp(resp);
    return result;
}

/** 手册 4.2.5 AT+SOCKETSEND=<ConID>,<length> -> '>' -> 原始数据（二进制安全） */
static int combo_socket_send(struct at_socket *socket, const char *buff, size_t bfsz,
                             enum at_socket_type type)
{
    at_response_t resp = RT_NULL;
    struct at_device *device = (struct at_device *)socket->device;
    rt_mutex_t lock = at_device_get_client_lock(device);
    size_t sent_size = 0, cur_pkt_size;
    rt_err_t result = RT_EOK;

    RT_UNUSED(type);

    RT_ASSERT(buff);
    RT_ASSERT(bfsz > 0);

    resp = at_create_resp(128, 1, COMBO_SOCKET_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (lock != RT_NULL)
    {
        rt_mutex_take(lock, RT_WAITING_FOREVER);
    }

    /* 手册 4.2.5：指令执行完会出现 '>'，看到它才能开始灌数据 */
    at_obj_set_end_sign(device->client, '>');

    while (sent_size < bfsz)
    {
        cur_pkt_size = (bfsz - sent_size < COMBO_SOCKET_SEND_MAX_SIZE) ?
                       (bfsz - sent_size) : COMBO_SOCKET_SEND_MAX_SIZE;

        if (at_obj_exec_cmd(device->client, resp, "AT+SOCKETSEND=%d,%d",
                            COMBO_SOCKET_CONID(socket), (int)cur_pkt_size) != RT_EOK)
        {
            LOG_E("socket(ConID=%d) send cmd failed.", COMBO_SOCKET_CONID(socket));
            result = -RT_ERROR;
            break;
        }

        if (at_client_obj_send(device->client, buff + sent_size, cur_pkt_size) != cur_pkt_size)
        {
            LOG_E("socket(ConID=%d) send data failed.", COMBO_SOCKET_CONID(socket));
            result = -RT_ERROR;
            break;
        }

        sent_size += cur_pkt_size;
    }

    at_obj_set_end_sign(device->client, 0);

    if (lock != RT_NULL)
    {
        rt_mutex_release(lock);
    }

    at_delete_resp(resp);

    return (result == RT_EOK) ? (int)sent_size : (int)result;
}

/** 手册 4.2.14 AT+WDOMAIN=<server name>（复用设备类里的实现） */
static int combo_socket_domain_resolve(const char *name, char ip[16])
{
    return combo_domain_resolve(name, ip);
}

static void combo_socket_set_event_cb(at_socket_evt_t event, at_evt_cb_t cb)
{
    if (event < (at_socket_evt_t)(sizeof(at_evt_cb_set) / sizeof(at_evt_cb_set[0])))
    {
        at_evt_cb_set[event] = cb;
    }
}

static const struct at_socket_ops combo_socket_ops =
{
    .at_connect       = combo_socket_connect,
    .at_closesocket   = combo_socket_close,
    .at_send          = combo_socket_send,
    .at_domain_resolve = combo_socket_domain_resolve,
    .at_set_event_cb  = combo_socket_set_event_cb,
    /* 这一版框架的 at_socket() 内部用 alloc_socket() 自行分配，不调用该回调，
     * 所以留空即可（class/esp8266 也是留空）。 */
    .at_socket        = RT_NULL,
};

/* ==========================================================================
 * 注册
 * ========================================================================== */

/** 设备 init 时调用：挂 socket URC + 切到主动上报模式 */
int combo_socket_init(struct at_device *device)
{
    at_response_t resp = RT_NULL;

    RT_ASSERT(device);

    /* 追加注册 socket URC（at_obj_set_urc_table 是追加式的，不会覆盖设备的表） */
    at_obj_set_urc_table(device->client, urc_table, sizeof(urc_table) / sizeof(urc_table[0]));

    /* 手册 4.2.10：1 = 主动模式，数据直接搭在 +EVENT:SocketDown 上 */
    resp = at_create_resp(128, 0, COMBO_SOCKET_RESP_TIMEOUT);
    if (resp == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    if (at_obj_exec_cmd(device->client, resp, "AT+SOCKETRECVCFG=1") != RT_EOK)
    {
        LOG_W("set AT+SOCKETRECVCFG=1 failed, received data may need AT+SOCKETREAD.");
    }
    at_delete_resp(resp);

    return RT_EOK;
}

/** 设备类注册时调用：填 socket 数量与 ops */
int combo_socket_class_register(struct at_device_class *class)
{
    RT_ASSERT(class);

    class->socket_num = AT_DEVICE_COMBO_SOCKETS_NUM;
    class->socket_ops = &combo_socket_ops;

    return RT_EOK;
}

#endif /* AT_DEVICE_USING_COMBO && AT_USING_SOCKET */
