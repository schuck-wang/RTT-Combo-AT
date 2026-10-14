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

/* 只需要给下面的 combo_sock_test 用，但放头部更符合习惯 */
#include <sys/socket.h>

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

/* 二进制补齐：往 URC 里捞数据时用的暂存块（只在 AT 客户端接收线程里用，无并发） */
#define COMBO_SOCKET_BIN_CHUNK         (256)
#define COMBO_SOCKET_BIN_TIMEOUT       200

static char combo_socket_bin_chunk[COMBO_SOCKET_BIN_CHUNK];

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
        /* ---- 二进制安全接收 ----
         * at_recv_readline() 一遇到 \r\n 就结束本行（见 at_client.c），所以：
         *
         *   数据里没有 0x0D0A（绝大多数情况，概率 ~1 - 1/65536 per byte）
         *     -> 整行 = 前缀 + 全部数据 + 模块行尾 \r\n，avail == len + 2
         *        直接投递 len 字节，行尾 \r\n 已被行解析器吃掉，不用管
         *
         *   数据里含 0x0D0A（行在数据中间被截断）
         *     -> avail <= len，剩下的还在串口里，用 at_client_obj_recv() 裸读补齐
         *
         * at_client_obj_recv() 的官方注释就是"只能用在 URC 回调里"：它直接
         * rt_device_read 串口，绕过行解析；用完把 RX 超时恢复成 RT_WAITING_NO
         * （at_client_init() 里的初始值），所以对接收线程没有副作用。
         * 注意：URC 回调里绝对不能改成 at_obj_exec_cmd()，那个会死等。 */
        rt_size_t off   = (rt_size_t)(p - data);
        rt_size_t avail = (size > off) ? (size - off) : 0;
        rt_size_t got   = ((rt_size_t)len < avail) ? (rt_size_t)len : avail;

        if (got > 0 && at_evt_cb_set[AT_SOCKET_EVT_RECV] != RT_NULL)
        {
            at_evt_cb_set[AT_SOCKET_EVT_RECV](sock, AT_SOCKET_EVT_RECV, p, got);
        }

        /* 被截断了就裸读补齐（分块投递，不需要大缓冲） */
        while (got < (rt_size_t)len)
        {
            rt_size_t  want = (rt_size_t)len - got;
            rt_ssize_t n;

            if (want > sizeof(combo_socket_bin_chunk))
            {
                want = sizeof(combo_socket_bin_chunk);
            }

            n = (rt_ssize_t)at_client_obj_recv(client, combo_socket_bin_chunk, want,
                                               COMBO_SOCKET_BIN_TIMEOUT);
            if (n <= 0)
            {
                LOG_W("SocketDown conid=%d: short read %d/%d bytes (peer reset?).",
                      conid, (int)got, len);
                break;
            }
            if ((rt_size_t)n > want)
            {
                n = (rt_ssize_t)want;       /* 防御上游 rt_device_read 异常返回值 */
            }

            if (at_evt_cb_set[AT_SOCKET_EVT_RECV] != RT_NULL)
            {
                at_evt_cb_set[AT_SOCKET_EVT_RECV](sock, AT_SOCKET_EVT_RECV,
                                                  combo_socket_bin_chunk, (size_t)n);
            }
            got += (rt_size_t)n;
        }

        /* 只有"提前断行"的情况才需要把模块行尾的 \r\n 吃掉：
         * avail > len 时它已经作为行结束符被行解析器吃掉了 */
        if (avail <= (rt_size_t)len)
        {
            at_client_obj_recv(client, combo_socket_bin_chunk, 2, COMBO_SOCKET_BIN_TIMEOUT);
        }
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

/* ==========================================================================
 * MSH：二进制链路验证
 * --------------------------------------------------------------------------
 * socket 数据是"搭在 +EVENT:SocketDown 这一行里"传的，而 at_client 的行解析器
 * 一遇到 \r\n 就断行 —— 所以 urc_socket_down_func() 必须用 at_client_obj_recv()
 * 裸读补齐。那套逻辑只有真实模组才能验证，这个命令就是干这个的：
 * 从 HTTP 服务器拉一个文件，算【实收字节数 + CRC32】，和服务器端对比。
 *
 * 为什么选 HTTP 而不是裸 TCP：HTTP 响应头**本身就是 CRLF 拼的**，
 * 所以它一次性把"含大量 CRLF 的文本"和"纯二进制 body"两条路都验证了。
 * ========================================================================== */
#ifdef FINSH_USING_MSH

#define COMBO_TEST_BUF_SIZE       (1024)
#define COMBO_TEST_MAX_ERR        (5)

/* 标准 CRC32（IEEE 802.3, poly 0xEDB88320, 初值 0xFFFFFFFF, 末尾取反）
 * PC 端对照（Python 的 zlib.crc32 就是同一条）：
 *   python -c "import zlib,sys;print('%08X'%zlib.crc32(open(sys.argv[1],'rb').read()))" fw.bin */
static rt_uint32_t combo_test_crc32(rt_uint32_t crc, const rt_uint8_t *buf, rt_size_t len)
{
    rt_size_t i;
    int       j;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (rt_uint32_t)(-(rt_int32_t)(crc & 1u)));
        }
    }

    return crc;
}

static void combo_sock_test(int argc, char **argv)
{
    struct sockaddr_in addr;
    char        ip[16]   = {0};
    char        req[256] = {0};
    char        hdr[512] = {0};
    char        buf[COMBO_TEST_BUF_SIZE];
    rt_uint32_t crc       = 0xFFFFFFFFu;
    rt_size_t   hdr_len   = 0;
    rt_size_t   body_len  = 0;
    rt_uint32_t hdr_state = 0;
    rt_bool_t   in_hdr    = RT_TRUE;
    int err_cnt = 0, fd, n, port, i;

    if (argc < 4)
    {
        rt_kprintf("usage: combo_sock_test <host> <port> <path>\n"
                   "   eg: combo_sock_test 192.168.1.10 80 /fw.bin\n"
                   "HTTP GET -> skip header -> print body bytes + CRC32 (no flash write).\n"
                   "PC check: python -c \"import zlib,sys;print('%%08X'%%zlib.crc32(open(sys.argv[1],'rb').read()))\" fw.bin\n");
        return;
    }
    if (combo_at_get_client() == RT_NULL)
    {
        rt_kprintf("combo: AT client not ready\n");
        return;
    }

    port = atoi(argv[2]);
    if (combo_domain_resolve(argv[1], ip) != RT_EOK)
    {
        rt_kprintf("dns \"%s\" failed\n", argv[1]);
        return;
    }

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

    rt_kprintf("GET http://%s:%d%s ...\n", ip, port, argv[3]);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        rt_kprintf("connect() failed\n");
        closesocket(fd);
        return;
    }

    rt_snprintf(req, sizeof(req),
                "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
                argv[3], argv[1]);
    if (send(fd, req, rt_strlen(req), 0) < 0)
    {
        rt_kprintf("send() failed\n");
        closesocket(fd);
        return;
    }

    for (;;)
    {
        n = recv(fd, buf, sizeof(buf), 0);
        if (n == 0)
        {
            break;                      /* 对端关闭，正常结束 */
        }
        if (n < 0)
        {
            if (++err_cnt > COMBO_TEST_MAX_ERR)
            {
                rt_kprintf("recv() error/timeout, give up\n");
                break;
            }
            continue;                   /* 偶发超时，再试 */
        }
        err_cnt = 0;

        i = 0;

        if (in_hdr)
        {
            /* 逐字节找响应头结束 \r\n\r\n（状态机可以跨 recv 边界） */
            for (; i < n; i++)
            {
                char c = (char)buf[i];

                if (hdr_len < sizeof(hdr) - 1)
                {
                    hdr[hdr_len++] = c;
                }

                if (c == '\r')
                {
                    hdr_state = (hdr_state == 0 || hdr_state == 2) ? hdr_state + 1 : 1;
                }
                else if (c == '\n')
                {
                    hdr_state = (hdr_state == 1 || hdr_state == 3) ? hdr_state + 1 : 0;
                }
                else
                {
                    hdr_state = 0;
                }

                if (hdr_state == 4)
                {
                    i++;                /* 跳过这个 \n */
                    in_hdr = RT_FALSE;
                    break;
                }
            }

            if (in_hdr)
            {
                continue;               /* 这一块整块都是响应头 */
            }
        }

        if (i < n)
        {
            rt_size_t old = body_len;

            crc = combo_test_crc32(crc, (const rt_uint8_t *)buf + i, (rt_size_t)(n - i));
            body_len += (rt_size_t)(n - i);

            if (body_len / (64 * 1024) != old / (64 * 1024))
            {
                rt_kprintf("  ... %d bytes\n", (int)body_len);
            }
        }
    }

    closesocket(fd);

    hdr[hdr_len] = '\0';

    rt_kprintf("--- response header ---\n%s------------------------\n", hdr);

    {
        char *cl = rt_strstr(hdr, "Content-Length:");

        if (cl == RT_NULL)
        {
            cl = rt_strstr(hdr, "content-length:");
        }
        if (cl != RT_NULL)
        {
            int want = atoi(cl + 15);

            rt_kprintf("Content-Length = %d, received = %d -> %s\n",
                       want, (int)body_len,
                       (want == (int)body_len) ? "MATCH" : "*** MISMATCH ***");
        }
    }

    rt_kprintf("body: %d bytes, CRC32 = %08X\n", (int)body_len, (unsigned)(~crc));
}
MSH_CMD_EXPORT_ALIAS(combo_sock_test, combo_sock_test, combo: HTTP body byte count + CRC32 (verify binary path));

#endif /* FINSH_USING_MSH */

#endif /* AT_DEVICE_USING_COMBO && AT_USING_SOCKET */
