#ifndef GRIDTALK_PIPE_H
#define GRIDTALK_PIPE_H
/*
 * GridTalk 应用层组件: 可靠有序字节管道
 *
 * 在一条 gridTalk_t 总线实例上定义若干互不影响的管道(管道号+对端ID),
 * 每个管道是一条停等式可靠有序字节流:
 *   - 分段传输: 写入按 GTPIPE_SEG_MAX 分段, 同一时刻至多一段在途(停等);
 *   - 有序+恰好一次: 复用 L2 的 needAck 重发与序号去重 -- 接收端只会在
 *     去重通过(onRx)时收数据, 重发副本被去重吞掉; 停等保证到达序=发送序;
 *   - 大数据传输: gtpipe_writeAll 从用户缓冲零拷贝分段(gtpipe_readAll 接收);
 *   - 文本会话: gtpipe_printf; 有 stdio 后端时 gtpipe_fopen 得到 FILE*
 *     直接 fprintf/fscanf(见 gridtalk_pipe_stdio.h)。
 *
 * 线上协议: CMD=GTPIPE_CMD_DATA, 载荷 [pipeId(1)][len(2,小端)][data(len≤GTPIPE_SEG_MAX)]。
 *
 * 语义边界(务必知晓):
 *   - writeAll 返回 -1(对端离线/L2放弃)时, 末段是否已达对端未知 -- 重试整个
 *     传输而非续传, 否则可能在对端产生重复数据;
 *   - 接收环形(GTPIPE_RX_SIZE)读得慢会溢出: 丢整段并置溢出标志(gtpipe_status),
 *     保有序不保完整 -- 大数据传输用 readAll 持续收或把环形调大;
 *   - 每 hub 的全部管道共享一段在途(共享半双工总线上这不损失吞吐),
 *     多管道并发 writeAll 按注册序先后完成。
 *
 * 线程模型: 状态经 GT_CRIT_* 保护(保存/恢复式, 可嵌套), 回调可能在ISR上下文
 * 触发; 阻塞API(writeAll/readAll)通过 pump 回调推进事件 -- 裸机自旋传NULL,
 * 仿真/协作调度传入事件泵。
 */
#include "GridTalk.h"
#include <stdarg.h>

#define GTPIPE_CMD_DATA 0x0F00     /* 管道数据帧CMD(L3命令空间, 应用勿占用0x0F00) */

#ifndef GTPIPE_MAX
#define GTPIPE_MAX 4               /* 每总线实例(每hub)最多管道数 */
#endif
#ifndef GTPIPE_RX_SIZE
#define GTPIPE_RX_SIZE 1024        /* 每管道接收环形(字节), 应≥GTPIPE_SEG_MAX */
#endif
#ifndef GTPIPE_TX_SIZE
#define GTPIPE_TX_SIZE 256         /* 每管道发送环形(printf/小写入用; writeAll零拷贝不占) */
#endif
#ifndef GTPIPE_PRINTF_BUF
#define GTPIPE_PRINTF_BUF 128      /* gtpipe_printf 格式化栈缓冲 */
#endif
#ifndef GTPIPE_HUB_MAX
#define GTPIPE_HUB_MAX 4           /* 全局hub注册表容量(每总线实例一个hub) */
#endif
#define GTPIPE_SEG_MAX ((GT_TX_FRAME_MAX) - 13)  /* 单段数据上限(帧载荷-管道头3B) */

/* 管道状态位(gtpipe_status, 读后清零) */
#define GTPIPE_ST_RXOVER 0x01u     /* 接收环形溢出过(丢过段) */
#define GTPIPE_ST_TXFAIL  0x02u    /* 发送环形内数据L2放弃过 */

typedef struct gtpipe gtpipe_t;
typedef struct gtpipe_hub gtpipe_hub_t;

/* 阻塞API的事件泵: 等待期间被周期调用以推进协议(仿真泵/RTOS yield); NULL=自旋 */
typedef int (*gtpipe_pump_cb)(void *user);
/* 进度回调: writeAll/readAll 等待期间在调用者上下文触发 */
typedef void (*gtpipe_progress_cb)(gtpipe_t *p, uint32_t done, uint32_t total);

/* ---- hub: 每个gridTalk_t实例一个, 管道表+共享停等发送机+回调链 ---- */
struct gtpipe_hub {
  gridTalk_t *gt;
  gt_readData_cb   prevOnRx;       /* 应用原回调(非管道流量透传给它) */
  gt_sendResult_cb prevOnResult;
  gtpipe_t *pipes[GTPIPE_MAX];
  uint8_t  pipeN;
  /* 停等发送机(全hub共享一段在途) */
  uint8_t  segBusy;                /* 1=段已上线, 等L2发送结果 */
  uint8_t  seg[3 + GTPIPE_SEG_MAX];
  gtpipe_t *segOwner;              /* 在途段所属管道 */
  uint8_t  segFromBulk;            /* 1=来自writeAll游标, 0=来自发送环形 */
  uint32_t segBulkStart;           /* writeAll游标段的首字节偏移 */
  uint16_t segDataLen;             /* 段内数据字节数 */
};

/* ---- 管道: (hub, 管道号, 对端ID) 唯一确定 ---- */
struct gtpipe {
  gtpipe_hub_t *h;
  uint8_t  id;
  uint16_t peer;
  uint8_t  rxBuf[GTPIPE_RX_SIZE];  /* 接收环形 */
  uint16_t rxHead, rxCount;
  uint8_t  stFlags;                /* 诊断标志(GTPIPE_ST_*, gtpipe_status读后清零) */
  uint8_t  txBuf[GTPIPE_TX_SIZE];  /* 发送环形(小写入/printf) */
  uint16_t txHead, txCount;
  const uint8_t *bulkSrc;          /* writeAll游标(零拷贝源) */
  uint32_t bulkLen, bulkPos;
  int8_t   bulkErr;                /* 0=无/完成, -1=L2放弃, -2=超时 */
};

/* 挂接总线实例: 链式接管 gt 的 onRx/onSendResult(管道流量被消化, 其余透传
   给应用已设置的回调); 须在首次收发前、应用回调设置之后调用 */
void gtpipe_attach(gtpipe_hub_t *h, gridTalk_t *gt);
/* 注册/注销管道(同hub内管道号+对端唯一; 注销取消未完成的writeAll) */
void gtpipe_register(gtpipe_t *p, gtpipe_hub_t *h, uint8_t pipeId, uint16_t peerId);
void gtpipe_detach(gtpipe_t *p);

/* 小写入(入发送环形, 尽力): 返回接受字节数(可部分, 满则0) */
int  gtpipe_write(gtpipe_t *p, const void *buf, uint16_t len);
/* 非阻塞读(从接收环形取, 可部分): 返回取出字节数(0=暂无数据) */
int  gtpipe_read(gtpipe_t *p, void *buf, uint16_t len);
uint16_t gtpipe_available(gtpipe_t *p);
uint8_t  gtpipe_status(gtpipe_t *p);   /* 诊断标志, 读后清零 */

/* 大块可靠传输(阻塞至完成/失败/超时):
   返回 len=成功, -1=对端不可达(L2放弃, 末段是否已达未知), -2=超时, -3=参数错 */
int  gtpipe_writeAll(gtpipe_t *p, const void *buf, uint32_t len,
                     uint32_t timeoutUs, gtpipe_progress_cb prog,
                     gtpipe_pump_cb pump, void *user);
int  gtpipe_readAll(gtpipe_t *p, void *buf, uint32_t len,
                    uint32_t timeoutUs, gtpipe_progress_cb prog,
                    gtpipe_pump_cb pump, void *user);

int  gtpipe_vprintf(gtpipe_t *p, const char *fmt, va_list ap);
int  gtpipe_printf(gtpipe_t *p, const char *fmt, ...);

/* 兜底驱动(发送曾因L2 FIFO满而挂起时由它续段): 周期调用(如1ms节拍)可加速恢复 */
void gtpipe_poll(gtpipe_hub_t *h);

#endif /* GRIDTALK_PIPE_H */
