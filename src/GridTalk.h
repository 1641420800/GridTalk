#ifndef _GRIDTALK_H_
#define _GRIDTALK_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

/* ================= 平台适配: 临界区 =================
   核心不包含任何芯片头文件; 中断屏蔽原语经宏注入, 各调用点在同一作用域成对使用:
     GT_CRIT_ENTER()  进入临界区(保存并关中断)
     GT_CRIT_EXIT()   退出临界区(恢复中断)
   须在任务与ISR上下文均安全(保存/恢复式, 而非成对开关式)。
   选择顺序:
   1. 定义 GT_PORT_HEADER(值为带引号的头文件名) -> 包含之, 由其提供两个宏;
   2. ARM/CMSIS编译器(armclang/armcc/gcc-arm/iar) -> 自动包含随库的
      port/gridtalk_port_cmsis.h(PRIMASK保存/恢复, 仅依赖cmsis_compiler.h);
   3. 其余(单上下文/PC仿真/协作式调度) -> 默认空操作。 */
#if defined(GT_PORT_HEADER)
  #include GT_PORT_HEADER
#elif defined(__ARMCC_VERSION) || defined(__ICCARM__) || \
      (defined(__GNUC__) && (defined(__arm__) || defined(__thumb__)))
  #include "../port/gridtalk_port_cmsis.h"
#else
  #define GT_CRIT_ENTER()
  #define GT_CRIT_EXIT()
#endif

/*
# 设计说明

- 多实例: 全部状态封装在gridTalk_t结构体内, 同一进程可创建多个独立实例,
  不再依赖全局单例; 平台通过钩子(nowUs/txByte)接入时间源与发送硬件。

- 帧格式: 目标ID(2) + 源ID(2) + 序号(2) + CMD(2) + 数据(N) + Fletcher-16(2)
  转义: FF 00 起始 ... FF FE 结束; 帧内FF转义为FF FF;
  应答: FF FC + 被确认方ID(2字节) + 确认方ID(2字节), 仅目标节点确认。
  (ACK携带确认方ID: 发送侧流水线允许多帧同时在途等ACK, 需按确认方对账)

- 自适应时序: 协议主动测量总线上相邻字节到达间隔得到"实测字节时间"byteUs,
  发送按byteUs节流(不依赖固定1ms tick)。
  空闲判定: 自主发送需总线静默4.5字符时间(防盲窗竞争);
  应答(ACK/响应帧): 收到帧结束后1字节时间即可开始, 保证"4字符时间内开始回复"。
  单跳最低延迟约等于"帧字节数+3"个byteUs。

- 可靠性: 单播等带地址ACK, 超时按实测byteUs指数退避重发; 序号去重防"重发被
  当新命令重复执行", 判定只看帧内序号, 与ACK是否丢失无关(见GT_SEQ_*与gtLinkRx);
  广播/NoAck尽力而为; 半双工发送期间(含1字节回显尾巴)忽略接收。
- 多信道冗余: 同一逻辑命令可经多个实例(gridTalk_sendMulti)各发一份, 全部副本
  共用同一序号; 接收端各实例经 gridTalk_linkSeq 并入同一序号域(共享发送计数与
  按源去重表)后, 跨信道副本被判为重发只ACK不执行 -- 单次执行语义, 且任一信道
  断链时另一信道独立承载(各自needAck重发互不影响)。

- 调用约定:
    gridTalk_init(gt, hw, id);        // hw=平台句柄(如USART2), 纯钩子接入可传NULL
    gridTalk_setBaud(gt, 9600);       // 可选: 已知波特率直接定标byteUs(推荐)
    gt->nowUs = 平台us时钟(如DWT);    // NULL则用usTick累加时钟
    gt->txByte = 平台发送函数;        // 唯一发送出口(逐字节)
    中断驱动(推荐): gt->txKick/txKickDone/txAbort = 发送中断控制钩子
                    (STM32一行绑定: gridTalk_portBindUart(gt), 见port/);
    轮询驱动: 上述三钩子保持NULL, usTick/rxByte入口按byteUs节流逐字节发送
              (可选 gt->txReady 背压: 硬件未就绪时不写)
    gt->onRx = 接收回调;  gt->onSendResult = 发送结果回调;
    周期调用 gridTalk_usTick(gt, us); // 建议1ms或更快
    UART RX中断调用 gridTalk_rxByte(gt, c);
    中断驱动: TXE中断调用 gridTalk_txIrq(gt)(可交付1字节),
              TC中断调用 gridTalk_txCompleteIrq(gt)(末字节移位完成)
    发送: gridTalk_send(单播可靠/自主帧), gridTalk_sendReply(响应,立即接入),
          gridTalk_sendNoAck(尽力而为), gridTalk_sendMulti(多信道冗余, 副本同序号)
*/

#define SEND_STREAM_BUFSIZE 2560
#define Bcast UINT16_MAX

#define GT_TX_FIFO_SIZE    8
#define GT_TX_FRAME_MAX    512
#define GT_ACK_QUEUE_SIZE  16
#define GT_DEFAULT_BYTE_US 1042u         /* 默认单字节时间(us): 9600bps 10bit */
#define GT_RESEND_COUNT    5

/* ---- 时序可配置宏 ----
   空闲判定: 自主发送前要求总线静默 >= GT_IDLE_CHARS_X10/10 个字符时间
   应答时限: 收到帧后须在 GT_REPLY_WINDOW_CHARS 个字符时间内开始回复
   应答就绪: 实际在帧结束后 GT_REPLY_START_CHARS 个字符时间开始(须<=时限)
   自主退避: 自主帧在空闲后再随机退避0..GT_AUTO_BACKOFF_CHARS-1个字符 */
#define GT_IDLE_CHARS_X10     45   /* 4.5字符: 静默间隔 */
#define GT_REPLY_WINDOW_CHARS 4    /* 应答开始时限(字符) */
#define GT_REPLY_START_CHARS  2    /* 应答就绪延迟(字符, <= 时限;
                                      须>1字符, 避免与帧内字节间隔(≈1字符)混淆,
                                      防止延迟应答插进正在传输的帧) */
#define GT_AUTO_BACKOFF_CHARS 96     /* 自主帧随机退避上限(字符): 随节点规模放大, 防"帧结束全员同槽"互撞 */
#define GT_ACK_WINDOW_CHARS 60 /* ACK窗口: 自本帧发送完成起算(非帧开始), 超时即重发;
                                  仅用于等待4字节ACK(快速通道2字符就绪+4字节发送,
                                  余量覆盖ACK队列排队); 下限250ms覆盖无线模块收发转换(实测约40ms)与对端节拍驱动
                                  的ACK发送延迟与byteUs自适应波动 */

/* 序号去重(见gtLinkRx): 发送方为每条新逻辑命令分配递增序号(重发复用同一号),
   接收方按源地址记忆最近执行的序号, 用无符号回卷差delta分四段判定:
     delta==0                        当前命令的重发 -> 只补ACK不执行
     65536-GT_SEQ_WINDOW+1..65535    迟到的旧帧(发送方串行, 最多落后1~2条) -> 同上
     1..GT_SEQ_WINDOW                新命令 -> 执行
     其余(远偏离)                    某端重启后序号失步 -> 执行并重新同步
   判定与ACK是否发出/丢失无关: "帧到而ACK丢"的经典重发不再被当第二条命令,
   同时保留"同内容不同序号"的故意重复发送语义(如连按两次启动). */
#ifndef GT_SEQ_WINDOW
#define GT_SEQ_WINDOW 16u   /* 新命令向前判定窗口(真重发最多落后1~2条, 16绰绰有余) */
#endif
#ifndef GT_SEQ_SLOTS
#define GT_SEQ_SLOTS  128   /* 按源地址记忆的槽位数(设备号0..127; 超出范围不去重) */
#endif
#ifndef GT_SEQ_DESYNC_FRAMES
#define GT_SEQ_DESYNC_FRAMES 8  /* 连续落后帧达此数=对端计数器重置失步, 强制重同步执行 */
#endif
/* 空闲保障窗口(us, 默认0): 接入与重发的随机退避范围扩展值
   (字符数 = idleGuardUs/byteUs)。覆盖"对端帧已上线、但本端尚未
   收到任何字节"的载波侦听盲区(打包型转发: 盲区≈打包门限)。
   注意: 不改4.5字符静默判定本身——固定等待会把各节点接入时刻对齐,
   实测反而加剧碰撞; 随机化才能既分散竞争又避开盲区。
   经 gridTalk_init 以 GT_IDLE_GUARD_US 初始化, 可运行时改 gt->idleGuardUs。 */
#ifndef GT_IDLE_GUARD_US
#define GT_IDLE_GUARD_US 0u
#endif

  /* 发送侧传输延迟上界(us, 默认0): 打包型转发(攒包后整包突发上线)等场景,
   本端"帧写入完毕"到"帧完整上线"之间的延迟上界。ACK窗口按
   GT_ACK_WINDOW_CHARS*byteUs + 2*txDelayUs 计算, 避免大帧被误判超时重发。
   经 gridTalk_init 以 GT_TX_DELAY_US 初始化, 可运行时改 gt->txDelayUs。 */
#ifndef GT_TX_DELAY_US
#define GT_TX_DELAY_US 0u
#endif

/* ACK窗口下限(us): 每实例可调(gt->ackWaitMinUs, init以GT_ACK_WINDOW_MIN_MS
   为默认)。默认250ms覆盖无线模块收发转换(~40ms)与对端节拍驱动的ACK延迟;
   快信道(485直连, RTT<1ms)应调小到~30ms, 否则每次丢帧的重发链路要付
   250ms×5 的队头阻塞代价 */

/* ---- 碰撞恢复调优(21台规模实测; 旧行为=200/250/0) ----
   旧行为的僵局: 多节点互撞后各自在<=208ms的小窗里重新随机->再次互撞;
   每次互撞发送队列冻结250ms(ACK窗口下限); 新鲜帧只需4.5字符即可上线,
   重发方却要付整个ACK超时 -> 重发持续挨撞, 总线利用率深坑后缓慢爬坡. */
#ifndef GT_BACKOFF_CAP_CHARS
#define GT_BACKOFF_CAP_CHARS 200  /* 重发退避窗口上限(字符): 打散互撞; 过大会拉长离线目标needAck帧的放弃时间, 堵塞发送队列 */
#endif
#ifndef GT_ACK_WINDOW_MIN_MS
#define GT_ACK_WINDOW_MIN_MS 250   /* ACK窗口下限(ms): 实测往返约54ms, 150≈2.8倍裕量 */
#endif
#ifndef GT_COLLISION_DEFER_CHARS
#define GT_COLLISION_DEFER_CHARS 0  /* 收到坏帧(碰撞残骸)后, 自主发送随机礼让0..N字符 */
#endif

/* 发送队列陈帧丢弃(us, 默认5000): 帧在FIFO中滞留超过此时长, 在开始发送前被丢弃
   并回报失败. 场景: 多信道冗余下快信道(485)已送达并推进了共享序号, 慢信道的
   同一副本若在积压队列里拖了数秒才首次上线, 其序号在接收端已落后超过去重窗口
   (GT_SEQ_WINDOW), 会落进"FAR=对端重启"分支被当新命令执行 -- 派生幻影波
   (级联环失控自由旋转的元凶之一). 丢弃陈帧切断该路径; 正常业务(方案下载
   344B≈0.4s/帧、Flash写冻结~2s)不会触及5s阈值. */
#ifndef GT_TX_STALE_MS
#define GT_TX_STALE_MS 5000u
#endif

/* 发送驱动模式:
   1 = 中断驱动(推荐): 平台提供 txKick/txKickDone/txAbort 钩子, 字节节奏由硬件
       TXE中断决定, 不受usTick周期限制; 最后一字节移位完成后由TC中断收尾
       (gridTalk_txIrq / gridTalk_txCompleteIrq 为平台中断入口);
   0 = 轮询: 上述钩子不接, 依赖usTick/rxByte入口, 每入口按byteUs节流发1字节
       (可选txReady背压) */
#ifndef GT_TX_IRQ
#define GT_TX_IRQ 1
#endif
#if GT_REPLY_START_CHARS > GT_REPLY_WINDOW_CHARS
#error "GT_REPLY_START_CHARS must be <= GT_REPLY_WINDOW_CHARS"
#endif

typedef struct gridTalk gridTalk_t;

typedef void (*gt_txByte_cb)(gridTalk_t *gt, uint8_t c);
typedef uint32_t (*gt_nowUs_cb)(void);
typedef void (*gt_txFrame_cb)(gridTalk_t *gt);   /* 帧开始/结束(485半双工方向控制, 可选) */
typedef void (*gt_txCtrl_cb)(gridTalk_t *gt);    /* 发送中断控制: txKick/txKickDone/txAbort */
typedef uint8_t (*gt_txReady_cb)(gridTalk_t *gt);/* 轮询模式背压: 硬件可再收1字节? */
typedef void (*gt_readData_cb)(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                               const uint8_t *data, uint16_t len);
typedef void (*gt_sendResult_cb)(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok);
/* 完整收帧回调(调试): 帧完成后被调, frame 为解帧后字节(目标ID+源ID+CMD+数据+Fletcher),
   crcOk=校验结果(0=坏帧). 该回调在UART中断上下文执行, 应只做轻量/入队操作. */
typedef void (*gt_onRxRaw_cb)(gridTalk_t *gt, const uint8_t *frame, uint16_t len, uint8_t crcOk);

/* 发送队列帧状态(流水线式发送: 在途等ACK的帧不占信道, 后续帧继续上线) */
#define GT_TXS_QUEUED    0u   /* 待发送(新帧, 或ACK超时后的重试) */
#define GT_TXS_ONWIRE    1u   /* 已选中接入/正在上线 */
#define GT_TXS_INFLIGHT  2u   /* 已上线, 等ACK中(不阻塞信道与队列) */

typedef struct {
  uint8_t  data[GT_TX_FRAME_MAX];
  uint16_t len;
  uint16_t targetId;
  uint16_t cmd;
  uint16_t seq;             /* 发送序号: 新逻辑命令取号, 重发复用(接收端据此去重) */
  uint32_t enqUs;           /* 入队时刻(us): 超龄帧在发送前丢弃(见GT_TX_STALE_MS) */
  uint8_t  needAck;
  uint8_t  reply;           /* 响应帧: 接入时立即发送, 不做随机退避 */
  uint8_t  st;              /* GT_TXS_* */
  uint8_t  resends;         /* 已重试次数 */
  uint32_t ackUntil;        /* 在途等ACK的截止时刻 */
  uint32_t retryUntilUs;    /* 重试退避截止(指数退避, 见gtDrain): 期间不被选中上线 */
} gtTxFrame_t;

typedef enum {
  GT_TX_IDLE = 0,           /* 信道空闲, 无选中帧 */
  GT_TX_ACCESS_WAIT,        /* 已选中帧, 等总线空闲+随机退避后上线 */
  GT_TX_SENDING,            /* 帧正在上线(TXE中断逐字节) */
  /* 注: 无WAIT_ACK状态 —— 帧上线后挂入队在途(GT_TXS_INFLIGHT), 信道立即可用,
     ACK异步回来按确认方ID对账销掉; 超时帧原位重排为QUEUED等下次接入 */
} gtTxState_t;

struct gridTalk {
  void *hw;                 /* 平台句柄(如USART_TypeDef*), 核心不解释; 供端口钩子取回硬件 */
  uint16_t id;

  /* 平台钩子 */
  gt_txByte_cb    txByte;   /* 发送出口(逐字节); NULL=无输出途径, 帧滞留队列 */
  /* 发送中断控制(GT_TX_IRQ=1时由平台提供, 见port/gridtalk_port_stm32.h);
     三者全NULL则该实例回退轮询节奏(usTick/rxByte入口按byteUs节流送txByte) */
  gt_txCtrl_cb    txKick;      /* 帧字节已备好, 启动发送(平台: 开TXE中断) */
  gt_txCtrl_cb    txKickDone;  /* 末字节已交付, 转等发送完成(平台: 关TXE开TC) */
  gt_txCtrl_cb    txAbort;     /* 发送卡死兜底(平台: 关发送中断/清挂起标志) */
  gt_txReady_cb   txReady;     /* 轮询模式可选背压: 硬件未就绪时不写 */
  gt_nowUs_cb     nowUs;
  gt_txFrame_cb   txFrameStart;      /* 帧开始发送(置485为发送方向, 可空) */
  gt_txFrame_cb   txFrameEnd;        /* 帧发送完成(485恢复接收方向, 可空) */
  gt_readData_cb  onRx;
  gt_sendResult_cb onSendResult;
  gt_onRxRaw_cb   onRxRaw;          /* 调试: 完整收帧(目标+源+CMD+数据+Fletcher) */

  /* 时钟 */
  uint32_t _nowUs;
  uint32_t _lastPollUs;

  /* 自适应实测字节速率(us) */
  uint32_t byteUs;
  uint32_t byteUsPending;   /* 自适应在途估计: 采样写入此值, 帧CRC通过才提交到
                               byteUs, 坏帧/截断帧回滚(碰撞字节交错的杂乱间隔
                               全落在坏帧里, 不得污染估计, 见gridTalk_rxByte) */
  uint32_t byteUsSamples;

  /* 空闲保障窗口(us): 随机接入/重发退避的范围扩展, GT_IDLE_GUARD_US 初始化 */
  uint32_t idleGuardUs;

  /* ACK窗口下限(us): 等ACK超时的下限, init以GT_ACK_WINDOW_MIN_MS*1000为默认;
     快信道(485)可调小, 见文件头注释 */
  uint32_t ackWaitMinUs;

  /* 发送侧传输延迟上界(us): ACK窗口据此放宽, GT_TX_DELAY_US 初始化 */
  uint32_t txDelayUs;

  /* 总线活动时刻(us) */
  uint32_t lastRxByteUs;
  uint32_t lastTxByteUs;
  uint32_t deferUntilUs;    /* 碰撞礼让: 收到坏帧后, 自主发送在此时刻前让路 */
  uint16_t rxBadFrames;     /* 诊断: 收到的坏帧(校验和错)累计, 反映信道质量 */
  uint8_t  rxBehindCnt[GT_SEQ_SLOTS]; /* 按源: 连续"落后帧"计数(失步重同步用) */
  uint16_t rxBytes;         /* 诊断: 总线收到的字节累计(不含自身回显), 反映信道活动 */
  uint16_t txGapMax200ms;   /* 诊断: 相邻两次成功入队的最大间隔(200ms单位,饱和) */
  uint32_t txLastEnqueueUs; /* 诊断: 上次成功入队时刻 */

  /* 本实例随机源(xorshift, 替代共享rand) */
  uint32_t rng;

  /* 发送 */
  /* FIFO为线性数组(下标0最旧): 终态帧可从任意位置移除(memmove),
     在途帧(INFLIGHT)保留槽位等待异步ACK对账 */
  gtTxFrame_t txFifo[GT_TX_FIFO_SIZE];
  uint8_t  txFifoCount;
  uint8_t  txRing[SEND_STREAM_BUFSIZE];
  uint16_t txRingHead;
  uint16_t txRingTail;
  uint16_t txRingCount;
  gtTxFrame_t txCurrent;      /* 正在上线/接入中的帧副本 */
  uint8_t  txCurIdx;          /* txCurrent对应FIFO槽位(发送完成时回写状态) */
  gtTxState_t txState;        /* 信道级状态(帧级状态见gtTxFrame_t.st) */
  uint32_t txWaitUs;        /* 接入等待剩余(us) */
  uint32_t txWaitUntilUs;
  uint32_t txSendStartUs;   /* 进入SENDING的时刻: SENDING卡死兜底用 */
  uint8_t  sendingAck;
  uint16_t ackQueue[GT_ACK_QUEUE_SIZE];
  uint8_t  ackHead;
  uint8_t  ackCount;
  uint16_t ackTargetId;
  gtTxState_t txResumeState;

  /* 接收 */
  uint8_t  rxBuff[SEND_STREAM_BUFSIZE + 8];
  uint16_t rxLen;
  uint8_t  rxStatus;
  uint8_t  ackTargetStage;   /* ACK载荷解析阶段: 1..4=被确认方hi/lo,确认方hi/lo */
  uint8_t  ackTargetHi;
  uint8_t  ackTargetLo;
  uint8_t  ackerHi;
  uint8_t  _c;              /* 转义解析状态 */

  /* 序号去重(见文件头GT_SEQ_*说明): 发送计数器 + 按源记忆的最近执行序号 */
  uint16_t txSeqCounter;                  /* 下一条新逻辑命令的发送序号 */
  uint16_t rxLastSeq[GT_SEQ_SLOTS];       /* 按源: 最近执行的发送序号 */
  uint8_t  rxSeqValid[GT_SEQ_SLOTS];      /* 按源: 已学习(0=该源首帧必执行) */
  uint16_t rxBehindLastSeq[GT_SEQ_SLOTS]; /* 按源: 失步兜底最近计数的落后序号
                                             (只计严格递增, 见gtLinkRx) */
  struct gridTalk *seqDomain;             /* 序号域(多信道冗余): NULL=独立域; 非NULL时
                                             发送计数与按源去重表改走宿主实例
                                             (见gridTalk_linkSeq/gridTalk_sendMulti) */

};

void gridTalk_init(gridTalk_t *gt, void *hw, uint16_t id);
void gridTalk_setBaud(gridTalk_t *gt, uint32_t baud);   /* 按波特率定标byteUs(10bit/字节) */
void gridTalk_setID(gridTalk_t *gt, uint16_t id);
void gridTalk_seed(gridTalk_t *gt, uint32_t entropy);   /* 注入每次上电不同的熵并重定序号基值: 防确定性基值跨上电复现被对端当重发吞掉 */
void gridTalk_usTick(gridTalk_t *gt, uint32_t us);
void gridTalk_rxByte(gridTalk_t *gt, uint8_t c);
#if GT_TX_IRQ
void gridTalk_txIrq(gridTalk_t *gt);          /* 平台TXE事件: 数据寄存器空, 可交付1字节 */
void gridTalk_txCompleteIrq(gridTalk_t *gt);  /* 平台TC事件: 末字节移位完成, 帧收尾 */
#endif
void gridTalk_send(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                   const uint8_t *data, uint16_t len);
void gridTalk_sendReply(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                        const uint8_t *data, uint16_t len);
void gridTalk_sendNoAck(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                        const uint8_t *data, uint16_t len);
/* 多信道冗余发送: 同一逻辑命令经 gts[0..count-1] 各信道实例发送一份, 全部副本
   带同一序号(自 gts[0] 所在序号域一次性分配)。语义同 gridTalk_send: 单播各信道
   独立带ACK重发(重发沿用同序号), 广播尽力而为; 某实例FIFO满则该信道静默丢弃
   (与单发一致), 其余信道不受影响。
   配合接收端 gridTalk_linkSeq 并域后, 跨信道副本被判为重发只ACK不执行(单次执行)。
   约束: 参与实例必须已并入同一序号域, 否则各信道其余帧的序号流交错,
   接收端共享去重表会把正常新命令误判FAR而反复重同步。 */
void gridTalk_sendMulti(gridTalk_t *gts[], uint8_t count, uint16_t target, uint16_t cmd,
                        const uint8_t *data, uint16_t len);
/* 序号域并入: member 的发送计数器与按源去重表改走 host 域 -- 发送侧各信道序号
   同源(单次递增), 接收侧跨信道去重(多信道冗余的配套)。init 之后、首次收发前
   调用; 并域后 member 自身的序号基值作废以 host 域为准(seed 建议在并域前完成,
   或统一对 host seed)。单实例固件(手柄)无需调用。 */
void gridTalk_linkSeq(gridTalk_t *host, gridTalk_t *member);
/* 轮询类应答专用: reply快速通道(帧尾2字符即发, 不做自主随机退避) + needAck=0
   (不等ACK不重发, 无重发风暴). 用于0x0402/0x0205这类"应答迟到无害, 下轮轮询
   自然补上"的周期应答 -- 若走自主通道, 大接入退避窗(GT_AUTO_BACKOFF_CHARS)会把
   应答拖进下一发轮询里对撞, 表现为扫描丢设备. */
void gridTalk_sendReplyNoAck(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                             const uint8_t *data, uint16_t len);
uint8_t gridTalk_pending(gridTalk_t *gt);
uint32_t gridTalk_byteUs(gridTalk_t *gt);

#endif
