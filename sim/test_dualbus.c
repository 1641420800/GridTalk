/*
 * test_dualbus.c -- 双信道冗余真实参数仿真
 * 信道1(无线): 9600bps(字节1042us) + 40ms单向延迟(无线模块回环)
 * 信道2(485) : 115200bps(字节87us) + 0.2ms换向(可忽略)
 *
 * 每条命令全流程: A端sendMulti双发 -> 并行上线 -> 按到达时刻归并投递 ->
 * B端并域去重单次执行 -> 各信道独立回L2 ACK -> B在首达信道回OK回执 -> A收OK。
 * 断言: 每条逻辑命令恰好执行1次/恰好1条OK/零发送失败/结束全部状态机IDLE。
 * 报告: 各命令执行信道、信道到达偏差、端到端时延、各实例byteUs自适应值。
 */
#include "GridTalk.h"
#include <stdio.h>
#include <string.h>

/* ---- 全局假时钟(us) ---- */
static uint32_t g_us = 1000;

/* ---- 信道参数 ---- */
#define CH1_BYTE_US 1042u    /* 无线 9600bps */
#define CH1_LAT_US  40000u   /* 无线模块单向延迟(实测) */
#define CH2_BYTE_US 87u      /* 485 115200bps */
#define CH2_LAT_US  200u     /* 485收发换向, 忽略量级 */

#define N_CMD 10

/* ---- 投递事件(按到达时刻排序) ---- */
typedef struct { uint32_t t; uint8_t b; gridTalk_t *dst; } ev_t;
static ev_t g_ev[4096];
static int g_evN;

static void ev_push(uint8_t b, uint32_t arrive, gridTalk_t *dst)
{
  if(g_evN >= (int)(sizeof(g_ev)/sizeof(g_ev[0]))) { printf("FAIL: 事件溢出\n"); exit(1); }
  int i = g_evN++;
  while(i > 0 && g_ev[i-1].t > arrive) { g_ev[i] = g_ev[i-1]; --i; }
  g_ev[i].t = arrive; g_ev[i].b = b; g_ev[i].dst = dst;
}

/* ---- 实例与信道 ---- */
typedef struct {
  gridTalk_t *gt;      /* 本端实例 */
  gridTalk_t *peer;    /* 对端实例(投递目标) */
  uint32_t   byteUs, latUs;
  uint32_t   nextTxUs; /* 下一字节/TC到期时刻(发送节拍) */
  uint32_t   lastTcUs; /* 最近一帧发送完成时刻(统计) */
  /* 仿真UART发送侧状态(经txKick/txKickDone/txAbort钩子置位, 替代伪造寄存器) */
  uint8_t    txIrqEn;  /* TXE中断已开(库请求交付字节) */
  uint8_t    tcArm;    /* 等TC(末字节已交付) */
  uint8_t    lastByte; /* 最近一次txByte交付的字节(DR内容) */
} link_t;

static gridTalk_t A1, A2, B1, B2;      /* A=发送端(id1) B=接收端(id2), 1=无线 2=485 */
static link_t g_links[4];              /* A1->B1, A2->B2, B1->A1, B2->A2 */

/* ---- 应用层钩子 ---- */
static int g_exec = 0;                 /* B端逻辑命令执行次数 */
static int g_ok = 0;                   /* A端收到OK回执次数 */
static int g_sendFail = 0;             /* 发送失败(giveup)次数 */
static gridTalk_t *g_execGt;           /* 最近一次执行所在实例 */
static uint32_t g_execT, g_okT, g_tc1, g_tc2;

static uint32_t fake_now_us(void) { return g_us; }

/* ---- 发送侧钩子(替代伪造寄存器) ---- */
static void db_tx_byte(gridTalk_t *gt, uint8_t c)
{
  ((link_t *)gt->hw)->lastByte = c;
}

static void db_tx_kick(gridTalk_t *gt)
{
  ((link_t *)gt->hw)->txIrqEn = 1;
}

static void db_tx_kick_done(gridTalk_t *gt)
{
  link_t *L = (link_t *)gt->hw;
  L->txIrqEn = 0;
  L->tcArm = 1;
}

static void db_tx_abort(gridTalk_t *gt)
{
  link_t *L = (link_t *)gt->hw;
  L->txIrqEn = 0;
  L->tcArm = 0;
}

static void on_rx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                  const uint8_t *d, uint16_t l)
{
  (void)l;
  if(cmd == 0x0100 && gt != &A1 && gt != &A2)
  { /* B端执行: 在首达信道回OK(真实固件行为: 回执走到达实例) */
    g_exec++; g_execGt = gt; g_execT = g_us;
    gridTalk_send(gt, src, 0x8301, (const uint8_t*)"OK", 2);
  }
  else if(cmd == 0x8301 && (gt == &A1 || gt == &A2))
  {
    g_ok++; g_okT = g_us;
  }
  (void)d;
}

static void on_send_result(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok)
{
  (void)gt; (void)target; (void)cmd;
  if(!ok) g_sendFail++;
}

/* ---- 发送泵: 到期发一字节/收尾, 字节完成时刻+延迟入投递队列 ---- */
static void link_pump(link_t *L)
{
  gridTalk_t *gt = L->gt;
  int guard = 0;
  while(guard++ < 512 && gt->txState == GT_TX_SENDING)
  {
    if(L->txIrqEn && gt->txRingCount > 0)
    {
      if((int32_t)(g_us - L->nextTxUs) < 0) break;
      gridTalk_txIrq(gt);   /* 字节经txByte钩子写入lastByte */
      ev_push(L->lastByte, g_us + L->byteUs + L->latUs, L->peer);
      L->nextTxUs = g_us + L->byteUs;
      if(gt->txRingCount == 0)
      { /* 末字节已写: 帧上线完成时刻=此刻+1字节时间. 只记命令窗口内首帧
           (=sendMulti的转发帧; 后续本端帧如ACK-of-OK不覆盖) */
        uint32_t frameEnd = g_us + L->byteUs;
        if(gt == &A1 && g_tc1 == 0) g_tc1 = frameEnd;
        if(gt == &A2 && g_tc2 == 0) g_tc2 = frameEnd;
      }
    }
    else if(L->tcArm && gt->txRingCount == 0)
    {
      if((int32_t)(g_us - L->nextTxUs) < 0) break;
      L->tcArm = 0;   /* TC已确认 */
      gridTalk_txCompleteIrq(gt);
      L->lastTcUs = g_us;
    }
    else break;
  }
}

/* 投递已到期事件(按时刻序) */
static void ev_flush_due(void)
{
  while(g_evN > 0 && (int32_t)(g_us - g_ev[0].t) >= 0)
  {
    gridTalk_rxByte(g_ev[0].dst, g_ev[0].b);
    memmove(&g_ev[0], &g_ev[1], (size_t)(g_evN-1)*sizeof(ev_t));
    g_evN--;
  }
}

static int inst_idle(gridTalk_t *gt)
{
  return gt->txState == GT_TX_IDLE && gt->txFifoCount == 0 && gt->ackCount == 0;
}

static int all_idle(void)
{
  return inst_idle(&A1) && inst_idle(&A2) && inst_idle(&B1) && inst_idle(&B2);
}

static uint32_t min_u32(uint32_t a, uint32_t b) { return a < b ? a : b; }

/* 主循环(事件驱动): 时钟精确跳到下一到期时刻(1ms节拍/到期投递/下一发送字节),
   保证接收字节间隔为真实值——否则1ms量化会污染byteUs自适应与tc统计.
   usTick为差值制(内部全部用nowUs差), 时钟跳变后调用一次等价于连续多次调用. */
static void settle(uint32_t maxMs)
{
  uint32_t endUs = g_us + maxMs * 1000u;
  int guard = 0;
  while(g_us < endUs && guard++ < 500000)
  {
    uint32_t t = g_us + 1000;                     /* 下一节拍 */
    if(g_evN) t = min_u32(t, g_ev[0].t);          /* 下一到期投递 */
    for(int i = 0; i < 4; ++i)                    /* 下一可写字节/TC */
      if(g_links[i].gt->txState == GT_TX_SENDING)
        t = min_u32(t, g_links[i].nextTxUs);
    if(t <= g_us) t = g_us + 1;
    g_us = t;
    gridTalk_usTick(&A1, 0); gridTalk_usTick(&A2, 0);
    gridTalk_usTick(&B1, 0); gridTalk_usTick(&B2, 0);
    for(int i = 0; i < 4; ++i) link_pump(&g_links[i]);
    ev_flush_due();
    if(all_idle() && g_evN == 0) return;
  }
  printf("FAIL: settle超时(状态机未归零: A1=%d A2=%d B1=%d B2=%d ev=%d)\n",
         (int)A1.txState, (int)A2.txState, (int)B1.txState, (int)B2.txState, g_evN);
  exit(1);
}

static void gt_setup(gridTalk_t *gt, uint16_t id, uint32_t entropy)
{
  gridTalk_init(gt, NULL, id);
  gridTalk_seed(gt, entropy);
  gt->nowUs = fake_now_us;
  gt->txByte = db_tx_byte;
  gt->txKick = db_tx_kick;
  gt->txKickDone = db_tx_kick_done;
  gt->txAbort = db_tx_abort;
  gt->onRx = on_rx;
  gt->onSendResult = on_send_result;
}

static int failures = 0;
#define CHECK(cond, msg) do { \
  if(cond) printf("PASS: %s\n", msg); \
  else { printf("FAIL: %s\n", msg); failures++; } } while(0)

int main(void)
{
  gt_setup(&A1, 1, 0x11111111u);
  gt_setup(&A2, 1, 0x22222222u);   /* 同id不同熵: 两信道退避序列独立(与固件一致) */
  gt_setup(&B1, 2, 0x33333333u);
  gt_setup(&B2, 2, 0x44444444u);
  gridTalk_linkSeq(&A1, &A2);
  gridTalk_linkSeq(&B1, &B2);
  /* 固件已按波特率定标byteUs(见main.c): 485实例从87us起步, 不再默认1042.
     无线实例由自适应维持1042, 定标值与之一致(1041), 二选一均可 */
  A2.byteUs = CH2_BYTE_US;
  B2.byteUs = CH2_BYTE_US;

  link_t *lA1 = &g_links[0], *lA2 = &g_links[1], *lB1 = &g_links[2], *lB2 = &g_links[3];
  lA1->gt=&A1; lA1->peer=&B1; lA1->byteUs=CH1_BYTE_US; lA1->latUs=CH1_LAT_US;
  lA2->gt=&A2; lA2->peer=&B2; lA2->byteUs=CH2_BYTE_US; lA2->latUs=CH2_LAT_US;
  lB1->gt=&B1; lB1->peer=&A1; lB1->byteUs=CH1_BYTE_US; lB1->latUs=CH1_LAT_US;
  lB2->gt=&B2; lB2->peer=&A2; lB2->byteUs=CH2_BYTE_US; lB2->latUs=CH2_LAT_US;
  A1.hw = lA1; A2.hw = lA2; B1.hw = lB1; B2.hw = lB2;   /* 发送钩子经hw取回链路 */

  gridTalk_t *gtsA[2] = {&A1, &A2};

  uint32_t skewSum = 0; int execBy485 = 0, execByWireless = 0;
  uint32_t e2eSum = 0;
  for(int n = 0; n < N_CMD; ++n)
  {
    int e0 = g_exec, o0 = g_ok, f0 = g_sendFail;
    uint32_t t0 = g_us;
    uint8_t payload = (uint8_t)n;
    g_tc1 = 0; g_tc2 = 0;   /* 只统计本命令的转发首帧(见link_pump注释) */
    gridTalk_sendMulti(gtsA, 2, 2, 0x0100, &payload, 1);
    settle(3000);
    if(g_exec - e0 != 1 || g_ok - o0 != 1 || g_sendFail - f0 != 0)
    {
      printf("FAIL: 命令#%d 执行%d次 OK%d次 失败%d次\n", n,
             g_exec-e0, g_ok-o0, g_sendFail-f0);
      failures++;
      break;
    }
    uint32_t skew = (g_tc1 + CH1_LAT_US) > (g_tc2 + CH2_LAT_US)
                  ? (g_tc1 + CH1_LAT_US) - (g_tc2 + CH2_LAT_US) : 0;
    skewSum += skew;
    e2eSum += g_okT - t0;
    if(g_execGt == &B2 || g_execGt == &B1)
      (g_execGt == &B2) ? execBy485++ : execByWireless++;
    printf("  命令#%02d: 执行信道=%s 到达偏差=%lu.%01lums 端到端=%lums OK经%s"
           " [dbg tc1=%lu tc2=%lu execdT=%lu]\n",
           n, (g_execGt==&B2)?"485":"无线",
           (unsigned long)(skew/1000), (unsigned long)((skew%1000)/100),
           (unsigned long)((g_okT - t0)/1000),
           (g_execGt==&B2)?"485":"无线",
           (unsigned long)g_tc1, (unsigned long)g_tc2,
           (unsigned long)(g_execT - t0));
  }

  printf("------------------------------------------------\n");
  CHECK(g_exec == N_CMD && g_ok == N_CMD, "每条逻辑命令恰好执行1次且恰好1条OK回执");
  CHECK(g_sendFail == 0, "零发送失败(无giveup, 无误超时重发耗尽)");
  CHECK(all_idle(), "全部状态机/FIFO/ACK队列归零(无残留卡死)");
  printf("统计: 执行信道 485=%d 无线=%d | 到达偏差均值=%lu.%01lums | 端到端均值=%lums\n",
         execBy485, execByWireless,
         (unsigned long)(skewSum/N_CMD/1000), (unsigned long)((skewSum/N_CMD%1000)/100),
         (unsigned long)(e2eSum/N_CMD/1000));
  printf("byteUs: 无线实例 A1=%lu B1=%lu (期望≈1042) | 485实例 A2=%lu B2=%lu\n",
         (unsigned long)gridTalk_byteUs(&A1), (unsigned long)gridTalk_byteUs(&B1),
         (unsigned long)gridTalk_byteUs(&A2), (unsigned long)gridTalk_byteUs(&B2));
  printf("------------------------------------------------\n");
  printf(failures ? "结果: %d 项FAIL\n" : "结果: 全部PASS (%d项FAIL)\n", failures);
  return failures ? 1 : 0;
}
/*
 * 构建运行: _t.bat (与 test_multich 一并构建)
 */
