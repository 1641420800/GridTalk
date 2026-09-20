/*
 * ring_sim.c -- 环形级联双总线 A/B 模拟器
 * 同一份应用代码, 构建时选库: -DUSE_OLD 用 gt_old.c(V2.17串行版=用户反馈问题的版本);
 * 默认用 GridTalk.c(V2.19流水线版), 同场景对比。
 *
 * 模型: 21台设备(跳过9号: 21->..->10->8->..->1->21) + 手柄(id0, 仅无线);
 *   总线0=无线(9600bps, 40ms单向延迟, 字节误码3‰, 延迟造成CSMA盲区→真实碰撞);
 *   总线1=485(115200bps, 0.2ms, 误码0.5‰);
 *   共享半双工: 同总线并发帧互撞, 双方整帧字节损坏(接收端Fletcher拒绝);
 *   应用=方案解释器: 0x06等信号(标志) -> 0x09延时100ms -> 0x07 sendMulti双信道转发;
 *   0x0100处理: 置标志+回OK(到达实例); 手柄200ms/ID轮询0x0402, 统计0x8402应答。
 */
#if defined(USE_OLD)
#include "gt_old.h"
#else
#include "GridTalk.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NNODE 22
#define SIM_MS 120000
#define DELAY_US 100000u
#define WBUS 0
#define FBUS 1
static const uint32_t BUS_BYTE_US[2] = {1042u, 87u};
static const uint32_t BUS_LAT_US[2]  = {40000u, 200u};
static const uint32_t BUS_LOSS_PM[2] = {30u, 5u};
/* 总线串行化模型: 模块/485收发器内部按总线排队, 帧字节在信道上串行下发
   (共享总线的真实物理: 并发交接若不排队, 接收端字节交错全灭).
   字节投递时刻 = max(交接时刻+延迟, 总线前一字节空出时刻);
   仅当两帧交接相差<=WCOLL_US(并发抢队竞态窗)才互撞, 后到者损坏(捕获效应).
   无线延迟40ms含排队, 485为0.2ms换向 */
#define WCOLL_US 2000u
static uint32_t busAirFreeUs[2] = {0, 0};   /* 该总线下一字节可下发的时刻 */

static uint32_t g_us = 1000;
static uint32_t rng_state = 1234567u;
static uint32_t rnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state >> 8; }

typedef struct {
  gridTalk_t gt;
  uint32_t nextTxUs;
  uint16_t node;
  uint8_t  bus;
  /* 仿真UART发送侧状态(经txKick/txKickDone/txAbort钩子置位, 替代伪造寄存器) */
  uint8_t  txIrqEn;        /* TXE中断已开(库请求交付字节) */
  uint8_t  tcArm;          /* 等TC(末字节已交付) */
  uint8_t  lastByte;       /* 最近一次txByte交付的字节(DR内容) */
  uint8_t  inFrame, frameDamaged;
  uint32_t frameStartUs;
} inst_t;
static inst_t I[NNODE][2];
static uint8_t nodeHas[NNODE][2];

static uint8_t  sig[NNODE];
static uint8_t  lapState[NNODE];
static uint32_t lapUntilUs[NNODE];
static uint32_t execT[NNODE][800]; static int execN[NNODE];
static uint32_t replyT[128];

static uint64_t st_bytes[2], st_frames[2], st_busyUs[2], st_coll[2];
static uint64_t st_sendFail = 0, st_replies = 0;
static uint32_t hsNextUs = 0; static uint16_t hsScanId = 1;

static uint16_t nextOf(uint16_t n)
{
  if(n == 1) return 21;
  if(n == 10) return 8;
  return (uint16_t)(n - 1);
}
static uint32_t fake_now(void) { return g_us; }

static uint64_t st_okByCmd[2][4];   /* [bus][cmd类] 成功: 0=0100转发 1=8301OK 2=8402应答 3=其他 */
static uint64_t st_failByCmd[2][4];
static int cmdIdx(uint16_t c)
{
  if(c == 0x0100) return 0;
  if(c == 0x8301) return 1;
  if(c == (0x0402 | 0x8000)) return 2;
  return 3;
}
static void on_send_result(gridTalk_t *gt, uint16_t t, uint16_t c, uint8_t ok)
{
  inst_t *me = (inst_t *)gt->hw;
  if(ok) st_okByCmd[me->bus][cmdIdx(c)]++;
  else { st_failByCmd[me->bus][cmdIdx(c)]++; st_sendFail++; }
  (void)t;
}

static void on_rx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                  const uint8_t *d, uint16_t l)
{
  (void)d; (void)l;
  inst_t *me = (inst_t *)gt->hw;
  uint16_t node = me->node;
  if(node == 0)
  { /* 手柄: 只关心在线应答 */
    if(cmd == (0x0402 | 0x8000) && src < 128)
    {
      replyT[src] = g_us;
      st_replies++;
    }
    return;
  }
  if(cmd == 0x0100)
  { /* Task_ctrl: 置标志 + 回OK(到达实例) */
    sig[node] = 1;
    gridTalk_send(gt, src, 0x8301, (const uint8_t*)"OK", 2);
    if(execN[node] < 800) execT[node][execN[node]++] = g_us;
  }
  else if(cmd == 0x0402)
  {
    gridTalk_sendReplyNoAck(gt, src, 0x0402 | 0x8000,
                            (const uint8_t*)&node, 1);
  }
}

/* ---- 投递事件 ---- */
typedef struct { uint32_t t; uint8_t b; inst_t *src; uint8_t damaged; } dlv_t;
static dlv_t D[8192]; static int DN;
static void dlv_push(uint8_t b, uint32_t arrive, inst_t *src, uint8_t damaged)
{
  if(DN >= (int)(sizeof(D)/sizeof(D[0]))) { printf("FAIL: 投递队列溢出\n"); exit(1); }
  int i = DN++;
  while(i > 0 && D[i-1].t > arrive) { D[i] = D[i-1]; --i; }
  D[i].t = arrive; D[i].b = b; D[i].src = src; D[i].damaged = damaged;
}
static void dlv_flush(void)
{
  while(DN > 0 && (int32_t)(g_us - D[0].t) >= 0)
  {
    inst_t *src = D[0].src;
    for(uint16_t n = 0; n < NNODE; ++n)
    {
      if(n == src->node || !nodeHas[n][src->bus]) continue;
      if(rnd() % 100000u < BUS_LOSS_PM[src->bus] * 100u) continue;  /* 字节误码 */
      uint8_t b = D[0].damaged ? (uint8_t)(D[0].b ^ (rnd() & 0xFF)) : D[0].b;
      gridTalk_rxByte(&I[n][src->bus].gt, b);
    }
    memmove(&D[0], &D[1], (size_t)(DN-1)*sizeof(dlv_t));
    DN--;
  }
}

/* ---- 发送侧钩子: txByte交付字节, txKick等控制发送中断(替代伪造寄存器) ---- */
static void ring_tx_byte(gridTalk_t *gt, uint8_t c)
{
  inst_t *me = (inst_t *)gt->hw;
  me->lastByte = c;              /* 写入"DR", 由inst_pump搬到线上 */
}

static void ring_tx_kick(gridTalk_t *gt)
{
  inst_t *me = (inst_t *)gt->hw;
  me->txIrqEn = 1;
}

static void ring_tx_kick_done(gridTalk_t *gt)
{
  inst_t *me = (inst_t *)gt->hw;
  me->txIrqEn = 0;
  me->tcArm = 1;
}

static void ring_tx_abort(gridTalk_t *gt)
{
  inst_t *me = (inst_t *)gt->hw;
  me->txIrqEn = 0;
  me->tcArm = 0;
}

/* ---- 发送泵: TXE逐字节 + TC收尾, 会话登记与碰撞检测 ---- */
static void inst_pump(inst_t *me)
{
  gridTalk_t *gt = &me->gt;
  int guard = 0;
  while(guard++ < 1024 && gt->txState == GT_TX_SENDING)
  {
    if(me->txIrqEn && gt->txRingCount > 0)
    {
      if((int32_t)(g_us - me->nextTxUs) < 0) break;
      if(!me->inFrame)
      { /* 帧起始: 起始时刻落入彼此接入盲窗=碰撞, 双方整帧损坏 */
        me->inFrame = 1; me->frameStartUs = g_us; me->frameDamaged = 0;
        st_frames[me->bus]++;
        /* 模块时槽结构: 排队帧之间保留帧间隙(否则接收端看不到空闲窗, CSMA全员饿死) */
        busAirFreeUs[me->bus] += (me->bus == WBUS) ? 6000u : 600u;
        for(uint16_t n = 0; n < NNODE; ++n)
        {
          if(n == me->node) continue;
          inst_t *o = &I[n][me->bus];
          if(!nodeHas[n][me->bus] || !o->inFrame) continue;
          uint32_t dt = (g_us > o->frameStartUs) ? g_us - o->frameStartUs
                                                  : o->frameStartUs - g_us;
          uint32_t win = (me->bus == WBUS) ? WCOLL_US : 400u;
          if(dt <= win)
          {
            st_coll[me->bus]++;
            me->frameDamaged = 1;   /* 交接竞态: 后到者死, 先到存活(捕获) */
          }
        }
      }
      gridTalk_txIrq(gt);           /* 字节经txByte钩子写入lastByte */
      uint8_t b = me->lastByte;
      { /* 模块内排队: 总线串行下发(并发交接的字节不交错) */
        uint32_t air = g_us + BUS_BYTE_US[me->bus] + BUS_LAT_US[me->bus];
        if((int32_t)(busAirFreeUs[me->bus] - air) > 0) air = busAirFreeUs[me->bus];
        busAirFreeUs[me->bus] = air + BUS_BYTE_US[me->bus];
        dlv_push(b, air, me, me->frameDamaged);
      }
      me->nextTxUs = g_us + BUS_BYTE_US[me->bus];
      st_bytes[me->bus]++;
      st_busyUs[me->bus] += BUS_BYTE_US[me->bus];
    }
    else if(me->tcArm && gt->txRingCount == 0)
    {
      if((int32_t)(g_us - me->nextTxUs) < 0) break;
      me->tcArm = 0;   /* TC已确认 */
      gridTalk_txCompleteIrq(gt);
      me->inFrame = 0;
    }
    else break;
  }
}

static uint32_t min_u32(uint32_t a, uint32_t b) { return a < b ? a : b; }

/* ---- 应用节拍: 解释器(0x06/0x09/0x07) + 手柄扫描 ---- */
static void app_tick(void)
{
  for(uint16_t n = 1; n < NNODE; ++n)
  {
    if(!nodeHas[n][0]) continue;
    if(lapState[n] == 1)
    {
      if((int32_t)(g_us - lapUntilUs[n]) >= 0)
      {
        gridTalk_t *gts[2] = {&I[n][WBUS].gt, &I[n][FBUS].gt};
        gridTalk_sendMulti(gts, 2, nextOf(n), 0x0100, NULL, 0);
        lapState[n] = 0;
      }
    }
    else if(sig[n])
    {
      sig[n] = 0;
      lapState[n] = 1;
      lapUntilUs[n] = g_us + DELAY_US;
    }
  }
  /* 手柄: 200ms/ID 轮询 1..127 */
  if((int32_t)(g_us - hsNextUs) >= 0)
  {
    gridTalk_sendNoAck(&I[0][WBUS].gt, hsScanId, 0x0402, NULL, 0);
    hsScanId = (hsScanId >= 127) ? 1 : (uint16_t)(hsScanId + 1);
    hsNextUs = g_us + 200000u;
  }
}

static void inst_setup(uint16_t n, uint8_t bus)
{
  inst_t *me = &I[n][bus];
  memset(me, 0, sizeof(*me));
  me->node = n; me->bus = bus;
  gridTalk_init(&me->gt, me, n);             /* hw=本实例, 钩子经hw取回 */
  gridTalk_seed(&me->gt, 0x01000000u * n + bus * 0x7777u + 9u);
  me->gt.nowUs = fake_now;
  me->gt.txByte = ring_tx_byte;
  me->gt.txKick = ring_tx_kick;
  me->gt.txKickDone = ring_tx_kick_done;
  me->gt.txAbort = ring_tx_abort;
  me->gt.onRx = on_rx;
  me->gt.onSendResult = on_send_result;
  me->gt.byteUs = BUS_BYTE_US[bus];
  me->gt.ackWaitMinUs = (bus == FBUS) ? 30000u : 250000u;
  nodeHas[n][bus] = 1;
}

static void stat_lap(const char *tag, int dev)
{
  int n = execN[dev];
  if(n < 3) { printf("  设备%d执行%d次(样本不足)\n", dev, n); return; }
  uint32_t mx = 0, sum = 0, cnt = 0, sub1s = 0;
  for(int i = 1; i < n; ++i)
  {
    uint32_t dt = execT[dev][i] - execT[dev][i-1];
    sum += dt; cnt++;
    if(dt > mx) mx = dt;
    if(dt < 1000000u) sub1s++;
  }
  printf("  设备%d: 执行%d次, 圈速均值=%lums 最大=%lums 最小圈=%lums, <1s圈占比=%lu%%\n",
         dev, n, (unsigned long)(sum/cnt/1000), (unsigned long)(mx/1000),
         (unsigned long)((execT[dev][n-1]-execT[dev][0])/(n-1)/1000),
         (unsigned long)(sub1s*100/cnt));
  (void)tag;
}

int main(void)
{
  inst_setup(0, WBUS);                       /* 手柄 */
  for(uint16_t n = 1; n < NNODE; ++n)
  {
    if(n == 9) continue;                     /* 9号离线 */
    inst_setup(n, WBUS);
    inst_setup(n, FBUS);
    gridTalk_linkSeq(&I[n][WBUS].gt, &I[n][FBUS].gt);
  }
  sig[21] = 1;                               /* t≈0 从21号触发 */

  uint32_t endUs = g_us + (uint32_t)SIM_MS * 1000u;
  int guard = 0;
  while(g_us < endUs && guard++ < 8000000)
  {
    uint32_t t = g_us + 1000;                          /* 1ms节拍 */
    if(DN) t = min_u32(t, D[0].t);
    for(uint16_t n = 0; n < NNODE; ++n)
      for(uint8_t b = 0; b < 2; ++b)
        if(nodeHas[n][b] && I[n][b].gt.txState == GT_TX_SENDING)
          t = min_u32(t, I[n][b].nextTxUs);
    if(t <= g_us) t = g_us + 1;
    g_us = t;
    for(uint16_t n = 0; n < NNODE; ++n)
      for(uint8_t b = 0; b < 2; ++b)
        if(nodeHas[n][b]) gridTalk_usTick(&I[n][b].gt, 0);
    for(uint16_t n = 0; n < NNODE; ++n)
      for(uint8_t b = 0; b < 2; ++b)
        if(nodeHas[n][b]) inst_pump(&I[n][b]);
    dlv_flush();
    app_tick();
    if(guard >= 8000000) { printf("FAIL: 主循环超限\n"); return 1; }
  }

  printf("================================================\n");
#if defined(USE_OLD)
  printf("环形级联A/B仿真: 【旧库 V2.17串行版】(用户反馈问题版本)\n");
#else
  printf("环形级联A/B仿真: 【新库 V2.19流水线版】\n");
#endif
  printf("场景: 21设备(跳9)双信道, 无线9600+40ms延迟, 485@115200, 0x09延时100ms, %ds\n",
         SIM_MS/1000);
  printf("---- 环 ----\n");
  stat_lap("lap", 1);
  stat_lap("lap", 21);
  printf("---- 信道 ----\n");
  for(int b = 0; b < 2; ++b)
    printf("  %s: 帧数=%llu 占用=%.1f%% 碰撞=%llu\n",
           b ? "485" : "无线",
           (unsigned long long)st_frames[b],
           (double)st_busyUs[b] / (SIM_MS*1000.0) * 100.0,
           (unsigned long long)st_coll[b]);
  printf("---- 手柄 ----\n");
  {
    int online = 0;
    for(int i = 1; i < 128; ++i)
      if((int32_t)(g_us - replyT[i]) < (int32_t)25400000u) online++;
    printf("  应答累计=%llu, 当前可见设备=%d/20\n",
           (unsigned long long)st_replies, online);
  }
  printf("  发送失败(giveup/陈丢)=%llu\n", (unsigned long long)st_sendFail);
  for(int b = 0; b < 2; ++b)
    printf("    %s 成功[转发=%llu OK=%llu 应答=%llu] 失败[转发=%llu OK=%llu 应答=%llu]\n",
           b?"485":"无线",
           (unsigned long long)st_okByCmd[b][0], (unsigned long long)st_okByCmd[b][1],
           (unsigned long long)st_okByCmd[b][2],
           (unsigned long long)st_failByCmd[b][0], (unsigned long long)st_failByCmd[b][1],
           (unsigned long long)st_failByCmd[b][2]);
  printf("================================================\n");
  return 0;
}
