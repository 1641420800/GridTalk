/*
 * test_pipe.c -- gtpipe 管道组件回归测试(两节点直连, 复用test_dualbus链路模型)
 *
 * 覆盖:
 *  T1  8KB随机数据 writeAll->readAll 字节级比对(分片/停等/零拷贝路径)
 *  T2  gtpipe_printf 文本会话(发送环形路径), 格式化内容比对
 *  T3  管道与业务流量共存: 大传输中途注入业务帧, 回调链正常透传
 *  T4  丢帧信道(每字节0.2%丢失/翻转, 含ACK)下4KB传输字节级完好
 *      -- 验证L2重发+去重经管道组合后仍"恰好一次"
 *  T5  未注册管道号的数据被对端丢弃(不透传应用, 不误入其他管道)
 *  T6  错误路径: 对端不存在 -> 短超时返回-2, 长超时返回-1(L2放弃)
 *
 * 构建运行: 与build_run.ps1同方式, 追加 /I..\app 并链接 ../app/gridtalk_pipe.c
 */
#include "GridTalk.h"
#include "gridtalk_pipe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BYTE_US 1042u
#define ID_A 1u
#define ID_B 2u

/* ---- 假时钟 ---- */
static uint32_t g_us = 1000;
static uint32_t fake_now_us(void) { return g_us; }

/* ---- 误码注入 ---- */
static uint32_t g_lossPm = 0;      /* 每字节丢失/翻转率(千分比) */
static uint32_t g_lcg = 0xABCD9u;
static uint32_t lcg_next(void) { g_lcg = g_lcg * 1664525u + 1013904223u; return g_lcg >> 8; }

/* ---- 投递事件(按到达时刻排序) ---- */
typedef struct { uint32_t t; uint8_t b; gridTalk_t *dst; } ev_t;
static ev_t g_ev[4096];
static int g_evN;
static uint64_t g_dropN, g_flipN;

static void ev_push(uint8_t b, uint32_t arrive, gridTalk_t *dst)
{
  if(g_evN >= (int)(sizeof(g_ev)/sizeof(g_ev[0]))) { printf("FAIL: 事件溢出\n"); exit(1); }
  int i = g_evN++;
  while(i > 0 && g_ev[i-1].t > arrive) { g_ev[i] = g_ev[i-1]; --i; }
  g_ev[i].t = arrive; g_ev[i].b = b; g_ev[i].dst = dst;
}

/* ---- 链路(与test_dualbus同构: 钩子胶水+发送泵) ---- */
typedef struct {
  gridTalk_t *gt, *peer;
  uint32_t   byteUs;
  uint32_t   nextTxUs;
  uint8_t    txIrqEn, tcArm, lastByte;
} link_t;

static gridTalk_t A, B;
static link_t lA, lB;

static void tx_byte(gridTalk_t *gt, uint8_t c) { ((link_t *)gt->hw)->lastByte = c; }
static void tx_kick(gridTalk_t *gt) { ((link_t *)gt->hw)->txIrqEn = 1; }
static void tx_kick_done(gridTalk_t *gt)
{
  link_t *L = (link_t *)gt->hw;
  L->txIrqEn = 0;
  L->tcArm = 1;
}
static void tx_abort(gridTalk_t *gt)
{
  link_t *L = (link_t *)gt->hw;
  L->txIrqEn = 0;
  L->tcArm = 0;
}

/* ---- 应用层回调(经hub链式透传的非管道流量) ---- */
static int g_appRxB = 0;      /* B端收到业务帧0x0100次数 */
static void app_rx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                   const uint8_t *d, uint16_t l)
{
  (void)src;
  if(cmd == 0x0100 && gt == &B)
  {
    g_appRxB++;
    (void)d; (void)l;
  }
}
static int g_appFail = 0;     /* 业务帧发送失败次数(链式) */
static void app_result(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok)
{
  (void)gt; (void)target; (void)cmd;
  if(!ok) g_appFail++;
}

/* B侧原始帧转储: 看总线上实际到达的管道帧载荷 */
static int g_rawDump = 0;
static void raw_dump_b(gridTalk_t *gt, const uint8_t *frame, uint16_t len, uint8_t crcOk)
{
  (void)gt;
  if(!g_rawDump || len < 10 || !crcOk) return;
  if(frame[6] != 0x0F || frame[7] != 0x00) return;   /* 只看管道帧 */
  printf("[raw] len=%u pipe=%u dlen=%u data=", len, frame[8],
         (unsigned)((unsigned)frame[9] | ((unsigned)frame[10] << 8)));
  for(int k = 0; k < 10; ++k) printf("%02X", frame[11 + k]);
  printf("\n");
}

/* ---- 管道 ---- */
static gtpipe_hub_t hubA, hubB;
static gtpipe_t pA, pB;       /* 管道1: A<->B */
static gtpipe_t pA5;          /* T5: 未在对端注册的管道2 */
static gtpipe_t pDead;        /* T6: 发往不存在节点 */

/* ---- 发送泵 ---- */
static void link_pump(link_t *L)
{
  gridTalk_t *gt = L->gt;
  int guard = 0;
  while(guard++ < 512 && gt->txState == GT_TX_SENDING)
  {
    if(L->txIrqEn && gt->txRingCount > 0)
    {
      if((int32_t)(g_us - L->nextTxUs) < 0) break;
      gridTalk_txIrq(gt);
      ev_push(L->lastByte, g_us + L->byteUs, L->peer);
      L->nextTxUs = g_us + L->byteUs;
    }
    else if(L->tcArm && gt->txRingCount == 0)
    {
      if((int32_t)(g_us - L->nextTxUs) < 0) break;
      L->tcArm = 0;
      gridTalk_txCompleteIrq(gt);
    }
    else break;
  }
}

static void ev_flush_due(void)
{
  while(g_evN > 0 && (int32_t)(g_us - g_ev[0].t) >= 0)
  {
    uint8_t b = g_ev[0].b;
    gridTalk_t *dst = g_ev[0].dst;
    memmove(&g_ev[0], &g_ev[1], (size_t)(g_evN-1)*sizeof(ev_t));
    g_evN--;
    if(g_lossPm && (lcg_next() % 1000u) < g_lossPm)
    {
      if(lcg_next() & 1u) { g_dropN++; continue; }
      b ^= (uint8_t)(1u << (lcg_next() & 7u));
      g_flipN++;
    }
    gridTalk_rxByte(dst, b);
  }
}

static int inst_idle(gridTalk_t *gt)
{
  return gt->txState == GT_TX_IDLE && gt->txFifoCount == 0 && gt->ackCount == 0;
}

/* 推进仿真: 至多maxMs毫秒(或全空闲即返回) */
static void settle(uint32_t maxMs)
{
  uint32_t endUs = g_us + maxMs * 1000u;
  int guard = 0;
  while(g_us < endUs && guard++ < 500000)
  {
    uint32_t t = g_us + 1000;
    if(g_evN && g_ev[0].t < t) t = g_ev[0].t;
    if(lA.gt->txState == GT_TX_SENDING && lA.nextTxUs < t) t = lA.nextTxUs;
    if(lB.gt->txState == GT_TX_SENDING && lB.nextTxUs < t) t = lB.nextTxUs;
    if(t <= g_us) t = g_us + 1;
    g_us = t;
    gridTalk_usTick(&A, 0);
    gridTalk_usTick(&B, 0);
    link_pump(&lA);
    link_pump(&lB);
    ev_flush_due();
    if(inst_idle(&A) && inst_idle(&B) && g_evN == 0) return;
  }
}

static int pump_plain(void *user) { (void)user; settle(5); return 0; }

/* ---- 数据与汇聚 ---- */
#define BULK_N 8192
static uint8_t g_src[BULK_N];
static uint8_t g_sink[BULK_N + 64];
static uint32_t g_sinkN;
static int g_appInjectDone;
static uint32_t g_progDone, g_progTotal, g_progCalls;

static void prog_cb(gtpipe_t *p, uint32_t done, uint32_t total)
{
  (void)p;
  g_progDone = done;
  g_progTotal = total;
  g_progCalls++;
}

static int pump_drain(void *user)
{
  (void)user;
  settle(5);
  /* T3: 大传输中途注入一帧业务流量(验证回调链共存) */
  if(!g_appInjectDone && g_sinkN > 500)
  {
    g_appInjectDone = 1;
    gridTalk_send(&A, ID_B, 0x0100, (const uint8_t *)"hi", 2);
  }
  g_sinkN += (uint32_t)gtpipe_read(&pB, g_sink + g_sinkN,
                (uint16_t)((sizeof(g_sink) - g_sinkN > 0xFFFFu) ? 0xFFFFu : sizeof(g_sink) - g_sinkN));
  return 0;
}

static int failures = 0;
#define CHECK(cond, msg) do { \
  if(cond) printf("PASS: %s\n", msg); \
  else { printf("FAIL: %s\n", msg); failures++; } } while(0)

static void gt_setup(gridTalk_t *gt, link_t *L, uint16_t id, uint32_t entropy,
                     gridTalk_t *peer)
{
  gridTalk_init(gt, L, id);
  gridTalk_seed(gt, entropy);
  gridTalk_setBaud(gt, 9600);
  gt->nowUs = fake_now_us;
  gt->txByte = tx_byte;
  gt->txKick = tx_kick;
  gt->txKickDone = tx_kick_done;
  gt->txAbort = tx_abort;
  gt->onRx = app_rx;             /* 先设应用回调, attach时被链式保存 */
  gt->onSendResult = app_result;
  gt->onRxRaw = raw_dump_b;
  L->gt = gt;
  L->peer = peer;
  L->byteUs = BYTE_US;
}

int main(void)
{
  gt_setup(&A, &lA, ID_A, 0x11111111u, &B);
  gt_setup(&B, &lB, ID_B, 0x22222222u, &A);
  gtpipe_attach(&hubA, &A);
  gtpipe_attach(&hubB, &B);
  gtpipe_register(&pA, &hubA, 1, ID_B);
  gtpipe_register(&pB, &hubB, 1, ID_A);
  CHECK(A.onRx != app_rx && B.onRx != app_rx, "hub已接管回调(链式安装)");
  settle(50);

  for(uint32_t i = 0; i < BULK_N; ++i) g_src[i] = (uint8_t)lcg_next();

  /* ---- T1: 8KB分片传输(停等/零拷贝/进度回调) ---- */
  {
    g_sinkN = 0;
    g_appInjectDone = 0;
    g_appRxB = 0;
    g_progCalls = 0;
    uint32_t t0 = g_us;
    int r = gtpipe_writeAll(&pA, g_src, BULK_N, 60000000u, prog_cb, pump_drain, NULL);
    CHECK(r == BULK_N, "T1. 8KB writeAll成功返回");
    int r2 = gtpipe_readAll(&pB, g_sink + g_sinkN, BULK_N - g_sinkN,
                            10000000u, NULL, pump_plain, NULL);
    CHECK(r2 == (int)(BULK_N - g_sinkN), "T1. readAll收齐剩余字节");
    if(memcmp(g_sink, g_src, BULK_N) != 0)
    {
      uint32_t off = 0;
      while(off < BULK_N && g_sink[off] == g_src[off]) off++;
      printf("      [首异@%lu: got %02X exp %02X]\n", (unsigned long)off,
             g_sink[off], g_src[off]);
    }
    CHECK(memcmp(g_sink, g_src, BULK_N) == 0, "T1. 8KB字节级一致");
    CHECK(gtpipe_status(&pB) == 0, "T1. 无接收溢出/发送失败标志");
    CHECK(g_progCalls > 0 && g_progDone == g_progTotal && g_progTotal == BULK_N,
          "T1. 进度回调触发且完成点=总量");
    CHECK(g_appInjectDone && g_appRxB == 1, "T3. 大传输中途业务帧经回调链正常到达");
    CHECK(gtpipe_available(&pB) == 0, "T1. 接收环形已取空");
    settle(50);
    CHECK(inst_idle(&A) && inst_idle(&B), "T1. 结束后L2状态机归零");
    printf("      [8KB用时%.1fs, 进度回调%lu次]\n",
           (double)(g_us - t0) / 1e6, (unsigned long)g_progCalls);
  }

  /* ---- T2: printf文本会话(发送环形路径) ---- */
  {
    const char *exp1 = "temp=25.50 mode=auto;";
    const char *exp2 = "seq=1024 crc=0xBEEF end\n";
    gtpipe_printf(&pA, "temp=%d.%02u mode=%s;", 25, 50, "auto");
    gtpipe_printf(&pA, "seq=%u crc=0x%04X end\n", 1024u, 0xBEEFu);
    char buf[64];
    int need = (int)(strlen(exp1) + strlen(exp2));
    int r = gtpipe_readAll(&pB, buf, (uint32_t)need, 5000000u, NULL, pump_plain, NULL);
    CHECK(r == need, "T2. printf会话字节数正确");
    buf[need] = 0;
    char exp[64];
    (void)snprintf(exp, sizeof(exp), "%s%s", exp1, exp2);
    CHECK(strcmp(buf, exp) == 0, "T2. 格式化内容一致(有序无丢失)");
    if(strcmp(buf, exp) != 0)
      printf("      [got: %s | exp: %s]\n", buf, exp);
    settle(50);
  }

  /* ---- T4: 丢帧信道下4KB传输(重发+去重 -> 恰好一次) ---- */
  {
    g_lossPm = 2;   /* 每字节0.2%: 丢失或翻转(ACK同样受损) */
    memset(g_sink, 0, 4096);
    g_sinkN = 0;
    int r = gtpipe_writeAll(&pA, g_src, 4096u, 300000000u, NULL, pump_drain, NULL);
    int r2 = gtpipe_readAll(&pB, g_sink + g_sinkN, 4096u - g_sinkN,
                            10000000u, NULL, pump_plain, NULL);
    (void)r2;
    g_lossPm = 0;
    CHECK(r == 4096, "T4. 丢帧信道下4KB传输完成");
    CHECK(memcmp(g_sink, g_src, 4096) == 0, "T4. 字节级完好(重发+去重恰好一次)");
    CHECK(gtpipe_status(&pB) == 0, "T4. 无溢出/失败标志");
    printf("      [注入丢失%llu字节/翻转%llu字节]\n",
           (unsigned long long)g_dropN, (unsigned long long)g_flipN);
    settle(50);
  }

  /* ---- T5: 未注册管道号 -> 对端丢弃 ---- */
  {
    gtpipe_register(&pA5, &hubA, 2, ID_B);   /* B端只注册了管道1 */
    uint16_t before = gtpipe_available(&pB);
    gtpipe_write(&pA5, (const uint8_t *)"junk", 4);
    settle(3000);
    CHECK(gtpipe_available(&pB) == before, "T5. 未注册管道数据被对端丢弃");
    CHECK(g_appRxB == 1, "T5. 管道流量未透传应用(业务计数不变)");
    gtpipe_detach(&pA5);
    settle(50);
  }

  /* ---- T6: 对端不存在 -> 超时/放弃路径 ---- */
  {
    gtpipe_register(&pDead, &hubA, 9, 3);    /* 节点3不存在 */
    uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    int r1 = gtpipe_writeAll(&pDead, d, 8, 300000u, NULL, pump_plain, NULL);
    CHECK(r1 == -2, "T6a. 短超时返回-2");
    int r2 = gtpipe_writeAll(&pDead, d, 8, 10000000u, NULL, pump_plain, NULL);
    CHECK(r2 == -1, "T6b. L2放弃返回-1");
    gtpipe_detach(&pDead);
    settle(100);
    /* 放弃路径后管道/hub可继续正常工作 */
    int r3 = gtpipe_writeAll(&pA, g_src, 100u, 10000000u, NULL, pump_plain, NULL);
    (void)gtpipe_readAll(&pB, g_sink, 100u, 10000000u, NULL, pump_plain, NULL);
    CHECK(r3 == 100, "T6c. 失败路径后管道恢复正常传输");
    settle(50);
    CHECK(inst_idle(&A) && inst_idle(&B), "T6. 结束后L2状态机归零");
  }

  printf("------------------------------------------------\n");
  printf(failures ? "结果: %d 项FAIL\n" : "结果: 全部PASS (%d项FAIL)\n", failures);
  return failures ? 1 : 0;
}
