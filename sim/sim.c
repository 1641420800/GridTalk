/*
 * GridTalk 485半双工总线仿真器(结构体多实例版)
 *
 * 直接创建多个gridTalk_t实例, 通过txByte/nowUs/onRx/onSendResult钩子
 * 接入共享半双工总线模型:
 *   - 按配置波特率收发(默认9600), 字节上线时间 = 10bit / 波特率;
 *   - 发送方: 帧首字节上线前附加随机小延时(0~1字节时间, 模拟UART启动抖动),
 *     帧内后续字节连续移位不加抖动;
 *   - 接收方: 每个接收方有独立的固有随机小延时(0~1字节时间, 初始化时确定),
 *     字节不回投给发送方(无自回显);
 *   - 误码: 接收时每字节独立按配置概率(默认0.4%)丢失或单比特翻转;
 *   - 碰撞: 两字节上线时间重叠即碰撞, 双方字节作废, 接收方采样到随机噪声;
 *   - 转发方式: 0=不打包(字节随上线随投递, 默认);
 *               1=打包(端到端整包: 发送侧攒包期间字节不上线, 静默门限内无新字节后
 *                 整包按字节节拍突发上线; 接收侧随突发连续收到完整一包);
 *     攒包期间总线静默, 其他节点无法侦听到"正在攒包"的节点(盲区=门限),
 *     空闲保障窗(idleGuardUs, 打包模式自动=门限)随机化接入予以缓解;
 *   - 场景: 1台A轮询255ID(前N台在线) + B分组建中继环 + 1台C随机向B发送2KB随机数据并校验回声
 *
 * 用法: sim.exe [时长秒=120] [每字节误码率%=0.4] [设备数=6] [种子=42] [波特率=9600] [打包=0] [打包门限ms=10] [空闲保障ms=-1自动]
 */
#include "GridTalk.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ================= 配置 ================= */
#define MAX_DEV     201              /* 200台B + 1台C(设备号200) */

/* B 中继分组: 每组 5 台(最后一组不足 5 台), 组间启动错峰, 避免所有组同刻开跑 */
#define RELAY_GROUP_BASE_SIZE 5
#define RELAY_GROUP_STAGGER_MS 500
#define ECHO_CMD_DATA   0x0500        /* C -> B: 数据分片(带序号+校验) */
#define ECHO_CMD_REPLY  0x0501        /* B -> C: 原样回声 */
#define ECHO_DATA_SIZE  2048
#define ECHO_CHUNK_DATA 494           /* 单帧净荷上限502(帧头含2字节发送序号后512-10) - 8字节(序号+校验) */
#define ECHO_CHUNKS     ((ECHO_DATA_SIZE + ECHO_CHUNK_DATA - 1) / ECHO_CHUNK_DATA)
#define ECHO_TRANSFER_GAP_MS 20000.0  /* 上一次传输结束后等待多久再发起下一次 */
#define ECHO_START_DELAY_MS 30000.0   /* C首次发送延迟, 让中继环先建立 */
#define ECHO_TXN_WINDOW_MS  15000.0   /* 回声事务窗口: 发片后在此时间内未收到回声则带新nonce重发 */
#define RTT_CAP     8192
#define ACK_CAP     8192
#define RELAY_FWD_DELAY_MS 1000.0     /* B收到中继后延迟多久才触发转发 */
#define RELAY_REINIT_MS 15000.0       /* 令牌环静默超过该时长, 组长重发令牌(转发应答无重发, 丢一跳即断环) */
#define RX_FIFO_SIZE 1400             /* 每接收方在途字节队列(打包模式需容纳整包回放) */
#define PKT_BUF_SIZE 1400             /* 打包模式单包缓存上限(帧最大约1200字节含转义) */
#define DEFAULT_PKT_IDLE_MS 10.0      /* 打包模式默认静默门限 */

/* 时序参数由波特率推导(main中计算) */
static double   g_byteTimeMs;         /* 单字节上线时间(ms) = 10000/波特率(10bit) */
static double   g_dtMs;               /* 微步时长(ms) = 字节时间/10 */
static uint32_t g_dtUs;               /* 微步时长(us), 供gridTalk_usTick */
static uint32_t g_stepsPerMs;         /* 每毫秒微步数 */
static double   g_burstGapMs;         /* 帧间隙判定阈值 = 2字节时间 */
static uint32_t g_baud = 9600;
static double   g_idleGuardMs = -1.0; /* 空闲保障窗口覆盖值(ms), <0=自动: 打包=门限 */
static uint32_t g_idleGuardUs = 0;    /* 实际下发的空闲保障窗口(us) */
static uint32_t g_txDelayUs = 0;      /* 发送侧传输延迟上界(us), 供协议ACK窗口放宽 */

static gridTalk_t gts[MAX_DEV];

/* ================= RNG(仿真噪声用) ================= */
static uint32_t g_rng = 0x12345678u;
static void rng_seed(uint32_t s) { g_rng = s ? s : 0x12345678u; }
static uint32_t rng_next(void)
{
  uint32_t x = g_rng;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  g_rng = x;
  return x;
}

/* ================= 统计 ================= */
typedef struct {
  uint64_t pollSent;        /* A发出在线探测次数 */
  uint64_t pollResp;        /* A收到探测回应次数 */
  uint64_t pollCycles;      /* A完成1..255轮询周期数 */
  uint8_t  onlineSeen[128];
  uint64_t onlineUnique;
  uint64_t relayRx;         /* B收到中继次数 */
  uint64_t relayTxResult;   /* B中继发送完成次数 */
  uint64_t relayLaps;
  double   lapSum, lapMin, lapMax; uint32_t lapN;
  double   hopLatSum, hopLatMin, hopLatMax; uint32_t hopLatN;
  uint64_t wireFrames;
  uint64_t wireAcks;
  uint64_t collidedBursts;
  uint64_t retransmits;
  uint64_t collisions;
  uint64_t sendOk;
  uint64_t sendFail;
  uint64_t rxUnicast;
  uint64_t rxPing;
  uint64_t rxPong;
  uint64_t rxBcast;
  uint64_t echoRx;          /* B收到C的数据分片次数 */
  uint64_t echoTxOk;        /* C回声帧发送完成(收到ACK)次数 */
  uint64_t echoTransferOk;  /* C完成2KB传输次数 */
  uint64_t echoBytesSent;   /* C发出的数据字节 */
  uint64_t echoBytesVerified;
  uint64_t echoVerifyFail;  /* 回声校验失败(序号/校验和不符) */
  uint64_t echoTxnTimeout;  /* 回声事务超时重发次数 */
  double   echoRttSum, echoRttMin, echoRttMax; uint32_t echoRttN;
  double  *echoRttSamples;  uint32_t echoRttCap, echoRttCnt;
  uint64_t pingDelivered;
  double   rttSum, rttMin, rttMax; uint32_t rttN;
  double   ackSum, ackMin, ackMax; uint32_t ackN;
  double  *rttSamples;  uint32_t rttCap, rttCnt;
  double  *ackSamples;  uint32_t ackCap, ackCnt;
} stats_t;

static void stats_init(stats_t *s)
{
  memset(s, 0, sizeof(*s));
  s->rttMin = 1e300; s->ackMin = 1e300; s->hopLatMin = 1e300;
  s->echoRttMin = 1e300;
  s->rttSamples = (double*)malloc(RTT_CAP * sizeof(double));
  s->ackSamples = (double*)malloc(ACK_CAP * sizeof(double));
  s->echoRttSamples = (double*)malloc(RTT_CAP * sizeof(double));
  s->rttCap = RTT_CAP; s->ackCap = ACK_CAP;
  s->echoRttCap = RTT_CAP;
}

static void samp_add(double *arr, uint32_t cap, uint32_t *cnt, double v)
{
  if(*cnt < cap) arr[(*cnt)++] = v;
}

static int cmp_d(const void *a, const void *b)
{
  double x = *(const double*)a, y = *(const double*)b;
  return (x > y) - (x < y);
}

static double pct(double *arr, uint32_t n, double p)
{
  uint32_t lo;
  double frac;
  if(n == 0) return 0.0;
  qsort(arr, n, sizeof(double), cmp_d);
  if(p <= 0.0) return arr[0];
  if(p >= 1.0) return arr[n - 1];
  frac = (double)(n - 1) * p;
  lo = (uint32_t)frac;
  frac -= (double)lo;
  return arr[lo] * (1.0 - frac) + arr[lo + 1] * frac;
}

/* ================= 仿真设备(总线模型) ================= */
typedef struct {
  uint16_t target;
  uint32_t seq;
  double   t0;
  uint8_t  active;
} pending_t;

typedef struct rxEntry rxEntry_t;
struct rxEntry {
  uint8_t  b;
  double   due;          /* 投递时刻(字节完成时刻+该接收方独立随机延时) */
};

typedef struct {
  uint16_t id;
  uint32_t tickPhase;
  double   rxDelay;        /* 本节点固有的接收延时(0~1字节时间, 初始化时随机) */

  /* 仿真UART发送侧状态(经txKick/txKickDone/txAbort钩子置位, 替代伪造寄存器):
     txIrqEn=TXE中断已开(库请求交付字节); tcArm=等TC(末字节已交付);
     txDrFull=字节待上线(DR占用); tcSet=字节上线完成(TC挂起) */
  uint8_t  txIrqEn;
  uint8_t  tcArm;
  uint8_t  txDrFull;
  uint8_t  txDrByte;
  uint8_t  txSchedActive;  /* 已排定上线时刻的字节(发送方启动抖动) */
  double   txSchedAt;
  uint8_t  txSchedByte;
  uint8_t  wireActive;
  uint8_t  wireByte;
  double   wireEndAt;
  uint8_t  wireCollided;
  uint8_t  tcSet;
  double   lastTxEndAt;

  /* 接收投递队列: 按字节完成顺序入队, 到期按序投递 */
  rxEntry_t rxFifo[RX_FIFO_SIZE];
  uint16_t rxHead, rxCount;

  /* 打包模式: 攒包缓存(静默门限后整包突发上线) */
  uint16_t pktLen;
  double   pktFlushAt;
  uint8_t  burstPlaying;
  uint16_t burstIdx;
  uint8_t  pktBuf[PKT_BUF_SIZE];

  uint8_t  burstActive;
  uint8_t  burstCollided;
  uint16_t burstBytes;
  uint32_t burstHash;
  uint32_t hashPrev;
  uint8_t  burstFirst;
  uint8_t  burstLog[64];

  double   nextPollAt;
  uint16_t nextPollId;
  int      group;
  int      member;
  int      groupSize;
  double   relayInitAt;
  uint8_t  relayInitDone;
  double   relayStartAt;
  uint32_t relaySeq;
  uint32_t relayInFlightSeq;
  uint8_t  relayFwdPending;
  uint16_t relayFwdTarget;
  double   relayFwdAt;
  uint8_t  relayFwdPayload[8];
  pending_t pending[4];

  /* C: 2KB随机数据分片回声 */
  uint16_t echoTarget;         /* 本传输目标B设备 */
  uint16_t echoChunkIdx;       /* 下一个待发送分片(0..ECHO_CHUNKS-1) */
  uint32_t echoSeq;            /* 分片序号(1起) */
  uint32_t echoInFlightSeq;    /* 在途分片序号 */
  uint16_t echoNonce;          /* 重试nonce: 每帧/每次重试都不同, 绕过B去重表 */
  double   echoChunkSentAt;    /* 在途分片发送时刻 */
  double   echoTransferStartAt;
  double   echoNextAt;         /* 发起下一次传输的时刻 */
  uint8_t  echoTransferActive;
  uint8_t  echoData[ECHO_DATA_SIZE];

  stats_t st;
} dev_t;

static dev_t devs[MAX_DEV];
static int g_devCount = 6;
static double g_now = 0.0;            /* 当前仿真时间(ms) */
static double sim_now_ms(void) { return g_now; }
static uint32_t sim_now_us(void) { return (uint32_t)(g_now * 1000.0); }

static uint32_t g_lossPermille = 4;   /* 每字节丢失/损坏概率(千分比), 默认0.4% */
static int      g_packetMode = 0;     /* 0=不打包(逐字节投递) 1=打包(静默后整包转发) */
static double   g_pktIdleMs = DEFAULT_PKT_IDLE_MS;  /* 打包静默门限(ms) */
static uint64_t g_busyUs = 0, g_totalUs = 0;
static int g_trace = 0, g_traceDev = 0;
static int g_burstLog = 0;
static uint64_t g_rxDropBytes = 0;    /* 接收侧丢失字节总数(所有接收方累计) */
static uint64_t g_rxModBytes = 0;     /* 接收侧被改写字节总数 */
static uint64_t g_rxDeliveredBytes = 0;
static uint64_t g_rxOverflow = 0;
static uint64_t g_unicastTo[MAX_DEV], g_ackTo[MAX_DEV];
static uint64_t g_collFrameFrame = 0, g_collFrameAck = 0, g_collAckAck = 0;

/* ================= B 分组辅助 ================= */
static double rand_sub_byte_delay(void);

static int group_count(int nB)
{
  return (nB + RELAY_GROUP_BASE_SIZE - 1) / RELAY_GROUP_BASE_SIZE;
}

static int group_base(int g)
{
  return g * RELAY_GROUP_BASE_SIZE + 1;   /* 组内成员编号 1..group_size */
}

static int group_size(int g, int nB)
{
  int first = group_base(g);
  int last = group_base(g) + RELAY_GROUP_BASE_SIZE - 1;
  if(last > nB) last = nB;
  return last - first + 1;
}

/* ================= 平台钩子 ================= */
static void relay_receive(int n, uint16_t srcId, const uint8_t *data, uint16_t len);
static void relay_send_now(int n);

static int echo_chunk_len(int idx)
{
  int left = ECHO_DATA_SIZE - idx * ECHO_CHUNK_DATA;
  return (left > ECHO_CHUNK_DATA) ? ECHO_CHUNK_DATA : left;
}

/* Fletcher-16(与协议帧校验相同算法), 用于回声数据完整性校验 */
static uint16_t fletcher16(const uint8_t *p, uint16_t len)
{
  uint8_t s1 = 0, s2 = 0;
  for(uint16_t i = 0; i < len; ++i)
  {
    s1 = (uint8_t)(s1 + p[i]);
    s2 = (uint8_t)(s2 + s1);
  }
  return (uint16_t)(((uint16_t)s2 << 8) | s1);
}

void sim_tx_byte(gridTalk_t *gt, uint8_t c)
{
  int n = (int)(gt - gts);
  dev_t *d = &devs[n];
  if(g_packetMode)
  {
    /* 打包转发: 模块立即收下发来的字节写入包缓存(TXE不等待, 协议全速倾倒),
       静默门限内无新字节后由总线层整包突发上线 */
    if(d->pktLen < PKT_BUF_SIZE)
    {
      d->pktBuf[d->pktLen++] = c;
      d->pktFlushAt = g_now + g_pktIdleMs;
    }
    else
    {
      g_rxOverflow++;
    }
    return;
  }
  d->txDrFull = 1;
  d->txDrByte = c;
  d->tcSet = 0;
}

/* 发送中断控制钩子: 替代伪造寄存器的CR1位操作 */
static void sim_tx_kick(gridTalk_t *gt)       /* 开TXE中断 */
{
  devs[(int)(gt - gts)].txIrqEn = 1;
}

static void sim_tx_kick_done(gridTalk_t *gt)  /* 关TXE开TC(末字节已交付) */
{
  dev_t *d = &devs[(int)(gt - gts)];
  d->txIrqEn = 0;
  d->tcArm = 1;
}

static void sim_tx_abort(gridTalk_t *gt)      /* 卡死兜底: 关发送中断 */
{
  dev_t *d = &devs[(int)(gt - gts)];
  d->txIrqEn = 0;
  d->tcArm = 0;
}

/* ================= 应用层回调 ================= */
static void on_rx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                  const uint8_t *data, uint16_t len)
{
  int n = (int)(gt - gts);
  dev_t *d = &devs[n];

  if(cmd == 0x0100)
  {
    d->st.rxPing++;
    d->st.rxUnicast++;
    if(len > 4) len = 4;
    gridTalk_sendReply(gt, src, 0x0101, data, len);
  }
  else if(cmd == 0x0101)
  {
    uint32_t seq = 0;
    d->st.rxPong++;
    if(len >= 4)
    {
      seq = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
            ((uint32_t)data[2] << 8) | (uint32_t)data[3];
    }
    for(int k = 0; k < 4; ++k)
    {
      if(d->pending[k].active && d->pending[k].seq == seq)
      {
        double rtt = sim_now_ms() - d->pending[k].t0;
        d->pending[k].active = 0;
        d->st.pingDelivered++;
        d->st.rttN++;
        d->st.rttSum += rtt;
        if(rtt < d->st.rttMin) d->st.rttMin = rtt;
        if(rtt > d->st.rttMax) d->st.rttMax = rtt;
        samp_add(d->st.rttSamples, RTT_CAP, &d->st.rttCnt, rtt);
        break;
      }
    }
  }
  else if(cmd == 0x0200)
  {
    d->st.rxBcast++;
  }
  else if(cmd == 0x0401)
  {
    if(n == 0) return;
    uint8_t myId = (uint8_t)n;
    gridTalk_sendReply(gt, src, 0x0401 | 0x8000, &myId, 1);
  }
  else if(cmd == (0x0401 | 0x8000))
  {
    if(n == 0 && len >= 1)
    {
      d->st.pollResp++;
      if(!d->st.onlineSeen[data[0] & 127])
      {
        d->st.onlineSeen[data[0] & 127] = 1;
        d->st.onlineUnique++;
      }
    }
  }
  else if(cmd == 0x0300)
  {
    if(n == 0) return;
    d->st.relayRx++;
    if(len >= 8)
    {
      uint32_t t0 = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                    ((uint32_t)data[6] << 8) | (uint32_t)data[7];
      double hop = sim_now_ms() - (double)t0;
      if(hop >= 0.0)
      {
        d->st.hopLatN++;
        d->st.hopLatSum += hop;
        if(hop < d->st.hopLatMin) d->st.hopLatMin = hop;
        if(hop > d->st.hopLatMax) d->st.hopLatMax = hop;
      }
    }
    relay_receive(n, src, data, len);
  }
  else if(cmd == ECHO_CMD_DATA)
  {
    if(n == 0) return;
    d->st.echoRx++;
    gridTalk_sendReply(gt, src, ECHO_CMD_REPLY, data, len);
  }
  else if(cmd == ECHO_CMD_REPLY)
  {
    if(n != g_devCount - 1) return;   /* 只有C(最后一台)处理回声 */
    if(len < 8 || !d->echoTransferActive) return;
    uint32_t seq = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) | (uint32_t)data[3];
    if(seq != d->echoInFlightSeq) return;
    uint16_t ck = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
    (void)data[6]; (void)data[7];   /* nonce 仅用于绕过去重, 不参与校验 */
    int idx = (int)(seq - 1);
    int clen = echo_chunk_len(idx);
    uint16_t sentCk = fletcher16(&d->echoData[idx * ECHO_CHUNK_DATA], (uint16_t)clen);
    double rtt = sim_now_ms() - d->echoChunkSentAt;
    d->st.echoRttN++;
    d->st.echoRttSum += rtt;
    if(rtt < d->st.echoRttMin) d->st.echoRttMin = rtt;
    if(rtt > d->st.echoRttMax) d->st.echoRttMax = rtt;
    samp_add(d->st.echoRttSamples, d->st.echoRttCap, &d->st.echoRttCnt, rtt);
    if(ck != sentCk)
    {
      d->st.echoVerifyFail++;
      return;   /* 校验失败: 不推进分片, 让超时重发 */
    }
    d->st.echoBytesVerified += (uint64_t)clen;
    d->echoChunkIdx++;
    d->echoInFlightSeq = 0;
    if(d->echoChunkIdx >= ECHO_CHUNKS)
    {
      d->st.echoTransferOk++;
      d->echoTransferActive = 0;
      d->echoNextAt = sim_now_ms() + ECHO_TRANSFER_GAP_MS;
    }
  }
}

static void on_send_result(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok)
{
  int n = (int)(gt - gts);
  dev_t *d = &devs[n];

  if(cmd == 0x0300)
  {
    if(ok) d->st.relayTxResult++;
    else d->st.sendFail++;
  }
  else if(cmd == 0x0100)
  {
    if(ok)
    {
      for(int k = 0; k < 4; ++k)
      {
        if(d->pending[k].active && d->pending[k].target == target)
        {
          double lat = sim_now_ms() - d->pending[k].t0;
          d->st.ackN++;
          d->st.ackSum += lat;
          if(lat < d->st.ackMin) d->st.ackMin = lat;
          if(lat > d->st.ackMax) d->st.ackMax = lat;
          samp_add(d->st.ackSamples, ACK_CAP, &d->st.ackCnt, lat);
          break;
        }
      }
      d->st.sendOk++;
    }
    else
    {
      d->st.sendFail++;
    }
  }
  else if(cmd == ECHO_CMD_DATA)
  {
    if(ok) d->st.echoTxOk++;
    else d->st.sendFail++;
  }
}

/* ================= B中继: 收到->延迟1s->触发转发 ================= */
static void relay_receive(int n, uint16_t srcId, const uint8_t *data, uint16_t len)
{
  dev_t *d = &devs[n];
  int g = d->group;
  int m = d->member;
  int size = d->groupSize;
  int nextIdx;

  if(g < 0) return;
  if(len > 8) len = 8;
  for(int k = 0; k < 8; ++k) d->relayFwdPayload[k] = (k < len) ? data[k] : 0;

  if(m == 0 && srcId == (uint16_t)(group_base(g) + size - 1))
  {
    uint32_t seq = ((uint32_t)d->relayFwdPayload[0] << 24) |
                   ((uint32_t)d->relayFwdPayload[1] << 16) |
                   ((uint32_t)d->relayFwdPayload[2] << 8) |
                   (uint32_t)d->relayFwdPayload[3];
    if(seq == d->relayInFlightSeq)
    {
      double lap = sim_now_ms() - d->relayStartAt;
      d->st.relayLaps++;
      d->st.lapN++;
      d->st.lapSum += lap;
      if(lap < d->st.lapMin) d->st.lapMin = lap;
      if(lap > d->st.lapMax) d->st.lapMax = lap;
    }
    d->relaySeq++;
    d->relayInFlightSeq = d->relaySeq;
    d->relayStartAt = sim_now_ms();
    d->relayFwdPayload[0] = (uint8_t)(d->relaySeq >> 24);
    d->relayFwdPayload[1] = (uint8_t)(d->relaySeq >> 16);
    d->relayFwdPayload[2] = (uint8_t)(d->relaySeq >> 8);
    d->relayFwdPayload[3] = (uint8_t)(d->relaySeq >> 0);
  }

  nextIdx = group_base(g) + ((m + 1) % size);
  d->relayFwdTarget = (uint16_t)nextIdx;
  d->relayFwdAt = sim_now_ms() + RELAY_FWD_DELAY_MS;
  d->relayFwdPending = 1;
}

static void relay_send_now(int n)
{
  dev_t *d = &devs[n];
  uint32_t t0 = (uint32_t)sim_now_ms();
  d->relayFwdPayload[4] = (uint8_t)(t0 >> 24);
  d->relayFwdPayload[5] = (uint8_t)(t0 >> 16);
  d->relayFwdPayload[6] = (uint8_t)(t0 >> 8);
  d->relayFwdPayload[7] = (uint8_t)(t0 >> 0);
  gridTalk_sendReply(&gts[n], d->relayFwdTarget, 0x0300, d->relayFwdPayload, 8);
  d->relayFwdPending = 0;
}

/* ================= 周期任务 ================= */
static void tick_one(int i)
{
  gridTalk_usTick(&gts[i], g_dtUs);

#if GT_TX_IRQ
  /* 仿真UART发送中断: TXE事件(DR空)逐字节交付, TC事件(移位完成)帧收尾 */
  if(devs[i].txIrqEn && gts[i].txRingCount > 0 && !devs[i].txDrFull)
  {
    gridTalk_txIrq(&gts[i]);
  }
  if(devs[i].tcArm && devs[i].tcSet && gts[i].txRingCount == 0)
  {
    gridTalk_txCompleteIrq(&gts[i]);
  }
  /* TC标志确认: 帧已离开发送态则视为软件已清除(含卡死兜底收尾路径) */
  if(devs[i].tcSet && gts[i].txState != GT_TX_SENDING) devs[i].tcSet = 0;
#endif
}

static void app_schedule_one(int i)
{
  dev_t *d = &devs[i];

  if(i == 0)
  {
    if(g_now >= d->nextPollAt && gridTalk_pending(&gts[0]) < 4)
    {
      uint16_t pollId = d->nextPollId;
      gridTalk_sendNoAck(&gts[0], pollId, 0x0401, NULL, 0);
      d->st.pollSent++;
      d->nextPollId = (uint16_t)((pollId == 255) ? 1 : pollId + 1);
      if(d->nextPollId == 1) d->st.pollCycles++;
      d->nextPollAt = g_now + 100.0;  /* 后台扫描: 每100ms探测一个ID */
    }
    return;
  }

  if(i == g_devCount - 1)
  {
    /* C: 随机目标 -> 2KB分片回声传输 */
    if(!d->echoTransferActive && g_now >= d->echoNextAt)
    {
      d->echoTarget = (uint16_t)(1 + rng_next() % (uint32_t)(g_devCount - 2));
      for(int k = 0; k < ECHO_DATA_SIZE; ++k) d->echoData[k] = (uint8_t)rng_next();
      d->echoChunkIdx = 0;
      d->echoSeq = 0;
      d->echoTransferActive = 1;
      d->echoTransferStartAt = g_now;
    }

    if(d->echoTransferActive && gridTalk_pending(&gts[i]) < 4)
    {
      if(!d->echoInFlightSeq)
      {
        /* 发送下一个未确认分片 */
        if(d->echoChunkIdx < ECHO_CHUNKS)
        {
          int idx = (int)d->echoChunkIdx;
          int clen = echo_chunk_len(idx);
          uint8_t payload[ECHO_CHUNK_DATA + 8];
          uint32_t seq = (uint32_t)idx + 1;
          payload[0] = (uint8_t)(seq >> 24);
          payload[1] = (uint8_t)(seq >> 16);
          payload[2] = (uint8_t)(seq >> 8);
          payload[3] = (uint8_t)(seq >> 0);
          uint16_t ck = fletcher16(&d->echoData[idx * ECHO_CHUNK_DATA], (uint16_t)clen);
          payload[4] = (uint8_t)(ck >> 8);
          payload[5] = (uint8_t)(ck >> 0);
          d->echoNonce = (uint16_t)rng_next();
          payload[6] = (uint8_t)(d->echoNonce >> 8);
          payload[7] = (uint8_t)(d->echoNonce >> 0);
          memcpy(&payload[8], &d->echoData[idx * ECHO_CHUNK_DATA], (size_t)clen);
          gridTalk_send(&gts[i], d->echoTarget, ECHO_CMD_DATA, payload, (uint16_t)(clen + 8));
          d->echoSeq = seq;
          d->echoInFlightSeq = seq;
          d->echoChunkSentAt = g_now;
          d->st.echoBytesSent += (uint64_t)clen;
        }
      }
      else if(g_now - d->echoChunkSentAt > ECHO_TXN_WINDOW_MS)
      {
        /* 回声事务超时: 同seq新nonce重发, 绕过接收端去重表 */
        int idx = (int)(d->echoInFlightSeq - 1);
        int clen = echo_chunk_len(idx);
        uint8_t payload[ECHO_CHUNK_DATA + 8];
        uint32_t seq = d->echoInFlightSeq;
        payload[0] = (uint8_t)(seq >> 24);
        payload[1] = (uint8_t)(seq >> 16);
        payload[2] = (uint8_t)(seq >> 8);
        payload[3] = (uint8_t)(seq >> 0);
        uint16_t ck = fletcher16(&d->echoData[idx * ECHO_CHUNK_DATA], (uint16_t)clen);
        payload[4] = (uint8_t)(ck >> 8);
        payload[5] = (uint8_t)(ck >> 0);
        d->echoNonce = (uint16_t)rng_next();
        payload[6] = (uint8_t)(d->echoNonce >> 8);
        payload[7] = (uint8_t)(d->echoNonce >> 0);
        memcpy(&payload[8], &d->echoData[idx * ECHO_CHUNK_DATA], (size_t)clen);
        gridTalk_send(&gts[i], d->echoTarget, ECHO_CMD_DATA, payload, (uint16_t)(clen + 8));
        d->echoChunkSentAt = g_now;
        d->st.echoTxnTimeout++;
      }
    }
    return;
  }

  if(d->group >= 0 && d->member == 0 && !d->relayInitDone && g_now >= d->relayInitAt)
  {
    d->relayInitDone = 1;
    d->relaySeq = 1;
    d->relayInFlightSeq = 1;
    d->relayStartAt = g_now;
    {
      uint8_t payload[8] = { 0, 0, 0, 1, 0, 0, 0, 0 };
      uint32_t t0 = (uint32_t)g_now;
      payload[4] = (uint8_t)(t0 >> 24);
      payload[5] = (uint8_t)(t0 >> 16);
      payload[6] = (uint8_t)(t0 >> 8);
      payload[7] = (uint8_t)(t0 >> 0);
      gridTalk_sendReply(&gts[i], (uint16_t)(group_base(d->group) + 1), 0x0300, payload, 8);
    }
  }

  /* 令牌丢失恢复: 组长发现环静默超过阈值则重发令牌(序号继续递增) */
  if(d->group >= 0 && d->member == 0 && d->relayInitDone &&
     !d->relayFwdPending && g_now - d->relayStartAt > RELAY_REINIT_MS)
  {
    uint8_t payload[8];
    uint32_t t0 = (uint32_t)g_now;
    d->relaySeq++;
    d->relayInFlightSeq = d->relaySeq;
    d->relayStartAt = g_now;
    payload[0] = (uint8_t)(d->relaySeq >> 24);
    payload[1] = (uint8_t)(d->relaySeq >> 16);
    payload[2] = (uint8_t)(d->relaySeq >> 8);
    payload[3] = (uint8_t)(d->relaySeq >> 0);
    payload[4] = (uint8_t)(t0 >> 24);
    payload[5] = (uint8_t)(t0 >> 16);
    payload[6] = (uint8_t)(t0 >> 8);
    payload[7] = (uint8_t)(t0 >> 0);
    gridTalk_sendReply(&gts[i], (uint16_t)(group_base(d->group) + 1), 0x0300, payload, 8);
  }

  if(d->relayFwdPending && g_now >= d->relayFwdAt)
  {
    relay_send_now(i);
  }
}

/* ================= 初始化 ================= */
static void init_all(void)
{
  for(int i = 0; i < MAX_DEV; ++i)
  {
    memset(&devs[i], 0, sizeof(devs[i]));
    stats_init(&devs[i].st);
    devs[i].lastTxEndAt = -1e9;
  }
  for(int i = 0; i < g_devCount; ++i)
  {
    devs[i].id = (uint16_t)i;
    gridTalk_init(&gts[i], NULL, (uint16_t)i);
    gts[i].nowUs = sim_now_us;
    gts[i].txByte = sim_tx_byte;
    gts[i].txKick = sim_tx_kick;
    gts[i].txKickDone = sim_tx_kick_done;
    gts[i].txAbort = sim_tx_abort;
    gts[i].onRx = on_rx;
    gts[i].onSendResult = on_send_result;
    gts[i].idleGuardUs = g_idleGuardUs;   /* 空闲判定保障窗口(打包模式消除攒包侦听盲区) */
    gts[i].txDelayUs = g_txDelayUs;       /* ACK窗口按 2*txDelayUs 放宽(打包转发往返延迟) */
    devs[i].tickPhase = rng_next() % g_stepsPerMs;
    devs[i].rxDelay = rand_sub_byte_delay();   /* 本节点固有接收延时(0~1字节时间) */
    devs[i].group = (i >= 1 && i < g_devCount) ? (i - 1) / RELAY_GROUP_BASE_SIZE : -1;
    devs[i].member = (i >= 1 && i < g_devCount) ? (i - 1) % RELAY_GROUP_BASE_SIZE : -1;
    devs[i].groupSize = (devs[i].group >= 0) ? group_size(devs[i].group, g_devCount - 1) : 0;
    devs[i].relayInitAt = (double)devs[i].group * RELAY_GROUP_STAGGER_MS;
    devs[i].nextPollAt = 50.0;
    devs[i].nextPollId = 1;
    if(i == g_devCount - 1 && g_devCount > 2)
    {
      devs[i].echoNextAt = ECHO_START_DELAY_MS;
      devs[i].echoTransferActive = 0;
    }
  }
}

/* ================= 帧(burst)识别 ================= */
static void begin_burst(dev_t *d, double t)
{
  if(d->burstActive)
  {
    if(g_burstLog)
    {
      printf("[t=%.1f] dev%u burst len=%u first=0x%02X hash=%08X prev=%08X bytes=",
             t, (unsigned)(d - devs) + 1, d->burstBytes, d->burstFirst,
             d->burstHash, d->hashPrev);
      for(uint16_t i = 0; i < d->burstBytes && i < 24; ++i)
        printf("%02X ", d->burstLog[i]);
      printf("\n");
    }
    if(d->burstBytes > 6)
    {
      d->st.wireFrames++;
      if(d->burstLog[2] != 0xFF || d->burstLog[3] != 0xFF)
      {
        uint8_t dst = d->burstLog[3];
        if(dst >= 1 && dst <= MAX_DEV) g_unicastTo[dst - 1]++;
      }
      if(d->burstCollided) d->st.collidedBursts++;
      if(d->hashPrev == d->burstHash) d->st.retransmits++;
    }
    else if(d->burstBytes == 6 && d->burstFirst == 0xFF && d->burstLog[1] == 0xFC)
    {
      /* V2.19起ACK为6字节: FF FC + 被确认方(2) + 确认方(2) */
      d->st.wireAcks++;
      if(d->burstLog[3] >= 1 && d->burstLog[3] <= MAX_DEV) g_ackTo[d->burstLog[3] - 1]++;
    }
    d->hashPrev = d->burstHash;
  }
  else
  {
    d->hashPrev = 0;
  }

  d->burstActive = 1;
  d->burstCollided = 0;
  d->burstBytes = 0;
  d->burstHash = 0;
  d->burstFirst = 0xFF;
}

static void end_burst(dev_t *d)
{
  if(!d->burstActive) return;
  if(d->burstBytes > 6)
  {
    d->st.wireFrames++;
    if(d->burstLog[2] != 0xFF || d->burstLog[3] != 0xFF)
    {
      uint8_t dst = d->burstLog[3];
      if(dst >= 1 && dst <= MAX_DEV) g_unicastTo[dst - 1]++;
    }
    if(d->burstCollided) d->st.collidedBursts++;
    if(d->hashPrev == d->burstHash) d->st.retransmits++;
  }
  else if(d->burstBytes == 6 && d->burstFirst == 0xFF && d->burstLog[1] == 0xFC)
  {
    d->st.wireAcks++;
    if(d->burstLog[3] >= 1 && d->burstLog[3] <= MAX_DEV) g_ackTo[d->burstLog[3] - 1]++;
  }
  d->burstActive = 0;
}

static uint32_t hash_byte(uint32_t h, uint8_t b)
{
  h = (h ^ b) * 16777619u;
  return h;
}

/* ================= 总线微步 ================= */

/* 每字节上线/投递的随机小延时: 0~1字节时间(模拟UART启动/采样抖动) */
static double rand_sub_byte_delay(void)
{
  uint32_t steps = (uint32_t)(g_byteTimeMs / g_dtMs + 0.5);
  if(steps < 1) steps = 1;
  return (double)(rng_next() % steps) * g_dtMs;
}

/* 发送排队: DR有字节且当前无在途/待上线字节时, 排定上线时刻.
   仅帧首字节(总线空闲超过帧间隙)附加随机启动抖动; 帧内后续字节
   由UART硬件连续移位, 不加抖动, 保证帧内字节间隔=字节时间. */
static void tx_sched(dev_t *d, double t)
{
  if(d->txDrFull && !d->txSchedActive && !d->wireActive)
  {
    double delay = (t - d->lastTxEndAt > g_burstGapMs) ? rand_sub_byte_delay() : 0.0;
    d->txSchedActive = 1;
    d->txSchedAt = t + delay;
    d->txSchedByte = d->txDrByte;
  }
}

/* 字节上线: 到达排定时刻即占用总线; 上线瞬间与其他在途字节重叠即碰撞 */
static void tx_activate(int i, double t)
{
  dev_t *d = &devs[i];
  int collide = 0;
  int j;

  if(!d->txSchedActive || t < d->txSchedAt || d->wireActive) return;

  for(j = 0; j < g_devCount; ++j)
    if(j != i && devs[j].wireActive) { collide = 1; break; }

  d->wireActive = 1;
  d->wireByte = d->txSchedByte;
  d->wireEndAt = t + g_byteTimeMs;
  d->wireCollided = 0;
  d->txSchedActive = 0;
  d->txDrFull = 0;

  if(t - d->lastTxEndAt > g_burstGapMs) begin_burst(d, t);
  d->burstBytes++;
  d->burstHash = hash_byte(d->burstHash, d->wireByte);
  if(d->burstBytes <= 64) d->burstLog[d->burstBytes - 1] = d->wireByte;
  if(d->burstBytes == 1) d->burstFirst = d->wireByte;

  if(collide)
  {
    d->wireCollided = 1;
    d->burstCollided = 1;
    d->st.collisions++;
    {
      uint8_t thisIsAck = (d->burstBytes >= 2 &&
                           d->burstLog[0] == 0xFF && d->burstLog[1] == 0xFC);
      int otherIsAck = 0;
      for(j = 0; j < g_devCount; ++j)
      {
        if(j != i && devs[j].wireActive)
        {
          if(devs[j].burstBytes >= 2 &&
             devs[j].burstLog[0] == 0xFF && devs[j].burstLog[1] == 0xFC) otherIsAck = 1;
        }
      }
      if(thisIsAck && otherIsAck) g_collAckAck++;
      else if(thisIsAck || otherIsAck) g_collFrameAck++;
      else g_collFrameFrame++;
      if(g_trace)
      {
        printf("[t=%.1f] COLL dev%u(%s len=%u) vs", t, (unsigned)(d - devs) + 1,
               thisIsAck ? "ACK" : "FRAME", d->burstBytes);
        for(j = 0; j < g_devCount; ++j)
        {
          if(j != i && devs[j].wireActive)
            printf(" dev%u(%s len=%u)", j + 1,
                   (devs[j].burstBytes >= 2 &&
                    devs[j].burstLog[0] == 0xFF && devs[j].burstLog[1] == 0xFC) ? "ACK" : "FRAME",
                   devs[j].burstBytes);
        }
        printf("\n");
      }
    }
    for(j = 0; j < g_devCount; ++j)
    {
      if(j != i && devs[j].wireActive)
      {
        devs[j].wireCollided = 1;
        devs[j].burstCollided = 1;
      }
    }
  }
}

/* 接收投递队列: 按字节完成顺序入队, 保持投递顺序 */
static void rx_push(dev_t *d, uint8_t b, double due)
{
  uint16_t slot;
  if(d->rxCount >= RX_FIFO_SIZE)
  {
    g_rxOverflow++;
    return;
  }
  slot = (uint16_t)((d->rxHead + d->rxCount) % RX_FIFO_SIZE);
  d->rxFifo[slot].b = b;
  d->rxFifo[slot].due = due;
  d->rxCount++;
}

/* 字节完成: 释放总线, 给每个接收方(不含发送方)按其固有延时排定投递.
   两种转发方式的接收侧相同: 突发的字节连续到达, 接收方收到的即完整一包 */
static void wire_complete(int i, double t)
{
  dev_t *d = &devs[i];

  if(!d->wireActive || t < d->wireEndAt) return;
  d->lastTxEndAt = t;
  d->wireActive = 0;
  if(!d->txDrFull) d->tcSet = 1;

  for(int k = 0; k < g_devCount; ++k)
  {
    if(k == i) continue;                    /* 不回投给发送方 */
    if(d->wireCollided)
    {
      /* 碰撞: 接收方采样到随机噪声 */
      rx_push(&devs[k], (uint8_t)(rng_next() & 0xFFu), t + devs[k].rxDelay);
    }
    else
    {
      rx_push(&devs[k], d->wireByte, t + devs[k].rxDelay);
    }
  }
}

/* 接收投递: 到期投递, 每字节独立按概率丢失或单比特翻转 */
static void rx_deliver(double t)
{
  for(int i = 0; i < g_devCount; ++i)
  {
    dev_t *d = &devs[i];
    while(d->rxCount && d->rxFifo[d->rxHead].due <= t)
    {
      uint8_t b = d->rxFifo[d->rxHead].b;
      d->rxHead = (uint16_t)((d->rxHead + 1) % RX_FIFO_SIZE);
      d->rxCount--;
      if((rng_next() % 1000u) < g_lossPermille)
      {
        if(rng_next() & 1u)
        {
          g_rxDropBytes++;            /* 字节丢失 */
          continue;
        }
        b = (uint8_t)(b ^ (1u << (rng_next() & 7u)));   /* 单比特翻转 */
        g_rxModBytes++;
      }
      g_rxDeliveredBytes++;
      if(g_trace && i == g_traceDev) printf("[t=%.1f] rx dev%d 0x%02X\n", t, i + 1, b);
      gridTalk_rxByte(&gts[i], b);
    }
  }
}

static void microstep(double t)
{
  int i;
  int busy = 0;

  for(i = 0; i < g_devCount; ++i)
  {
    tx_sched(&devs[i], t);
    tx_activate(i, t);
  }

  /* 打包模式: 攒包静默到期→整包上线; 在途包按字节节拍连续排上线 */
  if(g_packetMode)
  {
    for(i = 0; i < g_devCount; ++i)
    {
      dev_t *d = &devs[i];
      if(!d->burstPlaying && d->pktLen > 0 && t >= d->pktFlushAt &&
         !d->wireActive && !d->txSchedActive)
      {
        d->burstPlaying = 1;
        d->burstIdx = 0;
      }
      if(d->burstPlaying && !d->wireActive && !d->txSchedActive)
      {
        if(d->burstIdx >= d->pktLen)
        {
          d->burstPlaying = 0;   /* 整包上线完毕 */
          d->pktLen = 0;
        }
        else
        {
          /* 首字节(与上一包间隔>帧间隙)保留启动抖动; 包内字节按节拍连续上线 */
          double delay = (t - d->lastTxEndAt > g_burstGapMs) ? rand_sub_byte_delay() : 0.0;
          d->txSchedActive = 1;
          d->txSchedAt = t + delay;
          d->txSchedByte = d->pktBuf[d->burstIdx++];
        }
      }
    }
  }

  for(i = 0; i < g_devCount; ++i)
  {
    wire_complete(i, t);
  }

  /* 补排: 帧内后续字节在本拍紧跟上一字节上线(UART连续移位, 无帧内间隙) */
  for(i = 0; i < g_devCount; ++i)
  {
    tx_sched(&devs[i], t);
    tx_activate(i, t);
    if(devs[i].wireActive) busy = 1;
  }

  rx_deliver(t);

  if(busy) g_busyUs++;
  g_totalUs++;
}

/* ================= 报告 ================= */
static void print_report(double duration, uint32_t lossPermille, int nDev, uint32_t seed)
{
  uint64_t totFrames = 0, totCollide = 0, totRetx = 0;
  uint64_t totAcks = 0, totSendOk = 0, totSendFail = 0, totColl = 0;
  uint64_t totRelayTx = 0, totRelayRx = 0;
  double ackSumAll = 0; uint32_t ackAll = 0;

  for(int i = 0; i < g_devCount; ++i)
  {
    stats_t *s = &devs[i].st;
    totFrames += s->wireFrames;
    totCollide += s->collidedBursts; totRetx += s->retransmits;
    totAcks += s->wireAcks; totSendOk += s->sendOk; totSendFail += s->sendFail;
    totColl += s->collisions;
    totRelayTx += s->relayTxResult; totRelayRx += s->relayRx;
    ackSumAll += s->ackSum; ackAll += s->ackN;
  }

  printf("\n================================================================\n");
  printf("GridTalk 485 半双工总线仿真报告(结构体多实例版)\n");
  printf("================================================================\n");
  printf("配置   : 波特率=%u(字节时间%.3fms), 每字节丢失/损坏率=%.1f%%, 设备数=%d, 仿真时长=%.1fs, 种子=%u\n",
         (unsigned)g_baud, g_byteTimeMs, (double)lossPermille / 10.0, nDev, duration, seed);
  if(g_packetMode)
  {
    printf("模型   : 端到端整包转发(攒包门限%.1fms, 到期整包突发上线), 空闲保障%.0fms, 帧首字节随机启动抖动, 接收侧节点固有延时, 无自回显, 重叠即碰撞\n",
           g_pktIdleMs, (double)g_idleGuardUs / 1000.0);
  }
  else
  {
    printf("模型   : 逐字节转发, 空闲保障%.0fms, 帧首字节随机启动抖动(0~1字节时间), 接收侧节点固有延时, 无自回显, 重叠即碰撞\n",
           (double)g_idleGuardUs / 1000.0);
  }
  printf("----------------------------------------------------------------\n");
  printf("总线   : 利用率=%.2f%%, 字节级碰撞=%llu次\n",
         (double)g_busyUs * 100.0 / (g_totalUs ? g_totalUs : 1), (unsigned long long)totColl);
  printf("碰撞   : 帧vs帧=%llu, 帧vs应答=%llu, 应答vs应答=%llu\n",
         (unsigned long long)g_collFrameFrame,
         (unsigned long long)g_collFrameAck,
         (unsigned long long)g_collAckAck);
  printf("误码   : 交付字节=%llu, 丢失=%llu, 改写=%llu (合计%.3f%%), 队列溢出=%llu\n",
         (unsigned long long)g_rxDeliveredBytes,
         (unsigned long long)g_rxDropBytes,
         (unsigned long long)g_rxModBytes,
         (g_rxDeliveredBytes + g_rxDropBytes) ?
           (double)(g_rxDropBytes + g_rxModBytes) * 100.0 / (g_rxDeliveredBytes + g_rxDropBytes) : 0.0,
         (unsigned long long)g_rxOverflow);
  printf("帧     : 线上帧数(含重发)=%llu, 碰撞损坏=%llu(%.2f%%), 重发帧=%llu(%.2f%%)\n",
         (unsigned long long)totFrames,
         (unsigned long long)totCollide, totFrames ? (double)totCollide * 100.0 / totFrames : 0.0,
         (unsigned long long)totRetx, totFrames ? (double)totRetx * 100.0 / totFrames : 0.0);
  printf("应答   : 线上应答FF FC=%llu\n", (unsigned long long)totAcks);
  {
    double buMin = 1e300, buMax = 0, buSum = 0; int buN = 0;
    for(int i = 0; i < g_devCount; ++i)
    {
      uint32_t bu = gridTalk_byteUs(&gts[i]);
      if(bu)
      {
        buSum += bu; buN++;
        if(bu < buMin) buMin = bu;
        if(bu > buMax) buMax = bu;
      }
    }
    printf("自适应 : 实测单字节时间=%.0fus (min %.0f / max %.0f), 理论=%uus\n",
           buN ? buSum / buN : 0.0, buMin, buMax, (unsigned)(g_byteTimeMs * 1000.0));
  }

  {
    stats_t *a = &devs[0].st;
    printf("A轮询  : 探测=%llu次, 回应=%llu次, 在线唯一=%llu台, 完成轮询周期=%llu\n",
           (unsigned long long)a->pollSent, (unsigned long long)a->pollResp,
           (unsigned long long)a->onlineUnique, (unsigned long long)a->pollCycles);
    printf("A未发现: ");
    int miss = 0;
    for(int k = 1; k < g_devCount; ++k)
    {
      if(!a->onlineSeen[k & 127]) { printf("%d ", k); miss = 1; }
    }
    if(!miss) printf("(无, %d台全部发现)", g_devCount - 1);
    printf("\n");
  }

  {
    int nB = g_devCount - 1;
    int nGroups = group_count(nB);
    uint64_t totHops = 0, totLaps = 0;
    double lapSum = 0; uint32_t lapN = 0;
    double totHsum = 0, totHmin = 1e300, totHmax = 0; uint32_t totHn = 0;
    printf("B中继  : 组数=%d, 组(成员数) | 转发帧 | 完整圈数 | 圈耗时avg | 单跳延迟avg/min/max\n", nGroups);
    for(int g = 0; g < nGroups; ++g)
    {
      int size = group_size(g, nB);
      uint64_t hops = 0, laps = 0;
      double gsum = 0; uint32_t gn = 0;
      double hsum = 0, hmin = 1e300, hmax = 0; uint32_t hn = 0;
      for(int m = 0; m < size; ++m)
      {
        stats_t *s = &devs[group_base(g) + m].st;
        hops += s->relayRx; laps += s->relayLaps;
        gsum += s->lapSum; gn += s->lapN;
        hsum += s->hopLatSum; hn += s->hopLatN;
        if(s->hopLatN && s->hopLatMin < hmin) hmin = s->hopLatMin;
        if(s->hopLatMax > hmax) hmax = s->hopLatMax;
      }
      totHops += hops; totLaps += laps;
      lapSum += gsum; lapN += gn;
      totHsum += hsum; totHn += hn;
      if(hn && hmin < totHmin) totHmin = hmin;
      if(hmax > totHmax) totHmax = hmax;
      printf("  组%2d(%d) | %7llu | %6llu | %8.0f ms | %.1f/%.1f/%.1f ms\n", g + 1, size,
             (unsigned long long)hops, (unsigned long long)laps,
             gn ? gsum / gn : 0.0,
             hn ? hsum / hn : 0.0, hn ? hmin : 0.0, hn ? hmax : 0.0);
    }
    printf("  合计  | %7llu | %6llu | 圈均 %.0f ms (%.1f 圈/s) | 单跳 %.1f/%.1f/%.1f ms\n",
           (unsigned long long)totHops, (unsigned long long)totLaps,
           lapN ? lapSum / lapN : 0.0,
           duration > 0 ? (double)totLaps / duration : 0.0,
           totHn ? totHsum / totHn : 0.0,
           totHn ? totHmin : 0.0, totHn ? totHmax : 0.0);
  }

  if(g_devCount > 2)
  {
    stats_t *c = &devs[g_devCount - 1].st;
    uint64_t bEchoRx = 0;
    for(int i = 1; i < g_devCount - 1; ++i) bEchoRx += devs[i].st.echoRx;
    double dur = duration > 0 ? duration : 1.0;
    double thru = c->echoBytesVerified / dur;
    printf("C回声  : 2KB随机数据分片=%d片/帧(净荷%dB), 帧上限=%d\n",
           ECHO_CHUNKS, ECHO_CHUNK_DATA, GT_TX_FRAME_MAX - 10);
    printf("  传输  : 完成=%llu次, 数据字节已校验=%llu/%lluB, 校验失败=%llu, 有效吞吐=%.1f B/s\n",
           (unsigned long long)c->echoTransferOk,
           (unsigned long long)c->echoBytesVerified,
           (unsigned long long)c->echoBytesSent,
           (unsigned long long)c->echoVerifyFail, thru);
    printf("  回声  : B收分片=%llu, C数据帧收到ACK=%llu, 事务超时重发=%llu, 分片RTT avg=%.1fms (min %.1f / max %.1f)\n",
           (unsigned long long)bEchoRx, (unsigned long long)c->echoTxOk,
           (unsigned long long)c->echoTxnTimeout,
           c->echoRttN ? c->echoRttSum / c->echoRttN : 0.0,
           c->echoRttN ? c->echoRttMin : 0.0,
           c->echoRttN ? c->echoRttMax : 0.0);
  }

  printf("可靠   : 中继转发收到ACK=%llu, 组内中继帧(接收)=%llu, 发送失败=%llu\n",
         (unsigned long long)totRelayTx, (unsigned long long)totRelayRx,
         (unsigned long long)totSendFail);
  if(ackAll)
  {
    printf("延迟   : ping ACK延迟(发送完成) avg=%.1fms\n", ackSumAll / ackAll);
  }
  printf("----------------------------------------------------------------\n");
  printf("设备明细(每台):\n");
  printf(" ID | 线上帧 | 重发 | 转发ACK | 收单播 | 单跳min/avg/max\n");
  for(int i = 0; i < g_devCount; ++i)
  {
    stats_t *s = &devs[i].st;
    printf("%3d | %6llu | %4llu | %6llu | %6llu | %.1f/%.1f/%.1f\n", i,
           (unsigned long long)s->wireFrames, (unsigned long long)s->retransmits,
           (unsigned long long)s->relayTxResult, (unsigned long long)s->rxUnicast,
           s->hopLatN ? s->hopLatMin : 0.0,
           s->hopLatN ? s->hopLatSum / s->hopLatN : 0.0,
           s->hopLatN ? s->hopLatMax : 0.0);
  }
  printf("================================================================\n");
}

/* ================= 主循环 ================= */
int main(int argc, char **argv)
{
  double duration = 120.0;
  uint32_t lossPermille = 4;
  int nDev = 6;
  uint32_t seed = 42;

  if(argc > 1) duration = atof(argv[1]);
  if(argc > 2) lossPermille = (uint32_t)(atof(argv[2]) * 10.0 + 0.5);
  if(argc > 3) nDev = atoi(argv[3]);
  if(argc > 4) seed = (uint32_t)strtoul(argv[4], NULL, 0);
  if(argc > 5) g_baud = (uint32_t)strtoul(argv[5], NULL, 0);
  if(argc > 6) g_packetMode = atoi(argv[6]) ? 1 : 0;
  if(argc > 7) g_pktIdleMs = atof(argv[7]);
  if(argc > 8) g_idleGuardMs = atof(argv[8]);
  if(nDev < 1) nDev = 1;
  if(nDev > MAX_DEV) nDev = MAX_DEV;
  if(lossPermille > 1000) lossPermille = 1000;
  if(g_baud < 300) g_baud = 300;
  if(g_baud > 1000000u) g_baud = 1000000u;

  /* 时序推导: 字节时间=10bit/波特率, 微步=字节时间/10 */
  g_byteTimeMs = 10000.0 / (double)g_baud;
  g_dtMs = g_byteTimeMs / 10.0;
  if(g_dtMs < 0.001) g_dtMs = 0.001;
  g_dtUs = (uint32_t)(g_dtMs * 1000.0 + 0.5);
  if(g_dtUs < 1) g_dtUs = 1;
  g_stepsPerMs = (uint32_t)(1.0 / g_dtMs + 0.5);   /* 每毫秒微步数 */
  if(g_stepsPerMs < 1) g_stepsPerMs = 1;
  g_burstGapMs = 2.0 * g_byteTimeMs;
  if(g_pktIdleMs < 2.0 * g_byteTimeMs) g_pktIdleMs = 2.0 * g_byteTimeMs;   /* 门限不小于帧间隙, 保证整帧攒入同包 */

  /* 空闲保障窗口(随机化接入范围): 打包模式下攒包期间总线静默不可侦听(盲区=门限),
     自动取 门限; argv[8]>=0 显式覆盖(0=无保障), 逐字节转发默认0 */
  if(g_idleGuardMs < 0.0)
  {
    g_idleGuardMs = g_packetMode ? g_pktIdleMs : 0.0;
  }
  if(g_idleGuardMs < 0.0) g_idleGuardMs = 0.0;
  g_idleGuardUs = (uint32_t)(g_idleGuardMs * 1000.0 + 0.5);
  /* 发送侧传输延迟上界: 攒包门限+最大帧空中时间, 协议ACK窗口按往返(2x)放宽 */
  g_txDelayUs = g_packetMode ?
                (uint32_t)((g_pktIdleMs + g_byteTimeMs * GT_TX_FRAME_MAX) * 1000.0 + 0.5) : 0u;

  g_devCount = nDev;
  g_lossPermille = lossPermille;
  rng_seed(seed);
  if(getenv("GRIDTALK_TRACE")) g_trace = 1;
  if(getenv("GRIDTALK_BURSTLOG")) g_burstLog = 1;
  if(getenv("GRIDTALK_TRACE_DEV")) g_traceDev = atoi(getenv("GRIDTALK_TRACE_DEV"));
  init_all();

  long totalSteps = (long)(duration * 1000.0 / g_dtMs);
  for(long step = 0; step < totalSteps; ++step)
  {
    g_now = (double)step * g_dtMs;
    for(int i = 0; i < g_devCount; ++i)
    {
      tick_one(i);
      if((step + devs[i].tickPhase) % g_stepsPerMs == 0)
      {
        app_schedule_one(i);
      }
    }
    microstep(g_now);
  }
  for(int i = 0; i < g_devCount; ++i) end_burst(&devs[i]);

  /* 收尾: 让在途攒包上线、突发播完、接收队列排空, 保证统计完整 */
  for(long guard = 0; guard < 4000000L; ++guard)
  {
    int pending = 0;
    for(int i = 0; i < g_devCount; ++i)
    {
      pending += devs[i].rxCount + devs[i].pktLen + devs[i].burstPlaying +
                 devs[i].wireActive + devs[i].txSchedActive;
    }
    if(!pending) break;
    g_now += g_dtMs;
    microstep(g_now);
  }

  print_report(duration, lossPermille, nDev, seed);
  return 0;
}
