/*
 * test_multich.c -- gridTalk_sendMulti / gridTalk_linkSeq 多信道冗余语义单元测试
 * 编译运行: 见文件底部注释; 依赖 GridTalk.c(与 build_run.ps1 一并构建)
 *
 * 验证点:
 *  A. sendMulti 两信道副本带同一序号, 线上字节完全一致;
 *  B. 接收端 linkSeq 并域后, 跨信道副本被判重发只ACK不执行(单次执行);
 *  C. 迟到孪生帧(新命令已执行后旧副本才到)被"落后"分支吞掉, 不误执行;
 *  D. 对照组: 不并域的接收端两副本各执行一次(证明测试台真实投递了两份);
 *  E. ACK 跨信道独立: 各信道副本的ACK分别清各自发送实例的WAIT_ACK。
 */
#include "GridTalk.h"
#include <stdio.h>
#include <string.h>

/* ---- 假时钟(µs, 全局共享) ---- */
static uint32_t g_fakeUs = 1000;
static uint32_t fake_now_us(void) { return g_fakeUs; }
#define BYTE_US 1042u

/* ---- 信道: 线上字节采集(发送侧经txByte钩子写入, 替代伪造寄存器) ---- */
typedef struct {
  uint8_t  wire[4096];
  uint16_t wireLen;
  uint8_t  txIrqEn;        /* TXE中断已开(库请求交付字节) */
  uint8_t  tcArm;          /* 等TC(末字节已交付) */
  uint8_t  lastByte;       /* 最近一次txByte交付的字节(DR内容) */
} chan_t;

static void chan_init(chan_t *ch)
{
  memset(ch, 0, sizeof(*ch));
}

/* 发送侧钩子(实例经hw取回信道) */
static void ch_tx_byte(gridTalk_t *gt, uint8_t c)
{
  ((chan_t *)gt->hw)->lastByte = c;
}

static void ch_tx_kick(gridTalk_t *gt)
{
  ((chan_t *)gt->hw)->txIrqEn = 1;
}

static void ch_tx_kick_done(gridTalk_t *gt)
{
  chan_t *ch = (chan_t *)gt->hw;
  ch->txIrqEn = 0;
  ch->tcArm = 1;
}

static void ch_tx_abort(gridTalk_t *gt)
{
  chan_t *ch = (chan_t *)gt->hw;
  ch->txIrqEn = 0;
  ch->tcArm = 0;
}

/* 驱动实例把txRing发完(TXE逐字节+TC收尾), 采集交付字节为线上字节.
   每次调用采集一帧(调用前重置), 调用方需在帧启动后一次性泵完 */
static int g_pumpTxEvents = 0;   /* 累计"泵出过字节"的次数(≈上线次数, H用例) */
static void chan_pump(gridTalk_t *gt, chan_t *ch)
{
  int guard = 0;
  ch->wireLen = 0;   /* 每次泵送=采集当前这一帧, 不跨帧累积 */
  while(guard++ < 8192 && gt->txState == GT_TX_SENDING)
  {
    if(ch->txIrqEn && gt->txRingCount > 0)
    {
      gridTalk_txIrq(gt);   /* 字节经txByte钩子写入lastByte */
      ch->wire[ch->wireLen++] = ch->lastByte;
      if(ch->wireLen == 1) g_pumpTxEvents++;   /* 本次泵送产出首字节=一次上线 */
      g_fakeUs += BYTE_US;
    }
    else if(ch->tcArm && gt->txRingCount == 0)
    {
      ch->tcArm = 0;   /* TC已确认 */
      gridTalk_txCompleteIrq(gt);   /* 帧完成(离开SENDING) */
      g_fakeUs += BYTE_US;
    }
    else break;
  }
}

/* 驱动状态机直到进入SENDING(覆盖 ACCESS_WAIT 随机退避/等ACK超时重发) */
static void run_until_sending(gridTalk_t *gt, const char *tag)
{
  int guard = 0;
  while(gt->txState != GT_TX_SENDING && guard++ < 200000)
  {
    g_fakeUs += 1000;
    gridTalk_usTick(gt, 0);
  }
  if(gt->txState != GT_TX_SENDING)
  {
    printf("FAIL: %s 始终未进入SENDING(state=%d)\n", tag, (int)gt->txState);
    exit(1);
  }
}

/* 按线上字节节奏喂给接收实例 */
static void feed(gridTalk_t *gt, const uint8_t *bytes, uint16_t n)
{
  for(uint16_t i = 0; i < n; ++i)
  {
    g_fakeUs += BYTE_US;
    gridTalk_rxByte(gt, bytes[i]);
  }
}

/* ---- 接收回调计数 ---- */
static int rxCount = 0;
static uint16_t rxLastCmd = 0;
static void on_rx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                  const uint8_t *d, uint16_t l)
{
  (void)gt; (void)src; (void)d; (void)l;
  rxCount++;
  rxLastCmd = cmd;
}

static void gt_setup(gridTalk_t *gt, chan_t *ch, uint16_t id)
{
  gridTalk_init(gt, ch, id);   /* hw=信道, 发送钩子经hw取回 */
  gt->nowUs = fake_now_us;
  gt->txByte = ch_tx_byte;
  gt->txKick = ch_tx_kick;
  gt->txKickDone = ch_tx_kick_done;
  gt->txAbort = ch_tx_abort;
  gt->onRx = on_rx;
}

/* 手工构造一帧线上字节(dst/src/seq/cmd避开0xFF字节, 无需转义) */
static uint16_t build_frame(uint8_t *out, uint16_t dst, uint16_t src,
                            uint16_t seq, uint16_t cmd)
{
  uint8_t p[8];
  p[0]=dst>>8; p[1]=dst&0xFF; p[2]=src>>8; p[3]=src&0xFF;
  p[4]=seq>>8; p[5]=seq&0xFF; p[6]=cmd>>8; p[7]=cmd&0xFF;
  uint16_t s1=0,s2=0;
  for(int i=0;i<8;++i){ s1+=p[i]; if(s1>=255)s1-=255;
                        s2+=s1;    if(s2>=255)s2-=255; }
  out[0]=0xFF; out[1]=0x00;
  for(int i=0;i<8;++i) out[2+i]=p[i];
  out[10]=(uint8_t)s1; out[11]=(uint8_t)s2; out[12]=0xFF; out[13]=0xFE;
  return 14;
}

static int failures = 0;
#define CHECK(cond, msg) do { \
  if(cond) printf("PASS: %s\n", msg); \
  else { printf("FAIL: %s\n", msg); failures++; } } while(0)

static int g_localSendFail = 0;
static void on_send_fail(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok)
{
  (void)gt; (void)target; (void)cmd;
  if(!ok) g_localSendFail++;
}

int main(void)
{
  /* ===== 发送方A(id=1): 两信道实例并域; 接收方B(id=2): 两信道实例并域 ===== */
  chan_t chA1, chA2, chB1, chB2;
  gridTalk_t A1, A2, B1, B2;
  gt_setup(&A1, &chA1, 1);
  gt_setup(&A2, &chA2, 1);
  gt_setup(&B1, &chB1, 2);
  gt_setup(&B2, &chB2, 2);
  gridTalk_linkSeq(&A1, &A2);   /* 发送侧同源序号 */
  gridTalk_linkSeq(&B1, &B2);   /* 接收侧共享去重表 */

  /* ---- A. sendMulti副本同序号, 线上字节一致 ---- */
  gridTalk_t *gts[2] = {&A1, &A2};
  uint8_t payload = 'X';
  gridTalk_sendMulti(gts, 2, 2, 0x0100, &payload, 1);
  run_until_sending(&A1, "A1#1"); chan_pump(&A1, &chA1);
  run_until_sending(&A2, "A2#1"); chan_pump(&A2, &chA2);
  CHECK(A1.txCurrent.seq == A2.txCurrent.seq, "A. 两信道副本带同一序号");
  CHECK(chA1.wireLen == chA2.wireLen && memcmp(chA1.wire, chA2.wire, chA1.wireLen) == 0,
        "A. 两信道线上字节完全一致");

  /* ---- B. 跨信道副本只执行一次 ---- */
  int before = rxCount;
  feed(&B1, chA1.wire, chA1.wireLen);            /* 信道1副本先到: 执行 */
  run_until_sending(&B1, "B1 ACK#1"); chan_pump(&B1, &chB1);
  feed(&A1, chB1.wire, chB1.wireLen);            /* ACK回信道1发送实例 */
  feed(&B2, chA2.wire, chA2.wireLen);            /* 信道2副本后到: 应被吞 */
  run_until_sending(&B2, "B2 ACK#1"); chan_pump(&B2, &chB2);
  feed(&A2, chB2.wire, chB2.wireLen);            /* ACK回信道2发送实例 */
  CHECK(rxCount - before == 1, "B. 并域接收端: 跨信道副本单次执行");
  CHECK(chB1.wireLen >= 4 && chB1.wire[0] == 0xFF && chB1.wire[1] == 0xFC &&
        chB2.wireLen >= 4 && chB2.wire[0] == 0xFF && chB2.wire[1] == 0xFC,
        "E. 两信道各自回了L2 ACK(与执行无关, 先于去重)");
  CHECK(A1.txFifoCount == 0 && A2.txFifoCount == 0,
        "E. 各信道的ACK分别销掉各自发送实例的在途帧(FIFO清空)");
  CHECK(rxLastCmd == (0x0100), "B. 执行的是命令本身");

  /* ---- C. 新命令已执行后, 旧命令的孪生副本迟到 -> 落后分支吞掉 ---- */
  before = rxCount;
  gridTalk_sendMulti(gts, 2, 2, 0x0101, NULL, 0);          /* 序号 S2 */
  run_until_sending(&A1, "A1#2"); chan_pump(&A1, &chA1);
  run_until_sending(&A2, "A2#2"); chan_pump(&A2, &chA2);
  uint16_t s2 = (uint16_t)(A1.txSeqCounter - 1u);   /* 域计数回读(重试不改号, txCurrent可能是重试帧) */
  uint8_t twin2[512]; uint16_t twin2Len = chA2.wireLen;    /* 保存S2的信道2副本 */
  memcpy(twin2, chA2.wire, twin2Len);
  feed(&B1, chA1.wire, chA1.wireLen);                      /* 信道1: S2 执行 */
  run_until_sending(&B1, "B1 ACK#2"); chan_pump(&B1, &chB1);
  feed(&A1, chB1.wire, chB1.wireLen);

  gridTalk_sendMulti(gts, 2, 2, 0x0102, NULL, 0);          /* 序号 S3 */
  run_until_sending(&A1, "A1#3"); chan_pump(&A1, &chA1);
  run_until_sending(&A2, "A2#3"); chan_pump(&A2, &chA2);
  uint16_t s3 = (uint16_t)(A1.txSeqCounter - 1u);
  CHECK((uint16_t)(s3 - s2) == 1, "C. 并域后连续sendMulti序号严格+1");

  feed(&B1, chA1.wire, chA1.wireLen);                      /* 信道1: S3 执行 */
  run_until_sending(&B1, "B1 ACK#3"); chan_pump(&B1, &chB1);
  feed(&A1, chB1.wire, chB1.wireLen);
  feed(&B2, chA2.wire, chA2.wireLen);                      /* 信道2: S3孪生(delta=0) -> 吞 */
  run_until_sending(&B2, "B2 ACK#3"); chan_pump(&B2, &chB2);
  feed(&A2, chB2.wire, chB2.wireLen);
  feed(&B2, twin2, twin2Len);                              /* 信道2: S2迟到(落后1) -> 吞 */
  run_until_sending(&B2, "B2 ACK#2late"); chan_pump(&B2, &chB2);
  feed(&A2, chB2.wire, chB2.wireLen);
  CHECK(rxCount - before == 2, "C. 两条逻辑命令共执行2次(孪生/迟到副本全被吞)");

  /* ---- D. 对照组: 不并域的接收端两副本各执行一次(旧双发行为) ---- */
  chan_t chU1, chU2;
  gridTalk_t U1, U2;
  gt_setup(&U1, &chU1, 2);
  gt_setup(&U2, &chU2, 2);
  gridTalk_linkSeq(&A1, &A2);   /* 发送侧仍并域(同序号双发) */
  before = rxCount;
  gridTalk_sendMulti(gts, 2, 2, 0x0103, NULL, 0);
  run_until_sending(&A1, "A1#4"); chan_pump(&A1, &chA1);
  run_until_sending(&A2, "A2#4"); chan_pump(&A2, &chA2);
  feed(&U1, chA1.wire, chA1.wireLen);
  feed(&U2, chA2.wire, chA2.wireLen);
  CHECK(rxCount - before == 2, "D. 对照: 未并域接收端执行2次(跨信道不去重)");

  /* ---- F. 失步兜底防幻影(V2.17): 陈旧重试流不得执行; 递增落后流仍重同步 ---- */
  {
    chan_t chF; gridTalk_t F1;
    gt_setup(&F1, &chF, 2);          /* 单实例接收端, 直喂手工帧 */
    uint8_t f[16]; uint16_t n;
    before = rxCount;
    n = build_frame(f, 2, 1, 100, 0x0100); feed(&F1, f, n);   /* seq100: 首帧执行 */
    CHECK(rxCount - before == 1, "F1. 基线: 首帧执行");
    n = build_frame(f, 2, 1, 101, 0x0100); feed(&F1, f, n);   /* seq101: 新命令执行, 表=101 */
    CHECK(rxCount - before == 2, "F2. 基线: 递增新命令执行");
    /* 陈旧重试流: seq99/97交替×12帧(模拟经另一信道已执行之帧的本地重发+重发)
       —— 旧逻辑8连落后即强制执行(幻影波根源), 新逻辑必须全部吞掉 */
    before = rxCount;
    for(int i = 0; i < 12; ++i)
    {
      n = build_frame(f, 2, 1, (i % 2) ? 97 : 99, 0x0100); feed(&F1, f, n);
    }
    CHECK(rxCount - before == 0, "F3. 陈旧重发流(交替落后序号×12)全部吞并, 零执行");
    /* 同号重复重发×10: 同样不得执行 */
    for(int i = 0; i < 10; ++i)
    {
      n = build_frame(f, 2, 1, 99, 0x0100); feed(&F1, f, n);
    }
    CHECK(rxCount - before == 0, "F4. 同号重发×10全部吞并");
    /* 真失步(对端计数器回绕/复位到窗口内更低处, 序号持续递增的落后流):
       第8条仍须重同步执行(兜底保留). 注: 落后超过16会走FAR分支立即执行,
       那是重启换基值的另一条路径, 不经此兜底 */
    for(int i = 0; i < 8; ++i)
    {
      n = build_frame(f, 2, 1, 90 + i, 0x0100); feed(&F1, f, n);
    }
    CHECK(rxCount - before == 1, "F5. 递增落后流第8条强制重同步执行(兜底保留)");
  }

  /* ---- G. FIFO陈帧丢弃(V2.18): 超龄帧不上线, 直接丢弃并回报失败 ---- */
  {
    chan_t chG; gridTalk_t G1;
    gt_setup(&G1, &chG, 2);
    G1.ackWaitMinUs = 30000u;   /* 快信道参数, 缩短等待 */
    G1.onSendResult = on_send_fail;
    uint8_t pld = 'G';
    int f0 = g_localSendFail;
    /* 总线"恒忙": 持续喂字节让busIdle永不成立, 帧滞留FIFO超过GT_TX_STALE_MS */
    for(int ms = 0; ms < 5500; ++ms)
    {
      g_fakeUs += 1000;
      gridTalk_usTick(&G1, 0);
      if(ms % 3 == 0) gridTalk_rxByte(&G1, 0x55);   /* 噪声字节: 打断帧起始, 维持总线"忙" */
      if(ms == 100) gridTalk_send(&G1, 3, 0x0100, &pld, 1);   /* 入队一帧 */
    }
    CHECK(G1.txFifoCount == 0 && G1.txState == GT_TX_IDLE,
          "G1. 超龄帧被丢弃, 队列与状态机归零(未上线)");
    CHECK(g_localSendFail - f0 >= 1, "G2. 丢弃回报onSendResult失败");
    /* 对照: 信道恢复空闲后, 新帧正常上线 */
    int sent0 = (int)chG.wireLen;
    gridTalk_send(&G1, 3, 0x0100, &pld, 1);
    run_until_sending(&G1, "G3"); chan_pump(&G1, &chG);
    CHECK(chG.wireLen > sent0, "G3. 信道空闲后新帧正常上线(陈帧机制不误伤)");
  }

  /* ---- H. 流水线发送(V2.19): 在途等ACK不占信道, 多帧连续上线 ---- */
  {
    chan_t chH; gridTalk_t H1;
    gt_setup(&H1, &chH, 2);   /* ackWaitMinUs=默认250ms */
    uint8_t pld = 'H';
    int ev0 = g_pumpTxEvents;
    /* 4帧needAck发往不存在目标(无ACKer): 流水线下350ms内4帧应全部上线
       (各自背避0~96ms+帧15ms); 旧串行逻辑此时第1帧仍在WAIT_ACK,
       后3帧被队头阻塞, 上线次数≈1 */
    for(int k = 0; k < 4; ++k)
      gridTalk_send(&H1, 50 + k, 0x0100, &pld, 1);
    for(int ms = 0; ms < 350; ++ms)
    {
      g_fakeUs += 1000;
      gridTalk_usTick(&H1, 0);
      chan_pump(&H1, &chH);
    }
    CHECK(g_pumpTxEvents - ev0 >= 4,
          "H1. 无ACK应答时4帧在350ms内全部上线(在途等ACK不阻塞信道)");
    CHECK(H1.txFifoCount == 4,
          "H2. 4帧保留在途(未被丢弃), 信道不为其等待");
  }

  printf("------------------------------------------------\n");
  printf(failures ? "结果: %d 项FAIL\n" : "结果: 全部PASS (%d项FAIL)\n", failures);
  return failures ? 1 : 0;
}
/*
 * 编译运行(PowerShell, 本目录):
 *   $vs=& 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -latest -property installationPath
 *   cmd /c "call `"$vs\VC\Auxiliary\Build\vcvars64.bat`" >nul & cl /nologo /utf-8 /I. /I..\..\MX_autoMata\Core\GridTalk test_multich.c gridtalk.obj /Fe:test_multich.exe & test_multich.exe"
 * (gridtalk.obj 由 build_run.ps1 生成)
 */
