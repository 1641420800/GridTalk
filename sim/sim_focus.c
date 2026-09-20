/*
 * GridTalk 聚焦测试(2节点): 复现遥控器->控制器 与 0x07自指 的真实收发路径
 *
 * 场景A: t=50ms  节点0(遥控器,id=0) 单播 0x0100 到 节点1(控制器,id=1)
 *        预期: 节点1 onRx 收到 0x0100(src=0), 节点0 发送结果OK(收到ACK)
 * 场景B: t=2500ms 节点1 执行"0x07发启动信号给自身"(gridTalk_send(1,0x0100))
 *        预期(按当前实现): 自身回显被gtTxBusy过滤, onRx 收不到 src=1 的 0x0100
 * 场景C: t=5000ms 节点1 执行"0x08发停止信号给自身"(gridTalk_send(1,0x0101))
 *        预期: 同样收不到
 * 场景E: t=10000ms 节点1 执行"0x0B发复位信号给2号"(gridTalk_send(2,0x0104))
 * 场景F: t=13000ms 节点1 执行"0x0C发急停信号给2号"(gridTalk_send(2,0x0101))
 * 场景G: t=16000ms 节点1 执行"0x0D发恢复给2号"(gridTalk_send(2,0x0102))
 *
 * 编译(与build_run.ps1同一方式, 需GridTalk.c同目录):
 *   cl /nologo /std:c11 /W3 /O2 /utf-8 /I. /I..\..\MX_autoMata\Core\GridTalk ^
 *      /c ..\..\MX_autoMata\Core\GridTalk\GridTalk.c /Fogridtalk_focus.obj
 *   cl /nologo /std:c11 /W3 /O2 /utf-8 /I. /I..\..\MX_autoMata\Core\GridTalk ^
 *      sim_focus.c gridtalk_focus.obj /Fesim_focus.exe
 */
#include "GridTalk.h"
#include <stdio.h>
#include <string.h>

#define NDEV 3
#define SIM_US      10
#define DT_MS       0.1
#define DT_US       100
#define BYTE_TIME   1.0417

static gridTalk_t gts[NDEV];

typedef struct {
  uint16_t id;
  uint8_t  tickPhase;

  /* 仿真UART发送侧状态(经txKick/txKickDone/txAbort钩子置位, 替代伪造寄存器) */
  uint8_t  txIrqEn;        /* TXE中断已开(库请求交付字节) */
  uint8_t  tcArm;          /* 等TC(末字节已交付) */
  uint8_t  txDrFull;
  uint8_t  txDrByte;
  uint8_t  wireActive;
  uint8_t  wireByte;
  double   wireEndAt;
  uint8_t  tcSet;

  /* 收发包统计 */
  uint64_t rx0100_from0;
  uint64_t rx0100_from1;
  uint64_t rx0101_from1;
  uint64_t rx0104_from1;   /* 节点2: 收到1号发来的复位信号 */
  uint64_t rx0101_from_1;  /* 节点2: 收到1号发来的急停 */
  uint64_t rx0102_from_1;  /* 节点2: 收到1号发来的恢复 */
  uint64_t send0100Ok;
  uint64_t send0100Fail;
  uint64_t send0101Ok;
  uint64_t send0101Fail;
  uint64_t send0101To2Ok;    /* 1号->2号 急停发送结果 */
  uint64_t send0101To2Fail;
  uint64_t send0104Ok;
  uint64_t send0104Fail;
  uint64_t send0102Ok;
  uint64_t send0102Fail;
  uint64_t sentStart;
  uint64_t sentSelfStart;
  uint64_t sentSelfStop;
  uint64_t sentTo2Reset;
  uint64_t sentTo2Stop;
  uint64_t sentTo2Recover;
  uint64_t sentSelfFixed;
  uint64_t rx0100_selfFixed;   /* 场景D: 采用修复方案(本地投递)后的自触发计数 */
} dev_t;

static dev_t devs[NDEV];
static double g_now = 0.0;
static uint32_t sim_now_us(void) { return (uint32_t)(g_now * 1000.0); }

static int g_trace = 0;

/* ================= 平台钩子 ================= */
static void sim_tx_byte(gridTalk_t *gt, uint8_t c)
{
  int n = (int)(gt - gts);
  dev_t *d = &devs[n];
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

static void on_rx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                  const uint8_t *data, uint16_t len)
{
  int n = (int)(gt - gts);
  dev_t *d = &devs[n];
  (void)data; (void)len;

  if(n == 1)
  {
    if(cmd == 0x0100 && src == 0) d->rx0100_from0++;
    if(cmd == 0x0100 && src == 1) d->rx0100_from1++;
    if(cmd == 0x0101 && src == 1) d->rx0101_from1++;
  }
  if(n == 2)
  {
    if(cmd == 0x0104 && src == 1) d->rx0104_from1++;
    if(cmd == 0x0101 && src == 1) d->rx0101_from_1++;
    if(cmd == 0x0102 && src == 1) d->rx0102_from_1++;
  }
  if(n == 2 && (cmd == 0x0104 || cmd == 0x0101 || cmd == 0x0102) && src == 1)
  {
    /* 模拟真实控制器: 收到信号指令后回 "OK" (0x8301) */
    gridTalk_send(gt, src, 0x0301 | 0x8000, (const uint8_t*)"OK", 2);
  }
  if(g_trace)
    printf("[t=%.1fms] dev%d RX cmd=0x%04X src=%u\n", g_now, n, cmd, src);

  /* 模拟控制器收到启动指令后的行为: 投递队列 + 回"OK" */
  if(n == 1 && cmd == 0x0100)
  {
    gridTalk_send(gt, src, 0x0301 | 0x8000, (const uint8_t*)"OK", 2);
  }
}

/* 模拟修复方案: execute_signal 检测目标==本机时, 本地投递启动信号(不入队发送) */
static void local_deliver_start(int n, uint16_t cmd)
{
  dev_t *d = &devs[n];
  if(cmd == 0x0100)
  {
    d->rx0100_selfFixed++;
    /* 旧版485单字节启动行为: 队列投递 + 软件使能 */
    /* osMessagePut(Queue_signalHandle, 0x0100); Emm_V5_En_Control(1,1,0); */
  }
}

static void on_send_result(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok)
{
  int n = (int)(gt - gts);
  dev_t *d = &devs[n];
  printf("[t=%.1fms] dev%d SEND_RESULT target=%u cmd=0x%04X %s\n",
         g_now, n, target, cmd, ok ? "OK" : "FAIL");
  if(cmd == 0x0100)
  {
    if(ok) d->send0100Ok++; else d->send0100Fail++;
  }
  if(cmd == 0x0101)
  {
    if(ok) d->send0101Ok++; else d->send0101Fail++;
    if(n == 1 && target == 2) { if(ok) d->send0101To2Ok++; else d->send0101To2Fail++; }
  }
  if(cmd == 0x0104)
  {
    if(ok) d->send0104Ok++; else d->send0104Fail++;
  }
  if(cmd == 0x0102)
  {
    if(ok) d->send0102Ok++; else d->send0102Fail++;
  }
  if(g_trace)
    printf("[t=%.1fms] dev%d SEND_RESULT cmd=0x%04X %s\n", g_now, n, cmd,
           ok ? "OK" : "FAIL");
}

/* ================= 周期任务(与sim.c同构) ================= */
static void tick_one(int i)
{
  gridTalk_usTick(&gts[i], DT_US);

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
  if(devs[i].tcSet && gts[i].txState != GT_TX_SENDING) devs[i].tcSet = 0;
#endif
}

static void rx_all(uint8_t b)
{
  for(int i = 0; i < NDEV; ++i)
  {
    gridTalk_rxByte(&gts[i], b);
  }
}

static void microstep(double t)
{
  int i;
  for(i = 0; i < NDEV; ++i)
  {
    dev_t *d = &devs[i];
    if(d->txDrFull && !d->wireActive)
    {
      d->wireActive = 1;
      d->wireByte = d->txDrByte;
      d->wireEndAt = t + BYTE_TIME;
      d->txDrFull = 0;
    }
  }
  for(i = 0; i < NDEV; ++i)
  {
    dev_t *d = &devs[i];
    if(d->wireActive && t >= d->wireEndAt)
    {
      d->wireActive = 0;
      if(!d->txDrFull) d->tcSet = 1;
      rx_all(d->wireByte);
    }
  }
}

/* ================= 场景调度 ================= */
static void app_schedule(int i)
{
  dev_t *d = &devs[i];
  if(i == 0 && !d->sentStart && g_now >= 50.0)
  {
    d->sentStart = 1;
    printf("[t=%.1fms] 场景A: 遥控器(dev0) -> 控制器(dev1) 发送 0x0100 启动\n", g_now);
    gridTalk_send(&gts[0], 1, 0x0100, NULL, 0);
  }
  if(i == 1 && !d->sentSelfStart && g_now >= 2500.0)
  {
    d->sentSelfStart = 1;
    printf("[t=%.1fms] 场景B: 控制器(dev1) 执行0x07发启动信号给自身 -> 0x0100\n", g_now);
    gridTalk_send(&gts[1], 1, 0x0100, NULL, 0);
  }
  if(i == 1 && !d->sentSelfStop && g_now >= 5000.0)
  {
    d->sentSelfStop = 1;
    printf("[t=%.1fms] 场景C: 控制器(dev1) 执行0x08发停止信号给自身 -> 0x0101\n", g_now);
    gridTalk_send(&gts[1], 1, 0x0101, NULL, 0);
  }
  if(i == 1 && !d->sentSelfFixed && g_now >= 7500.0)
  {
    d->sentSelfFixed = 1;
    printf("[t=%.1fms] 场景D: 修复后 0x07发启动给自身 (发信道上行+本地投递)\n", g_now);
    gridTalk_send(&gts[1], 1, 0x0100, NULL, 0);
    if(1 == devs[1].id) local_deliver_start(1, 0x0100);
  }
  if(i == 1 && !d->sentTo2Reset && g_now >= 10000.0)
  {
    d->sentTo2Reset = 1;
    printf("[t=%.1fms] 场景E: 1号 -> 2号 发送 0x0104 复位信号\n", g_now);
    gridTalk_send(&gts[1], 2, 0x0104, NULL, 0);
  }
  if(i == 1 && !d->sentTo2Stop && g_now >= 13000.0)
  {
    d->sentTo2Stop = 1;
    printf("[t=%.1fms] 场景F: 1号 -> 2号 发送 0x0101 急停信号\n", g_now);
    gridTalk_send(&gts[1], 2, 0x0101, NULL, 0);
  }
  if(i == 1 && !d->sentTo2Recover && g_now >= 16000.0)
  {
    d->sentTo2Recover = 1;
    printf("[t=%.1fms] 场景G: 1号 -> 2号 发送 0x0102 恢复信号\n", g_now);
    gridTalk_send(&gts[1], 2, 0x0102, NULL, 0);
  }
}

static void init_all(void)
{
  for(int i = 0; i < NDEV; ++i)
  {
    memset(&devs[i], 0, sizeof(devs[i]));
    devs[i].id = (uint16_t)i;
    gridTalk_init(&gts[i], NULL, (uint16_t)i);
    gts[i].nowUs = sim_now_us;
    gts[i].txByte = sim_tx_byte;
    gts[i].txKick = sim_tx_kick;
    gts[i].txKickDone = sim_tx_kick_done;
    gts[i].txAbort = sim_tx_abort;
    gts[i].onRx = on_rx;
    gts[i].onSendResult = on_send_result;
    devs[i].tickPhase = (uint8_t)(i * 3 % SIM_US);
  }
}

int main(int argc, char **argv)
{
  double duration = 8000.0;
  if(argc > 1) duration = atof(argv[1]);
  if(getenv("GRIDTALK_TRACE")) g_trace = 1;

  init_all();
  long totalSteps = (long)(duration / DT_MS);
  for(long step = 0; step < totalSteps; ++step)
  {
    g_now = (double)step * DT_MS;
    for(int i = 0; i < NDEV; ++i)
    {
      tick_one(i);
      if((step + devs[i].tickPhase) % SIM_US == 0)
      {
        app_schedule(i);
      }
    }
    microstep(g_now);
  }

  printf("\n================ 聚焦测试结果 ================\n");
  printf("场景A 遥控器0x0100->控制器: onRx(src=0)=%llu  遥控器发送结果 OK=%llu FAIL=%llu\n",
         (unsigned long long)devs[1].rx0100_from0,
         (unsigned long long)devs[0].send0100Ok,
         (unsigned long long)devs[0].send0100Fail);
  printf("场景B 控制器0x07发启动给自身: onRx(src=1,0x0100)=%llu  (0=自触发失效)\n",
         (unsigned long long)devs[1].rx0100_from1);
  printf("场景C 控制器0x08发停止给自身: onRx(src=1,0x0101)=%llu  (0=自停止失效)\n",
         (unsigned long long)devs[1].rx0101_from1);
  printf("场景D 修复后自触发: 本地投递=%llu (1=有效)\n",
         (unsigned long long)devs[1].rx0100_selfFixed);
  printf("场景E 1->2 复位0x0104: 2号onRx=%llu, 1号发送结果 OK=%llu FAIL=%llu\n",
         (unsigned long long)devs[2].rx0104_from1,
         (unsigned long long)devs[1].send0104Ok,
         (unsigned long long)devs[1].send0104Fail);
  printf("场景F 1->2 急停0x0101: 2号onRx=%llu, 1号发送结果 OK=%llu FAIL=%llu\n",
         (unsigned long long)devs[2].rx0101_from_1,
         (unsigned long long)devs[1].send0101Ok,
         (unsigned long long)devs[1].send0101Fail);
  printf("场景G 1->2 恢复0x0102: 2号onRx=%llu, 1号发送结果 OK=%llu FAIL=%llu\n",
         (unsigned long long)devs[2].rx0102_from_1,
         (unsigned long long)devs[1].send0102Ok,
         (unsigned long long)devs[1].send0102Fail);
  printf("控制器发送结果: 0x0100 OK=%llu FAIL=%llu, 0x0101 OK=%llu FAIL=%llu\n",
         (unsigned long long)devs[1].send0100Ok,
         (unsigned long long)devs[1].send0100Fail,
         (unsigned long long)devs[1].send0101Ok,
         (unsigned long long)devs[1].send0101Fail);
  printf("自适应单字节时间: dev0=%uus dev1=%uus (9600bps理论1042us)\n",
         (unsigned)gridTalk_byteUs(&gts[0]), (unsigned)gridTalk_byteUs(&gts[1]));
  printf("===============================================\n");
  return 0;
}
