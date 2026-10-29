/* nowTick 换算累加器回归测试: 虚拟自由计数器 + 非整除计数率, 扫描 2^32 回绕。
 *
 * 覆盖病灶: 旧做法"读数/tickPerUs 再相减"(当年 CYCCNT/100)在 2^32 非整除时,
 * 计数回绕后差值读出 ~2^32 量级假值 -> 状态机假超时(级联环信号静默死亡)。
 * 独立真值模型: 自绑定时刻起的总增量 totalTicks(64位), 精确µs = totalTicks/
 * tickPerUs(长除法), 与库内 _nowUs (mod 2^32) 逐步比对; 并断言"先除后减"
 * 模型在回绕处确实失配(证明用例仍覆盖病灶)。
 *
 * 编译(MSVC, 参照 build_run.ps1 的环境):
 *   cl /nologo /std:c11 /W3 /O2 /utf-8 /I. /I..\src test_tickwrap.c gridtalk.obj
 */
#include <stdio.h>
#include "../src/GridTalk.h"

static uint32_t virtTick;             /* 虚拟自由计数器(uint32自然回绕) */
static uint32_t testTick(void){ return virtTick; }

static uint32_t fakeUsVal;            /* 差值回绕一致的假µs源(nowUs路径用) */
static uint32_t fakeUs(void){ return fakeUsVal; }

static uint32_t rngSt = 0x12345678u;
static uint32_t rng(void)
{
  uint32_t x = rngSt;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  rngSt = x;
  return x;
}

static int fails = 0;

static void expectUs(gridTalk_t *gt, uint64_t totalTicks, uint32_t tickPerUs,
                     const char *phase, long step)
{
  uint32_t expect = (uint32_t)((totalTicks / tickPerUs) & 0xFFFFFFFFu);
  if(gt->_nowUs != expect)
  {
    printf("FAIL[%s] step=%ld: _nowUs=%lu expect=%lu totalTicks=%llu\n",
           phase, step, (unsigned long)gt->_nowUs, (unsigned long)expect,
           (unsigned long long)totalTicks);
    fails++;
  }
}

int main(void)
{
  gridTalk_t gt;
  uint64_t total;
  long i;

  /* A: tickPerUs=3(2^32%3=1, 截断损失最大), 紧贴回绕点前起步细步跨过2^32 */
  gridTalk_init(&gt, NULL, 1);
  gt.nowTick = testTick;
  gt.tickPerUs = 3;
  virtTick = 0xFFFFF000u;
  gt._lastTick = virtTick;            /* 模拟绑定时与当前计数对齐 */
  total = 0;                          /* 真值: 自绑定时刻起的增量 */
  for(i = 0; i < 4096; i++)
  {
    virtTick += 3;
    total += 3;
    gridTalk_usTick(&gt, 1000);       /* us参数在nowTick路径必须被忽略 */
    expectUs(&gt, total, 3, "wrap", i);
  }
  /* 对照: "先除后减"旧模型此刻必须失配(否则用例不再覆盖病灶, 需检查) */
  {
    uint32_t naive = (virtTick / 3u) - (0xFFFFF000u / 3u);  /* 各自换算再相减 */
    uint32_t truth = (uint32_t)((total / 3u) & 0xFFFFFFFFu);
    if(naive == truth)
      printf("NOTE: naive divide-then-subtract matches -- case no longer covers the bug?\n");
    else
      printf("ok: naive model diverges at wrap (naive=%lu truth=%lu)\n",
             (unsigned long)naive, (unsigned long)truth);
  }

  /* B: 随机步长长跑, 跨越多次回绕; 中途穿插发送路径读时钟(txByte=NULL, 帧滞留队列) */
  gridTalk_init(&gt, NULL, 1);
  gt.nowTick = testTick;
  gt.tickPerUs = 3;
  virtTick = rng();
  gt._lastTick = virtTick;
  total = 0;
  for(i = 0; i < 2000000; i++)
  {
    uint32_t step = rng() % 20000u;
    if(step == 0) step = 1;
    virtTick += step;
    total += step;
    if((i & 4095) == 0)
      gridTalk_sendNoAck(&gt, 2, 0x0100, NULL, 0);
    gridTalk_usTick(&gt, 1000);
    expectUs(&gt, total, 3, "rand", i);
  }
  printf("ok: random walk covered %.1f counter wraps\n",
         (double)total / 4294967296.0);

  /* C: tickPerUs=1 快路径(计数即µs, 免除法) */
  gridTalk_init(&gt, NULL, 1);
  gt.nowTick = testTick;
  gt.tickPerUs = 1;
  virtTick = 0xFFFFFF00u;
  gt._lastTick = virtTick;
  total = 0;
  for(i = 0; i < 4096; i++)
  {
    virtTick += 7;
    total += 7;
    gridTalk_usTick(&gt, 1000);
    expectUs(&gt, total, 1, "divfree", i);
  }

  /* D: nowUs 钩子路径回归(gtNowUs重写不得破坏): 经enqUs观察时钟读数 */
  gridTalk_init(&gt, NULL, 1);
  gt.nowUs = fakeUs;
  fakeUsVal = 0xFFFF0000u;
  for(i = 0; i < 100000; i++)
    fakeUsVal += rng() % 5000u + 1u;
  gridTalk_sendNoAck(&gt, 2, 0x0100, NULL, 0);
  if(gt.txFifo[0].enqUs != fakeUsVal)
  {
    printf("FAIL[nowUs]: enqUs=%lu fakeUsVal=%lu\n",
           (unsigned long)gt.txFifo[0].enqUs, (unsigned long)fakeUsVal);
    fails++;
  }

  if(fails == 0) printf("test_tickwrap: ALL PASS\n");
  else           printf("test_tickwrap: %d FAILURES\n", fails);
  return fails != 0;
}
