#include "GridTalk.h"
#include <string.h>
#ifdef GT_DEBUG
#include <stdio.h>
#endif

/* ================= 内部工具 ================= */
/* 序号域: linkSeq并域后发送计数/按源去重表走宿主实例(多信道冗余:
   同一逻辑命令经多信道发送共用一个序号, 接收端跨信道去重) */
#define GT_SEQ_DOMAIN(gt) ((gt)->seqDomain ? (gt)->seqDomain : (gt))

static uint32_t gtNowUs(gridTalk_t *gt)
{
  return gt->nowUs ? gt->nowUs() : gt->_nowUs;
}

static uint32_t gtRng(gridTalk_t *gt)
{
  uint32_t x = gt->rng;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  gt->rng = x;
  return x;
}

/* 半双工: 发送期间(含最后一字节的移位尾巴)忽略接收, 过滤自身回显/冲突 */
static uint8_t gtTxBusy(gridTalk_t *gt)
{
  return (gt->txState == GT_TX_SENDING) ||
         (gtNowUs(gt) - gt->lastTxByteUs) < gt->byteUs;
}

/* 自主发送空闲判定: 距最后接收字节 >= GT_IDLE_CHARS_X10/10 个实测字节时间 */
static uint8_t gtBusIdle(gridTalk_t *gt)
{
  return (gtNowUs(gt) - gt->lastRxByteUs) >=
         (gt->byteUs * GT_IDLE_CHARS_X10 / 10u);
}

/* 应答就绪: 距最后接收字节 >= GT_REPLY_START_CHARS 个字节时间,
   保证应答在 GT_REPLY_WINDOW_CHARS 时限内开始 */
static uint8_t gtReplyReady(gridTalk_t *gt)
{
  return (gtNowUs(gt) - gt->lastRxByteUs) >= (gt->byteUs * GT_REPLY_START_CHARS);
}

/* 碰撞礼让(见GT_COLLISION_DEFER_CHARS): 坏帧=碰撞残骸/误码, 本节点自主发送
   随机让路一小段, 让正在重发的对端先走(重发方已付ACK超时代价, 新鲜帧只等
   4.5字符, 不让路则重发持续挨撞 -> 碰撞-冻结-再撞循环). 应答(ACK/响应)不受限 */
static void gtNoteCollision(gridTalk_t *gt)
{
#if GT_COLLISION_DEFER_CHARS
  gt->deferUntilUs = gtNowUs(gt) + (gtRng(gt) % GT_COLLISION_DEFER_CHARS) * gt->byteUs;
#endif
}

/* ================= 发送环形缓冲 ================= */
static void gtRingClear(gridTalk_t *gt)
{
  gt->txRingHead = 0;
  gt->txRingTail = 0;
  gt->txRingCount = 0;
}

static uint8_t gtRingWrite(gridTalk_t *gt, uint8_t c)
{
  if(gt->txRingCount >= SEND_STREAM_BUFSIZE) return 0;
  gt->txRing[gt->txRingTail] = c;
  gt->txRingTail = (uint16_t)((gt->txRingTail + 1) % SEND_STREAM_BUFSIZE);
  gt->txRingCount++;
  return 1;
}

static uint8_t gtRingRead(gridTalk_t *gt, uint8_t *c)
{
  if(gt->txRingCount == 0) return 0;
  *c = gt->txRing[gt->txRingHead];
  gt->txRingHead = (uint16_t)((gt->txRingHead + 1) % SEND_STREAM_BUFSIZE);
  gt->txRingCount--;
  return 1;
}

/* ================= 校验 ================= */
static void gtFletcher(const uint8_t* data, uint16_t len, uint8_t* sum1, uint8_t* sum2)
{
  uint16_t s1 = 0, s2 = 0;
  for(uint16_t i = 0; i < len; ++i)
  {
    s1 += data[i];
    if(s1 >= 255) s1 -= 255;
    s2 += s1;
    if(s2 >= 255) s2 -= 255;
  }
  *sum1 = (uint8_t)s1;
  *sum2 = (uint8_t)s2;
}

/* ================= 转义 ================= */
static void gtEscape(gridTalk_t *gt, const uint8_t* data, uint16_t len)
{
  gtRingWrite(gt, 0xFF);
  gtRingWrite(gt, 0x00);
  for(uint16_t i = 0; i < len; ++i)
  {
    gtRingWrite(gt, data[i]);
    if(data[i] == 0xFF) gtRingWrite(gt, data[i]);
  }
  gtRingWrite(gt, 0xFF);
  gtRingWrite(gt, 0xFE);
}

/* ================= L2 链路层: 媒体访问/发送状态机 ================= */
static void gtStartFrame(gridTalk_t *gt)
{
  gtRingClear(gt);
  gtEscape(gt, gt->txCurrent.data, gt->txCurrent.len);
  gt->txState = GT_TX_SENDING;
  gt->txSendStartUs = gtNowUs(gt);   /* SENDING卡死兜底的起点 */
  if(gt->txFrameStart) gt->txFrameStart(gt);   /* 485: 置发送方向 */
#if GT_TX_IRQ
  if(gt->uart) gt->uart->CR1 |= USART_CR1_TXEIE;  /* 开TXE中断, 逐字节发送 */
#endif
}

/* FIFO槽位移除(线性数组, 下标0最旧): 允许从任意位置销账(异步ACK乱序完成) */
static void gtFifoRemove(gridTalk_t *gt, uint8_t idx)
{
  if(idx >= gt->txFifoCount) return;
  memmove(&gt->txFifo[idx], &gt->txFifo[idx + 1],
          (size_t)(gt->txFifoCount - 1u - idx) * sizeof(gtTxFrame_t));
  gt->txFifoCount--;
}

/* 帧发送完成收尾: 切485方向/应答恢复/在途挂账(needAck)或终态(NoAck)/结果回调.
   流水线: needAck帧挂GT_TXS_INFLIGHT后信道立即可用(IDLE), ACK异步对账 */
static void gtTxFrameDone(gridTalk_t *gt)
{
  uint32_t now = gtNowUs(gt);
  if(gt->txFrameEnd) gt->txFrameEnd(gt);   /* 485: 帧发送完成, 恢复接收方向 */
  if(gt->sendingAck)
  {
    gt->sendingAck = 0;
    gt->txState = gt->txResumeState;
    gt->txResumeState = GT_TX_IDLE;
    return;
  }
  gt->txState = GT_TX_IDLE;
  if(gt->txCurrent.needAck == 0)
  {
    if(gt->onSendResult)
      gt->onSendResult(gt, gt->txCurrent.targetId, gt->txCurrent.cmd, 1);
    gtFifoRemove(gt, gt->txCurIdx);   /* NoAck帧: 发完即终态 */
    return;
  }
  /* needAck: 槽位转在途, 不阻塞信道/队列 */
  {
    gtTxFrame_t *f = (gt->txCurIdx < gt->txFifoCount) ? &gt->txFifo[gt->txCurIdx] : NULL;
    if(f && f->st == GT_TXS_ONWIRE)
    {
      /* ACK窗口自"发送完成"起算(TX_IRQ模式TC中断=最后字节已移出总线).
         下限(gt->ackWaitMinUs, init以GT_ACK_WINDOW_MIN_MS为默认): 覆盖无线模块
         收发转换(~40ms)与对端节拍驱动的ACK延迟; 快信道(485直连RTT<1ms)应调小.
         流水线下窗口不再阻塞信道, 取值只影响重发启动时机 */
      uint32_t ackWait = (uint32_t)GT_ACK_WINDOW_CHARS * gt->byteUs + 2u * gt->txDelayUs;
#if !GT_TX_IRQ
      ackWait += (uint32_t)gt->txCurrent.len * gt->byteUs;
#endif
      if(ackWait < gt->ackWaitMinUs) ackWait = gt->ackWaitMinUs;
      f->st = GT_TXS_INFLIGHT;
      f->ackUntil = now + ackWait;
    }
  }
}

static void gtDrain(gridTalk_t *gt)
{
  uint32_t now = gtNowUs(gt);

  /* ---- 应答绝对优先: 不发送中且总线空闲即发(响应, 立即接入).
     ACK格式FF FC+被确认方ID+确认方ID: 发送侧流水线多帧在途, 按确认方对账 ---- */
  if(gt->ackCount > 0 && gt->txState != GT_TX_SENDING && gtReplyReady(gt))
  {
    gt->txResumeState = gt->txState;
    gt->ackTargetId = gt->ackQueue[gt->ackHead];
    gt->ackHead = (uint8_t)((gt->ackHead + 1) % GT_ACK_QUEUE_SIZE);
    gt->ackCount--;
    gtRingClear(gt);
    gtRingWrite(gt, 0xFF);
    gtRingWrite(gt, 0xFC);
    gtRingWrite(gt, (uint8_t)(gt->ackTargetId >> 8));
    gtRingWrite(gt, (uint8_t)(gt->ackTargetId & 0xFF));
    gtRingWrite(gt, (uint8_t)(gt->id >> 8));       /* 确认方ID(流水线对账用) */
    gtRingWrite(gt, (uint8_t)(gt->id & 0xFF));
    gt->sendingAck = 1;
    gt->txState = GT_TX_SENDING;
    gt->txSendStartUs = now;   /* SENDING卡死兜底的起点 */
    if(gt->txFrameStart) gt->txFrameStart(gt); /* 485: ACK也是帧, 置发送方向 */
#if GT_TX_IRQ
    if(gt->uart) gt->uart->CR1 |= USART_CR1_TXEIE;  /* ACK字节流也由TXE中断发送 */
#endif
  }

  /* ---- 发送排空: 按实测字节速率节流, 不依赖固定tick ---- */
#if GT_TX_IRQ
  /* SENDING卡死兜底: TXE/TC中断全部停摆(硬件异常/配置被破坏)时, 状态机会永久停在
     SENDING且gtTxBusy恒真 -> 本信道把后续所有字节当自身回显丢弃, 全聋全哑:
     发不出ping -> 所有设备~50s内被判离线, 而手柄屏幕一切正常. 强制收尾自愈.
     Flash擦除冻结CPU期间全系统(含本函数)一起停摆, 不会误触发 */
  if(gt->txState == GT_TX_SENDING &&
     (int32_t)(now - gt->txSendStartUs) >
       (int32_t)(((uint32_t)gt->txCurrent.len * 2u + 16u) * gt->byteUs * 3u + 500000u))
  {
    if(gt->uart)
    {
      gt->uart->CR1 &= (uint32_t)~(USART_CR1_TXEIE | USART_CR1_TCIE);
      gt->uart->SR  &= (uint32_t)~USART_SR_TC;
    }
    gtRingClear(gt);
    gtTxFrameDone(gt);
  }
  /* TXE中断模式: 字节由UART中断逐字节写出, 此处不再轮询;
     TC收尾由gridTalk_txIrq处理, 这里仅作冗余兜底(TC中断异常丢失时) */
  if(gt->uart && gt->txState == GT_TX_SENDING && gt->txRingCount == 0 &&
     (gt->uart->CR1 & USART_CR1_TCIE) && (gt->uart->SR & USART_SR_TC))
  {
    gt->uart->CR1 &= (uint32_t)~USART_CR1_TCIE;
    gt->uart->SR  &= (uint32_t)~USART_SR_TC;   /* 清TC */
    gtTxFrameDone(gt);
  }
#else
  if(gt->uart && gt->txState == GT_TX_SENDING)
  {
    while(gt->txRingCount > 0 && (now - gt->lastTxByteUs) >= gt->byteUs)
    {
      if(!(gt->uart->SR & USART_SR_TXE)) break;
      uint8_t c;
      gtRingRead(gt, &c);
      if(gt->txByte) gt->txByte(gt, c); else gt->uart->DR = c;
      gt->lastTxByteUs = now;
      now = gtNowUs(gt);
    }

    /* 帧完成: 环形已空且最后一字节已移出(实测字节时间) */
    if(gt->txRingCount == 0 && (now - gt->lastTxByteUs) >= gt->byteUs)
    {
      gtTxFrameDone(gt);
    }
  }
#endif

  now = gtNowUs(gt);

  /* ---- 在途帧ACK超时扫描(流水线: 不占信道, 只处理自己的槽位) ---- */
  for(uint8_t i = 0; i < gt->txFifoCount; )
  {
    gtTxFrame_t *f = &gt->txFifo[i];
    if(f->st == GT_TXS_INFLIGHT && (int32_t)(now - f->ackUntil) >= 0)
    {
      if(f->resends >= GT_RESEND_COUNT)
      {
#ifdef GT_DEBUG
        printf("[gt id=%u] GIVEUP dst=%u cmd=%04X\n", gt->id, f->targetId, f->cmd);
#endif
        if(gt->onSendResult)
          gt->onSendResult(gt, f->targetId, f->cmd, 0);
        gtFifoRemove(gt, i);
        continue;   /* 移除后i位置是下一帧, 不递增 */
      }
      f->resends++;
      f->st = GT_TXS_QUEUED;   /* 原位重排(FIFO序, 重试优先于新帧但不占信道):
                                  下次接入按新抽的随机退避重发, 序号不变 */
      /* 指数退避(20<<n字符, 上限GT_BACKOFF_CAP_CHARS, 不低于保障窗): 重发风暴
         是碰撞恢复的关键 -- 无退避的激进重试会在高争用信道上越撞越多 */
      {
        uint32_t guardChars = gt->idleGuardUs / gt->byteUs;
        uint32_t win = (uint32_t)(20u << (f->resends > 8 ? 8 : f->resends));
        if(win > GT_BACKOFF_CAP_CHARS) win = GT_BACKOFF_CAP_CHARS;
        if(win < guardChars) win = guardChars;
        f->retryUntilUs = now + (gtRng(gt) % win) * gt->byteUs;
      }
#ifdef GT_DEBUG
      printf("[gt id=%u] RETRY dst=%u cmd=%04X n=%u\n", gt->id, f->targetId, f->cmd, f->resends);
#endif
    }
    i++;
  }

  /* ---- 状态机(信道级): 选取下一个可上线帧(跳过在途帧) ---- */
  switch(gt->txState)
  {
  case GT_TX_IDLE:
  {
    int idx = -1;
    /* 选取最早的非在途帧: QUEUED(新帧/重试)优先, 也恢复已选中但条件未就绪
       (总线忙/应答未到时)的ONWIRE帧, 避免其滞留; 重试退避中的帧跳过 */
    for(uint8_t i = 0; i < gt->txFifoCount; ++i)
    {
      if(gt->txFifo[i].st == GT_TXS_INFLIGHT) continue;
      if((int32_t)(now - gt->txFifo[i].retryUntilUs) < 0) continue;   /* 重试退避中 */
      idx = (int)i; break;
    }
    if(idx < 0) break;
    gtTxFrame_t *f = &gt->txFifo[idx];
#if GT_TX_STALE_MS
    /* 陈帧丢弃(见GT_TX_STALE_MS): 积压到超龄的帧在上线前丢弃.
       每次调用最多丢一帧, 剩余由后续节拍继续清理 */
    if((int32_t)(now - f->enqUs) > (int32_t)(GT_TX_STALE_MS * 1000u))
    {
      if(gt->onSendResult)
        gt->onSendResult(gt, f->targetId, f->cmd, 0);
      gtFifoRemove(gt, (uint8_t)idx);
      break;
    }
#endif
    f->st = GT_TXS_ONWIRE;
    gt->txCurrent = *f;
    gt->txCurIdx = (uint8_t)idx;
    if(f->reply)
    {
      /* 响应帧: 帧结束后1字节时间即发送(满足"4字符时间内开始回复") */
      if(gtReplyReady(gt))
      {
        gtStartFrame(gt);
      }
      /* 未就绪: 保持ONWIRE选中, 等下一拍重试replyReady */
    }
    else if(gtBusIdle(gt) && (int32_t)(now - gt->deferUntilUs) >= 0)
    {
      /* 自主帧: 总线空闲4.5字符且已过碰撞礼让期, 再随机退避, 分散竞争.
         随机范围覆盖 idleGuardUs: 既分散竞争, 又让接入点避开
         打包型转发等"帧已上线但本端不可见"的盲区(固定等待会同步各节点) */
      gt->txState = GT_TX_ACCESS_WAIT;
      gt->txWaitUs = (gtRng(gt) % (GT_AUTO_BACKOFF_CHARS + gt->idleGuardUs / gt->byteUs + 1u)) * gt->byteUs;
      gt->txWaitUntilUs = now + gt->txWaitUs;
    }
    /* 总线忙: 保持ONWIRE选中, IDLE态每拍重试接入条件 */
  }
  break;

  case GT_TX_ACCESS_WAIT:
    if(!gtBusIdle(gt))
    {
      /* 总线忙: 从总线空闲起重新计时 */
      gt->txWaitUntilUs = now + gt->txWaitUs;
      break;
    }
    if((int32_t)(now - gt->txWaitUntilUs) >= 0)
    {
      gtStartFrame(gt);
    }
    break;

  default:
    break;
  }
}

/* ================= L2 链路层: 组帧(含寻址)入队 ================= */
/* 取下一条新逻辑命令序号(经序号域: 多信道并域后各信道同源递增) */
static uint16_t gtSeqNext(gridTalk_t *gt)
{
  gridTalk_t *dom = GT_SEQ_DOMAIN(gt);
  return dom->txSeqCounter++;
}

/* 以指定序号入队一帧(sendMulti的副本共用同序号入口; 单发路径经gtSendFrame取号) */
static void gtSendFrameSeq(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                           const uint8_t* data, uint16_t len,
                           uint8_t needAck, uint8_t reply, uint16_t seq)
{
  if(len > GT_TX_FRAME_MAX - 10) return;
  if(gt->txFifoCount >= GT_TX_FIFO_SIZE) return;

  gtTxFrame_t *f = &gt->txFifo[gt->txFifoCount];   /* 线性数组: 尾部追加 */
  uint16_t _len = 0;
  /* 诊断: 入队间隔(200ms单位, 饱和65535). 手柄扫描靠它区分"MCU侧没入队"
     与"模块吞帧": 卡顿窗口里本间隔小=帧已交给模块(硬件问题), 大=MCU侧没入队 */
  {
    uint32_t nowUs = gtNowUs(gt);
    uint16_t gap = (uint16_t)((nowUs - gt->txLastEnqueueUs) / 200000u);
    if(gap > gt->txGapMax200ms) gt->txGapMax200ms = gap;
    gt->txLastEnqueueUs = nowUs;
  }
  f->seq = seq;                 /* 新逻辑命令取号; 重发沿用txCurrent里的同一号 */
  f->data[_len++] = (uint8_t)((target >> 8) & 0xFF);
  f->data[_len++] = (uint8_t)((target >> 0) & 0xFF);
  f->data[_len++] = (uint8_t)((gt->id >> 8) & 0xFF);
  f->data[_len++] = (uint8_t)((gt->id >> 0) & 0xFF);
  f->data[_len++] = (uint8_t)((f->seq >> 8) & 0xFF);
  f->data[_len++] = (uint8_t)((f->seq >> 0) & 0xFF);
  f->data[_len++] = (uint8_t)((cmd >> 8) & 0xFF);
  f->data[_len++] = (uint8_t)((cmd >> 0) & 0xFF);
  for(uint16_t i = 0; i < len; ++i) f->data[_len++] = data[i];
  gtFletcher(f->data, _len, &f->data[_len], &f->data[_len + 1]);
  _len += 2;

  f->len = _len;
  f->targetId = target;
  f->cmd = cmd;
  f->needAck = needAck;
  f->reply = reply;
  f->enqUs = gtNowUs(gt);   /* 陈帧丢弃的起点(见GT_TX_STALE_MS) */
  f->st = GT_TXS_QUEUED;
  f->resends = 0;
  f->retryUntilUs = 0;
  gt->txFifoCount++;

  gtDrain(gt);   /* 入队后立即尝试发送, 响应帧无需等待下一个tick */
}

static void gtSendFrame(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                        const uint8_t* data, uint16_t len,
                        uint8_t needAck, uint8_t reply)
{
  /* 序号在通过容量检查后才分配, FIFO满时不消耗序号(保持序号流无空洞) */
  if(len > GT_TX_FRAME_MAX - 10) return;
  if(gt->txFifoCount >= GT_TX_FIFO_SIZE) return;
  gtSendFrameSeq(gt, target, cmd, data, len, needAck, reply, gtSeqNext(gt));
}

/* 临界区采用PRIMASK保存/恢复: 任务与ISR上下文均可安全调用,
   避免成对__disable_irq/__enable_irq在ISR里错误地提前开中断 */
void gridTalk_send(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                   const uint8_t *data, uint16_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  gtSendFrame(gt, target, cmd, data, len, (target != Bcast) ? 1 : 0, 0);
  __set_PRIMASK(primask);
}

void gridTalk_sendReply(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                        const uint8_t *data, uint16_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  gtSendFrame(gt, target, cmd, data, len, (target != Bcast) ? 1 : 0, 1);
  __set_PRIMASK(primask);
}

void gridTalk_sendNoAck(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                        const uint8_t *data, uint16_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  gtSendFrame(gt, target, cmd, data, len, 0, 0);
  __set_PRIMASK(primask);
}

void gridTalk_sendReplyNoAck(gridTalk_t *gt, uint16_t target, uint16_t cmd,
                             const uint8_t *data, uint16_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  gtSendFrame(gt, target, cmd, data, len, 0, 1);
  __set_PRIMASK(primask);
}

/* 多信道冗余发送(见GridTalk.h): 一个逻辑命令只取一次序号, 全部副本共用。
   副本各自入队/ACK/重发独立, 任一信道FIFO满仅丢该信道。整段临界区保证
   各信道副本序号一致且不与单发路径交错取号。 */
void gridTalk_sendMulti(gridTalk_t *gts[], uint8_t count, uint16_t target, uint16_t cmd,
                        const uint8_t *data, uint16_t len)
{
  if(count == 0 || gts == NULL || gts[0] == NULL) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint16_t seq = gtSeqNext(gts[0]);   /* 自gts[0]所在序号域一次性分配 */
  uint8_t needAck = (target != Bcast) ? 1 : 0;
  for(uint8_t i = 0; i < count; ++i)
  {
    if(gts[i] == NULL) continue;
    gtSendFrameSeq(gts[i], target, cmd, data, len, needAck, 0, seq);
  }
  __set_PRIMASK(primask);
}

/* 序号域并入(见GridTalk.h): 解析host的域根(支持链式并入)后挂接, 防成环 */
void gridTalk_linkSeq(gridTalk_t *host, gridTalk_t *member)
{
  if(!host || !member || host == member) return;
  gridTalk_t *root = host;
  for(uint8_t i = 0; root->seqDomain && i < 8; ++i) root = root->seqDomain;
  if(root == member) return;   /* 会成环: 拒绝 */
  member->seqDomain = root;
}

uint8_t gridTalk_pending(gridTalk_t *gt)
{
  return gt->txFifoCount;
}

uint32_t gridTalk_byteUs(gridTalk_t *gt)
{
  return gt->byteUs;
}

/* ================= L3 应用层: 命令分发 ================= */
static void gtAppRx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                    const uint8_t *payload, uint16_t len)
{
  if(gt->onRx) gt->onRx(gt, src, cmd, payload, len);
}

/* ================= L2 链路层: 收帧(校验+寻址+应答+去重) ================= */
static void gtLinkRx(gridTalk_t *gt, const uint8_t* data, uint16_t len)
{
  uint8_t s1 = 0, s2 = 0;
  uint8_t crcOk = (len >= 10 && len <= SEND_STREAM_BUFSIZE);
  if(crcOk)
  {
    gtFletcher(data, len - 2, &s1, &s2);
    crcOk = (s1 == data[len - 2] && s2 == data[len - 1]);
  }

  /* 调试钩子: 每收到一个完整帧都上报(含坏帧), 供上位机实时看总线 */
  if(gt->onRxRaw) gt->onRxRaw(gt, data, len, crcOk);

  if(!crcOk && gt->rxBadFrames < 0xFFFF) gt->rxBadFrames++;   /* 诊断: 信道质量计数 */

  if(!crcOk) gtNoteCollision(gt);   /* 坏帧: 碰撞残骸/误码, 自主发送礼让重发方 */

  if(len < 10 || len > SEND_STREAM_BUFSIZE) return;
  if(!crcOk) return;

  uint16_t dst = (uint16_t)((data[0] << 8) | data[1]);
  uint16_t src = (uint16_t)((data[2] << 8) | data[3]);
  uint16_t seq = (uint16_t)((data[4] << 8) | data[5]);
  uint16_t cmd = (uint16_t)((data[6] << 8) | data[7]);

  if(dst != gt->id && dst != Bcast) return;

  /* 单播: 排队带地址应答 */
  if(dst == gt->id && gt->ackCount < GT_ACK_QUEUE_SIZE)
  {
    gt->ackQueue[(gt->ackHead + gt->ackCount) % GT_ACK_QUEUE_SIZE] = src;
    gt->ackCount++;
  }

  /* 执行去重(序号法, 见GT_SEQ_*说明): 判定只看序号, 与ACK是否发出/丢失无关 --
     "帧到而ACK丢"的经典重发不会再被当第二条命令执行.
     序号域: 多信道并域后各实例共享同一张按源表, 跨信道副本(同一逻辑命令
     的另一份, 序号相同)在此被判为重发只ACK不执行.
     远偏离(差值落在两窗口之外)只可能是某端重启失步, 执行并重新同步,
     避免失步后永久拒收. */
  if(dst == gt->id && src < GT_SEQ_SLOTS)
  {
    gridTalk_t *dom = GT_SEQ_DOMAIN(gt);
    uint16_t delta = (uint16_t)(seq - dom->rxLastSeq[src]);
    /* 落后帧: 回卷差落在窗口另一侧且非0(真重发是delta==0, 迟到旧帧最多落后1~2) */
    uint8_t behind = (delta != 0 && delta >= (uint16_t)(0u - GT_SEQ_WINDOW));
    if(dom->rxSeqValid[src] && (delta == 0 || behind))
    {
      if(delta == 0)
      {
        return;   /* 重发/迟到的旧帧(含跨信道副本): ACK已排队, 不再执行 */
      }
      /* 失步兜底(连续落后帧强制重同步): 只计"严格递增的不同落后序号".
         多信道冗余下, 经另一信道已执行之帧在本地信道的重发流全是落后帧
         (同号重复/两个旧号交替) —— 旧逻辑8连即强制执行, 会把陈旧重发当新
         命令执行, 派生幻影波并在环形级联中与真波互相吞并(表现为圈速忽快
         忽慢甚至长于单圈). 规则: 递增才计数; 同号重复不计不重置; 回退视为
         另一股落后流起点重新起算(交替残流的计数被钉死在其循环跨度内,
         永远到不了阈值); 对端真失步(重启换基值)时新序号持续递增,
         仍能在8条新命令内重同步. */
      if(dom->rxBehindCnt[src] == 0 ||
         (int16_t)(seq - dom->rxBehindLastSeq[src]) > 0)
      {
        dom->rxBehindLastSeq[src] = seq;
        if(++dom->rxBehindCnt[src] >= GT_SEQ_DESYNC_FRAMES)
        {
          dom->rxBehindCnt[src] = 0;
          dom->rxLastSeq[src] = seq;   /* 失步重同步: 执行本帧 */
        }
        else
        {
          return;
        }
      }
      else if(seq != dom->rxBehindLastSeq[src])
      {
        dom->rxBehindCnt[src] = 1;     /* 回退: 新落后流起点, 重新起算 */
        dom->rxBehindLastSeq[src] = seq;
        return;
      }
      else
      {
        return;   /* 同号重复(信道重发): 不计不重置 */
      }
    }
    else
    {
      dom->rxBehindCnt[src] = 0;
      dom->rxLastSeq[src] = seq;
      dom->rxSeqValid[src] = 1;
    }
  }

  gtAppRx(gt, src, cmd, data + 8, len - 10);
}

/* ================= L2 链路层: 转义/定界 ================= */
/* ACK对账: FF FC+被确认方ID+确认方ID 收齐后调用.
   清除对确认方acker最早一帧在途(GT_TXS_INFLIGHT)帧 -- 流水线下多帧同时在途,
   ACK只能按确认方匹配; 同目标多帧在途时按FIFO序(对端按接收序应答, 与本端
   发送序一致). 误匹配方向安全: 多余重发会被对端序号去重吞掉 */
static void gtAckMatch(gridTalk_t *gt, uint16_t acker)
{
  for(uint8_t i = 0; i < gt->txFifoCount; ++i)
  {
    if(gt->txFifo[i].st == GT_TXS_INFLIGHT && gt->txFifo[i].targetId == acker)
    {
      if(gt->onSendResult)
        gt->onSendResult(gt, gt->txFifo[i].targetId, gt->txFifo[i].cmd, 1);
      gtFifoRemove(gt, i);
      return;
    }
  }
}

static void gtUnescape(gridTalk_t *gt, uint8_t c)
{
  if(gt->_c == 0xFF)
  {
    if(c == 0x00)
    {
      gt->rxLen = 0;
      gt->rxStatus = 1;
    }
    else if(c == 0xFC)
    {
      gt->ackTargetStage = 1;   /* 应答开始: 后续4字节=被确认方ID(2)+确认方ID(2) */
    }
    else if(gt->rxStatus == 1)
    {
      if(c == 0xFE)
      {
        gt->rxStatus = 0;
        gtLinkRx(gt, gt->rxBuff, gt->rxLen);
      }
      else if(c == 0xFF)
      {
        if(gt->rxLen < sizeof(gt->rxBuff)) gt->rxBuff[gt->rxLen++] = 0xFF;
        c = 0;
      }
      else
      {
        gtNoteCollision(gt);   /* 帧被截断/打坏: 碰撞残骸, 礼让重发方 */
        gt->rxStatus = 0;
        gt->rxLen = 0;
      }
    }
  }
  else if(gt->ackTargetStage >= 1 && gt->ackTargetStage <= 4)
  {
    /* ACK载荷: 阶段1..4 = 被确认方hi/lo + 确认方hi/lo */
    switch(gt->ackTargetStage)
    {
    case 1: gt->ackTargetHi = c; break;
    case 2: gt->ackTargetLo = c; break;
    case 3: gt->ackerHi = c; break;
    default:
      gt->ackTargetStage = 0;
      gt->_c = 0;
      {
        uint16_t ackedId = (uint16_t)((gt->ackTargetHi << 8) | gt->ackTargetLo);
        uint16_t acker = (uint16_t)((gt->ackerHi << 8) | c);   /* c=确认方低字节 */
        if(ackedId == gt->id) gtAckMatch(gt, acker);
      }
      return;
    }
    gt->ackTargetStage++;
    gt->_c = 0;
    return;
  }
  else if(c != 0xFF && gt->rxStatus == 1)
  {
    if(gt->rxLen < sizeof(gt->rxBuff)) gt->rxBuff[gt->rxLen++] = c;
  }

  gt->_c = c;
}

void gridTalk_rxByte(gridTalk_t *gt, uint8_t c)
{
  uint32_t now = gtNowUs(gt);
  uint32_t gap = now - gt->lastRxByteUs;
  gt->lastRxByteUs = now;

  /* 半双工: 自身发送回显/冲突字节不解析 */
  if(gtTxBusy(gt))
  {
    gt->_c = 0;
    gt->ackTargetStage = 0;
    return;
  }
  if(gt->rxBytes < 0xFFFF) gt->rxBytes++;   /* 诊断: 总线活动字节计数(不含自身回显) */

  /* 字节速率自适应测量: 仅采样帧内字节间隔(第2字节起),
     避免把"接入立即发送"产生的帧间间隙混入均值 */
  if(gt->rxStatus == 1 && gt->rxLen >= 1 &&
     gap >= gt->byteUs / 2 && gap <= gt->byteUs * 2)
  {
    gt->byteUs = (gt->byteUs * 7 + gap) / 8;
    gt->byteUsSamples++;
  }

  /* 临界区(帧解析+排空): 防止更高优先级的usTick定时中断在同一实例上
     再入gtDrain, 造成txFifo/ackQueue双重消费与发送状态机错乱 */
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    gtUnescape(gt, c);
    /* 收到字节后立即尝试排空, 响应帧及时发出 */
    gtDrain(gt);
    __set_PRIMASK(primask);
  }
}

/* ================= 周期处理 ================= */
void gridTalk_usTick(gridTalk_t *gt, uint32_t us)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if(!gt->nowUs) gt->_nowUs += us;
  gtDrain(gt);
  __set_PRIMASK(primask);
}

#if GT_TX_IRQ
/* UART TXE/TC中断入口: 发送中断驱动模式下逐字节写出, 帧收尾.
   临界区: txRing/txState同样被usTick的gtDrain访问, 防止更高优先级定时中断再入 */
void gridTalk_txIrq(gridTalk_t *gt)
{
  uint32_t sr;
  if(!gt->uart || gt->txState != GT_TX_SENDING) return;
  sr = gt->uart->SR;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* TXE: 移位寄存器空闲, 可写下一字节 */
  if((sr & USART_SR_TXE) && gt->txRingCount > 0)
  {
    uint8_t c;
    gtRingRead(gt, &c);
    if(gt->txByte) gt->txByte(gt, c); else gt->uart->DR = c;
    gt->lastTxByteUs = gtNowUs(gt);
    if(gt->txRingCount == 0)
    {
      /* 最后一字节已写入DR: 关TXE中断, 等TC(移位完成)后收尾 */
      gt->uart->CR1 = (gt->uart->CR1 & (uint32_t)~USART_CR1_TXEIE) | USART_CR1_TCIE;
    }
    __set_PRIMASK(primask);
    return;
  }

  /* TC: 最后一字节移位完成, 帧真正发完 */
  if((sr & USART_SR_TC) && gt->txRingCount == 0)
  {
    gt->uart->CR1 &= (uint32_t)~USART_CR1_TCIE;
    gt->uart->SR  &= (uint32_t)~USART_SR_TC;   /* 清TC */
    gtTxFrameDone(gt);
  }

  __set_PRIMASK(primask);
}
#endif

/* ================= 初始化 ================= */
void gridTalk_init(gridTalk_t *gt, USART_TypeDef * uart, uint16_t id)
{
  memset(gt, 0, sizeof(*gt));
  gt->uart = uart;
  gt->byteUs = GT_DEFAULT_BYTE_US;
  gt->idleGuardUs = GT_IDLE_GUARD_US;
  gt->txDelayUs = GT_TX_DELAY_US;
  gt->ackWaitMinUs = (uint32_t)GT_ACK_WINDOW_MIN_MS * 1000u;
  gridTalk_setID(gt, id);
  /* 小种子(设备号)的xorshift输出近似线性(实测相邻设备基值差~8225), 先空转打散
     再取基值. 基值仍由ID确定性决定, 跨上电随机性由 gridTalk_seed(板级注入熵)
     与接收端连续落后重同步(GT_SEQ_DESYNC_FRAMES)共同保证 */
  for(uint8_t i = 0; i < 16; ++i) (void)gtRng(gt);
  gt->txSeqCounter = (uint16_t)gtRng(gt);
}

/* 板级注入每次上电不同的熵(如 DWT计数值^HAL_GetTick^UID字), 重定序号基值:
   同台设备每次上电的序号相对上次"远偏离", 对端立即走重同步分支执行.
   没有它, 确定性基值跨上电复现会被对端当重发吞掉 -- 现场表现为级联波在
   "跨过发送方重启的接收机"处静默死亡(发送方却收到ACK以为成功) */
void gridTalk_seed(gridTalk_t *gt, uint32_t entropy)
{
  gt->rng ^= entropy;
  for(uint8_t i = 0; i < 8; ++i) (void)gtRng(gt);
  /* 序号基值写序号域(并域后由宿主承载; 未link时即自身, 行为不变) */
  GT_SEQ_DOMAIN(gt)->txSeqCounter = (uint16_t)gtRng(gt);
}

void gridTalk_setID(gridTalk_t *gt, uint16_t id)
{
  gt->id = id;
  gt->rng = id ? (uint32_t)id : 0x9E3779B9u;
}
