/*
 * GridTalk 应用层组件: 可靠有序字节管道(实现)
 * 设计说明见 gridtalk_pipe.h 文件头; 依赖仅 GridTalk 核心与C标准库。
 */
#include "gridtalk_pipe.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>   /* vsnprintf */

/* ================= hub注册表(回调反查) =================
   核心回调签名携带gt, 组件经此表反查hub。静态有界(GTPIPE_HUB_MAX),
   容量不足时编译期调大; 单实例固件默认容量绰绰有余 */
static gtpipe_hub_t *s_hubs[GTPIPE_HUB_MAX];

static gtpipe_hub_t *hubOf(gridTalk_t *gt)
{
  for(uint8_t i = 0; i < GTPIPE_HUB_MAX; ++i)
    if(s_hubs[i] && s_hubs[i]->gt == gt) return s_hubs[i];
  return NULL;
}

static uint32_t pNowUs(gtpipe_hub_t *h)
{
  return h->gt->nowUs ? h->gt->nowUs() : h->gt->_nowUs;
}

/* ================= 环形缓冲 ================= */
static uint16_t pRxFree(gtpipe_t *p)
{
  return (uint16_t)(GTPIPE_RX_SIZE - p->rxCount);
}

static void pRxPush(gtpipe_t *p, const uint8_t *d, uint16_t n)
{
  if(n == 0 || pRxFree(p) < n)
  {
    if(n) p->stFlags |= GTPIPE_ST_RXOVER;   /* 放不下整段: 丢段保有序, 置标志 */
    return;
  }
  /* 写游标=(读游标+已有量): rxHead只由读取推进, 读写互不踩位 */
  uint16_t tail = (uint16_t)((p->rxHead + p->rxCount) % GTPIPE_RX_SIZE);
  p->rxCount = (uint16_t)(p->rxCount + n);
  for(uint16_t k = 0; k < n; ++k)
  {
    p->rxBuf[tail] = d[k];
    tail = (uint16_t)((tail + 1) % GTPIPE_RX_SIZE);
  }
}

/* ================= 停等发送机 =================
   从活跃源(writeAll游标优先, 其次各管道发送环形, 注册序)取一段发送。
   调用点: write/writeAll入队后、onSendResult续段、poll兜底。
   segBusy在gridTalk_send之前置位: send内部的drain可能同步触发前帧giveup
   -> pOnResult -> pKick重入, 先占位可防双段并发 */
static void pKick(gtpipe_hub_t *h)
{
  gridTalk_t *gt = h->gt;
  if(h->segBusy || !gt->txByte) return;
  if(gt->txFifoCount >= GT_TX_FIFO_SIZE) return;   /* L2 FIFO满: 由poll/结果回调续 */

  gtpipe_t *src = NULL;
  uint8_t fromBulk = 0;
  uint16_t n = 0;
  for(uint8_t i = 0; i < h->pipeN; ++i)   /* writeAll游标优先(尽快完成独占) */
  {
    gtpipe_t *p = h->pipes[i];
    if(p->bulkSrc && p->bulkPos < p->bulkLen)
    {
      src = p; fromBulk = 1;
      n = (uint16_t)(p->bulkLen - p->bulkPos);
      break;
    }
  }
  if(!src)
  {
    for(uint8_t i = 0; i < h->pipeN; ++i)
    {
      gtpipe_t *p = h->pipes[i];
      if(p->txCount)
      {
        src = p;
        n = p->txCount;
        break;
      }
    }
  }
  if(!src) return;
  if(n > GTPIPE_SEG_MAX) n = GTPIPE_SEG_MAX;

  h->seg[0] = src->id;
  h->seg[1] = (uint8_t)(n & 0xFFu);
  h->seg[2] = (uint8_t)(n >> 8);
  if(fromBulk)
  {
    memcpy(&h->seg[3], src->bulkSrc + src->bulkPos, n);
    h->segBulkStart = src->bulkPos;
  }
  else
  {
    for(uint16_t k = 0; k < n; ++k)
    {
      h->seg[3 + k] = src->txBuf[src->txHead];
      src->txHead = (uint16_t)((src->txHead + 1) % GTPIPE_TX_SIZE);
    }
    src->txCount = (uint16_t)(src->txCount - n);
  }
  h->segOwner = src;
  h->segFromBulk = fromBulk;
  h->segDataLen = n;
  h->segBusy = 1;
  gridTalk_send(gt, src->peer, GTPIPE_CMD_DATA, h->seg, (uint16_t)(n + 3u));
}

/* ================= L2回调(链式) =================
   总在核心临界区内被调(发送结果来自ACK对账/giveup扫描, 接收来自收帧路径);
   组件自身的GT_CRIT为保存/恢复式, 嵌套安全 */
static void pOnRx(gridTalk_t *gt, uint16_t src, uint16_t cmd,
                  const uint8_t *data, uint16_t len)
{
  gtpipe_hub_t *h = hubOf(gt);
  if(!h) return;
  if(cmd == GTPIPE_CMD_DATA)
  {
    if(len >= 3)
    {
      uint16_t n = (uint16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
      if(n == (uint16_t)(len - 3u))
      {
        GT_CRIT_ENTER();
        for(uint8_t i = 0; i < h->pipeN; ++i)
        {
          gtpipe_t *p = h->pipes[i];
          if(p->id == data[0] && p->peer == src)
          {
            pRxPush(p, data + 3, n);   /* 去重已过(恰好一次), 入接收环形 */
            GT_CRIT_EXIT();
            return;
          }
        }
        GT_CRIT_EXIT();
      }
    }
    return;   /* 管道流量(未注册管道/畸形段): 消化, 不透传应用 */
  }
  if(h->prevOnRx) h->prevOnRx(gt, src, cmd, data, len);
}

static void pOnResult(gridTalk_t *gt, uint16_t target, uint16_t cmd, uint8_t ok)
{
  gtpipe_hub_t *h = hubOf(gt);
  if(!h) return;
  if(cmd == GTPIPE_CMD_DATA)
  {
    GT_CRIT_ENTER();
    if(h->segBusy && h->segOwner)
    {
      gtpipe_t *p = h->segOwner;
      if(h->segFromBulk)
      {
        if(ok) p->bulkPos = h->segBulkStart + h->segDataLen;
        else   p->bulkErr = -1;
      }
      else if(!ok)
      {
        p->stFlags |= GTPIPE_ST_TXFAIL;   /* 环形内该段数据已失 */
      }
      h->segBusy = 0;
      h->segOwner = NULL;
      GT_CRIT_EXIT();
      pKick(h);   /* 续下一段 */
      return;
    }
    GT_CRIT_EXIT();
    return;       /* 迟到的管道段结果(如超时后的ACK): 消化掉 */
  }
  if(h->prevOnResult) h->prevOnResult(gt, target, cmd, ok);
  /* 应用帧结果释放了FIFO空间: 顺带续段(管道段可能正因此挂起) */
  GT_CRIT_ENTER();
  pKick(h);
  GT_CRIT_EXIT();
}

/* ================= 挂接/注册 ================= */
void gtpipe_attach(gtpipe_hub_t *h, gridTalk_t *gt)
{
  if(!h || !gt || !gt->txByte) return;
  if(hubOf(gt)) return;   /* 该实例已挂接(一个实例一个hub) */
  uint8_t slot;
  for(slot = 0; slot < GTPIPE_HUB_MAX; ++slot)
    if(!s_hubs[slot]) break;
  if(slot >= GTPIPE_HUB_MAX) return;
  memset(h, 0, sizeof(*h));
  h->gt = gt;
  h->prevOnRx = gt->onRx;             /* 应用回调在此刻之前设置, 链式保存 */
  h->prevOnResult = gt->onSendResult;
  gt->onRx = pOnRx;
  gt->onSendResult = pOnResult;
  s_hubs[slot] = h;
}

void gtpipe_register(gtpipe_t *p, gtpipe_hub_t *h, uint8_t pipeId, uint16_t peerId)
{
  if(!p || !h || h->pipeN >= GTPIPE_MAX) return;
  memset(p, 0, sizeof(*p));
  p->h = h;
  p->id = pipeId;
  p->peer = peerId;
  h->pipes[h->pipeN++] = p;
}

void gtpipe_detach(gtpipe_t *p)
{
  if(!p || !p->h) return;
  gtpipe_hub_t *h = p->h;
  GT_CRIT_ENTER();
  for(uint8_t i = 0; i < h->pipeN; ++i)
  {
    if(h->pipes[i] == p)
    {
      if(p->bulkSrc) { p->bulkErr = -2; p->bulkSrc = NULL; }   /* 取消writeAll */
      memmove(&h->pipes[i], &h->pipes[i + 1],
              (size_t)(h->pipeN - 1u - i) * sizeof(gtpipe_t *));
      h->pipeN--;
      break;
    }
  }
  if(h->segOwner == p) h->segOwner = NULL;   /* 在途段结果迟到时不再触达已注销管道 */
  GT_CRIT_EXIT();
  memset(p, 0, sizeof(*p));
}

/* ================= 数据面 ================= */
int gtpipe_write(gtpipe_t *p, const void *buf, uint16_t len)
{
  if(!p || !p->h) return 0;
  GT_CRIT_ENTER();
  uint16_t free_ = (uint16_t)(GTPIPE_TX_SIZE - p->txCount);
  uint16_t n = (len < free_) ? len : free_;
  if(n)
  {
    const uint8_t *d = (const uint8_t *)buf;
    for(uint16_t k = 0; k < n; ++k)
    {
      p->txBuf[(uint16_t)((p->txHead + p->txCount) % GTPIPE_TX_SIZE)] = d[k];
      p->txCount++;
    }
    pKick(p->h);
  }
  GT_CRIT_EXIT();
  return (int)n;
}

int gtpipe_read(gtpipe_t *p, void *buf, uint16_t len)
{
  if(!p || !p->h || !len) return 0;
  GT_CRIT_ENTER();
  uint16_t n = (len < p->rxCount) ? len : p->rxCount;
  uint8_t *d = (uint8_t *)buf;
  for(uint16_t k = 0; k < n; ++k)
  {
    d[k] = p->rxBuf[p->rxHead];
    p->rxHead = (uint16_t)((p->rxHead + 1) % GTPIPE_RX_SIZE);
    p->rxCount--;
  }
  GT_CRIT_EXIT();
  return (int)n;
}

uint16_t gtpipe_available(gtpipe_t *p)
{
  if(!p) return 0;
  GT_CRIT_ENTER();
  uint16_t n = p->rxCount;
  GT_CRIT_EXIT();
  return n;
}

uint8_t gtpipe_status(gtpipe_t *p)
{
  if(!p) return 0;
  GT_CRIT_ENTER();
  uint8_t f = p->stFlags;
  p->stFlags = 0;
  GT_CRIT_EXIT();
  return f;
}

int gtpipe_writeAll(gtpipe_t *p, const void *buf, uint32_t len,
                    uint32_t timeoutUs, gtpipe_progress_cb prog,
                    gtpipe_pump_cb pump, void *user)
{
  if(!p || !p->h || !p->h->gt->txByte || (!buf && len)) return -3;
  uint32_t t0 = pNowUs(p->h);
  GT_CRIT_ENTER();
  p->bulkSrc = (const uint8_t *)buf;
  p->bulkLen = len;
  p->bulkPos = 0;
  p->bulkErr = 0;
  pKick(p->h);
  GT_CRIT_EXIT();
  while(1)
  {
    uint32_t pos;
    int8_t err;
    GT_CRIT_ENTER();
    pos = p->bulkPos; err = p->bulkErr;
    GT_CRIT_EXIT();
    if(prog) prog(p, pos, len);
    if(err)
    {
      GT_CRIT_ENTER();
      p->bulkSrc = NULL;
      GT_CRIT_EXIT();
      return err;   /* -1=对端不可达 -2=超时(由下方置位) */
    }
    if(pos >= len)
    {
      GT_CRIT_ENTER();
      p->bulkSrc = NULL;
      GT_CRIT_EXIT();
      return (int)len;
    }
    if(timeoutUs && (uint32_t)(pNowUs(p->h) - t0) > timeoutUs)
    {
      GT_CRIT_ENTER();
      p->bulkErr = -2;
      p->bulkSrc = NULL;
      GT_CRIT_EXIT();
      return -2;
    }
    if(pump) pump(user);
  }
}

int gtpipe_readAll(gtpipe_t *p, void *buf, uint32_t len,
                   uint32_t timeoutUs, gtpipe_progress_cb prog,
                   gtpipe_pump_cb pump, void *user)
{
  if(!p || !p->h) return -3;
  uint32_t t0 = pNowUs(p->h);
  uint32_t got = 0;
  while(got < len)
  {
    uint16_t chunk = (len - got > 0xFFFFu) ? 0xFFFFu : (uint16_t)(len - got);
    int n = gtpipe_read(p, (uint8_t *)buf + got, chunk);
    if(n > 0) got += (uint32_t)n;
    if(prog) prog(p, got, len);
    if(got >= len) return (int)len;
    if(timeoutUs && (uint32_t)(pNowUs(p->h) - t0) > timeoutUs) return -2;
    if(pump) pump(user);
  }
  return (int)len;
}

int gtpipe_vprintf(gtpipe_t *p, const char *fmt, va_list ap)
{
  char s[GTPIPE_PRINTF_BUF];
  int n = vsnprintf(s, sizeof(s), fmt, ap);
  if(n <= 0) return n;
  if(n > (int)sizeof(s) - 1) n = (int)sizeof(s) - 1;
  return gtpipe_write(p, s, (uint16_t)n);
}

int gtpipe_printf(gtpipe_t *p, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int n = gtpipe_vprintf(p, fmt, ap);
  va_end(ap);
  return n;
}

void gtpipe_poll(gtpipe_hub_t *h)
{
  if(!h) return;
  GT_CRIT_ENTER();
  pKick(h);
  GT_CRIT_EXIT();
}
