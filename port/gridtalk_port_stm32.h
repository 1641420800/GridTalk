#ifndef GRIDTALK_PORT_STM32_H
#define GRIDTALK_PORT_STM32_H
/*
 * GridTalk STM32 USART 端口(可选): 寄存器直写绑定, 等价旧版 uart 直写模式。
 *
 * 用法(三行接入):
 *   gridTalk_init(&gt, USART2, myID);   // hw = USART外设
 *   gridTalk_portBindUart(&gt);          // txByte写DR + txKick等中断控制, 一行绑定
 *   // USART2_IRQHandler 里调用 gridTalk_portUsartIsr(&gt);
 * 时基(推荐再加一行): gridTalk_portBindDwt(&gt); // DWT->CYCCNT, 库内余数精确换算µs
 *
 * 包含顺序: 须先让 USART_TypeDef / USART_CR1_TXEIE / USART_SR_TC 可见。
 * 默认包含 "main.h"(Cube工程惯例), 可经 GT_STM32_DEVICE_HEADER 覆盖。
 * 临界区经 GridTalk.h 自动选择(ARM/CMSIS下为PRIMASK保存/恢复)。
 */
#ifndef GT_STM32_DEVICE_HEADER
#define GT_STM32_DEVICE_HEADER "main.h"
#endif
#include GT_STM32_DEVICE_HEADER
#include "GridTalk.h"

/* ---- 发送钩子(寄存器直写, static inline 无调用开销) ---- */

/* 发送出口: 字节写入数据寄存器(TXE中断/轮询节拍保证DR已空) */
static inline void gtPort_txByte(gridTalk_t *gt, uint8_t c)
{
  ((USART_TypeDef *)gt->hw)->DR = c;
}

/* 帧字节已备好(库已置SENDING并填好txRing): 开TXE中断逐字节交付 */
static inline void gtPort_txKick(gridTalk_t *gt)
{
  ((USART_TypeDef *)gt->hw)->CR1 |= USART_CR1_TXEIE;
}

/* 末字节已交付(库内txRing已空): 关TXE中断, 改等TC(移位完成) */
static inline void gtPort_txKickDone(gridTalk_t *gt)
{
  USART_TypeDef *u = (USART_TypeDef *)gt->hw;
  u->CR1 = (u->CR1 & (uint32_t)~USART_CR1_TXEIE) | USART_CR1_TCIE;
}

/* 发送卡死兜底: 关发送中断, 清TC挂起标志 */
static inline void gtPort_txAbort(gridTalk_t *gt)
{
  USART_TypeDef *u = (USART_TypeDef *)gt->hw;
  u->CR1 &= (uint32_t)~(USART_CR1_TXEIE | USART_CR1_TCIE);
  u->SR  &= (uint32_t)~USART_SR_TC;
}

/* 一键绑定(在 gridTalk_init 之后调用) */
static inline void gridTalk_portBindUart(gridTalk_t *gt)
{
  gt->txByte     = gtPort_txByte;
  gt->txKick     = gtPort_txKick;
  gt->txKickDone = gtPort_txKickDone;
  gt->txAbort    = gtPort_txAbort;
}

/* USART中断服务程序统一入口:
   void USART2_IRQHandler(void){ gridTalk_portUsartIsr(&gt2); }
   TXE事件 -> gridTalk_txIrq(交付下一字节);
   TC事件  -> 清标志关中断后调 gridTalk_txCompleteIrq(帧收尾/485换向)。 */
static inline void gridTalk_portUsartIsr(gridTalk_t *gt)
{
  USART_TypeDef *u = (USART_TypeDef *)gt->hw;
  uint32_t sr = u->SR;
  if(sr & USART_SR_TXE)
  {
    gridTalk_txIrq(gt);
  }
  if(sr & USART_SR_TC)
  {
    u->SR  &= (uint32_t)~USART_SR_TC;    /* 写0清除(TC为rc_w0) */
    u->CR1 &= (uint32_t)~USART_CR1_TCIE;
    gridTalk_txCompleteIrq(gt);
  }
}

/* ---- µs时基绑定(DWT CYCCNT): 库内余数精确换算, 免回绕失真 ----
   不得把"自由计数器先除后换算"(如CYCCNT/100)直接接 gt->nowUs: 2^32/100
   非整数, 计数回绕后差值失真(100MHz下每~43s制造一批假超时, 差值读出
   ~71.5min假值, 见docs/history)。绑定后µs基由库内累加器维护, 采样率由
   周期调用 gridTalk_usTick 保证(须小于计数回绕周期, 100MHz约42.9s;
   1ms节拍即满足)。时基取 SystemCoreClock(须与实际主频一致, 特殊频率可
   事后改写 gt->tickPerUs)。可多实例共用/运行中绑定, 不清零CYCCNT。 */
static inline uint32_t gtPort_dwtTick(void)
{
  return DWT->CYCCNT;
}

/* 一键绑定(在 gridTalk_init 之后调用) */
static inline void gridTalk_portBindDwt(gridTalk_t *gt)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 释放DWT(已使能则无害) */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  gt->nowTick   = gtPort_dwtTick;
  gt->tickPerUs = SystemCoreClock / 1000000u;
  if(gt->tickPerUs == 0u) gt->tickPerUs = 1u;
  gt->_lastTick = DWT->CYCCNT;   /* 与当前计数对齐(不清零, 兼容多实例) */
}
#endif /* GRIDTALK_PORT_STM32_H */
