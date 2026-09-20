#ifndef GRIDTALK_PORT_STM32_H
#define GRIDTALK_PORT_STM32_H
/*
 * GridTalk STM32 USART 端口(可选): 寄存器直写绑定, 等价旧版 uart 直写模式。
 *
 * 用法(三行接入):
 *   gridTalk_init(&gt, USART2, myID);   // hw = USART外设
 *   gridTalk_portBindUart(&gt);          // txByte写DR + txKick等中断控制, 一行绑定
 *   // USART2_IRQHandler 里调用 gridTalk_portUsartIsr(&gt);
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

/* 轮询模式可选背压: DR空(可再收1字节)? */
static inline uint8_t gtPort_txReady(gridTalk_t *gt)
{
  return (((USART_TypeDef *)gt->hw)->SR & USART_SR_TXE) != 0u;
}

/* 一键绑定(在 gridTalk_init 之后调用) */
static inline void gridTalk_portBindUart(gridTalk_t *gt)
{
  gt->txByte     = gtPort_txByte;
  gt->txKick     = gtPort_txKick;
  gt->txKickDone = gtPort_txKickDone;
  gt->txAbort    = gtPort_txAbort;
  gt->txReady    = gtPort_txReady;
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
#endif /* GRIDTALK_PORT_STM32_H */
