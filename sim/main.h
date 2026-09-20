#ifndef _SIM_MAIN_H_
#define _SIM_MAIN_H_

/*
 * PC仿真用假HAL头: 替代STM32的main.h
 * 只提供GridTalk.c实际用到的寄存器/中断原语
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct USART_TypeDef USART_TypeDef;
struct USART_TypeDef {
  volatile uint32_t SR;      /* 状态寄存器(仿真维护TXE/TC位) */
  volatile uint32_t CR1;     /* 控制寄存器(仿真维护TXEIE/TCIE位) */
  volatile uint32_t DR;      /* 数据寄存器(仿真只读, 写入经GT_UART_DR_WRITE截获) */
  void *sim;                 /* 指向仿真设备状态 */
};

#define USART_SR_TXE  0x80u
#define USART_SR_TC   0x40u
#define USART_SR_RXNE 0x20u
#define USART_CR1_TXEIE 0x80u
#define USART_CR1_TCIE  0x40u

/* 中断原语: PC仿真为空操作(单线程) */
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __set_PRIMASK(uint32_t v) { (void)v; }

#endif /* _SIM_MAIN_H_ */
