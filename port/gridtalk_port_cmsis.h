#ifndef GRIDTALK_PORT_CMSIS_H
#define GRIDTALK_PORT_CMSIS_H
/*
 * GridTalk ARM(CMSIS)端口: 临界区 = PRIMASK 保存/恢复。
 *
 * 仅依赖 cmsis_compiler.h(CMSIS-V5, 随STM32Cube等固件包提供)。
 * 由 GridTalk.h 在检测到 ARM/CMSIS 编译器时自动包含, 无需手工引入;
 * 若工程不可用该头: 定义 GT_PORT_NO_CMSIS 并自行提供 GT_CRIT_ENTER/GT_CRIT_EXIT
 * (或经 GT_PORT_HEADER 指定自备端口头)。
 */
#if defined(GT_PORT_NO_CMSIS)
  #if !defined(GT_CRIT_ENTER) || !defined(GT_CRIT_EXIT)
  #error "GT_PORT_NO_CMSIS: 须自行定义 GT_CRIT_ENTER / GT_CRIT_EXIT"
  #endif
#else
  #include "cmsis_compiler.h"
  #ifndef GT_CRIT_ENTER
  /* 保存/恢复式: 任务与ISR上下文均可安全调用,
     避免成对__disable_irq/__enable_irq在ISR里错误地提前开中断 */
  #define GT_CRIT_ENTER() uint32_t gtCritPm_ = __get_PRIMASK(); __disable_irq()
  #define GT_CRIT_EXIT()  __set_PRIMASK(gtCritPm_)
  #endif
#endif
#endif /* GRIDTALK_PORT_CMSIS_H */
