#include "main.h"
void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void DebugMon_Handler(void) { }
/* SVC, PendSV, SysTick: defined exclusively by FreeRTOS ARM_CM3/port.c.
 * TIM2: defined by the independent HAL timebase implementation. */
