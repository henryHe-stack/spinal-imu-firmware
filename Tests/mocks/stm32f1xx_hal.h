#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; DMA_HandleTypeDef *hdmatx; } UART_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
typedef struct { void *Instance; struct { uint32_t Prescaler, Period; } Init;
    uint32_t counter, flags; } TIM_HandleTypeDef;
#define TIM2 ((void *)2)
#define TIM_FLAG_UPDATE 1U
#define __HAL_TIM_SET_COUNTER(t,v) ((t)->counter=(v))
#define __HAL_TIM_CLEAR_FLAG(t,v) ((t)->flags &= ~(v))
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter)
#define __HAL_TIM_GET_FLAG(t,v) ((t)->flags & (v))
#define __DMB() ((void)0)
extern uint32_t mock_primask;
#define __get_PRIMASK() mock_primask
#define __disable_irq() (mock_primask=1)
#define __set_PRIMASK(v) (mock_primask=(v))
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n,uint32_t ms);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h,uint16_t n);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
#endif
