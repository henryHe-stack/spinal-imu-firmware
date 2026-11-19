#pragma once
#include <stdint.h>
typedef struct { unsigned unused; } I2C_HandleTypeDef;
typedef struct { uint32_t EGR, CNT, SR, PSC; } FakeTimer;
typedef struct { FakeTimer *Instance; } TIM_HandleTypeDef;
#define TIM_EGR_UG 1U
#define TIM_FLAG_UPDATE 1U
#define HAL_OK 0
#define __HAL_TIM_SET_PRESCALER(h,v) ((h)->Instance->PSC=(v))
#define __HAL_TIM_CLEAR_FLAG(h,v) ((h)->Instance->SR &= ~(v))
#define __HAL_TIM_GET_FLAG(h,v) ((h)->Instance->SR & (v))
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
static inline unsigned __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(unsigned v) { (void)v; }
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h);
