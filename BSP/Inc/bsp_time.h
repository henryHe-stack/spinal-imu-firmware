#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdbool.h>
#include <stdint.h>

#include "app_status.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    TIM_HandleTypeDef *timer;
    bool high_resolution_ready;
} BspTime;

AppStatus BspTime_Init(BspTime *self, TIM_HandleTypeDef *timer);
uint32_t BspTime_NowUs(const BspTime *self);
bool BspTime_IsHighResolution(const BspTime *self);

#endif /* BSP_TIME_H */
