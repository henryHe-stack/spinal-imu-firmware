#include "bsp_time.h"

#include "FreeRTOS.h"
#include "task.h"

static volatile uint64_t overflow_base;
void BspTime_OnOverflow(void)
{
    overflow_base += 65536ULL;
}

AppStatus BspTime_Init(BspTime *self, TIM_HandleTypeDef *timer)
{
    if (self == NULL)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    self->timer = timer;
    overflow_base = 0ULL;
    self->high_resolution_ready = false;

    if ((timer != NULL) && (timer->Instance != NULL))
    {
        /* Ensure 1 MHz tick rate on 72 MHz APB1 timer clock (PSC = 71). */
        __HAL_TIM_SET_PRESCALER(timer, 71U);
        timer->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
        if (HAL_TIM_Base_Start_IT(timer) == HAL_OK)
        {
            self->high_resolution_ready = true;
        }
    }

    return APP_STATUS_OK;
}

uint64_t BspTime_NowUs64(BspTime *self)
{
    uint16_t current_cnt;
    uint32_t irq_mask;
    uint64_t now;

    if ((self == NULL) || !self->high_resolution_ready || (self->timer == NULL))
    {
        return ((uint64_t)xTaskGetTickCount()) * 1000ULL;
    }

    irq_mask = __get_PRIMASK();
    __disable_irq();
    now = overflow_base;
    current_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(self->timer);
    if (__HAL_TIM_GET_FLAG(self->timer, TIM_FLAG_UPDATE)) {
        now += 65536ULL;
        current_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(self->timer);
    }
    __set_PRIMASK(irq_mask);
    now += current_cnt;

    return now;
}

uint32_t BspTime_NowUs(BspTime *self)
{
    return (uint32_t)BspTime_NowUs64(self);
}

bool BspTime_IsHighResolution(const BspTime *self)
{
    return (self != NULL) && self->high_resolution_ready;
}
