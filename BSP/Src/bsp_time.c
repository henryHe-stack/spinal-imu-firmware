#include "bsp_time.h"

static BspTime *active_time;
AppStatus BspTime_Init(BspTime *self, TIM_HandleTypeDef *timer)
{
    if (!self || !timer || timer->Instance != TIM2 || timer->Init.Prescaler != 71U
        || timer->Init.Period != 65535U) return APP_STATUS_INVALID_ARGUMENT;
    self->timer = timer; self->overflow_us = 0; self->high_resolution_ready = false;
    active_time = self;
    __HAL_TIM_SET_COUNTER(timer, 0); __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(timer) != HAL_OK) return APP_STATUS_IO_ERROR;
    self->high_resolution_ready = true;
    return APP_STATUS_OK;
}
void BspTime_OnOverflow(TIM_HandleTypeDef *timer)
{
    if (active_time && timer == active_time->timer) active_time->overflow_us += 65536U;
}
uint32_t BspTime_NowUs(const BspTime *self)
{
    if (!self || !self->high_resolution_ready) return 0;
    /* Atomic high/low snapshot, including one overflow pending in NVIC. Never
     * mask interrupts for >=65.536ms; uint32_t wraps after ~71.6 minutes. */
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    uint32_t high = self->overflow_us;
    uint32_t low = __HAL_TIM_GET_COUNTER(self->timer);
    if (__HAL_TIM_GET_FLAG(self->timer, TIM_FLAG_UPDATE))
    { high += 65536U; low = __HAL_TIM_GET_COUNTER(self->timer); }
    __set_PRIMASK(mask);
    return high + low;
}
bool BspTime_IsHighResolution(const BspTime *self)
{ return self && self->high_resolution_ready; }
