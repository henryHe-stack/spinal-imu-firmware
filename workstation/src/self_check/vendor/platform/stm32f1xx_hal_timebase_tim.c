#include "main.h"
/* HAL timeouts use TIM2, not the FreeRTOS-owned SysTick. TIM2 is reserved. */
static TIM_HandleTypeDef hal_tick_timer;

HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency, timer_clock;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) timer_clock *= 2U;
    if (priority >= (1U << __NVIC_PRIO_BITS) || timer_clock < 1000000U) return HAL_ERROR;
    __HAL_RCC_TIM2_CLK_ENABLE();
    hal_tick_timer.Instance = TIM2;
    hal_tick_timer.Init.Period = 999U;
    hal_tick_timer.Init.Prescaler = timer_clock / 1000000U - 1U;
    hal_tick_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hal_tick_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    hal_tick_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&hal_tick_timer) != HAL_OK) return HAL_ERROR;
    __HAL_TIM_CLEAR_FLAG(&hal_tick_timer, TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM2_IRQn, priority, 0U);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    uwTickPrio = priority;
    return HAL_TIM_Base_Start_IT(&hal_tick_timer);
}

void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&hal_tick_timer, TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&hal_tick_timer, TIM_IT_UPDATE); }
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&hal_tick_timer); }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM2) HAL_IncTick();
}
