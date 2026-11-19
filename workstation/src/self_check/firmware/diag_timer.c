#include "diag_platform.h"
#include "diag_stats.h"
#ifndef DIAG_ENABLE_MARKER
#define DIAG_ENABLE_MARKER 0
#endif
static volatile uint64_t overflow_base;

int diag_timer_start(void)
{
    RCC_ClkInitTypeDef clk;uint32_t latency,hz;
    HAL_RCC_GetClockConfig(&clk,&latency);hz=HAL_RCC_GetPCLK1Freq();
    if(clk.APB1CLKDivider!=RCC_HCLK_DIV1) hz*=2u;
    if(hz<1000000u || hz%1000000u) return 0;
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1=0;TIM3->DIER=0;TIM3->PSC=hz/1000000u-1u;
    TIM3->ARR=65535;TIM3->EGR=TIM_EGR_UG;TIM3->SR=0;TIM3->CNT=0;
    overflow_base=0;
    HAL_NVIC_SetPriority(TIM3_IRQn,4u,0u);HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);TIM3->DIER=TIM_DIER_UIE;TIM3->CR1=TIM_CR1_CEN;
#if DIAG_ENABLE_MARKER
    {
        GPIO_InitTypeDef gpio={0};__HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
        gpio.Pin=GPIO_PIN_8;gpio.Mode=GPIO_MODE_OUTPUT_PP;gpio.Speed=GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA,&gpio);
    }
#endif
    return 1;
}
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) && (TIM3->DIER & TIM_DIER_UIE)) {
        TIM3->SR=(uint16_t)~TIM_SR_UIF;
        overflow_base+=65536u;
    }
}
uint64_t diag_now_us(void)
{
    uint32_t prior=__get_PRIMASK();uint64_t base,result;
    uint16_t before,after;uint8_t pending;
    __disable_irq();
    base=overflow_base;before=(uint16_t)TIM3->CNT;
    pending=(TIM3->SR&TIM_SR_UIF)?1u:0u;after=(uint16_t)TIM3->CNT;
    result=diag_timer_snapshot(base,before,pending,after);
    __set_PRIMASK(prior);return result;
}
void diag_marker(int high)
{
#if DIAG_ENABLE_MARKER
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,high?GPIO_PIN_SET:GPIO_PIN_RESET);
#else
    (void)high;
#endif
}
