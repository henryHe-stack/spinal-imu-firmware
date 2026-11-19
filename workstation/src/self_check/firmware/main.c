#include "diag_platform.h"
#include "FreeRTOS.h"
#include "task.h"
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

static void clock_init(void)
{
    RCC_OscInitTypeDef osc={0}; RCC_ClkInitTypeDef clk={0};
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSI;
    osc.HSIState=RCC_HSI_ON;osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState=RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc)!=HAL_OK) Error_Handler();
    clk.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_HSI;clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV1;clk.APB2CLKDivider=RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_0)!=HAL_OK) Error_Handler();
}
int main(void)
{
    HAL_Init();clock_init();
    hi2c1.Instance=I2C1;hi2c1.Init.ClockSpeed=100000;
    hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1)!=HAL_OK) Error_Handler();
    huart2.Instance=USART2;huart2.Init.BaudRate=115200;
    huart2.Init.WordLength=UART_WORDLENGTH_8B;huart2.Init.StopBits=UART_STOPBITS_1;
    huart2.Init.Parity=UART_PARITY_NONE;huart2.Init.Mode=UART_MODE_TX;
    huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;huart2.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart2)!=HAL_OK || !diag_timer_start()) Error_Handler();
    diag_start();vTaskStartScheduler();Error_Handler();return 0;
}
void Error_Handler(void)
{ __disable_irq();g_diag_fatal=1;for(;;) { __NOP(); } }
void neck_assert_failed(const char *file,unsigned long line)
{ (void)file;(void)line;Error_Handler(); }
void vApplicationMallocFailedHook(void) { Error_Handler(); }
void vApplicationStackOverflowHook(TaskHandle_t task,char *name)
{ (void)task;(void)name;Error_Handler(); }
