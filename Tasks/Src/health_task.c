#include "app_tasks.h"
#include "app_config.h"
#include "diagnostics_service.h"
#include "stm32f1xx_hal.h"

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{ (void)task; (void)name; taskDISABLE_INTERRUPTS(); for (;;) {} }
void vApplicationMallocFailedHook(void)
{ taskDISABLE_INTERRUPTS(); for (;;) {} }
void HealthTask_Entry(void *argument)
{
    AppTaskContext *context=argument;
    AppDiagnosticsSnapshot previous={0}, current={0};
    uint8_t missed[3]={0}; bool watchdog_started=false;
    uint32_t reset_flags=RCC->CSR; __HAL_RCC_CLEAR_RESET_FLAGS();
    configASSERT(context);
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(APP_HEALTH_PERIOD_MS));
        DiagnosticsService_GetSnapshot(&current); current.healthy=1; current.reset_flags=reset_flags;
        for (unsigned i=0;i<3;++i)
        {
            missed[i]=current.heartbeat[i]==previous.heartbeat[i] ? missed[i]+1 : 0;
            if (missed[i]>=3) { missed[i]=3; current.healthy=0; }
        }
        for (unsigned i=0;i<4;++i)
        {
            current.stack_free_words[i]=(uint16_t)uxTaskGetStackHighWaterMark(context->task_handles[i]);
            if (current.stack_free_words[i]<24) current.healthy=0;
        }
        for (unsigned i=0;i<APP_PORT_COUNT;++i)
        { current.uart_errors[i]=context->ports[i]->errors; current.uart_overflows[i]=context->ports[i]->rx_overflows; }
        current.bluetooth_connected=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==GPIO_PIN_SET;
        if (current.healthy) HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        else HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
        DiagnosticsService_UpdateHealth(&current); previous=current;
        /* Register-level IWDG avoids adding a Cube-generated IWDG instance.
         * /256, reload4095 is nominal ~26s at 40kHz LSI; oscillator varies.
         * Remains opt-in for bench bring-up; never claim watchdog is enabled. */
        if (APP_ENABLE_WATCHDOG && current.healthy)
        {
            if (!watchdog_started)
            {
                /* Start first: this enables LSI, as HAL_IWDG_Init does.
                 * A configuration failure deliberately allows a reset. */
                IWDG->KR=0xCCCC;
                IWDG->KR=0x5555; IWDG->PR=6; IWDG->RLR=4095;
                uint32_t started=HAL_GetTick();
                while (IWDG->SR && (uint32_t)(HAL_GetTick()-started)<1000U) {}
                if (IWDG->SR) { for (;;) { vTaskDelay(pdMS_TO_TICKS(100)); } }
                watchdog_started=true;
            }
            IWDG->KR=0xAAAA;
        }
        DiagnosticsService_Heartbeat(APP_HEARTBEAT_HEALTH);
    }
}
