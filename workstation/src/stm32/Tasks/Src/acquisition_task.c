#include "app_tasks.h"

#include <stdbool.h>

#include "app_config.h"
#include "diagnostics_service.h"
#include "task.h"

void AcquisitionTask_Entry(void *argument)
{
    AppTaskContext *context = (AppTaskContext *)argument;
    AppSampleFrame frame;
    AppControlCommand command;
    bool streaming_enabled = true;
    uint32_t active_session = 1U;
    uint32_t period_ms = APP_ACQUISITION_PERIOD_MS;
    uint32_t reinitialize_countdown = 0U;
    AppStatus sensor_status;
    TickType_t last_wake_tick;

    configASSERT(context != NULL);
    configASSERT(context->sensor_service != NULL);
    configASSERT(context->i2c != NULL);
    configASSERT(context->timebase != NULL);
    configASSERT(context->sample_queue != NULL);
    configASSERT(context->command_queue != NULL);

    /* Sensor discovery and every later I2C transaction stay in this task. */
    sensor_status = SensorService_Init(context->sensor_service,
                                       context->i2c,
                                       context->timebase);

    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        /* This task is the sole owner of I2C1, TCA9548A and all BNO055 nodes. */
        while (xQueueReceive(context->command_queue, &command, 0U) == pdPASS)
        {
            if (command.boot != 1U || command.config != 1U || command.session != active_session) {
                command.result = 1U;
            } else if (command.result == 0U && command.opcode == 1U) {
                command.value = streaming_enabled ? 0U : 1U;
            }
            if (command.result == 0U && command.id == APP_COMMAND_START_STREAM)
            {
                streaming_enabled = true;
            }
            else if (command.result == 0U && command.id == APP_COMMAND_STOP_STREAM)
            {
                streaming_enabled = false;
            }
            if (command.result == 0U && (command.opcode == 4U || command.opcode == 5U)) {
                xQueueReset(context->sample_queue);
                ++active_session;
            }

            if ((context->reply_queue != NULL) && (command.opcode != 8U))
            {
                (void)xQueueSend(context->reply_queue, &command, portMAX_DELAY);
            }
        }

        if (streaming_enabled)
        {
            if (sensor_status != APP_STATUS_OK)
            {
                /* Retry approximately once per second without blocking other tasks. */
                if (reinitialize_countdown == 0U)
                {
                    sensor_status = SensorService_Init(context->sensor_service,
                                                       context->i2c,
                                                       context->timebase);
                    reinitialize_countdown = 1000U / period_ms;
                    last_wake_tick = xTaskGetTickCount();
                }
                else
                {
                    reinitialize_countdown--;
                }
            }

            if (SensorService_ReadCycle(context->sensor_service, &frame) != APP_STATUS_OK)
            {
                DiagnosticsService_Increment(APP_DIAG_SENSOR_READ_ERROR);
            }

            frame.stream_session = active_session;
            /* Bounded policy: keep acquisition periodic and drop the newest frame if full. */
            if (xQueueSend(context->sample_queue, &frame, 0U) != pdPASS)
            {
                DiagnosticsService_Increment(APP_DIAG_SAMPLE_QUEUE_DROP);
            }
        }

        DiagnosticsService_Heartbeat(APP_HEARTBEAT_ACQUISITION);
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(period_ms));
    }
}
