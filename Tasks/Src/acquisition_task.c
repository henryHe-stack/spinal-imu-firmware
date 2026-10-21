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
    uint32_t reinitialize_countdown = 0U;
    AppStatus sensor_status;

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

    for (;;)
    {
        /* This task is the sole owner of I2C1, TCA9548A and all BNO055 nodes. */
        while (xQueueReceive(context->command_queue, &command, 0U) == pdPASS)
        {
            if (command.id == APP_COMMAND_START_STREAM)
            {
                streaming_enabled = true;
            }
            else if (command.id == APP_COMMAND_STOP_STREAM)
            {
                streaming_enabled = false;
            }
            else
            {
                (void)SensorService_ExecuteCommand(context->sensor_service, &command);
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
                    reinitialize_countdown = 1000U / APP_ACQUISITION_PERIOD_MS;
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

            /* Bounded policy: keep acquisition periodic and drop the newest frame. */
            if (xQueueSend(context->sample_queue, &frame, 0U) != pdPASS)
            {
                DiagnosticsService_Increment(APP_DIAG_SAMPLE_QUEUE_DROP);
            }
        }

        DiagnosticsService_Heartbeat(APP_HEARTBEAT_ACQUISITION);
        vTaskDelay(pdMS_TO_TICKS(APP_ACQUISITION_PERIOD_MS));
    }
}
