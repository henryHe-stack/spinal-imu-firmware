#include "app_tasks.h"
#include "app_config.h"
#include "diagnostics_service.h"
#include "task.h"

void AcquisitionTask_Entry(void *argument)
{
    AppTaskContext *context=argument;
    AppSampleFrame frame; AppControlCommand command;
    bool streaming=true; uint32_t period_ms=APP_ACQUISITION_PERIOD_MS;
    configASSERT(context && context->sensor_service && context->reply_queue);
    (void)SensorService_Init(context->sensor_service,context->i2c,context->timebase);
    TickType_t wake=xTaskGetTickCount();
    for (;;)
    {
        /* Bound command work: a continuously active UART must not starve
         * sensor acquisition. Overflow is reported by CommandTask. */
        for (unsigned budget=0; budget<APP_COMMAND_QUEUE_LENGTH &&
             xQueueReceive(context->command_queue,&command,0)==pdPASS; ++budget)
        {
            AppStatus status=APP_STATUS_OK; uint32_t value=command.value;
            switch(command.id)
            {
                case APP_COMMAND_START_STREAM: streaming=true; break;
                case APP_COMMAND_STOP_STREAM: streaming=false; break;
                case APP_COMMAND_SET_SAMPLE_PERIOD_MS:
                    /* 50Hz max until real I2C cycle timing is measured. */
                    if (value<20 || value>1000) status=APP_STATUS_INVALID_ARGUMENT;
                    else { period_ms=value; wake=xTaskGetTickCount(); }
                    break;
                case APP_COMMAND_GET_INFO: value=0x00010001UL; break; /* firmware 0.1.0.1 */
                case APP_COMMAND_GET_STATUS:
                    value=streaming ? 0x100U : 0U;
                    for (unsigned i=0;i<3;++i) if(context->sensor_service->sensors[i].configured) value|=1U<<i;
                    break;
                case APP_COMMAND_SAVE_CALIBRATION:
                    status=streaming ? APP_STATUS_BUSY : SensorService_ExecuteCommand(context->sensor_service,&command);
                    wake=xTaskGetTickCount(); break;
                default: status=SensorService_ExecuteCommand(context->sensor_service,&command); break;
            }
            AppCommandReply reply={.request_id=command.request_id,.value=value,.command=(uint8_t)command.id,
                .status=(uint8_t)status,.destination=command.source};
            if (xQueueSend(context->reply_queue,&reply,0)!=pdPASS) DiagnosticsService_Increment(APP_DIAG_REPLY_DROP);
        }
        AppStatus sensor_status=SensorService_ReadCycle(context->sensor_service,&frame);
        if (sensor_status!=APP_STATUS_OK)
            DiagnosticsService_Increment(APP_DIAG_SENSOR_READ_ERROR);
        /* Do not couple transmission to the aggregate service status. A
         * single healthy node is enough to publish a frame; invalid nodes
         * remain in their fixed slots with SENSOR_ERROR for the host to see.
         * An all-invalid frame is suppressed so the PC cannot mistake a
         * zero-filled placeholder for a real measurement. */
        if (streaming && SensorService_FrameHasValidSample(&frame))
        {
            if (xQueueSend(context->sample_queue,&frame,0)!=pdPASS)
                DiagnosticsService_Increment(APP_DIAG_SAMPLE_QUEUE_DROP);
        }
        DiagnosticsService_Heartbeat(APP_HEARTBEAT_ACQUISITION);
        TickType_t period=pdMS_TO_TICKS(period_ms), now=xTaskGetTickCount();
        if ((TickType_t)(now-wake)>=period)
        { DiagnosticsService_Increment(APP_DIAG_ACQUISITION_OVERRUN); wake=now; }
        vTaskDelayUntil(&wake,period);
    }
}
