#include "app_tasks.h"

#include "app_config.h"
#include "diagnostics_service.h"
#include "protocol_service.h"
#include "task.h"

void CommunicationTask_Entry(void *argument)
{
    AppTaskContext *context = (AppTaskContext *)argument;
    AppSampleFrame frame;
    uint8_t tx_buffer[APP_PROTOCOL_MAX_FRAME_SIZE];
    size_t tx_length;
    AppStatus status;

    configASSERT(context != NULL);
    configASSERT(context->uart != NULL);
    configASSERT(context->sample_queue != NULL);

    for (;;)
    {
        if (xQueueReceive(context->sample_queue, &frame, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        status = ProtocolService_EncodeSampleFrame(&frame,
                                                   tx_buffer,
                                                   sizeof(tx_buffer),
                                                   &tx_length);
        if (status == APP_STATUS_NOT_IMPLEMENTED)
        {
            /* Expected during framework bring-up; no bytes are transmitted. */
        }
        else if (status != APP_STATUS_OK)
        {
            DiagnosticsService_Increment(APP_DIAG_PROTOCOL_ERROR);
        }
        else if (BspUart_SendBlocking(context->uart, tx_buffer, tx_length) != APP_STATUS_OK)
        {
            DiagnosticsService_Increment(APP_DIAG_UART_TX_ERROR);
        }

        DiagnosticsService_Heartbeat(APP_HEARTBEAT_COMMUNICATION);
    }
}
