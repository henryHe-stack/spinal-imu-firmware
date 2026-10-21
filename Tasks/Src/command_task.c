#include "app_tasks.h"

#include "diagnostics_service.h"
#include "protocol_service.h"
#include "task.h"

#define COMMAND_RX_CHUNK_SIZE (64U)
#define COMMAND_POLL_PERIOD_MS (10U)

void CommandTask_Entry(void *argument)
{
    AppTaskContext *context = (AppTaskContext *)argument;
    uint8_t rx_data[COMMAND_RX_CHUNK_SIZE];
    size_t rx_length;
    size_t consumed;
    AppControlCommand command;
    AppStatus status;

    configASSERT(context != NULL);
    configASSERT(context->uart != NULL);
    configASSERT(context->command_queue != NULL);

    for (;;)
    {
        rx_length = BspUart_ReadAvailable(context->uart, rx_data, sizeof(rx_data));
        if (rx_length > 0U)
        {
            status = ProtocolService_ParseCommand(rx_data,
                                                  rx_length,
                                                  &command,
                                                  &consumed);
            (void)consumed;

            if (status == APP_STATUS_OK)
            {
                if (xQueueSend(context->command_queue, &command, 0U) != pdPASS)
                {
                    DiagnosticsService_Increment(APP_DIAG_COMMAND_PARSE_ERROR);
                }
            }
            else if (status != APP_STATUS_NOT_IMPLEMENTED)
            {
                DiagnosticsService_Increment(APP_DIAG_COMMAND_PARSE_ERROR);
            }
        }

        DiagnosticsService_Heartbeat(APP_HEARTBEAT_COMMAND);
        vTaskDelay(pdMS_TO_TICKS(COMMAND_POLL_PERIOD_MS));
    }
}
