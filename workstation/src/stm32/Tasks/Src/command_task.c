#include "app_tasks.h"

#include <string.h>

#include "diagnostics_service.h"
#include "protocol_service.h"
#include "task.h"

#define COMMAND_RX_BUFFER_SIZE (128U)
#define COMMAND_POLL_PERIOD_MS (10U)

void CommandTask_Entry(void *argument)
{
    AppTaskContext *context = (AppTaskContext *)argument;
    uint8_t rx_buffer[COMMAND_RX_BUFFER_SIZE];
    size_t buffered = 0U;
    size_t rx_length;
    size_t consumed;
    AppControlCommand command;
    AppStatus status;

    configASSERT(context != NULL);
    configASSERT(context->uart != NULL);
    configASSERT(context->command_queue != NULL);

    for (;;)
    {
        if (buffered >= sizeof(rx_buffer))
        {
            buffered = 0U;
        }

        rx_length = BspUart_ReadAvailable(context->uart,
                                          &rx_buffer[buffered],
                                          sizeof(rx_buffer) - buffered);
        buffered += rx_length;

        while (buffered > 0U)
        {
            consumed = 0U;
            status = ProtocolService_ParseCommand(rx_buffer,
                                                  buffered,
                                                  &command,
                                                  &consumed);
            if (consumed > 0U)
            {
                if (consumed < buffered)
                {
                    memmove(rx_buffer, &rx_buffer[consumed], buffered - consumed);
                }
                buffered -= consumed;
            }

            if (status == APP_STATUS_OK)
            {
                if (xQueueSend(context->command_queue, &command, 0U) != pdPASS)
                {
                    DiagnosticsService_Increment(APP_DIAG_COMMAND_PARSE_ERROR);
                }
            }
            else
            {
                break;
            }
        }

        DiagnosticsService_Heartbeat(APP_HEARTBEAT_COMMAND);
        vTaskDelay(pdMS_TO_TICKS(COMMAND_POLL_PERIOD_MS));
    }
}
