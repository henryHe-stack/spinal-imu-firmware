#include "app_tasks.h"
#include "diagnostics_service.h"
#include "protocol_service.h"
#include "task.h"

void CommandTask_Entry(void *argument)
{
    AppTaskContext *context=argument;
    static ProtocolParser parsers[APP_PORT_COUNT];
    uint32_t previous_overflows[APP_PORT_COUNT]={0}, previous_errors[APP_PORT_COUNT]={0};
    uint8_t chunk[64]; AppControlCommand command;
    configASSERT(context && context->command_queue);
    for (;;)
    {
        for (unsigned port=0;port<APP_PORT_COUNT;++port)
        {
            BspUart *uart=context->ports[port]; BspUart_Service(uart);
            if (uart->rx_overflows!=previous_overflows[port] || uart->errors!=previous_errors[port])
            {
                parsers[port].length=0;
                previous_overflows[port]=uart->rx_overflows; previous_errors[port]=uart->errors;
            }
            /* Bound work per turn so one noisy port cannot starve its peer. */
            size_t length=BspUart_ReadAvailable(uart,chunk,sizeof(chunk));
            uint32_t errors=parsers[port].errors;
            for (size_t i=0;i<length;++i) if (ProtocolService_FeedCommand(&parsers[port],chunk[i],&command))
            {
                command.source=(AppPortId)port;
                if (xQueueSend(context->command_queue,&command,0)!=pdPASS)
                {
                    AppCommandReply reply={.request_id=command.request_id,.command=command.id,
                        .status=APP_STATUS_QUEUE_FULL,.destination=(AppPortId)port};
                    if(xQueueSend(context->reply_queue,&reply,0)!=pdPASS) DiagnosticsService_Increment(APP_DIAG_REPLY_DROP);
                }
            }
            if (errors!=parsers[port].errors) DiagnosticsService_Increment(APP_DIAG_COMMAND_PARSE_ERROR);
        }
        DiagnosticsService_Heartbeat(APP_HEARTBEAT_COMMAND);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
