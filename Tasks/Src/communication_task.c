#include "app_tasks.h"
#include "diagnostics_service.h"
#include "protocol_service.h"
#include "task.h"

/* One dispatcher fans out a frame; two consumers on the sample queue would
 * divide frames rather than broadcast. Each UART owns its own DMA TX copy. */
void CommunicationTask_Entry(void *argument)
{
    AppTaskContext *context=argument;
    AppSampleFrame frame; AppCommandReply reply;
    uint8_t bytes[APP_PROTOCOL_MAX_FRAME_SIZE]; size_t length;
    bool reply_pending=false;
    configASSERT(context && context->sample_queue && context->reply_queue);
    for (;;)
    {
        if (!reply_pending) reply_pending=xQueueReceive(context->reply_queue,&reply,0)==pdPASS;
        if (reply_pending)
        {
            if (ProtocolService_EncodeReply(&reply,bytes,sizeof(bytes),&length)==APP_STATUS_OK)
            {
                AppStatus status=BspUart_SendDma(context->ports[reply.destination],bytes,length);
                if (status!=APP_STATUS_BUSY) reply_pending=false;
            }
            else reply_pending=false;
        }
        if (xQueueReceive(context->sample_queue,&frame,pdMS_TO_TICKS(2))==pdPASS)
        {
            if (ProtocolService_EncodeSampleFrame(&frame,bytes,sizeof(bytes),&length)!=APP_STATUS_OK)
                DiagnosticsService_Increment(APP_DIAG_PROTOCOL_ERROR);
            else for (unsigned i=0;i<APP_PORT_COUNT;++i)
            {
                if (i==APP_PORT_BT && !APP_BT_STREAM_ENABLED) continue;
                /* Prioritize a queued reply on that port; acquisition never blocks. */
                if (reply_pending && i==(unsigned)reply.destination) continue;
                if (BspUart_SendDma(context->ports[i],bytes,length)!=APP_STATUS_OK)
                    DiagnosticsService_Increment(APP_DIAG_UART_TX_ERROR);
            }
        }
        DiagnosticsService_Heartbeat(APP_HEARTBEAT_COMMUNICATION);
    }
}
