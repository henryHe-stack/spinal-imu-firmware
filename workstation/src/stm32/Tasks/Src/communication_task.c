#include "app_tasks.h"

#include "app_config.h"
#include "diagnostics_service.h"
#include "protocol_service.h"
#include "task.h"

#define SESSION_ANNOUNCE_PERIOD_US (200000ULL)

void CommunicationTask_Entry(void *argument)
{
    AppTaskContext *context = (AppTaskContext *)argument;
    AppSampleFrame frame;
    AppControlCommand reply_cmd;
    uint8_t tx_buffer[APP_PROTOCOL_MAX_FRAME_SIZE];
    size_t tx_length = 0U;
    uint32_t tx_sequence = 1U;
    const uint32_t boot_id = 1U;
    uint32_t session_id = 1U;
    uint32_t config_id = 1U;
    uint64_t last_session_emit_us = 0ULL;
    bool session_sent = false;
    AppStatus status;

    configASSERT(context != NULL);
    configASSERT(context->uart != NULL);
    configASSERT(context->timebase != NULL);
    configASSERT(context->sample_queue != NULL);

    for (;;)
    {
        uint64_t now_us = BspTime_NowUs64(context->timebase);

        while ((context->reply_queue != NULL) &&
               (xQueueReceive(context->reply_queue, &reply_cmd, 0U) == pdPASS))
        {
            now_us = BspTime_NowUs64(context->timebase);
            status = ProtocolService_EncodeReplyFrame(&reply_cmd,
                                                      reply_cmd.result,
                                                      tx_sequence++,
                                                      boot_id,
                                                      session_id,
                                                      config_id,
                                                      now_us,
                                                      tx_buffer,
                                                      sizeof(tx_buffer),
                                                      &tx_length);
            if (status == APP_STATUS_OK)
            {
                if (BspUart_SendBlocking(context->uart, tx_buffer, tx_length) != APP_STATUS_OK)
                {
                    DiagnosticsService_Increment(APP_DIAG_UART_TX_ERROR);
                }
            }
            if (reply_cmd.result == 0U && ((reply_cmd.opcode == 4U) || (reply_cmd.opcode == 5U)))
            {
                session_id++;
                session_sent = false;
            }
        }

        now_us = BspTime_NowUs64(context->timebase);
        if (!session_sent || ((now_us - last_session_emit_us) >= SESSION_ANNOUNCE_PERIOD_US))
        {
            status = ProtocolService_EncodeSessionFrame(tx_sequence++,
                                                        boot_id,
                                                        session_id,
                                                        config_id,
                                                        now_us,
                                                        APP_ACQUISITION_PERIOD_MS * 1000U,
                                                        tx_buffer,
                                                        sizeof(tx_buffer),
                                                        &tx_length);
            if (status == APP_STATUS_OK)
            {
                if (BspUart_SendBlocking(context->uart, tx_buffer, tx_length) == APP_STATUS_OK)
                {
                    session_sent = true;
                    last_session_emit_us = now_us;
                }
                else
                {
                    DiagnosticsService_Increment(APP_DIAG_UART_TX_ERROR);
                }
            }
        }

        if (xQueueReceive(context->sample_queue, &frame, pdMS_TO_TICKS(100U)) != pdPASS)
        {
            DiagnosticsService_Heartbeat(APP_HEARTBEAT_COMMUNICATION);
            continue;
        }

        now_us = BspTime_NowUs64(context->timebase);
        if (frame.stream_session != session_id) continue;
        status = ProtocolService_EncodeSampleFrame(&frame,
                                                   tx_sequence++,
                                                   boot_id,
                                                   session_id,
                                                   config_id,
                                                   now_us,
                                                   APP_ACQUISITION_PERIOD_MS * 1000U,
                                                   tx_buffer,
                                                   sizeof(tx_buffer),
                                                   &tx_length);
        if (status != APP_STATUS_OK)
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
