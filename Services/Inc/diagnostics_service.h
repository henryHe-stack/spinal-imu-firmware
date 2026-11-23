#ifndef DIAGNOSTICS_SERVICE_H
#define DIAGNOSTICS_SERVICE_H

#include <stdint.h>

typedef enum
{
    APP_HEARTBEAT_ACQUISITION = 0,
    APP_HEARTBEAT_COMMUNICATION,
    APP_HEARTBEAT_COMMAND,
    APP_HEARTBEAT_HEALTH,
    APP_HEARTBEAT_COUNT
} AppHeartbeatId;

typedef enum
{
    APP_DIAG_SENSOR_READ_ERROR = 0,
    APP_DIAG_SAMPLE_QUEUE_DROP,
    APP_DIAG_PROTOCOL_ERROR,
    APP_DIAG_UART_TX_ERROR,
    APP_DIAG_COMMAND_PARSE_ERROR,
    APP_DIAG_ACQUISITION_OVERRUN,
    APP_DIAG_REPLY_DROP,
    APP_DIAG_COUNT
} AppDiagnosticCounterId;

typedef struct
{
    uint32_t heartbeat[APP_HEARTBEAT_COUNT];
    uint32_t counters[APP_DIAG_COUNT];
    uint32_t reset_flags;
    uint32_t uart_errors[2];
    uint32_t uart_overflows[2];
    uint16_t stack_free_words[APP_HEARTBEAT_COUNT];
    uint8_t healthy;
    uint8_t bluetooth_connected;
} AppDiagnosticsSnapshot;
void DiagnosticsService_UpdateHealth(const AppDiagnosticsSnapshot *health);

void DiagnosticsService_Init(void);
void DiagnosticsService_Heartbeat(AppHeartbeatId id);
void DiagnosticsService_Increment(AppDiagnosticCounterId id);
void DiagnosticsService_GetSnapshot(AppDiagnosticsSnapshot *snapshot);

#endif /* DIAGNOSTICS_SERVICE_H */
