#include "diagnostics_service.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static AppDiagnosticsSnapshot g_diagnostics;

void DiagnosticsService_Init(void)
{
    taskENTER_CRITICAL();
    memset(&g_diagnostics, 0, sizeof(g_diagnostics));
    taskEXIT_CRITICAL();
}

void DiagnosticsService_Heartbeat(AppHeartbeatId id)
{
    if (id >= APP_HEARTBEAT_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_diagnostics.heartbeat[id]++;
    taskEXIT_CRITICAL();
}

void DiagnosticsService_Increment(AppDiagnosticCounterId id)
{
    if (id >= APP_DIAG_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_diagnostics.counters[id]++;
    taskEXIT_CRITICAL();
}

void DiagnosticsService_GetSnapshot(AppDiagnosticsSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = g_diagnostics;
    taskEXIT_CRITICAL();
}
