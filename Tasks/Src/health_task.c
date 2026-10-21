#include "app_tasks.h"

#include "app_config.h"
#include "diagnostics_service.h"
#include "task.h"

void HealthTask_Entry(void *argument)
{
    AppTaskContext *context = (AppTaskContext *)argument;
    AppDiagnosticsSnapshot previous = {0};
    AppDiagnosticsSnapshot current;

    configASSERT(context != NULL);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(APP_HEALTH_PERIOD_MS));
        DiagnosticsService_GetSnapshot(&current);

        /*
         * TODO(SYSTEM owner): compare per-task heartbeat deltas, queue watermarks
         * and reset causes. Feed the independent watchdog only if every required
         * task is healthy. Do not let this task consume application data queues.
         */
        previous = current;
        (void)previous;

        DiagnosticsService_Heartbeat(APP_HEARTBEAT_HEALTH);
    }
}
