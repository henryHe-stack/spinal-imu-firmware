#include "app_bootstrap.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_config.h"
#include "app_tasks.h"
#include "app_types.h"
#include "bsp_i2c.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "diagnostics_service.h"
#include "sensor_service.h"

static BspI2c g_i2c;
static BspUart g_uart;
static BspTime g_timebase;
static SensorService g_sensor_service;
static AppTaskContext g_task_context;

static StaticQueue_t g_sample_queue_control;
static uint8_t g_sample_queue_storage[APP_SAMPLE_QUEUE_LENGTH * sizeof(AppSampleFrame)];
static StaticQueue_t g_command_queue_control;
static uint8_t g_command_queue_storage[APP_COMMAND_QUEUE_LENGTH * sizeof(AppControlCommand)];

static StaticTask_t g_acquisition_task_control;
static StackType_t g_acquisition_stack[APP_ACQUISITION_STACK_WORDS];
static StaticTask_t g_communication_task_control;
static StackType_t g_communication_stack[APP_COMMUNICATION_STACK_WORDS];
static StaticTask_t g_command_task_control;
static StackType_t g_command_stack[APP_COMMAND_STACK_WORDS];
static StaticTask_t g_health_task_control;
static StackType_t g_health_stack[APP_HEALTH_STACK_WORDS];

static bool AppBootstrap_CreateTasks(void)
{
    bool success = true;

    vTaskSuspendAll();

    success = success && (xTaskCreateStatic(AcquisitionTask_Entry,
                                            "acquisition",
                                            APP_ACQUISITION_STACK_WORDS,
                                            &g_task_context,
                                            APP_ACQUISITION_TASK_PRIORITY,
                                            g_acquisition_stack,
                                            &g_acquisition_task_control) != NULL);

    success = success && (xTaskCreateStatic(CommunicationTask_Entry,
                                            "communication",
                                            APP_COMMUNICATION_STACK_WORDS,
                                            &g_task_context,
                                            APP_COMMUNICATION_TASK_PRIORITY,
                                            g_communication_stack,
                                            &g_communication_task_control) != NULL);

    success = success && (xTaskCreateStatic(CommandTask_Entry,
                                            "command",
                                            APP_COMMAND_STACK_WORDS,
                                            &g_task_context,
                                            APP_COMMAND_TASK_PRIORITY,
                                            g_command_stack,
                                            &g_command_task_control) != NULL);

    success = success && (xTaskCreateStatic(HealthTask_Entry,
                                            "health",
                                            APP_HEALTH_STACK_WORDS,
                                            &g_task_context,
                                            APP_HEALTH_TASK_PRIORITY,
                                            g_health_stack,
                                            &g_health_task_control) != NULL);

    (void)xTaskResumeAll();
    return success;
}

AppStatus AppBootstrap_Start(const AppBootstrapConfig *config)
{
    AppStatus status;

    if ((config == NULL) || (config->i2c == NULL) || (config->uart == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    status = BspI2c_Init(&g_i2c, config->i2c);
    if (status != APP_STATUS_OK)
    {
        return status;
    }
    status = BspUart_Init(&g_uart, config->uart);
    if (status != APP_STATUS_OK)
    {
        return status;
    }
    status = BspTime_Init(&g_timebase, config->timestamp_timer);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    DiagnosticsService_Init();

    g_task_context.sample_queue = xQueueCreateStatic(APP_SAMPLE_QUEUE_LENGTH,
                                                     sizeof(AppSampleFrame),
                                                     g_sample_queue_storage,
                                                     &g_sample_queue_control);
    g_task_context.command_queue = xQueueCreateStatic(APP_COMMAND_QUEUE_LENGTH,
                                                      sizeof(AppControlCommand),
                                                      g_command_queue_storage,
                                                      &g_command_queue_control);
    g_task_context.sensor_service = &g_sensor_service;
    g_task_context.i2c = &g_i2c;
    g_task_context.timebase = &g_timebase;
    g_task_context.uart = &g_uart;

    if ((g_task_context.sample_queue == NULL) || (g_task_context.command_queue == NULL))
    {
        return APP_STATUS_INTERNAL_ERROR;
    }

    vQueueAddToRegistry(g_task_context.sample_queue, "sample_queue");
    vQueueAddToRegistry(g_task_context.command_queue, "command_queue");

    return AppBootstrap_CreateTasks() ? APP_STATUS_OK : APP_STATUS_INTERNAL_ERROR;
}
