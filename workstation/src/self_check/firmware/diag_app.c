#include "diag_platform.h"
#include "diag_stats.h"
#include "neck_sensor_bus_stm32.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

#ifndef DIAG_NODE_MASK
#define DIAG_NODE_MASK 7
#endif
#ifndef DIAG_EXTERNAL_CLOCK
#define DIAG_EXTERNAL_CLOCK 1
#endif
#ifndef DIAG_ENABLE_MARKER
#define DIAG_ENABLE_MARKER 0
#endif
volatile uint32_t g_diag_fatal;
volatile DiagReport g_diag_latest;
static QueueHandle_t report_queue;
static TaskHandle_t acquire_handle,report_handle;
static char tx_buffer[1500];
static uint32_t uart_errors;

static void transmit(const char *s,size_t n)
{
    if(n && n<=65535u && HAL_UART_Transmit(&huart2,(uint8_t *)s,(uint16_t)n,200u)!=HAL_OK)
        if(uart_errors!=UINT32_MAX) ++uart_errors;
}
static void header(void)
{
    size_t n=diag_format_header(DIAG_NODE_MASK,DIAG_EXTERNAL_CLOCK,
                                DIAG_ENABLE_MARKER,tx_buffer,sizeof(tx_buffer));
    if(!n) Error_Handler();
    transmit(tx_buffer,n);
}
static void acquire_task(void *argument)
{
    NeckStm32SensorBusContext adapter={&hi2c1,20};
    NeckI2cBus bus=neck_stm32_sensor_bus(&adapter);
    NeckBno055 nodes[DIAG_NODES];
    NeckSensorResult init_result[DIAG_NODES];
    DiagReport report;DiagCycle cycle;
    TickType_t last_wake;uint64_t last_report;unsigned i,j;
    (void)argument;memset(&report,0,sizeof(report));memset(nodes,0,sizeof(nodes));
    if(!diag_stats_init(&report.stats,DIAG_NODE_MASK)) Error_Handler();
    vTaskDelay(pdMS_TO_TICKS(750u));
    for(i=0;i<DIAG_NODES;++i) {
        init_result[i]=NECK_SENSOR_NOT_CONFIGURED;
        if(DIAG_NODE_MASK & (1u<<i))
            init_result[i]=neck_bno055_configure(&bus,&nodes[i],(uint8_t)i,
                NECK_BNO055_MODE_NDOF,DIAG_EXTERNAL_CLOCK);
    }
    last_wake=xTaskGetTickCount();last_report=diag_now_us();
    for(;;) {
        uint64_t end;TickType_t elapsed;
        memset(&cycle,0,sizeof(cycle));diag_marker(1);cycle.start_us=diag_now_us();
        for(i=0;i<DIAG_NODES;++i) if(DIAG_NODE_MASK & (1u<<i)) {
            NeckBno055Observation observation;
            DiagNodeRead *n=&cycle.nodes[i];
            n->start_us=diag_now_us();
            n->result=init_result[i]==NECK_SENSOR_OK?
                (uint32_t)neck_bno055_read(&bus,&nodes[i],&observation):(uint32_t)init_result[i];
            n->end_us=diag_now_us();
            if(n->result==NECK_SENSOR_OK) {
                n->observed=observation.observation_valid;
                n->calibration_system=observation.calibration_system;
                n->calibration_accel=observation.calibration_accel;
                for(j=0;j<3;++j) n->accel[j]=observation.accel_m_s2[j];
                for(j=0;j<4;++j) n->quaternion[j]=observation.quaternion_wxyz[j];
            }
        }
        end=diag_now_us();cycle.end_us=end;diag_marker(0);
        (void)diag_stats_add(&report.stats,&cycle);
        if(end-last_report>=1000000u) {
            last_report=end;report.free_heap_bytes=(uint32_t)xPortGetFreeHeapSize();
            report.acquire_stack_words=(uint32_t)uxTaskGetStackHighWaterMark(acquire_handle);
            if(uxQueueMessagesWaiting(report_queue) && report.report_drops!=UINT32_MAX)
                ++report.report_drops;
            if(xQueueOverwrite(report_queue,&report)!=pdPASS) Error_Handler();
        }
        elapsed=xTaskGetTickCount()-last_wake;
        if(elapsed>=pdMS_TO_TICKS(10u)) {
            /* Do not burst-catch-up after an overrun. Counted by cycle/period stats. */
            last_wake=xTaskGetTickCount();vTaskDelay(pdMS_TO_TICKS(10u));
            last_wake=xTaskGetTickCount();
        } else vTaskDelayUntil(&last_wake,pdMS_TO_TICKS(10u));
    }
}
static void report_task(void *argument)
{
    DiagReport report;size_t n;(void)argument;header();
    for(;;) {
        if(xQueueReceive(report_queue,&report,pdMS_TO_TICKS(1500u))!=pdPASS) {
            const char wait[]="WAIT state=WAIT_INIT_OR_REPORT\r\n";
            transmit(wait,sizeof(wait)-1u);continue;
        }
        report.uart_errors=uart_errors;
        report.report_stack_words=(uint32_t)uxTaskGetStackHighWaterMark(report_handle);
        g_diag_latest=report;
        header();n=diag_format_report(&report,tx_buffer,sizeof(tx_buffer));
        if(!n) Error_Handler();
        transmit(tx_buffer,n);
    }
}
void diag_start(void)
{
    report_queue=xQueueCreate(1u,sizeof(DiagReport));
    if(!report_queue) Error_Handler();
    if(xTaskCreate(acquire_task,"SelfAcquire",640u,NULL,3u,&acquire_handle)!=pdPASS ||
       xTaskCreate(report_task,"SelfReport",640u,NULL,1u,&report_handle)!=pdPASS) Error_Handler();
}
