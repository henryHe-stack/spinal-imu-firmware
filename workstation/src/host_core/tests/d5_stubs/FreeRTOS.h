#ifndef D5_TEST_FREERTOS_H
#define D5_TEST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef void *TaskHandle_t;
typedef struct TestQueue *QueueHandle_t;
#define configTICK_RATE_HZ 1000u
#define configASSERT(x) assert(x)
#define pdPASS 1
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *,TickType_t);
void vTaskDelay(TickType_t);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t);
BaseType_t xTaskCreate(void (*)(void *),const char *,unsigned,void *,unsigned,TaskHandle_t *);
void xTaskNotifyGive(TaskHandle_t);
uint32_t ulTaskNotifyTake(BaseType_t,TickType_t);
size_t xPortGetFreeHeapSize(void);
QueueHandle_t xQueueCreate(unsigned,size_t);
BaseType_t xQueueSend(QueueHandle_t,const void *,TickType_t);
BaseType_t xQueueReceive(QueueHandle_t,void *,TickType_t);
BaseType_t xQueueOverwrite(QueueHandle_t,const void *);
BaseType_t xQueueReset(QueueHandle_t);
unsigned uxQueueMessagesWaiting(QueueHandle_t);
unsigned uxQueueSpacesAvailable(QueueHandle_t);
#endif
