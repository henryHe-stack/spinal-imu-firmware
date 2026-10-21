#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "FreeRTOS.h"
#include "queue.h"

#include "bsp_i2c.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "sensor_service.h"

typedef struct
{
    SensorService *sensor_service;
    BspI2c *i2c;
    BspTime *timebase;
    BspUart *uart;
    QueueHandle_t sample_queue;
    QueueHandle_t command_queue;
} AppTaskContext;

void AcquisitionTask_Entry(void *argument);
void CommunicationTask_Entry(void *argument);
void CommandTask_Entry(void *argument);
void HealthTask_Entry(void *argument);

#endif /* APP_TASKS_H */
