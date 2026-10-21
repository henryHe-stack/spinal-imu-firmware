#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_status.h"
#include "app_types.h"
#include "bno055.h"
#include "bsp_i2c.h"
#include "bsp_time.h"
#include "tca9548a.h"

typedef struct
{
    Tca9548a mux;
    Bno055 sensors[APP_IMU_NODE_COUNT];
    BspI2c *bus;
    BspTime *timebase;
    uint32_t next_sequence;
    bool initialized;
} SensorService;

/* All functions in this module must be called by AcquisitionTask only. */
AppStatus SensorService_Init(SensorService *self, BspI2c *bus, BspTime *timebase);
AppStatus SensorService_ReadCycle(SensorService *self, AppSampleFrame *frame);
AppStatus SensorService_ExecuteCommand(SensorService *self, const AppControlCommand *command);

#endif /* SENSOR_SERVICE_H */
