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
    uint32_t next_recovery_ms;
} SensorService;

/* All functions in this module must be called by AcquisitionTask only. */
AppStatus SensorService_Init(SensorService *self, BspI2c *bus, BspTime *timebase);
AppStatus SensorService_ReadCycle(SensorService *self, AppSampleFrame *frame);
AppStatus SensorService_ExecuteCommand(SensorService *self, const AppControlCommand *command);

/*
 * A cycle keeps one slot for each physical node so the PC can identify
 * missing C0/C4/C7 samples. Transmission is allowed when at least one slot
 * carries a valid sample; callers must not use the aggregate service status
 * as the transmission gate because the remaining nodes may still be usable.
 */
static inline bool SensorService_FrameHasValidSample(const AppSampleFrame *frame)
{
    if (!frame || frame->sample_count > APP_IMU_NODE_COUNT) return false;
    for (unsigned i = 0; i < frame->sample_count; ++i)
    {
        if ((frame->samples[i].flags & APP_SAMPLE_FLAG_VALID) != 0U) return true;
    }
    return false;
}

#endif /* SENSOR_SERVICE_H */
