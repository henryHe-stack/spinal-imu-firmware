#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include <stdint.h>

#include "app_status.h"
#include "app_types.h"
#include "bsp_i2c.h"
#include "bsp_time.h"
#include "tca9548a.h"

typedef struct
{
    BspI2c *bus;
    BspTime *timebase;
    Tca9548a *mux;
    uint8_t address_7bit;
    uint8_t tca_channel;
    AppImuNodeId node_id;
    bool present;
    bool configured;
    uint8_t init_step;
    uint8_t consecutive_errors;
    uint32_t due_ms;
    uint8_t calibration[22];
    bool calibration_available;
} Bno055;

AppStatus Bno055_InitObject(Bno055 *self,
                            BspI2c *bus,
                            BspTime *timebase,
                            Tca9548a *mux,
                            uint8_t address_7bit,
                            uint8_t tca_channel,
                            AppImuNodeId node_id);
AppStatus Bno055_Probe(Bno055 *self);
AppStatus Bno055_Configure(Bno055 *self);
/* One state-machine step per poll. BUSY means a required settling interval. */
AppStatus Bno055_Service(Bno055 *self);
AppStatus Bno055_CaptureCalibration(Bno055 *self);
AppStatus Bno055_ReadSample(Bno055 *self, AppImuSample *sample);

#endif /* BNO055_H */
