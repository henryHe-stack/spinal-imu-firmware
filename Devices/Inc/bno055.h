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
AppStatus Bno055_ReadSample(Bno055 *self, AppImuSample *sample);

#endif /* BNO055_H */
