#ifndef TCA9548A_H
#define TCA9548A_H

#include <stdbool.h>
#include <stdint.h>

#include "app_status.h"
#include "bsp_i2c.h"

typedef struct
{
    BspI2c *bus;
    uint8_t address_7bit;
    uint8_t selected_mask;
    bool initialized;
} Tca9548a;

AppStatus Tca9548a_Init(Tca9548a *self, BspI2c *bus, uint8_t address_7bit);
AppStatus Tca9548a_SelectChannel(Tca9548a *self, uint8_t channel);
AppStatus Tca9548a_DisableAll(Tca9548a *self);

#endif /* TCA9548A_H */
