#include "tca9548a.h"

#include <stddef.h>

AppStatus Tca9548a_Init(Tca9548a *self, BspI2c *bus, uint8_t address_7bit)
{
    AppStatus status;

    if ((self == NULL) || (bus == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    self->bus = bus;
    self->address_7bit = address_7bit;
    self->selected_mask = 0U;
    self->initialized = false;

    status = BspI2c_IsReady(bus, address_7bit);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    status = BspI2c_MasterWrite(bus, address_7bit, &self->selected_mask, 1U);
    if (status == APP_STATUS_OK)
    {
        self->initialized = true;
    }
    return status;
}

AppStatus Tca9548a_SelectChannel(Tca9548a *self, uint8_t channel)
{
    uint8_t mask;
    AppStatus status;

    if ((self == NULL) || !self->initialized)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if (channel > 7U)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    mask = (uint8_t)(1U << channel);
    if (mask == self->selected_mask)
    {
        return APP_STATUS_OK;
    }

    status = BspI2c_MasterWrite(self->bus, self->address_7bit, &mask, 1U);
    if (status == APP_STATUS_OK)
    {
        self->selected_mask = mask;
    }
    return status;
}

AppStatus Tca9548a_DisableAll(Tca9548a *self)
{
    uint8_t mask = 0U;
    AppStatus status;

    if ((self == NULL) || !self->initialized)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    status = BspI2c_MasterWrite(self->bus, self->address_7bit, &mask, 1U);
    if (status == APP_STATUS_OK)
    {
        self->selected_mask = 0U;
    }
    return status;
}
