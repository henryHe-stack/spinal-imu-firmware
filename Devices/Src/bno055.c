#include "bno055.h"

#include <string.h>

#include "app_config.h"

#define BNO055_REG_CHIP_ID (0x00U)

AppStatus Bno055_InitObject(Bno055 *self,
                            BspI2c *bus,
                            BspTime *timebase,
                            Tca9548a *mux,
                            uint8_t address_7bit,
                            uint8_t tca_channel,
                            AppImuNodeId node_id)
{
    if ((self == NULL) || (bus == NULL) || (timebase == NULL) || (mux == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    self->bus = bus;
    self->timebase = timebase;
    self->mux = mux;
    self->address_7bit = address_7bit;
    self->tca_channel = tca_channel;
    self->node_id = node_id;
    self->present = false;
    self->configured = false;
    return APP_STATUS_OK;
}

AppStatus Bno055_Probe(Bno055 *self)
{
    uint8_t chip_id = 0U;
    AppStatus status;

    if ((self == NULL) || (self->bus == NULL) || (self->mux == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    status = Tca9548a_SelectChannel(self->mux, self->tca_channel);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    status = BspI2c_ReadRegister(self->bus,
                                 self->address_7bit,
                                 BNO055_REG_CHIP_ID,
                                 &chip_id,
                                 1U);
    if ((status != APP_STATUS_OK) || (chip_id != APP_BNO055_CHIP_ID))
    {
        self->present = false;
        return (status == APP_STATUS_OK) ? APP_STATUS_DEVICE_NOT_FOUND : status;
    }

    self->present = true;
    return APP_STATUS_OK;
}

AppStatus Bno055_Configure(Bno055 *self)
{
    if ((self == NULL) || !self->present)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    /*
     * TODO(IMU driver owner): implement reset, power mode, axis remap, units,
     * operation mode, startup delays, calibration restore and readback checks.
     * Keep the datasheet-required mode transition delays outside the I2C lock.
     */
    self->configured = false;
    return APP_STATUS_NOT_IMPLEMENTED;
}

AppStatus Bno055_ReadSample(Bno055 *self, AppImuSample *sample)
{
    AppStatus status;

    if ((self == NULL) || (sample == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(sample, 0, sizeof(*sample));
    sample->node_id = (uint8_t)self->node_id;
    sample->tca_channel = self->tca_channel;
    sample->flags = APP_SAMPLE_FLAG_SENSOR_ERROR;
    if (!BspTime_IsHighResolution(self->timebase))
    {
        sample->flags |= APP_SAMPLE_FLAG_TIME_COARSE;
    }

    if (!self->configured)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    status = Tca9548a_SelectChannel(self->mux, self->tca_channel);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    sample->timestamp_start_us = BspTime_NowUs(self->timebase);

    /*
     * TODO(IMU driver owner): burst-read quaternion, acceleration, gyroscope,
     * temperature and calibration registers; decode little-endian fixed-point
     * values and validate the operation/calibration status.
     */

    sample->timestamp_end_us = BspTime_NowUs(self->timebase);
    sample->timestamp_mid_us = sample->timestamp_start_us
                             + ((sample->timestamp_end_us - sample->timestamp_start_us) / 2U);
    return APP_STATUS_NOT_IMPLEMENTED;
}
