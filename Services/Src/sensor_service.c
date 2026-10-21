#include "sensor_service.h"

#include <string.h>

#include "app_config.h"

static const uint8_t kSensorChannels[APP_IMU_NODE_COUNT] =
{
    APP_IMU_C0_TCA_CHANNEL,
    APP_IMU_C4_TCA_CHANNEL,
    APP_IMU_C7_TCA_CHANNEL
};

AppStatus SensorService_Init(SensorService *self, BspI2c *bus, BspTime *timebase)
{
    uint32_t index;
    AppStatus status;

    if ((self == NULL) || (bus == NULL) || (timebase == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(self, 0, sizeof(*self));
    self->bus = bus;
    self->timebase = timebase;

    status = Tca9548a_Init(&self->mux, bus, APP_TCA9548A_ADDRESS_7BIT);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    for (index = 0U; index < APP_IMU_NODE_COUNT; ++index)
    {
        (void)Bno055_InitObject(&self->sensors[index],
                                bus,
                                timebase,
                                &self->mux,
                                APP_BNO055_ADDRESS_7BIT,
                                kSensorChannels[index],
                                (AppImuNodeId)index);

        /* A missing/unconfigured node must not prevent other nodes from starting. */
        if (Bno055_Probe(&self->sensors[index]) == APP_STATUS_OK)
        {
            (void)Bno055_Configure(&self->sensors[index]);
        }
    }

    self->initialized = true;
    return APP_STATUS_OK;
}

AppStatus SensorService_ReadCycle(SensorService *self, AppSampleFrame *frame)
{
    uint32_t index;
    uint32_t valid_count = 0U;

    if (frame == NULL)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(frame, 0, sizeof(*frame));
    frame->sample_count = APP_IMU_NODE_COUNT;
    for (index = 0U; index < APP_IMU_NODE_COUNT; ++index)
    {
        frame->samples[index].node_id = (uint8_t)index;
        frame->samples[index].tca_channel = kSensorChannels[index];
        frame->samples[index].flags = APP_SAMPLE_FLAG_SENSOR_ERROR;
    }

    if ((self == NULL) || !self->initialized)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    frame->sequence = self->next_sequence++;

    /* Fixed C0 -> C4 -> C7 order makes the software timing offset observable. */
    for (index = 0U; index < APP_IMU_NODE_COUNT; ++index)
    {
        if (Bno055_ReadSample(&self->sensors[index], &frame->samples[index]) == APP_STATUS_OK)
        {
            frame->samples[index].flags |= APP_SAMPLE_FLAG_VALID;
            frame->samples[index].flags &= (uint16_t)~APP_SAMPLE_FLAG_SENSOR_ERROR;
            valid_count++;
        }
    }

    return (valid_count > 0U) ? APP_STATUS_OK : APP_STATUS_IO_ERROR;
}

AppStatus SensorService_ExecuteCommand(SensorService *self, const AppControlCommand *command)
{
    if ((self == NULL) || !self->initialized)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if (command == NULL)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    /*
     * TODO(SENSOR owner): implement re-initialization and neutral capture here.
     * Stream start/stop and sample-period policy should be applied by the
     * acquisition service, not by the low-level BNO055 driver.
     */
    return APP_STATUS_NOT_IMPLEMENTED;
}
