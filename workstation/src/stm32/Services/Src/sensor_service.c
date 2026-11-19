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

        self->next_attempt_us[index] = BspTime_NowUs64(timebase) + index * 20000ULL;
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
        Bno055 *node = &self->sensors[index];
        uint64_t now = BspTime_NowUs64(self->timebase);
        if (!node->configured) {
            if (now < self->next_attempt_us[index]) continue;
            AppStatus status = node->present ? APP_STATUS_OK : Bno055_Probe(node);
            if (status == APP_STATUS_OK) status = Bno055_Configure(node);
            if (status == APP_STATUS_CONFIG_PENDING) {
                self->next_attempt_us[index] = BspTime_NowUs64(self->timebase) + 20000ULL;
                continue;
            }
            if (status != APP_STATUS_OK) {
                node->present = false;
                node->config_step = 0U;
                self->next_attempt_us[index] = now + 1000000ULL;
                continue;
            }
        }
        if (Bno055_ReadSample(node, &frame->samples[index]) == APP_STATUS_OK)
        {
            frame->samples[index].flags |= APP_SAMPLE_FLAG_VALID;
            frame->samples[index].flags &= (uint16_t)~APP_SAMPLE_FLAG_SENSOR_ERROR;
            valid_count++;
        } else {
            node->configured = false;
            node->present = false;
            node->config_step = 0U;
            self->next_attempt_us[index] = now + 1000000ULL;
        }
    }

    return (valid_count > 0U) ? APP_STATUS_OK : APP_STATUS_IO_ERROR;
}

AppStatus SensorService_ExecuteCommand(SensorService *self, const AppControlCommand *command)
{
    if (self == NULL)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if (command == NULL)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    if (command->id == APP_COMMAND_REINITIALIZE_SENSORS)
    {
        return SensorService_Init(self, self->bus, self->timebase);
    }

    if (command->id == APP_COMMAND_CAPTURE_NEUTRAL)
    {
        /* Neutral capture is solved on the PC host brain; acknowledge receipt. */
        return APP_STATUS_OK;
    }

    return APP_STATUS_OK;
}
