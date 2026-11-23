#include "bno055.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"

#define BNO055_REG_CHIP_ID      (0x00U)
#define BNO055_REG_PAGE_ID      (0x07U)
#define BNO055_REG_ACC_DATA     (0x08U)
#define BNO055_REG_GYR_DATA     (0x14U)
#define BNO055_REG_QUAT_DATA    (0x20U)
#define BNO055_REG_TEMP         (0x34U)
#define BNO055_REG_CALIB_STAT   (0x35U)
#define BNO055_REG_SYS_STATUS   (0x39U)
#define BNO055_REG_UNIT_SEL     (0x3BU)
#define BNO055_REG_OPR_MODE     (0x3DU)
#define BNO055_REG_PWR_MODE     (0x3EU)

#define BNO055_MODE_CONFIG      (0x00U)
#define BNO055_MODE_NDOF        (0x0CU)
#define BNO055_SYS_FUSION_RUN   (0x05U)

static int16_t Bno055_DecodeLe16(const uint8_t *bytes)
{
    uint16_t raw = (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8U);
    return (int16_t)raw;
}

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
    self->config_step = 0U;
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
    AppStatus status;
    uint8_t val = 0U;

    if ((self == NULL) || !self->present)
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    self->configured = false;

    status = Tca9548a_SelectChannel(self->mux, self->tca_channel);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    if (self->config_step == 0U) {
        val = BNO055_MODE_CONFIG;
        status = BspI2c_WriteRegister(self->bus, self->address_7bit, BNO055_REG_OPR_MODE, &val, 1U);
        if (status != APP_STATUS_OK) return status;
        self->config_step = 1U;
        return APP_STATUS_CONFIG_PENDING;
    }
    if (self->config_step == 1U) {
        /* The caller yields a full mode-transition interval between steps. */
        const uint8_t regs[] = { BNO055_REG_PAGE_ID, BNO055_REG_PWR_MODE, BNO055_REG_UNIT_SEL, BNO055_REG_OPR_MODE };
        const uint8_t values[] = { 0U, 0U, 0U, BNO055_MODE_NDOF };
        for (unsigned i = 0; i < sizeof(regs); ++i) {
            status = BspI2c_WriteRegister(self->bus, self->address_7bit, regs[i], &values[i], 1U);
            if (status != APP_STATUS_OK) return status;
        }
        self->config_step = 2U;
        return APP_STATUS_CONFIG_PENDING;
    }

    val = 0U;
    status = BspI2c_ReadRegister(self->bus, self->address_7bit, BNO055_REG_OPR_MODE, &val, 1U);
    if ((status != APP_STATUS_OK) || ((val & 0x0FU) != BNO055_MODE_NDOF))
    {
        return (status == APP_STATUS_OK) ? APP_STATUS_IO_ERROR : status;
    }

    self->configured = true;
    return APP_STATUS_OK;
}

AppStatus Bno055_ReadSample(Bno055 *self, AppImuSample *sample)
{
    AppStatus status;
    uint8_t sys_status = 0U;
    uint8_t acc_raw[6];
    uint8_t gyr_raw[6];
    uint8_t quat_raw[8];
    uint8_t tail_raw[2]; /* 0x34 TEMP, 0x35 CALIB_STAT */

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

    status = BspI2c_ReadRegister(self->bus, self->address_7bit, BNO055_REG_SYS_STATUS, &sys_status, 1U);
    if ((status != APP_STATUS_OK) || (sys_status != BNO055_SYS_FUSION_RUN))
    {
        sample->timestamp_end_us = BspTime_NowUs(self->timebase);
        sample->timestamp_mid_us = sample->timestamp_start_us + ((sample->timestamp_end_us - sample->timestamp_start_us) / 2U);
        return (status == APP_STATUS_OK) ? APP_STATUS_IO_ERROR : status;
    }

    status = BspI2c_ReadRegister(self->bus, self->address_7bit, BNO055_REG_ACC_DATA, acc_raw, sizeof(acc_raw));
    if (status != APP_STATUS_OK)
    {
        return status;
    }
    status = BspI2c_ReadRegister(self->bus, self->address_7bit, BNO055_REG_GYR_DATA, gyr_raw, sizeof(gyr_raw));
    if (status != APP_STATUS_OK)
    {
        return status;
    }
    status = BspI2c_ReadRegister(self->bus, self->address_7bit, BNO055_REG_QUAT_DATA, quat_raw, sizeof(quat_raw));
    if (status != APP_STATUS_OK)
    {
        return status;
    }
    status = BspI2c_ReadRegister(self->bus, self->address_7bit, BNO055_REG_TEMP, tail_raw, sizeof(tail_raw));
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    sample->timestamp_end_us = BspTime_NowUs(self->timebase);
    sample->timestamp_mid_us = sample->timestamp_start_us
                             + ((sample->timestamp_end_us - sample->timestamp_start_us) / 2U);

    sample->acceleration.x = Bno055_DecodeLe16(&acc_raw[0]);
    sample->acceleration.y = Bno055_DecodeLe16(&acc_raw[2]);
    sample->acceleration.z = Bno055_DecodeLe16(&acc_raw[4]);

    sample->gyroscope.x = Bno055_DecodeLe16(&gyr_raw[0]);
    sample->gyroscope.y = Bno055_DecodeLe16(&gyr_raw[2]);
    sample->gyroscope.z = Bno055_DecodeLe16(&gyr_raw[4]);

    sample->quaternion.w = Bno055_DecodeLe16(&quat_raw[0]);
    sample->quaternion.x = Bno055_DecodeLe16(&quat_raw[2]);
    sample->quaternion.y = Bno055_DecodeLe16(&quat_raw[4]);
    sample->quaternion.z = Bno055_DecodeLe16(&quat_raw[6]);

    sample->temperature_c = (int8_t)tail_raw[0];
    sample->calibration_status = tail_raw[1];

    sample->flags &= (uint16_t)~APP_SAMPLE_FLAG_SENSOR_ERROR;
    sample->flags |= APP_SAMPLE_FLAG_VALID;
    if ((((tail_raw[1] >> 6U) & 0x03U) < 3U) || (((tail_raw[1] >> 2U) & 0x03U) < 3U))
    {
        sample->flags |= APP_SAMPLE_FLAG_NOT_CALIBRATED;
    }

    return APP_STATUS_OK;
}
