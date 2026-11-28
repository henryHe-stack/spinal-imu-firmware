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
    if ((self == NULL) || (bus == NULL) || (timebase == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(self, 0, sizeof(*self));
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

/* The same BNO055 driver supports both topologies:
 *   - mux != NULL: select one TCA9548A channel before every transaction;
 *   - mux == NULL: access a single BNO055 directly on the I2C bus.
 */
static AppStatus SelectPath(Bno055 *self)
{
    if (self == NULL || self->bus == NULL) return APP_STATUS_NOT_INITIALIZED;
    return self->mux == NULL ? APP_STATUS_OK :
        Tca9548a_SelectChannel(self->mux, self->tca_channel);
}

AppStatus Bno055_Probe(Bno055 *self)
{
    uint8_t chip_id = 0U;
    AppStatus status;

    if ((self == NULL) || (self->bus == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    status = SelectPath(self);
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

/* Bosch BNO055 page-0 map; reference URLs and mode delays are documented in
 * Docs/DRIVER_BRINGUP.md. No dependency on the Bosch global-device API. */
static AppStatus Write(Bno055 *self, uint8_t reg, uint8_t value)
{
    AppStatus result = SelectPath(self);
    if (result != APP_STATUS_OK) return result;
    return BspI2c_WriteRegister(self->bus, self->address_7bit, reg, &value, 1);
}
static AppStatus Read(Bno055 *self, uint8_t reg, uint8_t *data, size_t length)
{
    AppStatus result = SelectPath(self);
    if (result != APP_STATUS_OK) return result;
    return BspI2c_ReadRegister(self->bus, self->address_7bit, reg, data, length);
}
AppStatus Bno055_Configure(Bno055 *self)
{
    if (!self || !self->bus) return APP_STATUS_INVALID_ARGUMENT;
    self->configured = false; self->present = false; self->init_step = 0;
    self->consecutive_errors = 0; self->due_ms = HAL_GetTick() + 700U;
    return APP_STATUS_OK;
}
AppStatus Bno055_Service(Bno055 *self)
{
    if (!self || !self->bus) return APP_STATUS_INVALID_ARGUMENT;
    if (self->configured) return APP_STATUS_OK;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - self->due_ms) < 0) return APP_STATUS_BUSY;
    AppStatus status = APP_STATUS_OK;
    uint8_t value = 0;
    uint32_t wait = 0;
    switch (self->init_step)
    {
        case 0: status = Bno055_Probe(self); break;
        case 1: status = Write(self, 0x3D, 0x00); wait = 25; break; /* CONFIGMODE */
        case 2: status = Write(self, 0x3F, 0x20); wait = 700; break; /* soft reset */
        case 3: status = Bno055_Probe(self); break;
        case 4: status = Write(self, 0x07, 0x00); break; /* PAGE_ID */
        case 5: status = Write(self, 0x3E, 0x00); wait = 10; break; /* normal power */
        case 6: status = Write(self, 0x3B, 0x00); break; /* m/s2, dps, deg, C */
        case 7: status = Write(self, 0x41, APP_BNO055_AXIS_MAP); break;
        case 8: status = Write(self, 0x42, APP_BNO055_AXIS_SIGN); break;
        case 9:
            status = Write(self, 0x3F, APP_BNO055_EXTERNAL_CRYSTAL ? 0x80 : 0);
            wait = 10; break;
        case 10:
            if (self->calibration_available)
            {
                status = SelectPath(self);
                if (status == APP_STATUS_OK) status = BspI2c_WriteRegister(self->bus,
                    self->address_7bit, 0x55, self->calibration, sizeof(self->calibration));
            }
            break;
        case 11: status = Write(self, 0x3D, APP_BNO055_OPERATION_MODE); wait = 20; break;
        case 12:
            status = Read(self, 0x3D, &value, 1);
            if (status == APP_STATUS_OK && value != APP_BNO055_OPERATION_MODE) status = APP_STATUS_IO_ERROR;
            break;
        case 13:
            /* SYS_TRIGGER.CLK_SEL readback confirms the requested source;
             * physical clock stability still requires a bench check. */
            status = Read(self, 0x3F, &value, 1);
            if (status == APP_STATUS_OK && ((value >> 7) & 1U) != APP_BNO055_EXTERNAL_CRYSTAL)
                status = APP_STATUS_IO_ERROR;
            if (status == APP_STATUS_OK) { self->configured = true; return APP_STATUS_OK; }
            break;
        default: return APP_STATUS_INTERNAL_ERROR;
    }
    if (status != APP_STATUS_OK)
    {
        self->present = false; self->init_step = 0;
        self->due_ms = now + APP_SENSOR_RETRY_MS;
        return status;
    }
    self->init_step++; self->due_ms = now + wait;
    return APP_STATUS_BUSY;
}
static int16_t Le16(const uint8_t *p)
{ return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
AppStatus Bno055_ReadSample(Bno055 *self, AppImuSample *sample)
{
    if (!self || !sample) return APP_STATUS_INVALID_ARGUMENT;
    memset(sample, 0, sizeof(*sample));
    sample->node_id = (uint8_t)self->node_id; sample->tca_channel = self->tca_channel;
    sample->flags = APP_SAMPLE_FLAG_SENSOR_ERROR;
    if (!self->configured) return APP_STATUS_NOT_INITIALIZED;
    uint8_t raw[51]; /* 0x08..0x3A: accel, gyro, quaternion, temp, cal, sys */
    sample->timestamp_start_us = BspTime_NowUs(self->timebase);
    AppStatus status = Read(self, 0x08, raw, sizeof(raw));
    sample->timestamp_end_us = BspTime_NowUs(self->timebase);
    sample->timestamp_mid_us = sample->timestamp_start_us +
        (uint32_t)(sample->timestamp_end_us - sample->timestamp_start_us) / 2U;
    if (status != APP_STATUS_OK) goto failed;
    sample->acceleration = (AppVector3Raw){ Le16(raw), Le16(raw+2), Le16(raw+4) };
    sample->gyroscope = (AppVector3Raw){ Le16(raw+12), Le16(raw+14), Le16(raw+16) };
    sample->quaternion = (AppQuaternionRaw){ Le16(raw+24), Le16(raw+26), Le16(raw+28), Le16(raw+30) };
    sample->temperature_c = (int8_t)raw[44]; sample->calibration_status = raw[45];
    sample->system_status = raw[49]; sample->system_error = raw[50];
    /* Reject a reset/configuring sensor and impossible quaternion magnitudes.
     * Calibration quality is reported separately: uncalibrated != transport error. */
    int64_t norm = (int64_t)sample->quaternion.w * sample->quaternion.w
        + (int64_t)sample->quaternion.x * sample->quaternion.x
        + (int64_t)sample->quaternion.y * sample->quaternion.y
        + (int64_t)sample->quaternion.z * sample->quaternion.z;
    if (sample->system_status != 5U || sample->system_error || norm < 150000000LL || norm > 400000000LL)
    { status = APP_STATUS_IO_ERROR; goto failed; }
    sample->flags = APP_SAMPLE_FLAG_VALID;
    if (sample->calibration_status != 0xFF) sample->flags |= APP_SAMPLE_FLAG_NOT_CALIBRATED;
    self->consecutive_errors = 0;
    return APP_STATUS_OK;
failed:
    if (++self->consecutive_errors >= 3) (void)Bno055_Configure(self);
    return status;
}
AppStatus Bno055_CaptureCalibration(Bno055 *self)
{
    /* Called only while streaming is stopped. CONFIGMODE makes this blocking
     * maintenance operation explicit; do not call in a regular read cycle. */
    if (!self || !self->configured) return APP_STATUS_NOT_INITIALIZED;
    uint8_t cal;
    AppStatus status = Read(self, 0x35, &cal, 1);
    if (status != APP_STATUS_OK) return status;
    if (cal != 0xFF) return APP_STATUS_BUSY;
    status = Write(self, 0x3D, 0); HAL_Delay(25);
    if (status == APP_STATUS_OK) status = Read(self, 0x55, self->calibration, sizeof(self->calibration));
    AppStatus restore = Write(self, 0x3D, APP_BNO055_OPERATION_MODE); HAL_Delay(20);
    self->calibration_available = status == APP_STATUS_OK;
    if (restore != APP_STATUS_OK) { (void)Bno055_Configure(self); return restore; }
    return status;
}
