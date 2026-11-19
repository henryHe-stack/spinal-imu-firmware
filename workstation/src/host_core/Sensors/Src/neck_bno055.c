#include "neck_bno055.h"
#include <string.h>

enum {
    REG_CHIP_ID = 0x00, REG_PAGE_ID = 0x07, REG_ACC_DATA = 0x08,
    REG_QUATERNION = 0x20, REG_CALIB_STAT = 0x35,
    REG_SYS_CLK_STATUS = 0x38, REG_SYS_STATUS = 0x39, REG_UNIT_SEL = 0x3B,
    REG_OPR_MODE = 0x3D, REG_PWR_MODE = 0x3E, REG_SYS_TRIGGER = 0x3F,
    CHIP_ID = 0xA0
};

static int bus_ok(const NeckI2cBus *bus, int needs_delay)
{
    return bus != NULL && bus->write != NULL && bus->read_register != NULL &&
           (!needs_delay || bus->delay_ms != NULL);
}

static NeckSensorResult select_channel(const NeckI2cBus *bus, uint8_t channel)
{
    uint8_t mask;
    if (channel > 2u) return NECK_SENSOR_BAD_ARGUMENT;
    mask = (uint8_t)(1u << channel);
    /* The TCA9548A latches this selection only after STOP. Never enable
     * multiple identical-address BNO055s on the same upstream bus.
     */
    if (bus->write(bus->context, NECK_TCA9548A_ADDRESS7, &mask, 1u) != 0)
        return NECK_SENSOR_BUS_ERROR;
    return NECK_SENSOR_OK;
}

static NeckSensorResult read_byte(const NeckI2cBus *bus, uint8_t reg, uint8_t *value)
{
    return bus->read_register(bus->context, NECK_BNO055_ADDRESS7, reg, value, 1u) == 0
               ? NECK_SENSOR_OK : NECK_SENSOR_BUS_ERROR;
}

static NeckSensorResult write_byte(const NeckI2cBus *bus, uint8_t reg, uint8_t value)
{
    uint8_t bytes[2];
    bytes[0] = reg;
    bytes[1] = value;
    return bus->write(bus->context, NECK_BNO055_ADDRESS7, bytes, 2u) == 0
               ? NECK_SENSOR_OK : NECK_SENSOR_BUS_ERROR;
}

static int16_t signed_le16(const uint8_t *bytes)
{
    uint16_t raw = (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8);
    return raw < 0x8000u ? (int16_t)raw : (int16_t)((int32_t)raw - 65536);
}

NeckSensorResult neck_bno055_probe(const NeckI2cBus *bus, uint8_t channel)
{
    uint8_t value;
    NeckSensorResult result;
    if (!bus_ok(bus, 0) || channel > 2u) return NECK_SENSOR_BAD_ARGUMENT;
    result = select_channel(bus, channel);
    if (result != NECK_SENSOR_OK) return result;
    result = read_byte(bus, REG_CHIP_ID, &value);
    if (result != NECK_SENSOR_OK) return result;
    return value == CHIP_ID ? NECK_SENSOR_OK : NECK_SENSOR_CHIP_ID_MISMATCH;
}

NeckSensorResult neck_bno055_configure(const NeckI2cBus *bus,
                                      NeckBno055 *node, uint8_t channel,
                                      uint8_t fusion_mode, uint8_t external_clock)
{
    NeckSensorResult result;
    uint8_t value;
    if (node == NULL || !bus_ok(bus, 1) || channel > 2u ||
        (fusion_mode != NECK_BNO055_MODE_IMU && fusion_mode != NECK_BNO055_MODE_NDOF) ||
        external_clock > 1u) return NECK_SENSOR_BAD_ARGUMENT;
    memset(node, 0, sizeof(*node));
    node->channel = channel;
    node->fusion_mode = fusion_mode;
    node->external_clock = external_clock;
    result = neck_bno055_probe(bus, channel);
    if (result != NECK_SENSOR_OK) return result;

    /* CONFIGMODE is required before changing page, units, power or clock.
     * Worst-case transition from an already running mode is 19 ms.
     */
    result = write_byte(bus, REG_OPR_MODE, 0x00);
    if (result != NECK_SENSOR_OK) return result;
    bus->delay_ms(bus->context, 20u);
    result = write_byte(bus, REG_PAGE_ID, 0x00);
    if (result != NECK_SENSOR_OK) return result;
    result = write_byte(bus, REG_PWR_MODE, 0x00); /* Normal power. */
    if (result != NECK_SENSOR_OK) return result;
    result = write_byte(bus, REG_UNIT_SEL, 0x00); /* Windows, m/s^2, dps, deg. */
    if (result != NECK_SENSOR_OK) return result;
    if (external_clock != 0u) {
        result = write_byte(bus, REG_SYS_TRIGGER, 0x80);
        if (result != NECK_SENSOR_OK) return result;
        bus->delay_ms(bus->context, 650u); /* Datasheet minimum approximately 600 ms. */
        result = read_byte(bus, REG_SYS_CLK_STATUS, &value);
        if (result != NECK_SENSOR_OK) return result;
        if ((value & 0x01u) != 0u) return NECK_SENSOR_EXTERNAL_CLOCK_FAILED;
        result = read_byte(bus, REG_SYS_TRIGGER, &value);
        if (result != NECK_SENSOR_OK) return result;
        if ((value & 0x80u) == 0u) return NECK_SENSOR_EXTERNAL_CLOCK_FAILED;
    }
    result = write_byte(bus, REG_OPR_MODE, fusion_mode);
    if (result != NECK_SENSOR_OK) return result;
    bus->delay_ms(bus->context, 10u); /* CONFIGMODE -> fusion: datasheet 7 ms. */
    result = read_byte(bus, REG_OPR_MODE, &value);
    if (result != NECK_SENSOR_OK) return result;
    if ((value & 0x0Fu) != fusion_mode) return NECK_SENSOR_CONFIG_MISMATCH;
    result = read_byte(bus, REG_UNIT_SEL, &value);
    if (result != NECK_SENSOR_OK) return result;
    if (value != 0x00u) return NECK_SENSOR_CONFIG_MISMATCH;
    node->configured = 1u;
    return NECK_SENSOR_OK;
}

NeckSensorResult neck_bno055_read(const NeckI2cBus *bus,
                                 const NeckBno055 *node,
                                 NeckBno055Observation *out)
{
    uint8_t accel[6], quat[8], calibration, system_status;
    size_t i;
    NeckSensorResult result;
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (!bus_ok(bus, 0) || node == NULL || out == NULL || node->channel > 2u)
        return NECK_SENSOR_BAD_ARGUMENT;
    if (!node->configured) return NECK_SENSOR_NOT_CONFIGURED;
    result = select_channel(bus, node->channel);
    if (result != NECK_SENSOR_OK) return result;
    result = read_byte(bus, REG_SYS_STATUS, &system_status);
    if (result != NECK_SENSOR_OK) return result;
    if (system_status != 5u) return NECK_SENSOR_FUSION_NOT_RUNNING;
    if (bus->read_register(bus->context, NECK_BNO055_ADDRESS7,
                           REG_ACC_DATA, accel, sizeof(accel)) != 0)
        return NECK_SENSOR_BUS_ERROR;
    if (bus->read_register(bus->context, NECK_BNO055_ADDRESS7,
                           REG_QUATERNION, quat, sizeof(quat)) != 0)
        return NECK_SENSOR_BUS_ERROR;
    result = read_byte(bus, REG_CALIB_STAT, &calibration);
    if (result != NECK_SENSOR_OK) return result;
    for (i = 0; i < 3u; ++i)
        out->accel_m_s2[i] = (double)signed_le16(&accel[2u * i]) / 100.0;
    for (i = 0; i < 4u; ++i)
        out->quaternion_wxyz[i] = (double)signed_le16(&quat[2u * i]) / 16384.0;
    out->calibration_system = (uint8_t)((calibration >> 6) & 3u);
    out->calibration_gyro = (uint8_t)((calibration >> 4) & 3u);
    out->calibration_accel = (uint8_t)((calibration >> 2) & 3u);
    out->calibration_mag = (uint8_t)(calibration & 3u);
    out->system_status = system_status;
    out->observation_valid = 1u;
    /* No sensor sample time or sample counter is available in these reads. */
    out->sample_time_known = 0u;
    return NECK_SENSOR_OK;
}
