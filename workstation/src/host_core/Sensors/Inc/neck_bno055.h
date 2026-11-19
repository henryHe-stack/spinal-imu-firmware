#ifndef NECK_BNO055_H
#define NECK_BNO055_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All addresses in this API are 7-bit I2C addresses. Each write MUST end
 * with STOP; a register read MUST use repeated START before the data phase.
 * The caller owns the entire shared I2C bus during each driver call.
 */
typedef struct {
    void *context;
    int (*write)(void *context, uint8_t address7, const uint8_t *data, size_t length);
    int (*read_register)(void *context, uint8_t address7, uint8_t reg,
                         uint8_t *data, size_t length);
    void (*delay_ms)(void *context, uint32_t milliseconds);
} NeckI2cBus;

typedef enum {
    NECK_SENSOR_OK = 0,
    NECK_SENSOR_BAD_ARGUMENT,
    NECK_SENSOR_BUS_ERROR,
    NECK_SENSOR_CHIP_ID_MISMATCH,
    NECK_SENSOR_CONFIG_MISMATCH,
    NECK_SENSOR_EXTERNAL_CLOCK_FAILED,
    NECK_SENSOR_FUSION_NOT_RUNNING,
    NECK_SENSOR_NOT_CONFIGURED,
    NECK_SENSOR_CLOCK_ERROR,
    NECK_SENSOR_NO_OBSERVATION
} NeckSensorResult;

enum {
    NECK_BNO055_ADDRESS7 = 0x28,
    NECK_TCA9548A_ADDRESS7 = 0x70,
    NECK_BNO055_MODE_IMU = 0x08,
    NECK_BNO055_MODE_NDOF = 0x0C
};

typedef struct {
    uint8_t channel; /* Schematic: C0=0, C4=1, C7=2. */
    uint8_t fusion_mode;
    uint8_t external_clock; /* Only if 32.768 kHz crystal is fitted. */
    uint8_t configured;
} NeckBno055;

typedef struct {
    double accel_m_s2[3]; /* ACC_DATA, including gravity; never LIA_DATA. */
    double quaternion_wxyz[4];
    uint8_t calibration_system;
    uint8_t calibration_gyro;
    uint8_t calibration_accel;
    uint8_t calibration_mag;
    uint8_t system_status;
    uint8_t observation_valid;
    uint8_t sample_time_known; /* Always zero: register read is not sample time. */
} NeckBno055Observation;

/* Probe after power-up delay (datasheet start-up 400 ms typical). Does not
 * configure fusion or synthesize a timestamp. channel must be 0, 1 or 2.
 */
NeckSensorResult neck_bno055_probe(const NeckI2cBus *bus, uint8_t channel);
/* Blocking start-up only, not for the 10 ms acquisition loop. Re-init after
 * reset/fault as a deliberate task-level action. Config remains false on error.
 */
NeckSensorResult neck_bno055_configure(const NeckI2cBus *bus,
                                      NeckBno055 *node, uint8_t channel,
                                      uint8_t fusion_mode, uint8_t external_clock);
/* One channel is selected, fusion status checked, then three reads made. The returned
 * observation has no trustworthy BNO055 sample instant or sync proof.
 */
NeckSensorResult neck_bno055_read(const NeckI2cBus *bus,
                                 const NeckBno055 *node,
                                 NeckBno055Observation *out);

#ifdef __cplusplus
}
#endif
#endif
