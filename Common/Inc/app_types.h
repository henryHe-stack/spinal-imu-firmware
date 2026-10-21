#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_IMU_NODE_COUNT (3U)

typedef enum
{
    APP_IMU_NODE_C0 = 0,
    APP_IMU_NODE_C4 = 1,
    APP_IMU_NODE_C7 = 2
} AppImuNodeId;

typedef enum
{
    APP_SAMPLE_FLAG_NONE             = 0U,
    APP_SAMPLE_FLAG_VALID            = (1U << 0),
    APP_SAMPLE_FLAG_TIME_COARSE      = (1U << 1),
    APP_SAMPLE_FLAG_SENSOR_ERROR     = (1U << 2),
    APP_SAMPLE_FLAG_NOT_CALIBRATED   = (1U << 3)
} AppSampleFlags;

typedef struct
{
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} AppQuaternionRaw;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} AppVector3Raw;

/*
 * Internal sample representation. It is deliberately not packed and must never
 * be sent directly over UART. ProtocolService owns byte order and wire layout.
 */
typedef struct
{
    uint32_t timestamp_start_us;
    uint32_t timestamp_end_us;
    uint32_t timestamp_mid_us;
    AppQuaternionRaw quaternion;
    AppVector3Raw acceleration;
    AppVector3Raw gyroscope;
    int8_t temperature_c;
    uint8_t calibration_status;
    uint8_t node_id;
    uint8_t tca_channel;
    uint16_t flags;
} AppImuSample;

typedef struct
{
    uint32_t sequence;
    uint8_t sample_count;
    uint8_t reserved[3];
    AppImuSample samples[APP_IMU_NODE_COUNT];
} AppSampleFrame;

typedef enum
{
    APP_COMMAND_NONE = 0,
    APP_COMMAND_START_STREAM,
    APP_COMMAND_STOP_STREAM,
    APP_COMMAND_CAPTURE_NEUTRAL,
    APP_COMMAND_REINITIALIZE_SENSORS,
    APP_COMMAND_SET_SAMPLE_PERIOD_MS
} AppCommandId;

typedef struct
{
    AppCommandId id;
    uint32_t request_id;
    uint32_t value;
} AppControlCommand;

#endif /* APP_TYPES_H */
