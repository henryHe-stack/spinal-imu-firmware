/* 单节点倾角接口。含重力的加速度以 m/s² 输入，正轴依次朝前、朝左、朝上。 */
/* S03 C99 port. No HAL, RTOS, dynamic allocation, filter, yaw or alarm.
 * Body axes: +X anterior, +Y left, +Z superior (right handed).
 * Input is specific force INCLUDING gravity, in m/s^2. Output is degrees.
 * Paper magnitudes + explicit project sign/zero adaptation, NOT Euler angles.
 * double deliberately preserves the Python baseline's numerical boundaries.
 * Cortex-M3 uses software floating point: hardware timing remains unverified.
 */
#ifndef NECK_TILT_H
#define NECK_TILT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NECK_ALGORITHM_ID "s03-paper-magnitude-signed-adaptation-v1"
#define NECK_NODE_COUNT 3u
#define NECK_MIN_ACCEL_NORM 1.0e-9
#define NECK_DIRECTION_EPS 1.0e-10
#define NECK_CALIBRATION_UNKNOWN 255u

typedef enum {
    NECK_NODE_C0 = 0, NECK_NODE_C4 = 1, NECK_NODE_C7 = 2
} NeckNodeId;
typedef enum {
    NECK_SOURCE_SIMULATION = 0, NECK_SOURCE_HARDWARE = 1
} NeckSource;
typedef enum {
    NECK_NODE_OK = 0, NECK_NODE_PARTIAL = 1, NECK_NODE_MISSING = 2,
    NECK_NODE_FAULT = 3, NECK_NODE_STALE = 4
} NeckNodeStatus;
typedef enum {
    NECK_COMPUTED = 0, NECK_PARTIAL = 1, NECK_UNCALIBRATED = 2,
    NECK_NO_ACCEL = 3, NECK_INVALID = 4, NECK_DIRECTION_UNDETERMINED = 5
} NeckTiltStatus;
typedef enum {
    NECK_OK = 0, NECK_BAD_ARGUMENT = 1, NECK_BAD_CONFIG = 2,
    NECK_CONFIG_MISMATCH = 3, NECK_BAD_REFERENCE = 4,
    NECK_SIM_REFERENCE_ON_HARDWARE = 5, NECK_BAD_FRAME = 6
} NeckResult;
typedef enum {
    NECK_REFERENCE_SIMULATION = 0, NECK_REFERENCE_EXTERNAL = 1
} NeckReferenceOrigin;

enum {
    NECK_SIGNAL_QUATERNION = 1u, NECK_SIGNAL_ACCEL = 2u,
    NECK_SIGNAL_GYRO = 4u
};
/* A numeric member is usable ONLY when its corresponding validity bit is set.
 * Cleared members are initialized to zero, but do NOT mean a zero measurement.
 */
enum {
    NECK_VALID_BODY = 1u << 0, NECK_VALID_NORM = 1u << 1,
    NECK_VALID_TOTAL = 1u << 2, NECK_VALID_PITCH_RAW = 1u << 3,
    NECK_VALID_LATERAL_RAW = 1u << 4, NECK_VALID_FLEXION = 1u << 5,
    NECK_VALID_RIGHT_LATERAL = 1u << 6, NECK_VALID_SAMPLE_TIME = 1u << 7,
    NECK_VALID_SAMPLE_AGE = 1u << 8, NECK_VALID_REFERENCE = 1u << 9
};
enum {
    NECK_FLAG_QUALITY_NOT_EVALUATED = 1u << 0,
    NECK_FLAG_SAMPLE_TIME_UNKNOWN = 1u << 1,
    NECK_FLAG_CALIBRATION_UNKNOWN = 1u << 2,
    NECK_FLAG_CALIBRATION_PENDING = 1u << 3,
    NECK_FLAG_NEUTRAL_UNSET = 1u << 4,
    NECK_FLAG_GRAVITY_ONLY = 1u << 5,
    NECK_FLAG_PITCH_AMBIGUOUS = 1u << 6,
    NECK_FLAG_INVALID_ACCEL_VECTOR = 1u << 7,
    NECK_FLAG_INVALID_ACCEL_NORM = 1u << 8,
    NECK_FLAG_ZERO_OR_TINY_ACCEL = 1u << 9,
    NECK_FLAG_OUTSIDE_UPRIGHT = 1u << 10,
    NECK_FLAG_ACCEL_UNAVAILABLE = 1u << 11,
    NECK_FLAG_NODE_OK = 1u << 12,
    NECK_FLAG_NODE_PARTIAL = 1u << 13,
    NECK_FLAG_NODE_MISSING = 1u << 14,
    NECK_FLAG_NODE_FAULT = 1u << 15,
    NECK_FLAG_NODE_STALE = 1u << 16,
    NECK_FLAG_INPUT_REJECTED = 1u << 17,
    NECK_FLAG_QUALITY_REJECTED = 1u << 18,
    NECK_FLAG_INPUT_TIMEOUT = 1u << 19
};

/* Fixed-size algorithm INPUT SUBSET, not a full JSON contract or wire packet.
 * quat/gyro values, full calibration and read intervals remain upstream.
 * nodes[] order is always C0/C4/C7; retain signal mask from validated upstream.
 * sample_time_known=0 is unknown, never infer a sensor acquisition timestamp
 * from the MCU register-read time. accel_present must match ACCEL validity.
 */
typedef struct {
    double accel_m_s2[3];
    uint64_t sample_time_us;
    uint8_t signal_valid_mask;
    uint8_t accel_present;
    uint8_t sample_time_known;
    uint8_t calibration_accel;
    uint8_t status;
} NeckNodeSample;

typedef struct {
    uint32_t config_id;
    uint32_t session_id;
    uint64_t sequence;
    uint64_t frame_time_us;
    uint8_t source;
    NeckNodeSample nodes[NECK_NODE_COUNT];
} NeckFrame;

/* IDs are caller-assigned nonzero tokens from an explicit registry, NOT hashes
 * of Python strings. Changing config or session requires an explicit boundary.
 */
typedef struct {
    uint32_t reference_id;
    uint32_t config_id;
    uint8_t node_id;
    uint8_t origin;
    double pitch_deg;
    double lateral_deg;
} NeckNeutralReference;

typedef struct {
    uint32_t valid_mask;
    uint32_t flags;
    uint8_t status;
    uint8_t pitch_status;
    uint8_t lateral_status;
    double accel_norm_m_s2;
    double total_tilt_deg;
    double pitch_raw_deg;
    double lateral_raw_deg;
} NeckRawTilt;

typedef struct {
    uint32_t config_id;
    uint32_t session_id;
    uint32_t reference_id;
    uint32_t valid_mask;
    uint32_t flags;
    uint64_t sequence;
    uint64_t frame_time_us;
    uint64_t sample_time_us;
    uint64_t sample_age_us;
    uint8_t node_id;
    uint8_t source;
    uint8_t status;
    uint8_t pitch_status;
    uint8_t lateral_status;
    double accel_body_m_s2[3];
    double accel_norm_m_s2;
    double total_tilt_deg;
    double pitch_raw_deg;
    double lateral_raw_deg;
    double flexion_deg;
    double right_lateral_deg;
} NeckTiltEstimate;

typedef struct {
    uint32_t config_id;
    uint8_t node_id;
    uint8_t initialized;
    double sensor_to_body[3][3];
} NeckTiltContext;

/* Normalize an approximately-unit Hamilton active sensor->body quaternion.
 * Initialization always invalidates context on error. tolerance is [0,1).
 */
NeckResult neck_tilt_init(NeckTiltContext *context, uint32_t config_id,
    uint8_t node_id, const double mount_wxyz[4], double quaternion_tolerance);
NeckResult neck_sensor_to_body(const double accel_sensor[3],
    const double mount_wxyz[4], double quaternion_tolerance, double accel_body[3]);
/* 输入已转到人体坐标系的三轴加速度，输出总倾角、俯仰角和侧倾角，单位均为度。 */
NeckResult neck_tilt_raw(const double accel_body[3], NeckRawTilt *out);
NeckResult neck_reference_init(NeckNeutralReference *reference,
    uint32_t reference_id, uint32_t config_id, uint8_t node_id, uint8_t origin,
    double pitch_deg, double lateral_deg);
/* reference==NULL: raw values available but no corrected angles; no first-frame
 * auto-zero. Explicit simulation reference is rejected for hardware source.
 * NECK_OK can still mean INVALID/NO_ACCEL; examine output status and valid bits.
 * Error returns clear every output validity bit. No stream/timing quality gate.
 */
/* 读取角度前检查 valid_mask 中对应的位；返回 NECK_OK 也可能表示本帧缺少加速度。 */
NeckResult neck_tilt_estimate(const NeckTiltContext *context,
    const NeckFrame *frame, const NeckNeutralReference *reference,
    NeckTiltEstimate *out);
const char *neck_tilt_status_name(uint8_t status);

#ifdef __cplusplus
}
#endif
#endif
