/* 相对姿态接口。三个节点须使用同一世界坐标系；每组姿态分别记录原始值和中立位校正值。 */
/* S06: relative quaternion poses; C99, no HAL/RTOS/allocation.
 * Hamilton active wxyz. Measured q maps SENSOR -> REGISTERED COMMON WORLD.
 * Mount q maps SENSOR -> LOCAL ANATOMICAL BODY. These are different rotations.
 * q_world_body = q_world_sensor * inverse(q_body_sensor).
 * Relative Euler values use Rz(axial)*Ry(flexion)*Rx(right_lateral), NOT S03.
 * Common-world registration and timing metadata are caller declarations,
 * not evidence of actual heading alignment or hardware synchronization.
 */
#ifndef NECK_RELATIVE_H
#define NECK_RELATIVE_H
#include "neck_tilt.h"
#ifdef __cplusplus
extern "C" {
#endif
#define NECK_RELATIVE_ALGORITHM_ID "s06-common-world-mounted-relative-v1"
#define NECK_RELATIVE_PAIR_COUNT 4u
#define NECK_WORLD_REFERENCE 3u
typedef enum {
    NECK_REL_OVERALL=0, NECK_REL_UPPER=1, NECK_REL_LOWER=2, NECK_REL_BASE=3
} NeckRelativePairId;
typedef enum {
    NECK_REL_UNAVAILABLE=0, NECK_REL_RAW_ONLY=1, NECK_REL_CORRECTED=2
} NeckRelativeStatus;
enum {
    NECK_REL_RAW_QUAT=1u, NECK_REL_RAW_EULER=2u, NECK_REL_RAW_ANGLE=4u,
    NECK_REL_CORRECTED_QUAT=8u, NECK_REL_CORRECTED_EULER=16u,
    NECK_REL_CORRECTED_ANGLE=32u, NECK_REL_KNOWN_SKEW=64u
};
enum {
    NECK_REL_QUALITY_NOT_EVALUATED=1u, NECK_REL_THRESHOLDS_UNSET=2u,
    NECK_REL_NODE_UNAVAILABLE=4u, NECK_REL_INVALID_QUATERNION=8u,
    NECK_REL_WORLD_MISMATCH=16u, NECK_REL_TIME_UNKNOWN=32u,
    NECK_REL_STALE=64u, NECK_REL_EXCESSIVE_SKEW=128u,
    NECK_REL_RAW_GIMBAL_LOCK=256u, NECK_REL_CORRECTED_GIMBAL_LOCK=512u,
    NECK_REL_NEUTRAL_UNSET=1024u, NECK_REL_INPUT_REJECTED=2048u,
    NECK_REL_NORMALIZED=4096u, NECK_REL_CALIBRATION_UNKNOWN=8192u,
    NECK_REL_CALIBRATION_PENDING=16384u, NECK_REL_STREAM_GAP=32768u,
    NECK_REL_QUALITY_REJECTED=65536u, NECK_REL_INPUT_TIMEOUT=131072u
};
typedef struct {
    double sensor_to_world_wxyz[4];
    uint64_t sample_time_us;
    uint32_t world_frame_id;
    uint8_t quat_present, sample_time_known, calibration_system, status;
} NeckOrientationSample;
typedef struct {
    uint32_t config_id, session_id;
    uint64_t sequence, frame_time_us;
    uint8_t source;
    NeckOrientationSample nodes[NECK_NODE_COUNT];
} NeckOrientationFrame;
typedef struct {
    double quaternion_norm_tolerance, gimbal_cos_min;
    /* 前者限制样本相对帧时间的最大延迟，后者限制配对节点之间的采样时间差。 */
    uint64_t stale_after_us, max_node_skew_us;
} NeckRelativeParameters;
typedef struct {
    double raw_wxyz[4], corrected_wxyz[4];
    double raw_euler_deg[3], corrected_euler_deg[3]; /* flexion, lateral, axial */
    double raw_rotation_deg, corrected_rotation_deg; /* geodesic 0..180 deg */
    uint64_t sample_skew_us;
    uint32_t valid_mask, flags;
    uint8_t status, reference_node, target_node;
} NeckRelativePairOutput;
typedef struct {
    NeckRelativePairOutput pairs[NECK_RELATIVE_PAIR_COUNT];
    uint64_t sequence, frame_time_us;
    uint32_t config_id, session_id, world_frame_id, reference_id, flags;
    uint8_t source;
} NeckRelativeOutput;
typedef struct {
    double pair_wxyz[NECK_RELATIVE_PAIR_COUNT][4];
    uint32_t reference_id, config_id, world_frame_id;
    uint8_t origin;
} NeckRelativeReference;
typedef struct {
    NeckRelativeParameters parameters;
    double inverse_mount_wxyz[NECK_NODE_COUNT][4];
    NeckRelativeReference reference;
    uint64_t previous_sequence, previous_frame_time_us;
    uint32_t config_id, world_frame_id, session_id;
    uint8_t initialized, reference_set, stream_bound, source;
} NeckRelativeContext;

void neck_relative_default_parameters(NeckRelativeParameters *parameters);
NeckResult neck_relative_init(NeckRelativeContext *context, uint32_t config_id,
    uint32_t world_frame_id, const double sensor_to_body_wxyz[NECK_NODE_COUNT][4],
    const NeckRelativeParameters *parameters);
void neck_relative_reset_stream(NeckRelativeContext *context, uint8_t clear_reference);
NeckResult neck_relative_process(NeckRelativeContext *context,
    const NeckOrientationFrame *frame, NeckRelativeOutput *output);
NeckResult neck_relative_set_reference(NeckRelativeContext *context,
    const NeckRelativeReference *reference);
/* Explicit instantaneous capture, NOT a stationary-window detector. All four
 * raw quaternions must be available from the latest processed frame. Caller
 * must justify neutrality, provenance and stationarity; app waits for S04. */
/* 抓取最近一帧中四组有效的原始四元数作为中立位，静止条件由调用方确认。 */
NeckResult neck_relative_capture_reference(NeckRelativeContext *context,
    const NeckRelativeOutput *latest, uint32_t reference_id, uint8_t origin);
#ifdef __cplusplus
}
#endif
#endif
