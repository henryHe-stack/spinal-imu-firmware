/* 逐节点判断加速度和姿态数据能否用于后续计算。帧、样本和当前时间均使用同一单调时钟。 */
/* S07 software eligibility, NOT physical/clinical confidence. All time values
 * must share one monotonic epoch. No HAL/RTOS/allocation. Invalid envelopes do
 * not refresh freshness. Unknown timestamps are rejected, not synthesized. */
#ifndef NECK_QUALITY_H
#define NECK_QUALITY_H
#include "neck_relative.h"
#ifdef __cplusplus
extern "C" {
#endif
#define NECK_QUALITY_ALGORITHM_ID "s07-wall-freshness-hysteresis-recovery-v1"
enum {
    NECK_Q_FRAME_REJECTED=1u, NECK_Q_FUTURE_FRAME=2u, NECK_Q_OLD_FRAME=4u,
    NECK_Q_STREAM_CHANGED=8u, NECK_Q_BAD_SEQUENCE=16u, NECK_Q_BAD_FRAME_TIME=32u,
    NECK_Q_CLOCK_FAULT=64u, NECK_Q_TIMEOUT=128u, NECK_Q_STREAM_GAP=256u,
    NECK_Q_METADATA_MISMATCH=512u, NECK_Q_NO_FRAME=1024u,
    NECK_Q_TIMEOUT_EDGE=2048u, NECK_Q_RECOVERY_EDGE=4096u,
    NECK_Q_RESET=8192u, NECK_Q_TRANSPORT_STALE=16384u, NECK_Q_ENGINEERING_ONLY=65536u
};
enum {
    NECK_QN_ACCEL_MISSING=1u, NECK_QN_ACCEL_NONFINITE=2u, NECK_QN_NORM_REJECTED=4u,
    NECK_QN_ACCEL_TIME_UNKNOWN=8u, NECK_QN_ACCEL_TIME_INVALID=16u,
    NECK_QN_ACCEL_CAL_UNKNOWN=32u, NECK_QN_ACCEL_CAL_PENDING=64u,
    NECK_QN_QUAT_MISSING=128u, NECK_QN_QUAT_INVALID=256u,
    NECK_QN_QUAT_TIME_UNKNOWN=512u, NECK_QN_QUAT_TIME_INVALID=1024u,
    NECK_QN_QUAT_CAL_UNKNOWN=2048u, NECK_QN_QUAT_CAL_PENDING=4096u,
    NECK_QN_WORLD_MISMATCH=8192u, NECK_QN_ACCEL_RECOVERING=16384u,
    NECK_QN_QUAT_RECOVERING=32768u, NECK_QN_QUAT_REQUIRES_ACCEL=65536u,
    NECK_QN_ACCEL_NOT_NEW=131072u, NECK_QN_QUAT_NOT_NEW=262144u,
    NECK_QN_ACCEL_METADATA=524288u, NECK_QN_QUAT_METADATA=1048576u
};
enum { NECK_Q_UNKNOWN=0, NECK_Q_SOFTWARE_READY=1, NECK_Q_PARTIAL=2 };
typedef struct {
    double gravity_m_s2, norm_reentry_error_m_s2, norm_exit_error_m_s2;
    double quaternion_norm_tolerance;
    uint64_t max_frame_age_us, max_sample_age_us, max_gap_us;
    uint64_t no_frame_timeout_us, recovery_us;
    uint8_t minimum_accel_calibration, minimum_system_calibration;
    uint8_t allow_simulation_uncalibrated;
} NeckQualityParameters;
/* frame_accepted 表示整帧通过基础检查；两个 usable_mask 的第 i 位分别表示节点 i 对应信号可用。 */
typedef struct {
    uint64_t sequence, frame_time_us, decision_time_us;
    uint32_t config_id, session_id, flags, node_flags[NECK_NODE_COUNT];
    uint8_t accel_usable_mask, orientation_usable_mask, state, source, frame_accepted;
} NeckQualityOutput;
typedef struct {
    uint64_t last_accel_time_us, last_quat_time_us, accel_good_since_us, quat_good_since_us;
    uint8_t accel_seen, quat_seen, accel_recovering, quat_recovering, norm_rejected;
} NeckQualityNodeState;
typedef struct {
    NeckQualityParameters parameters;
    NeckQualityNodeState nodes[NECK_NODE_COUNT];
    uint64_t previous_sequence, previous_frame_time_us, previous_wall_us, start_wall_us;
    uint32_t config_id, world_frame_id, session_id;
    uint8_t initialized, stream_bound, source, timeout_latched, clock_fault;
} NeckQualityContext;
void neck_quality_default_parameters(NeckQualityParameters *parameters);
NeckResult neck_quality_init(NeckQualityContext *context, uint32_t config_id,
    uint32_t world_frame_id, uint64_t now_us, const NeckQualityParameters *parameters);
void neck_quality_reset_stream(NeckQualityContext *context, uint64_t now_us);
NeckResult neck_quality_process(NeckQualityContext *context, const NeckFrame *acceleration,
    const NeckOrientationFrame *orientation, uint64_t now_us, NeckQualityOutput *output);
/* 0=no expiry, 1=expired/clock-fault. Output contains no new valid sample;
 * TIMEOUT_EDGE occurs once. Caller polls even when the input queue is empty. */
/* 输入队列为空时也要定期调用。持续超时期间 TIMEOUT 保持置位，TIMEOUT_EDGE 只在首次检出时置位。 */
int neck_quality_poll(NeckQualityContext *context, uint64_t now_us, NeckQualityOutput *output);
#ifdef __cplusplus
}
#endif
#endif
