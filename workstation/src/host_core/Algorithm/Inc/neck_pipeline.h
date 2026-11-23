/* 单节点滤波与调零接口。raw 用于查看原始倾角，filtered 提供滤波结果和可用的中立位校正角。 */
#ifndef NECK_PIPELINE_H
#define NECK_PIPELINE_H
#include "neck_tilt.h"
#ifdef __cplusplus
extern "C" {
#endif
#define NECK_PIPELINE_ALGORITHM_ID "s04-body-vector-zoh-lpf-static-neutral-v1"
enum {
    NECK_FILTER_SEEDED=1u, NECK_FILTER_UPDATED=2u, NECK_FILTER_RESET=4u,
    NECK_FILTER_WARMUP=8u, NECK_FILTER_DATA_INVALID=16u,
    NECK_FILTER_TIMING_REJECTED=32u, NECK_FILTER_FRAME_TIME_FALLBACK=64u,
    NECK_ZERO_WAIT_WARMUP=128u, NECK_ZERO_WINDOW_RESTART=256u,
    NECK_ZERO_COMMITTED=512u, NECK_FILTER_TIME_BASIS_CHANGED=1024u
};
typedef enum {
    NECK_ZERO_IDLE=0, NECK_ZERO_COLLECTING=1, NECK_ZERO_WAITING_WARMUP=2,
    NECK_ZERO_SUCCEEDED=3, NECK_ZERO_FAILED_INPUT=4,
    NECK_ZERO_FAILED_AMBIGUOUS=5, NECK_ZERO_CANCELED=6, NECK_ZERO_TIMED_OUT=7
} NeckZeroStatus;
typedef struct {
    double cutoff_hz;
    uint64_t nominal_period_us, max_gap_us, warmup_us;
    /* zero_duration_us 是单个稳定窗口的最短时长，zero_timeout_us 限制整次调零等待时间。 */
    uint64_t zero_duration_us, zero_timeout_us;
    uint32_t zero_min_samples;
    double gravity_m_s2, gravity_tolerance_m_s2, norm_span_m_s2, angle_span_deg;
} NeckPipelineParameters;
/* 调零进度记录当前窗口的样本数、持续时间及角度和加速度模长的波动范围。 */
typedef struct {
    uint8_t status;
    uint32_t sample_count;
    uint64_t window_elapsed_us;
    double pitch_span_deg, lateral_span_deg, norm_span_m_s2;
} NeckZeroProgress;
typedef struct {
    NeckTiltEstimate raw; /* Raw diagnostic; neutral correction not applied. */
    NeckTiltEstimate filtered;
    NeckZeroProgress zero;
    uint32_t filter_flags;
    uint8_t alpha_valid;
    /* alpha_valid 置位时，alpha 表示本帧加速度在滤波更新中的权重。 */
    double alpha;
} NeckPipelineOutput;
typedef struct {
    NeckTiltContext tilt;
    NeckPipelineParameters parameters;
    NeckNeutralReference reference;
    double nominal_alpha, filtered_vector[3], mean[3];
    double min_pitch, max_pitch, min_lateral, max_lateral, min_norm, max_norm;
    uint64_t previous_sequence, previous_frame_time_us, previous_time_us;
    uint64_t filter_start_us, request_start_us, window_start_us, window_last_us;
    uint32_t session_id, pending_reference_id, count;
    uint8_t initialized, stream_bound, source, filter_initialized;
    uint8_t time_basis_known, reference_set, reset_pending, zero_status;
    uint8_t pending_origin, request_time_set;
} NeckPipelineContext;

void neck_pipeline_default_parameters(NeckPipelineParameters *parameters);
NeckResult neck_pipeline_init(NeckPipelineContext *context, uint32_t config_id,
    uint8_t node_id, const double mount_wxyz[4], double quaternion_tolerance,
    const NeckPipelineParameters *parameters);
void neck_pipeline_reset_stream(NeckPipelineContext *context, uint8_t clear_reference);
NeckResult neck_pipeline_set_reference(NeckPipelineContext *context,
    const NeckNeutralReference *reference);
/* Explicit request. Existing reference remains usable until a new one commits.
 * Stationarity from acceleration is only an engineering candidate test.
 */
NeckResult neck_pipeline_begin_zero(NeckPipelineContext *context,
    uint32_t reference_id, uint8_t origin);
void neck_pipeline_cancel_zero(NeckPipelineContext *context);
/* Clear filter and any active candidate window without changing references
 * or accepted-stream chronology. Use FAILED_INPUT/TIMED_OUT as the reason. */
void neck_pipeline_invalidate(NeckPipelineContext *context, uint8_t zero_failure_status);
NeckResult neck_pipeline_process(NeckPipelineContext *context,
    const NeckFrame *frame, NeckPipelineOutput *output);
#ifdef __cplusplus
}
#endif
#endif
