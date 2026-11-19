/* 单节点持续姿态检测。输入为中立位校正后的滤波角度，分别判断屈伸和侧屈。 */
/* S05 single-node sustained posture detection. C99, no HAL/RTOS/allocation.
 * Paper thresholds are reproduction defaults, NOT clinical recommendations.
 * Warning condition includes danger; each condition has its own 10 s clock.
 * Consume S04 FILTERED, neutral-corrected output. Default application node C4.
 */
#ifndef NECK_POSTURE_H
#define NECK_POSTURE_H
#include "neck_pipeline.h"
#ifdef __cplusplus
extern "C" {
#endif
#define NECK_POSTURE_ALGORITHM_ID "s05-nested-threshold-continuous-v1"
typedef enum {
    NECK_POSTURE_UNKNOWN=0, NECK_POSTURE_NORMAL=1,
    NECK_POSTURE_WARNING=2, NECK_POSTURE_DANGER=3,
    NECK_POSTURE_EXTENSION=4 /* A category, NOT higher severity than danger. */
} NeckPostureLevel;
enum {
    NECK_EVENT_TRIGGER=1u, NECK_EVENT_CLEAR=2u, NECK_EVENT_LEVEL_CHANGE=4u,
    NECK_EVENT_INVALIDATED=8u, NECK_EVENT_DIRECTION_RESET=16u
};
enum {
    NECK_POSTURE_QUALITY_NOT_EVALUATED=1u,
    NECK_POSTURE_FRAME_TIME_FALLBACK=2u, NECK_POSTURE_INPUT_INVALID=4u,
    NECK_POSTURE_WARMUP=8u, NECK_POSTURE_STREAM_RESET=16u,
    NECK_POSTURE_TIMING_REJECTED=32u, NECK_POSTURE_REFERENCE_CHANGED=64u,
    NECK_POSTURE_NO_REFERENCE=128u, NECK_POSTURE_PIPELINE_RESET=256u,
    NECK_POSTURE_QUALITY_REJECTED=512u, NECK_POSTURE_INPUT_TIMEOUT=1024u
};
typedef struct {
    double flexion_warning_deg, flexion_danger_deg;
    double lateral_warning_deg, lateral_danger_deg;
    uint64_t continuous_duration_us, max_gap_us;
} NeckPostureParameters;
/* instant_level 表示本帧的角度分级，alarm_level 表示满足持续时间要求后的告警级别。 */
typedef struct {
    uint64_t warning_elapsed_us, danger_elapsed_us, extension_elapsed_us;
    /* events 记录本帧发生的告警变化；查询持续中的告警应读取 alarm_level。 */
    uint32_t events;
    uint8_t valid, instant_level, alarm_level, previous_alarm_level;
    int8_t direction; /* +1=flexion/right, -1=extension/left, 0=normal/unknown */
} NeckPostureAxisOutput;
typedef struct {
    NeckPostureAxisOutput flexion, lateral;
    uint64_t sequence, frame_time_us, time_us;
    uint32_t config_id, session_id, reference_id, flags;
    uint8_t source, node_id, sample_time_known;
} NeckPostureOutput;
typedef struct {
    uint64_t warning_start_us, danger_start_us, extension_start_us;
    uint8_t warning_running, danger_running, extension_running;
    uint8_t known, alarm_level;
    int8_t direction;
} NeckPostureAxisState;
typedef struct {
    NeckPostureParameters parameters;
    NeckPostureAxisState flexion, lateral;
    uint64_t previous_sequence, previous_frame_time_us, previous_time_us;
    uint32_t config_id, session_id, reference_id;
    uint8_t initialized, node_id, stream_bound, source, time_basis_known;
} NeckPostureContext;

void neck_posture_default_parameters(NeckPostureParameters *parameters);
NeckResult neck_posture_init(NeckPostureContext *context, uint32_t config_id,
    uint8_t node_id, const NeckPostureParameters *parameters);
/* Explicit stream reset clears clocks and binding. Caller owns any event
 * required at an administrative reset; this function produces no output. */
void neck_posture_reset(NeckPostureContext *context);
/* Wall-clock decision without fabricating an input sample. Emits at most one
 * CLEAR|INVALIDATED per active axis; retains stream metadata, clears clocks. */
NeckResult neck_posture_invalidate(NeckPostureContext *context, uint64_t now_us,
    uint32_t reason_flags, NeckPostureOutput *output);
NeckResult neck_posture_process(NeckPostureContext *context,
    const NeckPipelineOutput *input, NeckPostureOutput *output);
#ifdef __cplusplus
}
#endif
#endif
