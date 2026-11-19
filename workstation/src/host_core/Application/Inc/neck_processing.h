/* Shared PC/STM32 processing path. A single owner calls process/poll/commands.
 * No HAL/RTOS/allocation. Poll is REQUIRED even in complete input silence. */
#ifndef NECK_PROCESSING_H
#define NECK_PROCESSING_H
#include "neck_posture.h"
#include "neck_quality.h"
typedef struct { NeckFrame acceleration; NeckOrientationFrame orientation; } NeckAcquisitionFrame;
enum { NECK_REQUEST_IDLE=0, NECK_REQUEST_WAIT_QUALITY=1, NECK_REQUEST_COLLECTING=2 };
enum { NECK_STEP_FRAME=1u, NECK_STEP_REJECTED=2u, NECK_STEP_TIMEOUT_EDGE=4u };
typedef struct {
    NeckTiltEstimate nodes[NECK_NODE_COUNT];
    uint32_t filter_flags[NECK_NODE_COUNT];
    uint8_t zero_status[NECK_NODE_COUNT];
    NeckPostureOutput posture;
    NeckQualityOutput quality;
    uint64_t valid_until_us;
    uint32_t pending_reference_id, invalidation_epoch;
    uint8_t zero_request_state;
} NeckPoseBatch;
typedef struct {
    NeckPoseBatch pose;
    NeckPipelineOutput c4;
    NeckRelativeOutput relative;
    uint32_t step_flags;
} NeckProcessingOutput;
typedef struct {
    NeckQualityParameters quality;
    NeckPipelineParameters pipeline;
    NeckPostureParameters posture;
    NeckRelativeParameters relative;
} NeckProcessingParameters;
typedef struct {
    uint32_t zero_requests, zero_cancels, command_errors, node_commits, filter_resets;
    uint32_t pipeline_errors, posture_errors, relative_errors, relative_commits, relative_aborts;
    uint32_t rejected_frames, timeout_transitions;
} NeckProcessingStats;
typedef struct {
    NeckPipelineContext pipelines[NECK_NODE_COUNT];
    NeckPostureContext posture;
    NeckRelativeContext relative;
    NeckQualityContext quality;
    NeckProcessingStats stats;
    uint64_t request_wall_us;
    uint32_t next_reference_id, pending_reference_id, invalidation_epoch;
    uint32_t previous_tilt_masks[NECK_NODE_COUNT], previous_relative_masks[NECK_RELATIVE_PAIR_COUNT];
    uint8_t initialized, request_state, request_origin, previous_accel_mask, previous_quat_mask;
    uint8_t single_c4;
} NeckProcessingContext;
void neck_processing_default_parameters(NeckProcessingParameters *parameters);
/* C4 acceleration tilt only. Mount is a verified sensor-to-body quaternion.
 * No common world or relative orientation is claimed in this mode. */
NeckResult neck_processing_init_single_c4(NeckProcessingContext *context, uint32_t config_id,
    const double mount[4], uint64_t now_us, const NeckProcessingParameters *parameters);
NeckResult neck_processing_init(NeckProcessingContext *context, uint32_t config_id,
    uint32_t world_frame_id, const double mounts[NECK_NODE_COUNT][4], uint64_t now_us,
    const NeckProcessingParameters *parameters);
NeckResult neck_processing_request_zero(NeckProcessingContext *context,
    uint64_t now_us, uint8_t origin);
void neck_processing_cancel_zero(NeckProcessingContext *context);
NeckResult neck_processing_process(NeckProcessingContext *context,
    const NeckAcquisitionFrame *input, uint64_t now_us, NeckProcessingOutput *output);
/* Return 1 when output is an UNKNOWN wall-clock decision, 0 otherwise.
 * Repeated expired polls contain zero events: they are safe as heartbeats. */
int neck_processing_poll(NeckProcessingContext *context, uint64_t now_us,
    NeckProcessingOutput *output);
/* Explicit stream boundary clears references; emits invalidation first. */
NeckResult neck_processing_reset_stream(NeckProcessingContext *context,
    uint64_t now_us, NeckProcessingOutput *output);
/* For a queued COPY only: never touches detector state or invents events.
 * Expire by freshness/epoch immediately before each transmission. */
int neck_processing_expire_pose(NeckPoseBatch *pose, uint64_t now_us, uint32_t current_epoch);
#endif
