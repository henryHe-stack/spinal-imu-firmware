/* Host-only scalar/array adapter. Production firmware never links this file. */
#include "neck_tilt_host_wrapper.h"
#include "neck_tilt.h"
#include <string.h>

static NeckTiltContext context;

size_t ntw_input_size(void) { return sizeof(NtwInput); }
size_t ntw_output_size(void) { return sizeof(NtwOutput); }

int ntw_init(uint32_t config_id, uint32_t node_id, const double mount[4], double tolerance) {
    if (node_id >= NECK_NODE_COUNT) return NECK_BAD_CONFIG;
    return neck_tilt_init(&context, config_id, (uint8_t)node_id, mount, tolerance);
}

static uint32_t mapped_flags(uint32_t flags) {
    uint32_t result = 0;
    if (flags & NECK_FLAG_QUALITY_NOT_EVALUATED) result |= NTW_QUALITY_NOT_EVALUATED;
    if (flags & NECK_FLAG_SAMPLE_TIME_UNKNOWN) result |= NTW_SAMPLE_TIME_UNKNOWN;
    if (flags & NECK_FLAG_CALIBRATION_UNKNOWN) result |= NTW_CALIBRATION_METADATA_UNKNOWN;
    if (flags & NECK_FLAG_CALIBRATION_PENDING) result |= NTW_CALIBRATION_METADATA_PENDING;
    if (flags & NECK_FLAG_NEUTRAL_UNSET) result |= NTW_NEUTRAL_REFERENCE_UNSET;
    if (flags & NECK_FLAG_GRAVITY_ONLY) result |= NTW_GRAVITY_ONLY_ASSUMPTION;
    if (flags & NECK_FLAG_PITCH_AMBIGUOUS) result |= NTW_PITCH_DIRECTION_AMBIGUOUS;
    if (flags & NECK_FLAG_ACCEL_UNAVAILABLE) result |= NTW_ACCEL_UNAVAILABLE;
    if (flags & NECK_FLAG_INVALID_ACCEL_VECTOR) result |= NTW_INVALID_ACCEL_VECTOR;
    if (flags & NECK_FLAG_INVALID_ACCEL_NORM) result |= NTW_INVALID_ACCEL_NORM;
    if (flags & NECK_FLAG_ZERO_OR_TINY_ACCEL) result |= NTW_ZERO_OR_TINY_ACCEL_VECTOR;
    if (flags & NECK_FLAG_OUTSIDE_UPRIGHT) result |= NTW_OUTSIDE_UPRIGHT_DOMAIN;
    if (flags & NECK_FLAG_NODE_OK) result |= NTW_NODE_OK;
    if (flags & NECK_FLAG_NODE_PARTIAL) result |= NTW_NODE_PARTIAL;
    if (flags & NECK_FLAG_NODE_MISSING) result |= NTW_NODE_MISSING;
    if (flags & NECK_FLAG_NODE_FAULT) result |= NTW_NODE_FAULT;
    if (flags & NECK_FLAG_NODE_STALE) result |= NTW_NODE_STALE;
    return result;
}

int ntw_estimate(const NtwInput *input, NtwOutput *output) {
    NeckFrame frame;
    NeckNodeSample *node;
    NeckNeutralReference reference;
    NeckTiltEstimate estimate;
    NeckResult result;
    unsigned int i;
    if (input == NULL || output == NULL) return NECK_BAD_ARGUMENT;
    memset(output, 0, sizeof(*output));
    memset(&frame, 0, sizeof(frame));
    if (input->source > 1 || input->sample_time_known > 1 || input->node_status > 4 ||
        input->signal_valid_mask > 7 || input->calibration_accel < -1 || input->calibration_accel > 3 ||
        input->reference_origin > 1 || input->reference_node_id > 2) return NECK_BAD_ARGUMENT;
    frame.config_id = input->config_id;
    frame.session_id = 7;
    frame.source = (uint8_t)input->source;
    frame.sequence = input->sequence;
    frame.frame_time_us = input->frame_time_us;
    for (i = 0; i < NECK_NODE_COUNT; ++i) {
        frame.nodes[i].status = NECK_NODE_MISSING;
        frame.nodes[i].calibration_accel = NECK_CALIBRATION_UNKNOWN;
    }
    node = &frame.nodes[context.node_id];
    node->status = (uint8_t)input->node_status;
    node->signal_valid_mask = (uint8_t)input->signal_valid_mask;
    node->accel_present = (input->signal_valid_mask & NECK_SIGNAL_ACCEL) != 0;
    node->sample_time_known = (uint8_t)input->sample_time_known;
    node->sample_time_us = input->sample_time_us;
    node->calibration_accel = input->calibration_accel < 0 ? NECK_CALIBRATION_UNKNOWN : (uint8_t)input->calibration_accel;
    memcpy(node->accel_m_s2, input->accel, sizeof(input->accel));
    if (input->reference_present) {
        result = neck_reference_init(&reference, 9, input->reference_config_id,
            (uint8_t)input->reference_node_id, (uint8_t)input->reference_origin,
            input->reference_pitch_deg, input->reference_lateral_deg);
        if (result != NECK_OK) return result;
    }
    result = neck_tilt_estimate(&context, &frame, input->reference_present ? &reference : NULL, &estimate);
    output->sequence = estimate.sequence;
    output->frame_time_us = estimate.frame_time_us;
    output->sample_time_us = estimate.sample_time_us;
    output->sample_age_us = estimate.sample_age_us;
    output->status = estimate.status;
    output->pitch_status = estimate.pitch_status;
    output->lateral_status = estimate.lateral_status;
    output->valid_fields = estimate.valid_mask & 127u;
    output->flags = mapped_flags(estimate.flags);
    output->sample_time_known = (estimate.valid_mask & NECK_VALID_SAMPLE_TIME) != 0;
    memcpy(output->accel_body, estimate.accel_body_m_s2, sizeof(output->accel_body));
    output->accel_norm = estimate.accel_norm_m_s2;
    output->total_tilt = estimate.total_tilt_deg;
    output->pitch_raw = estimate.pitch_raw_deg;
    output->lateral_raw = estimate.lateral_raw_deg;
    output->flexion = estimate.flexion_deg;
    output->right_lateral = estimate.right_lateral_deg;
    return result;
}
