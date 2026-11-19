#ifndef NECK_TILT_HOST_WRAPPER_H
#define NECK_TILT_HOST_WRAPPER_H

/* Host-test ABI only; this header must not be added to firmware source paths. */
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint64_t sequence, frame_time_us, sample_time_us;
    uint32_t config_id, source, sample_time_known, node_status, signal_valid_mask;
    int32_t calibration_accel;
    uint32_t reference_present, reference_origin, reference_config_id, reference_node_id;
    double accel[3], reference_pitch_deg, reference_lateral_deg;
} NtwInput;

typedef struct {
    uint64_t sequence, frame_time_us, sample_time_us, sample_age_us;
    uint32_t status, pitch_status, lateral_status, flags, valid_fields, sample_time_known;
    double accel_body[3], accel_norm, total_tilt, pitch_raw, lateral_raw, flexion, right_lateral;
} NtwOutput;

enum { NTW_COMPUTED, NTW_PARTIAL, NTW_UNCALIBRATED, NTW_NO_ACCEL, NTW_INVALID, NTW_DIRECTION_UNDETERMINED };
enum { NTW_BODY = 1, NTW_NORM = 2, NTW_TOTAL = 4, NTW_PITCH = 8, NTW_LATERAL = 16, NTW_FLEXION = 32, NTW_RIGHT_LATERAL = 64 };
enum {
    NTW_QUALITY_NOT_EVALUATED = 1u << 0,
    NTW_SAMPLE_TIME_UNKNOWN = 1u << 1,
    NTW_CALIBRATION_METADATA_UNKNOWN = 1u << 2,
    NTW_CALIBRATION_METADATA_PENDING = 1u << 3,
    NTW_NEUTRAL_REFERENCE_UNSET = 1u << 4,
    NTW_GRAVITY_ONLY_ASSUMPTION = 1u << 5,
    NTW_PITCH_DIRECTION_AMBIGUOUS = 1u << 6,
    NTW_ACCEL_UNAVAILABLE = 1u << 7,
    NTW_INVALID_ACCEL_VECTOR = 1u << 8,
    NTW_INVALID_ACCEL_NORM = 1u << 9,
    NTW_ZERO_OR_TINY_ACCEL_VECTOR = 1u << 10,
    NTW_OUTSIDE_UPRIGHT_DOMAIN = 1u << 11,
    NTW_NODE_OK = 1u << 12,
    NTW_NODE_PARTIAL = 1u << 13,
    NTW_NODE_MISSING = 1u << 14,
    NTW_NODE_FAULT = 1u << 15,
    NTW_NODE_STALE = 1u << 16
};

size_t ntw_input_size(void);
size_t ntw_output_size(void);
int ntw_init(uint32_t config_id, uint32_t node_id, const double mount[4], double tolerance);
int ntw_estimate(const NtwInput *input, NtwOutput *output);

#endif
