/* Analytic host assertions: no simulator, Python or production formula oracle. */
#include "neck_tilt_host_wrapper.h"
#include "neck_tilt.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
static const double G = 9.80665;
static const double PI = 3.14159265358979323846264338327950288;

#define CHECK(expr) do { ++checks; if (!(expr)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); } } while (0)
static void close_number(double value, double expected) { CHECK(isfinite(value)); CHECK(fabs(value - expected) <= 1e-8); }
static void initialize(const double mount[4]) { CHECK(ntw_init(1, 1, mount, 0.001) == 0); }
static void identity(void) { const double mount[4] = {1, 0, 0, 0}; initialize(mount); }

static NtwInput frame(double pitch, double lateral) {
    NtwInput input;
    double p = pitch * PI / 180.0, r = lateral * PI / 180.0;
    memset(&input, 0, sizeof(input));
    input.sequence = 7;
    input.frame_time_us = 70000;
    input.sample_time_us = 70000;
    input.config_id = 1;
    input.sample_time_known = 1;
    input.signal_valid_mask = 7;
    input.calibration_accel = 3;
    input.reference_present = 1;
    input.reference_config_id = 1;
    input.reference_node_id = 1;
    input.accel[0] = -G * sin(p);
    input.accel[1] = G * cos(p) * sin(r);
    input.accel[2] = G * cos(p) * cos(r);
    return input;
}

static NtwOutput estimate(const NtwInput *input) {
    NtwOutput output;
    memset(&output, 0xa5, sizeof(output));
    CHECK(ntw_estimate(input, &output) == 0);
    return output;
}

static void test_upright_and_sagittal(void) {
    static const double pitches[] = {-30, -15, -5, 0, 5, 15, 30};
    size_t index;
    identity();
    for (index = 0; index < sizeof(pitches) / sizeof(pitches[0]); ++index) {
        NtwInput input = frame(pitches[index], 0);
        NtwOutput output = estimate(&input);
        CHECK(output.status == NTW_COMPUTED);
        CHECK(output.pitch_status == NTW_COMPUTED && output.lateral_status == NTW_COMPUTED);
        CHECK(output.valid_fields == 127);
        close_number(output.accel_norm, G);
        close_number(output.total_tilt, fabs(pitches[index]));
        close_number(output.pitch_raw, pitches[index]);
        close_number(output.flexion, pitches[index]);
        close_number(output.right_lateral, 0);
        CHECK(output.flags == (NTW_QUALITY_NOT_EVALUATED | NTW_GRAVITY_ONLY_ASSUMPTION));
    }
}

static void test_pure_lateral_and_direction_edges(void) {
    static const double lateral[] = {-10, 10};
    size_t index;
    identity();
    for (index = 0; index < 2; ++index) {
        NtwInput input = frame(0, lateral[index]);
        NtwOutput output = estimate(&input);
        CHECK(output.status == NTW_PARTIAL);
        CHECK(output.pitch_status == NTW_DIRECTION_UNDETERMINED);
        CHECK(output.lateral_status == NTW_COMPUTED);
        CHECK(!(output.valid_fields & (NTW_PITCH | NTW_FLEXION)));
        CHECK(output.valid_fields & NTW_TOTAL);
        close_number(output.total_tilt, 10);
        close_number(output.right_lateral, lateral[index]);
        CHECK(output.flags & NTW_PITCH_DIRECTION_AMBIGUOUS);
        input.reference_pitch_deg = 4;
        output = estimate(&input);
        CHECK(!(output.valid_fields & NTW_FLEXION));
        input.reference_present = 0;
        output = estimate(&input);
        CHECK(output.status == NTW_UNCALIBRATED);
        CHECK(output.pitch_status == NTW_DIRECTION_UNDETERMINED);
        CHECK(output.lateral_status == NTW_UNCALIBRATED);
        CHECK(!(output.valid_fields & (NTW_FLEXION | NTW_RIGHT_LATERAL)));
    }
    {
        NtwInput input = frame(0, 10);
        NtwOutput output;
        /* Normalized ax is below/above the 1e-10 numerical boundary. */
        input.accel[0] = G * 0.5e-10;
        output = estimate(&input);
        CHECK(output.pitch_status == NTW_DIRECTION_UNDETERMINED);
        input.accel[0] = G * 2e-10;
        output = estimate(&input);
        CHECK(output.pitch_status == NTW_COMPUTED);
        CHECK(output.pitch_raw < -9.99);
        input.accel[0] = -G * 2e-10;
        output = estimate(&input);
        CHECK(output.pitch_raw > 9.99);
        input = frame(0, 0);
        input.accel[0] = G * 0.5e-10;
        output = estimate(&input);
        CHECK(output.pitch_status == NTW_COMPUTED);
        close_number(output.pitch_raw, 0);
    }
}

static void test_combined_and_offsets(void) {
    double p = 20 * PI / 180.0, r = -10 * PI / 180.0;
    double total = acos(cos(p) * cos(r)) * 180 / PI;
    double lateral = asin(cos(p) * sin(r)) * 180 / PI;
    NtwInput input = frame(20, -10);
    NtwOutput output;
    identity();
    output = estimate(&input);
    close_number(output.total_tilt, total);
    close_number(output.pitch_raw, total);
    close_number(output.lateral_raw, lateral);
    CHECK(fabs(output.pitch_raw - 20) > 2);
    input.reference_pitch_deg = 10;
    input.reference_lateral_deg = 3;
    output = estimate(&input);
    close_number(output.flexion, total - 10);
    close_number(output.right_lateral, lateral - 3);
    input = frame(-5, 0);
    input.reference_pitch_deg = 10;
    output = estimate(&input);
    close_number(output.flexion, -15);
    input.reference_present = 0;
    output = estimate(&input);
    CHECK(output.status == NTW_UNCALIBRATED);
    CHECK(output.pitch_status == NTW_UNCALIBRATED);
    CHECK(output.valid_fields & NTW_PITCH);
    CHECK(!(output.valid_fields & NTW_FLEXION));
    CHECK(output.flags & NTW_NEUTRAL_REFERENCE_UNSET);
    input = frame(5, 0);
    input.reference_present = 0;
    output = estimate(&input);
    close_number(output.pitch_raw, 5);
    CHECK(!(output.valid_fields & NTW_FLEXION)); /* First frame never auto-zeros. */
}

static void test_mounts_and_validation(void) {
    const double h = sqrt(0.5);
    double mount[4] = {h, 0, 0, h};
    NtwInput input = frame(15, 0);
    NtwOutput output;
    double body[3] = {input.accel[0], input.accel[1], input.accel[2]};
    size_t index;
    initialize(mount);
    input.accel[0] = body[1]; input.accel[1] = -body[0];
    output = estimate(&input);
    for (index = 0; index < 3; ++index) close_number(output.accel_body[index], body[index]);
    close_number(output.flexion, 15);
    for (index = 0; index < 4; ++index) mount[index] = -mount[index];
    initialize(mount);
    output = estimate(&input);
    close_number(output.flexion, 15);
    for (index = 0; index < 4; ++index) mount[index] *= 1.0005;
    initialize(mount);
    output = estimate(&input);
    close_number(output.flexion, 15);
    mount[0] = 0; mount[1] = 0; mount[2] = 0; mount[3] = 0;
    CHECK(ntw_init(1, 1, mount, 0.001) != 0);
    mount[0] = 2;
    CHECK(ntw_init(1, 1, mount, 0.001) != 0);
    mount[0] = NAN;
    CHECK(ntw_init(1, 1, mount, 0.001) != 0);
    mount[0] = 1;
    CHECK(ntw_init(1, 1, mount, -0.001) != 0);
    CHECK(ntw_init(1, 1, mount, 1.0) != 0);
    CHECK(ntw_init(1, 1, mount, NAN) != 0);
    CHECK(ntw_init(1, 3, mount, 0.001) != 0);
}

static void test_missing_and_metadata(void) {
    NtwInput input = frame(15, 0);
    NtwOutput output;
    uint32_t status;
    identity();
    for (status = 2; status <= 4; ++status) {
        input.node_status = status;
        input.signal_valid_mask = 0;
        input.sample_time_known = 0;
        output = estimate(&input);
        CHECK(output.status == NTW_NO_ACCEL);
        CHECK(output.pitch_status == NTW_NO_ACCEL && output.lateral_status == NTW_NO_ACCEL);
        CHECK(output.valid_fields == 0);
        CHECK(output.flags & NTW_ACCEL_UNAVAILABLE);
        CHECK(output.flags & (1u << (12 + status)));
    }
    input = frame(15, 0);
    input.node_status = 1; input.signal_valid_mask = 2;
    output = estimate(&input);
    CHECK(output.status == NTW_COMPUTED);
    close_number(output.flexion, 15);
    input.signal_valid_mask = 5;
    output = estimate(&input);
    CHECK(output.status == NTW_NO_ACCEL);
    input = frame(15, 0);
    input.sample_time_us = 30000;
    input.calibration_accel = 1;
    output = estimate(&input);
    CHECK(output.sample_age_us == 40000);
    CHECK(output.status == NTW_COMPUTED); /* Age/calibration are annotated, not S07 gating. */
    CHECK(output.flags & NTW_CALIBRATION_METADATA_PENDING);
    input.sample_time_known = 0;
    input.calibration_accel = -1;
    output = estimate(&input);
    CHECK(output.sample_time_known == 0);
    CHECK(output.flags & NTW_SAMPLE_TIME_UNKNOWN);
    CHECK(output.flags & NTW_CALIBRATION_METADATA_UNKNOWN);
    input = frame(15, 0);
    input.sequence = UINT64_C(9007199254741001);
    input.frame_time_us = UINT64_MAX - 8;
    input.sample_time_us = UINT64_MAX - 12;
    output = estimate(&input);
    CHECK(output.sequence == input.sequence);
    CHECK(output.frame_time_us == input.frame_time_us);
    CHECK(output.sample_age_us == 4);
}

static void test_bad_numeric_domain(void) {
    static const double bad[] = {NAN, INFINITY, -INFINITY};
    NtwInput input;
    NtwOutput output;
    size_t value, axis;
    identity();
    for (value = 0; value < 3; ++value) for (axis = 0; axis < 3; ++axis) {
        input = frame(0, 0);
        input.accel[axis] = bad[value];
        output = estimate(&input);
        CHECK(output.status == NTW_INVALID);
        CHECK(output.valid_fields == 0);
        CHECK(output.flags & NTW_INVALID_ACCEL_VECTOR);
    }
    input = frame(0, 0); input.accel[2] = 0;
    output = estimate(&input);
    CHECK(output.status == NTW_INVALID);
    CHECK(output.flags & NTW_ZERO_OR_TINY_ACCEL_VECTOR);
    CHECK(output.valid_fields == (NTW_BODY | NTW_NORM));
    input.accel[2] = 1e-10;
    output = estimate(&input);
    CHECK(output.status == NTW_INVALID);
    input = frame(0, 0); input.accel[2] = -G;
    input.reference_present = 0;
    output = estimate(&input);
    CHECK(output.status == NTW_INVALID);
    CHECK(output.flags & NTW_OUTSIDE_UPRIGHT_DOMAIN);
    input = frame(0, 0); input.accel[0] = G; input.accel[2] = 0;
    output = estimate(&input);
    CHECK(output.status == NTW_INVALID);
    input = frame(20, 8);
    for (axis = 0; axis < 3; ++axis) input.accel[axis] *= 1e299;
    output = estimate(&input);
    CHECK(output.status == NTW_COMPUTED);
    CHECK(isfinite(output.accel_norm));
    for (axis = 0; axis < 3; ++axis) input.accel[axis] = 1.7e308;
    output = estimate(&input);
    CHECK(output.status == NTW_INVALID);
    CHECK(output.flags & NTW_INVALID_ACCEL_NORM);
    {
        const double mount[4] = {cos(PI / 8), 0, 0, sin(PI / 8)};
        initialize(mount);
        input = frame(0, 0);
        input.accel[0] = 1.3e308; input.accel[1] = 1.3e308; input.accel[2] = 0;
        output = estimate(&input);
        CHECK(output.status == NTW_INVALID);
        CHECK(output.valid_fields == 0);
        CHECK(output.flags & NTW_INVALID_ACCEL_VECTOR);
    }
}

static void test_reference_guards(void) {
    NtwInput input = frame(12, 0);
    NtwOutput output;
    identity();
    input.config_id = 2;
    CHECK(ntw_estimate(&input, &output) != 0);
    input = frame(12, 0); input.reference_config_id = 2;
    CHECK(ntw_estimate(&input, &output) != 0);
    input = frame(12, 0); input.reference_node_id = 0;
    CHECK(ntw_estimate(&input, &output) != 0);
    input = frame(12, 0); input.source = 1;
    CHECK(ntw_estimate(&input, &output) != 0); /* Simulation zero must not calibrate hardware. */
    input.reference_origin = 1;
    output = estimate(&input);
    close_number(output.flexion, 12);
    input.reference_pitch_deg = NAN;
    CHECK(ntw_estimate(&input, &output) != 0);
    input = frame(12, 0); input.reference_lateral_deg = INFINITY;
    CHECK(ntw_estimate(&input, &output) != 0);
}

static NeckFrame native_frame(void) {
    NeckFrame value;
    unsigned int index;
    memset(&value, 0, sizeof(value));
    value.config_id = 1; value.session_id = 7;
    value.sequence = 7; value.frame_time_us = 70000;
    for (index = 0; index < NECK_NODE_COUNT; ++index) {
        value.nodes[index].status = NECK_NODE_MISSING;
        value.nodes[index].calibration_accel = NECK_CALIBRATION_UNKNOWN;
    }
    value.nodes[NECK_NODE_C4].status = NECK_NODE_PARTIAL;
    value.nodes[NECK_NODE_C4].signal_valid_mask = NECK_SIGNAL_ACCEL;
    value.nodes[NECK_NODE_C4].accel_present = 1;
    value.nodes[NECK_NODE_C4].sample_time_known = 1;
    value.nodes[NECK_NODE_C4].sample_time_us = 70000;
    value.nodes[NECK_NODE_C4].calibration_accel = 3;
    value.nodes[NECK_NODE_C4].accel_m_s2[2] = G;
    return value;
}

static void assert_rejected(const NeckTiltContext *context, const NeckFrame *input, const NeckNeutralReference *reference, NeckResult expected) {
    NeckTiltEstimate output;
    memset(&output, 0xa5, sizeof(output));
    CHECK(neck_tilt_estimate(context, input, reference, &output) == expected);
    CHECK(output.valid_mask == 0);
    CHECK(output.status == NECK_INVALID);
    CHECK(output.flags == NECK_FLAG_INPUT_REJECTED);
}

static void test_native_api_contract(void) {
    double mount[4] = {1, 0, 0, 0};
    double vector[3] = {0, 0, G}, transformed[3];
    NeckTiltContext context;
    NeckNeutralReference reference;
    NeckFrame input = native_frame();
    NeckTiltEstimate output;
    NeckRawTilt raw;
    unsigned int index;
    CHECK(neck_tilt_init(&context, 1, NECK_NODE_C4, mount, 0.001) == NECK_OK);
    CHECK(neck_reference_init(&reference, 9, 1, NECK_NODE_C4, NECK_REFERENCE_SIMULATION, 0, 0) == NECK_OK);
    CHECK(neck_tilt_estimate(&context, &input, &reference, &output) == NECK_OK);
    CHECK(output.config_id == 1 && output.session_id == 7 && output.reference_id == 9);
    CHECK(output.node_id == NECK_NODE_C4 && output.source == NECK_SOURCE_SIMULATION);
    CHECK(output.valid_mask & NECK_VALID_REFERENCE);
    CHECK(output.valid_mask & NECK_VALID_SAMPLE_AGE);
    CHECK(neck_tilt_estimate(&context, &input, NULL, &output) == NECK_OK);
    CHECK(!(output.valid_mask & NECK_VALID_REFERENCE));
    assert_rejected(NULL, &input, NULL, NECK_BAD_ARGUMENT);
    assert_rejected(&context, NULL, NULL, NECK_BAD_ARGUMENT);
    CHECK(neck_tilt_estimate(&context, &input, NULL, NULL) == NECK_BAD_ARGUMENT);
    CHECK(neck_tilt_init(NULL, 1, 1, mount, 0.001) == NECK_BAD_ARGUMENT);
    CHECK(neck_tilt_raw(vector, NULL) == NECK_BAD_ARGUMENT);
    CHECK(neck_tilt_raw(NULL, &raw) == NECK_OK);
    CHECK(raw.status == NECK_INVALID && raw.valid_mask == 0);
    CHECK(neck_sensor_to_body(vector, mount, 0.001, NULL) == NECK_BAD_ARGUMENT);
    CHECK(neck_sensor_to_body(vector, NULL, 0.001, transformed) == NECK_BAD_CONFIG);
    for (index = 0; index < 3; ++index) CHECK(transformed[index] == 0);
    CHECK(neck_reference_init(NULL, 9, 1, 1, 0, 0, 0) == NECK_BAD_ARGUMENT);
    CHECK(neck_reference_init(&reference, 0, 1, 1, 0, 0, 0) == NECK_BAD_REFERENCE);
    CHECK(reference.reference_id == 0);
    CHECK(neck_reference_init(&reference, 9, 1, 1, 2, 0, 0) == NECK_BAD_REFERENCE);
    CHECK(neck_reference_init(&reference, 9, 1, 3, 0, 0, 0) == NECK_BAD_REFERENCE);
    CHECK(neck_reference_init(&reference, 9, 0, 1, 0, 0, 0) == NECK_BAD_REFERENCE);
    CHECK(neck_reference_init(&reference, 9, 1, 1, 0, 0, 0) == NECK_OK);
    input.session_id = 0;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.source = 2;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[1].sample_time_us = 70001;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[1].status = NECK_NODE_OK;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[1].signal_valid_mask = 7;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[1].accel_present = 0;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[1].sample_time_known = 2;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[1].calibration_accel = 4;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[0].sample_time_known = 1;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[2].status = 5;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    input = native_frame(); input.nodes[2].signal_valid_mask = 8;
    assert_rejected(&context, &input, &reference, NECK_BAD_FRAME);
    CHECK(neck_tilt_init(&context, 0, 1, mount, 0.001) == NECK_BAD_CONFIG);
    CHECK(context.initialized == 0);
    input = native_frame();
    assert_rejected(&context, &input, &reference, NECK_BAD_CONFIG);
    CHECK(neck_tilt_init(&context, 1, NECK_NODE_C0, mount, 0.001) == NECK_OK);
    input.nodes[0] = input.nodes[1];
    input.nodes[0].accel_m_s2[0] = -G * sin(9 * PI / 180);
    input.nodes[0].accel_m_s2[2] = G * cos(9 * PI / 180);
    CHECK(neck_reference_init(&reference, 9, 1, NECK_NODE_C0, NECK_REFERENCE_SIMULATION, 0, 0) == NECK_OK);
    CHECK(neck_tilt_estimate(&context, &input, &reference, &output) == NECK_OK);
    CHECK(output.node_id == NECK_NODE_C0);
    close_number(output.flexion_deg, 9);
    mount[0] = 0; mount[3] = 1; /* Caller mutation must not mutate cached matrix. */
    CHECK(neck_tilt_estimate(&context, &input, &reference, &output) == NECK_OK);
    close_number(output.flexion_deg, 9);
    CHECK(strcmp(neck_tilt_status_name(NECK_DIRECTION_UNDETERMINED), "DIRECTION_UNDETERMINED") == 0);
    CHECK(strcmp(neck_tilt_status_name(255), "INVALID_STATUS") == 0);
}

int main(void) {
    test_upright_and_sagittal();
    test_pure_lateral_and_direction_edges();
    test_combined_and_offsets();
    test_mounts_and_validation();
    test_missing_and_metadata();
    test_bad_numeric_domain();
    test_reference_guards();
    test_native_api_contract();
    printf("PASS: 8 analytic host test groups, %u assertions\n", checks);
    printf("HOST sizeof: double=%zu, node=%zu, frame=%zu, reference=%zu, raw=%zu, estimate=%zu, context=%zu bytes\n",
        sizeof(double), sizeof(NeckNodeSample), sizeof(NeckFrame), sizeof(NeckNeutralReference),
        sizeof(NeckRawTilt), sizeof(NeckTiltEstimate), sizeof(NeckTiltContext));
    return 0;
}
