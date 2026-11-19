#include "neck_tilt.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Do not enable -ffast-math: NaN/Inf guards and ambiguity checks are required. */
static const double radians_to_degrees = 57.2957795130823208768;

static int finite_vector(const double *v, unsigned int length)
{
    unsigned int i;
    if (v == NULL) return 0;
    for (i = 0; i < length; ++i) {
        if (!isfinite(v[i])) return 0;
    }
    return 1;
}

static double norm3(const double v[3])
{
    return hypot(v[0], hypot(v[1], v[2]));
}

/* 将接近单位长度的安装四元数归一化，得到从传感器坐标系到人体坐标系的旋转矩阵。 */
static NeckResult make_matrix(const double q[4], double tolerance,
    double matrix[3][3])
{
    double norm, w, x, y, z;
    if (!isfinite(tolerance) || tolerance < 0.0 || tolerance >= 1.0 ||
        !finite_vector(q, 4)) return NECK_BAD_CONFIG;
    norm = hypot(q[0], hypot(q[1], hypot(q[2], q[3])));
    if (!isfinite(norm) || fabs(norm - 1.0) > tolerance || norm == 0.0)
        return NECK_BAD_CONFIG;
    w = q[0] / norm; x = q[1] / norm; y = q[2] / norm; z = q[3] / norm;
    matrix[0][0] = 1.0 - 2.0 * (y*y + z*z);
    matrix[0][1] = 2.0 * (x*y - z*w);
    matrix[0][2] = 2.0 * (x*z + y*w);
    matrix[1][0] = 2.0 * (x*y + z*w);
    matrix[1][1] = 1.0 - 2.0 * (x*x + z*z);
    matrix[1][2] = 2.0 * (y*z - x*w);
    matrix[2][0] = 2.0 * (x*z - y*w);
    matrix[2][1] = 2.0 * (y*z + x*w);
    matrix[2][2] = 1.0 - 2.0 * (x*x + y*y);
    return NECK_OK;
}

static int apply_matrix(const double matrix[3][3], const double input[3],
    double output[3])
{
    unsigned int row, col;
    if (!finite_vector(input, 3)) return 0;
    for (row = 0; row < 3; ++row) {
        output[row] = 0.0;
        for (col = 0; col < 3; ++col)
            output[row] += matrix[row][col] * input[col];
    }
    return finite_vector(output, 3);
}

NeckResult neck_tilt_init(NeckTiltContext *context, uint32_t config_id,
    uint8_t node_id, const double mount_wxyz[4], double quaternion_tolerance)
{
    NeckResult result;
    if (context == NULL) return NECK_BAD_ARGUMENT;
    memset(context, 0, sizeof(*context));
    if (config_id == 0 || node_id >= NECK_NODE_COUNT) return NECK_BAD_CONFIG;
    result = make_matrix(mount_wxyz, quaternion_tolerance, context->sensor_to_body);
    if (result != NECK_OK) return result;
    context->config_id = config_id;
    context->node_id = node_id;
    context->initialized = 1;
    return NECK_OK;
}

NeckResult neck_sensor_to_body(const double accel_sensor[3],
    const double mount_wxyz[4], double quaternion_tolerance, double accel_body[3])
{
    double matrix[3][3], transformed[3];
    NeckResult result;
    if (accel_body == NULL) return NECK_BAD_ARGUMENT;
    result = make_matrix(mount_wxyz, quaternion_tolerance, matrix);
    /* C99 does not implicitly add const inside an array-pointer type. */
    if (result == NECK_OK && !apply_matrix((const double (*)[3])matrix,
                                         accel_sensor, transformed))
        result = NECK_BAD_ARGUMENT;
    if (result == NECK_OK) memcpy(accel_body, transformed, sizeof(transformed));
    else memset(accel_body, 0, 3u * sizeof(double));
    return result;
}

static void invalid_raw(NeckRawTilt *out, uint32_t reason)
{
    memset(out, 0, sizeof(*out));
    out->status = NECK_INVALID;
    out->pitch_status = NECK_INVALID;
    out->lateral_status = NECK_INVALID;
    out->flags = reason;
}

NeckResult neck_tilt_raw(const double accel_body[3], NeckRawTilt *out)
{
    double norm, ax, ay, az, horizontal;
    if (out == NULL) return NECK_BAD_ARGUMENT;
    invalid_raw(out, NECK_FLAG_INVALID_ACCEL_VECTOR);
    if (!finite_vector(accel_body, 3)) return NECK_OK;
    norm = norm3(accel_body);
    if (!isfinite(norm)) {
        invalid_raw(out, NECK_FLAG_INVALID_ACCEL_NORM);
        return NECK_OK;
    }
    if (norm <= NECK_MIN_ACCEL_NORM) {
        invalid_raw(out, NECK_FLAG_ZERO_OR_TINY_ACCEL);
        out->accel_norm_m_s2 = norm;
        out->valid_mask = NECK_VALID_NORM;
        return NECK_OK;
    }
    /* 算法只在人体 Z 轴分量为正时解算倾角。超出该范围时，仍可读取加速度模长。 */
    if (accel_body[2] <= 0.0) {
        invalid_raw(out, NECK_FLAG_OUTSIDE_UPRIGHT);
        out->accel_norm_m_s2 = norm;
        out->valid_mask = NECK_VALID_NORM;
        return NECK_OK;
    }
    ax = accel_body[0] / norm;
    ay = accel_body[1] / norm;
    az = accel_body[2] / norm;
    horizontal = hypot(ax, ay);
    memset(out, 0, sizeof(*out));
    out->status = NECK_COMPUTED;
    out->pitch_status = NECK_COMPUTED;
    out->lateral_status = NECK_COMPUTED;
    out->accel_norm_m_s2 = norm;
    out->total_tilt_deg = atan2(horizontal, az) * radians_to_degrees;
    out->lateral_raw_deg = atan2(ay, hypot(ax, az)) * radians_to_degrees;
    out->valid_mask = NECK_VALID_NORM | NECK_VALID_TOTAL | NECK_VALID_LATERAL_RAW;
    out->flags = NECK_FLAG_GRAVITY_ONLY;
    /* 直立时俯仰角取零；有水平倾斜但前后分量接近零时，保留侧倾角并标记俯仰方向不确定。 */
    if (horizontal <= NECK_DIRECTION_EPS) {
        out->pitch_raw_deg = 0.0;
        out->valid_mask |= NECK_VALID_PITCH_RAW;
    } else if (fabs(ax) <= NECK_DIRECTION_EPS) {
        out->status = NECK_PARTIAL;
        out->pitch_status = NECK_DIRECTION_UNDETERMINED;
        out->flags |= NECK_FLAG_PITCH_AMBIGUOUS;
    } else {
        /* 俯仰角取总倾角的大小，并按人体前向分量的反号确定屈伸方向。 */
        out->pitch_raw_deg = copysign(out->total_tilt_deg, -ax);
        out->valid_mask |= NECK_VALID_PITCH_RAW;
    }
    return NECK_OK;
}

NeckResult neck_reference_init(NeckNeutralReference *reference,
    uint32_t reference_id, uint32_t config_id, uint8_t node_id, uint8_t origin,
    double pitch_deg, double lateral_deg)
{
    if (reference == NULL) return NECK_BAD_ARGUMENT;
    memset(reference, 0, sizeof(*reference));
    if (reference_id == 0 || config_id == 0 || node_id >= NECK_NODE_COUNT ||
        origin > NECK_REFERENCE_EXTERNAL || !isfinite(pitch_deg) ||
        !isfinite(lateral_deg)) return NECK_BAD_REFERENCE;
    reference->reference_id = reference_id;
    reference->config_id = config_id;
    reference->node_id = node_id;
    reference->origin = origin;
    reference->pitch_deg = pitch_deg;
    reference->lateral_deg = lateral_deg;
    return NECK_OK;
}

static int frame_metadata_valid(const NeckFrame *frame)
{
    unsigned int i;
    if (frame->config_id == 0 || frame->session_id == 0 ||
        frame->source > NECK_SOURCE_HARDWARE) return 0;
    for (i = 0; i < NECK_NODE_COUNT; ++i) {
        const NeckNodeSample *node = &frame->nodes[i];
        const unsigned int mask = node->signal_valid_mask;
        if (mask > 7 || node->status > NECK_NODE_STALE ||
            node->accel_present > 1 || node->sample_time_known > 1 ||
            (node->calibration_accel > 3 &&
             node->calibration_accel != NECK_CALIBRATION_UNKNOWN)) return 0;
        if ((node->status == NECK_NODE_OK && mask != 7) ||
            (node->status == NECK_NODE_PARTIAL && (mask == 0 || mask == 7)) ||
            (node->status >= NECK_NODE_MISSING && mask != 0)) return 0;
        if (((mask & NECK_SIGNAL_ACCEL) != 0) != (node->accel_present != 0))
            return 0;
        if (node->sample_time_known &&
            (mask == 0 || node->sample_time_us > frame->frame_time_us)) return 0;
    }
    return 1;
}

static NeckResult rejected(NeckTiltEstimate *out, NeckResult reason)
{
    memset(out, 0, sizeof(*out));
    out->status = NECK_INVALID;
    out->pitch_status = NECK_INVALID;
    out->lateral_status = NECK_INVALID;
    out->flags = NECK_FLAG_INPUT_REJECTED;
    return reason;
}

NeckResult neck_tilt_estimate(const NeckTiltContext *context,
    const NeckFrame *frame, const NeckNeutralReference *reference,
    NeckTiltEstimate *out)
{
    const NeckNodeSample *node;
    NeckRawTilt raw;
    if (out == NULL) return NECK_BAD_ARGUMENT;
    if (context == NULL || frame == NULL) return rejected(out, NECK_BAD_ARGUMENT);
    if (context->initialized != 1 || context->config_id == 0 ||
        context->node_id >= NECK_NODE_COUNT) return rejected(out, NECK_BAD_CONFIG);
    if (frame->config_id != context->config_id)
        return rejected(out, NECK_CONFIG_MISMATCH);
    if (!frame_metadata_valid(frame)) return rejected(out, NECK_BAD_FRAME);
    if (reference != NULL) {
        if (reference->reference_id == 0 ||
            reference->config_id != context->config_id ||
            reference->node_id != context->node_id ||
            reference->origin > NECK_REFERENCE_EXTERNAL ||
            !isfinite(reference->pitch_deg) || !isfinite(reference->lateral_deg))
            return rejected(out, NECK_BAD_REFERENCE);
        if (reference->origin == NECK_REFERENCE_SIMULATION &&
            frame->source != NECK_SOURCE_SIMULATION)
            return rejected(out, NECK_SIM_REFERENCE_ON_HARDWARE);
    }
    memset(out, 0, sizeof(*out));
    node = &frame->nodes[context->node_id];
    out->config_id = context->config_id;
    out->session_id = frame->session_id;
    out->node_id = context->node_id;
    out->source = frame->source;
    out->sequence = frame->sequence;
    out->frame_time_us = frame->frame_time_us;
    out->flags = NECK_FLAG_QUALITY_NOT_EVALUATED;
    if (reference != NULL) {
        out->reference_id = reference->reference_id;
        out->valid_mask |= NECK_VALID_REFERENCE;
    }
    if (node->sample_time_known) {
        out->sample_time_us = node->sample_time_us;
        out->sample_age_us = frame->frame_time_us - node->sample_time_us;
        out->valid_mask |= NECK_VALID_SAMPLE_TIME | NECK_VALID_SAMPLE_AGE;
    } else out->flags |= NECK_FLAG_SAMPLE_TIME_UNKNOWN;
    if (node->calibration_accel == NECK_CALIBRATION_UNKNOWN)
        out->flags |= NECK_FLAG_CALIBRATION_UNKNOWN;
    else if (node->calibration_accel < 3)
        out->flags |= NECK_FLAG_CALIBRATION_PENDING;
    if (!(node->signal_valid_mask & NECK_SIGNAL_ACCEL) || !node->accel_present) {
        invalid_raw(&raw, NECK_FLAG_ACCEL_UNAVAILABLE);
        raw.status = NECK_NO_ACCEL;
        raw.pitch_status = NECK_NO_ACCEL;
        raw.lateral_status = NECK_NO_ACCEL;
        out->flags |= NECK_FLAG_NODE_OK << node->status;
    } else if (!apply_matrix(context->sensor_to_body, node->accel_m_s2,
                           out->accel_body_m_s2)) {
        memset(out->accel_body_m_s2, 0, sizeof(out->accel_body_m_s2));
        invalid_raw(&raw, NECK_FLAG_INVALID_ACCEL_VECTOR);
    } else {
        out->valid_mask |= NECK_VALID_BODY;
        (void)neck_tilt_raw(out->accel_body_m_s2, &raw);
    }
    out->status = raw.status;
    out->pitch_status = raw.pitch_status;
    out->lateral_status = raw.lateral_status;
    out->valid_mask |= raw.valid_mask;
    out->flags |= raw.flags;
    out->accel_norm_m_s2 = raw.accel_norm_m_s2;
    out->total_tilt_deg = raw.total_tilt_deg;
    out->pitch_raw_deg = raw.pitch_raw_deg;
    out->lateral_raw_deg = raw.lateral_raw_deg;
    if (raw.status != NECK_INVALID && raw.status != NECK_NO_ACCEL) {
        if (reference == NULL) {
            out->status = NECK_UNCALIBRATED;
            if (raw.valid_mask & NECK_VALID_PITCH_RAW)
                out->pitch_status = NECK_UNCALIBRATED;
            if (raw.valid_mask & NECK_VALID_LATERAL_RAW)
                out->lateral_status = NECK_UNCALIBRATED;
            out->flags |= NECK_FLAG_NEUTRAL_UNSET;
        } else {
            /* 各轴分别扣除中立位角度，只有方向有效的轴才输出校正值。 */
            if (raw.valid_mask & NECK_VALID_PITCH_RAW) {
                out->flexion_deg = raw.pitch_raw_deg - reference->pitch_deg;
                out->valid_mask |= NECK_VALID_FLEXION;
            }
            if (raw.valid_mask & NECK_VALID_LATERAL_RAW) {
                out->right_lateral_deg = raw.lateral_raw_deg - reference->lateral_deg;
                out->valid_mask |= NECK_VALID_RIGHT_LATERAL;
            }
        }
    }
    return NECK_OK;
}

const char *neck_tilt_status_name(uint8_t status)
{
    switch (status) {
    case NECK_COMPUTED: return "COMPUTED";
    case NECK_PARTIAL: return "PARTIAL";
    case NECK_UNCALIBRATED: return "UNCALIBRATED";
    case NECK_NO_ACCEL: return "NO_ACCEL";
    case NECK_INVALID: return "INVALID";
    case NECK_DIRECTION_UNDETERMINED: return "DIRECTION_UNDETERMINED";
    default: return "INVALID_STATUS";
    }
}
