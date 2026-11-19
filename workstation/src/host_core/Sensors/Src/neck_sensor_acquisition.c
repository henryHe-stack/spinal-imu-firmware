#include "neck_sensor_acquisition.h"
#include <string.h>

NeckSensorResult neck_sensor_acquisition_init(NeckSensorAcquisition *acquisition,
    const NeckI2cBus *bus, uint8_t selected_mask, uint8_t fusion_mode,
    uint8_t external_clock, uint32_t config_id, uint32_t session_id)
{
    unsigned i;
    if (acquisition == NULL) return NECK_SENSOR_BAD_ARGUMENT;
    memset(acquisition, 0, sizeof(*acquisition));
    if (bus == NULL || bus->write == NULL || bus->read_register == NULL ||
        bus->delay_ms == NULL ||
        (selected_mask != NECK_SENSOR_C4_ONLY && selected_mask != NECK_SENSOR_ALL_NODES) ||
        (fusion_mode != NECK_BNO055_MODE_IMU && fusion_mode != NECK_BNO055_MODE_NDOF) ||
        external_clock > 1u || config_id == 0u || session_id == 0u)
        return NECK_SENSOR_BAD_ARGUMENT;
    acquisition->bus = *bus;
    acquisition->selected_mask = selected_mask;
    acquisition->config_id = config_id;
    acquisition->session_id = session_id;
    for (i = 0u; i < NECK_NODE_COUNT; ++i) {
        acquisition->init_result[i] = NECK_SENSOR_NOT_CONFIGURED;
        if ((selected_mask & (1u << i)) == 0u) continue;
        acquisition->init_result[i] = neck_bno055_configure(&acquisition->bus,
            &acquisition->node[i], (uint8_t)i, fusion_mode, external_clock);
        if (acquisition->init_result[i] == NECK_SENSOR_OK)
            acquisition->ready_mask |= (uint8_t)(1u << i);
    }
    return acquisition->ready_mask != 0u ? NECK_SENSOR_OK : NECK_SENSOR_NOT_CONFIGURED;
}

NeckSensorResult neck_sensor_acquisition_frame(NeckSensorAcquisition *acquisition,
    uint64_t sequence, NeckSensorNowUs now_us, void *clock_context,
    NeckAcquisitionFrame *frame, NeckSensorReadTrace *trace)
{
    unsigned i, j;
    uint64_t last_time = 0u;
    if (frame != NULL) memset(frame, 0, sizeof(*frame));
    if (trace != NULL) memset(trace, 0, sizeof(*trace));
    if (acquisition == NULL || frame == NULL || trace == NULL || now_us == NULL ||
        acquisition->config_id == 0u || acquisition->session_id == 0u ||
        acquisition->selected_mask == 0u)
        return NECK_SENSOR_BAD_ARGUMENT;
    trace->selected_mask = acquisition->selected_mask;
    frame->acceleration.config_id = frame->orientation.config_id = acquisition->config_id;
    frame->acceleration.session_id = frame->orientation.session_id = acquisition->session_id;
    frame->acceleration.sequence = frame->orientation.sequence = sequence;
    frame->acceleration.source = frame->orientation.source = NECK_SOURCE_HARDWARE;
    for (i = 0u; i < NECK_NODE_COUNT; ++i) {
        NeckNodeSample *accel = &frame->acceleration.nodes[i];
        NeckOrientationSample *orient = &frame->orientation.nodes[i];
        NeckBno055Observation observation;
        uint64_t start, end;
        accel->calibration_accel = NECK_CALIBRATION_UNKNOWN;
        orient->calibration_system = NECK_CALIBRATION_UNKNOWN;
        accel->status = orient->status = NECK_NODE_MISSING;
        trace->result[i] = NECK_SENSOR_NOT_CONFIGURED;
        if ((acquisition->selected_mask & (1u << i)) == 0u) continue;
        accel->status = orient->status = NECK_NODE_FAULT;
        trace->result[i] = acquisition->init_result[i];
        if ((acquisition->ready_mask & (1u << i)) == 0u) continue;
        start = now_us(clock_context);
        if (start < last_time) goto clock_error;
        trace->read_start_us[i] = start;
        trace->result[i] = neck_bno055_read(&acquisition->bus,
            &acquisition->node[i], &observation);
        end = now_us(clock_context);
        if (end < start) goto clock_error;
        trace->read_end_us[i] = end;
        last_time = end;
        if (trace->result[i] != NECK_SENSOR_OK || !observation.observation_valid) continue;
        trace->observed_mask |= (uint8_t)(1u << i);
        /* No gyro sample is supplied: OK requires all three signal bits. */
        accel->status = orient->status = NECK_NODE_PARTIAL;
        accel->signal_valid_mask = NECK_SIGNAL_ACCEL | NECK_SIGNAL_QUATERNION;
        accel->accel_present = 1u;
        accel->calibration_accel = observation.calibration_accel;
        orient->quat_present = 1u;
        orient->calibration_system = observation.calibration_system;
        for (j = 0u; j < 3u; ++j)
            accel->accel_m_s2[j] = observation.accel_m_s2[j];
        for (j = 0u; j < 4u; ++j)
            orient->sensor_to_world_wxyz[j] = observation.quaternion_wxyz[j];
        /* Both sample time and common world remain explicitly unknown. */
    }
    frame->acceleration.frame_time_us = now_us(clock_context);
    if (frame->acceleration.frame_time_us < last_time) goto clock_error;
    frame->orientation.frame_time_us = frame->acceleration.frame_time_us;
    return trace->observed_mask != 0u ? NECK_SENSOR_OK : NECK_SENSOR_NO_OBSERVATION;

clock_error:
    memset(frame, 0, sizeof(*frame));
    memset(trace, 0, sizeof(*trace));
    return NECK_SENSOR_CLOCK_ERROR;
}
