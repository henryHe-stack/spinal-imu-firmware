#ifndef NECK_MEASUREMENT_PORT_H
#define NECK_MEASUREMENT_PORT_H
#include "neck_sensor_acquisition.h"
typedef struct {
    uint32_t config_id,world_id;
    double sensor_to_body[3][4],local_to_common_world[3][4];
    uint8_t node_mask;
} NeckMeasurementProfile;
typedef struct {
    uint32_t config_id,session_id;
    uint64_t sequence,read_start_us[3],read_end_us[3],accel_time_us[3],quat_time_us[3];
    uint8_t accel_mask,quat_mask;
} NeckMeasurementTimes;
int neck_measurement_profile_valid(const NeckMeasurementProfile *profile,uint32_t config);
/* All time fields are evidence from the board provider, not read-time guesses.
 * Validates per-node read binding; quality still enforces age/skew/calibration.
 * Rotation is local fusion world -> independently registered common world. */
uint8_t neck_measurement_apply(const NeckMeasurementProfile *profile,
    NeckAcquisitionFrame *frame,const NeckSensorReadTrace *trace,const NeckMeasurementTimes *times);
/* Strong board overrides require physical validation. Defaults return 0. */
int neck_board_three_profile(uint32_t config,NeckMeasurementProfile *profile);
int neck_board_three_times(const NeckAcquisitionFrame *frame,
    const NeckSensorReadTrace *trace,NeckMeasurementTimes *times);
#endif
