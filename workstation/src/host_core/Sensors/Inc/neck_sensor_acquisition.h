#ifndef NECK_SENSOR_ACQUISITION_H
#define NECK_SENSOR_ACQUISITION_H

#include "neck_bno055.h"
#include "neck_processing.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NECK_SENSOR_C4_ONLY = 1u << NECK_NODE_C4,
    NECK_SENSOR_ALL_NODES = (1u << NECK_NODE_COUNT) - 1u
};

typedef uint64_t (*NeckSensorNowUs)(void *context);

typedef struct {
    NeckI2cBus bus;
    NeckBno055 node[NECK_NODE_COUNT];
    NeckSensorResult init_result[NECK_NODE_COUNT];
    uint32_t config_id, session_id;
    uint8_t selected_mask, ready_mask;
} NeckSensorAcquisition;

typedef struct {
    uint64_t read_start_us[NECK_NODE_COUNT];
    uint64_t read_end_us[NECK_NODE_COUNT];
    NeckSensorResult result[NECK_NODE_COUNT];
    uint8_t selected_mask, observed_mask;
} NeckSensorReadTrace;

/* Configure one C4 or three nodes. A failed node does not prevent other
 * channels from being probed; inspect ready_mask and init_result per node.
 * Only successful BNO055 register configuration is claimed here.
 */
NeckSensorResult neck_sensor_acquisition_init(NeckSensorAcquisition *acquisition,
    const NeckI2cBus *bus, uint8_t selected_mask, uint8_t fusion_mode,
    uint8_t external_clock, uint32_t config_id, uint32_t session_id);

/* Read each selected/ready channel once, sequentially. read_start/end are MCU
 * observation windows, NOT BNO055 sample times. The output source is HARDWARE,
 * every sample_time_known remains false and world_frame_id remains 0 until
 * independently verified timing/common-world registration is implemented.
 * Return OK if any selected node was observed; per-node faults are in trace.
 */
NeckSensorResult neck_sensor_acquisition_frame(NeckSensorAcquisition *acquisition,
    uint64_t sequence, NeckSensorNowUs now_us, void *clock_context,
    NeckAcquisitionFrame *frame, NeckSensorReadTrace *trace);

#ifdef __cplusplus
}
#endif
#endif
