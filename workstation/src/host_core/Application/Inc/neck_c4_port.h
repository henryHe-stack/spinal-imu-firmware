#ifndef NECK_C4_PORT_H
#define NECK_C4_PORT_H
#include "neck_sensor_acquisition.h"

/* Evidence supplied by a board-specific, independently validated timing
 * implementation. Same epoch/units as the acquisition MCU clock. Binding to
 * this exact read prevents accidentally reusing another frame's evidence.
 * This is a trust boundary, NOT a timestamp estimator or proof of timing. */
typedef struct {
    uint32_t config_id, session_id;
    uint64_t sequence, read_start_us, read_end_us, sample_time_us;
} NeckC4TimeEvidence;

/* Clears existing C4 timestamps first; approves only acceleration, never quat.
 * Caller must additionally validate mounting and acquisition clock semantics. */
int neck_c4_apply_time(NeckAcquisitionFrame *frame, const NeckSensorReadTrace *trace,
    const NeckC4TimeEvidence *evidence);

/* Board port hooks. Default weak definitions return 0 (unavailable).
 * Override only after measuring mounting and sample-time correspondence.
 * mount: sensor-to-body normalized wxyz, fixed for this config/session.
 * time: actual data-generation timestamp, NOT I2C read completion or task tick.
 * Called in task context. Repeated sample timestamps are rejected downstream.
 * The default 1-ms MCU time base must be accounted for in hardware validation. */
int neck_c4_board_mount(double mount[4]);
int neck_c4_board_time(const NeckAcquisitionFrame *frame,
    const NeckSensorReadTrace *trace, NeckC4TimeEvidence *evidence);
#endif
