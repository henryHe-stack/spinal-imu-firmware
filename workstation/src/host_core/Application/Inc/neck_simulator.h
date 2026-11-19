#ifndef NECK_SIMULATOR_H
#define NECK_SIMULATOR_H
#include "neck_app.h"
/* Ideal gravity/quaternion fixtures, not BNO055 fusion or physical dynamics.
 * A shared world ID is justified ONLY by this known simulation construction. */
const double *neck_simulator_mount(uint8_t node_id);
const double (*neck_simulator_mounts(void))[4];
void neck_simulator_frame(NeckAcquisitionFrame *frame, uint64_t sequence, uint64_t time_us);
#endif
