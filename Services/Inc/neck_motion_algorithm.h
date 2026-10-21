#ifndef NECK_MOTION_ALGORITHM_H
#define NECK_MOTION_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>

#include "app_status.h"
#include "app_types.h"

typedef struct
{
    bool neutral_pose_valid;
    AppQuaternionRaw neutral_pose[APP_IMU_NODE_COUNT];
} NeckMotionAlgorithm;

typedef struct
{
    float c0_to_c4_deg[3];
    float c4_to_c7_deg[3];
    float c0_to_c7_deg[3];
    uint16_t valid_mask;
} NeckMotionResult;

AppStatus NeckMotionAlgorithm_Init(NeckMotionAlgorithm *self);
AppStatus NeckMotionAlgorithm_CaptureNeutral(NeckMotionAlgorithm *self,
                                             const AppSampleFrame *frame);
AppStatus NeckMotionAlgorithm_Process(NeckMotionAlgorithm *self,
                                      const AppSampleFrame *frame,
                                      NeckMotionResult *result);

#endif /* NECK_MOTION_ALGORITHM_H */
