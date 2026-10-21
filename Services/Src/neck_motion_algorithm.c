#include "neck_motion_algorithm.h"

#include <string.h>

AppStatus NeckMotionAlgorithm_Init(NeckMotionAlgorithm *self)
{
    if (self == NULL)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(self, 0, sizeof(*self));
    return APP_STATUS_OK;
}

AppStatus NeckMotionAlgorithm_CaptureNeutral(NeckMotionAlgorithm *self,
                                             const AppSampleFrame *frame)
{
    if ((self == NULL) || (frame == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    /*
     * TODO(ALGORITHM owner): validate all nodes, normalize quaternions, reject
     * motion during capture and store a stable neutral-pose estimate.
     */
    self->neutral_pose_valid = false;
    return APP_STATUS_NOT_IMPLEMENTED;
}

AppStatus NeckMotionAlgorithm_Process(NeckMotionAlgorithm *self,
                                      const AppSampleFrame *frame,
                                      NeckMotionResult *result)
{
    if ((self == NULL) || (frame == NULL) || (result == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));

    /*
     * TODO(ALGORITHM owner): time-align samples, compute neutral-corrected
     * relative rotations C0-C4/C4-C7/C0-C7, then apply the documented Euler
     * convention. Do not claim per-vertebra C1-C7 angles from only three IMUs.
     */
    return APP_STATUS_NOT_IMPLEMENTED;
}
