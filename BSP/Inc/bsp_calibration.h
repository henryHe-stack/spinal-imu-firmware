#ifndef BSP_CALIBRATION_H
#define BSP_CALIBRATION_H
#include "app_status.h"
#include <stdint.h>
/* Two reserved 1KB Flash pages, alternating committed records. Writes only
 * by AcquisitionTask with streaming stopped; no automatic per-sample writes. */
AppStatus BspCalibration_Load(uint8_t offsets[3][22]);
AppStatus BspCalibration_Save(const uint8_t offsets[3][22]);
#endif
