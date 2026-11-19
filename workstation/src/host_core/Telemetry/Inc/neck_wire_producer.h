/* MCU-side D2 projection of shared algorithm results into the D0 wire model.
 * Pure C99; no HAL/FreeRTOS, allocations, or UART ownership. */
#ifndef NECK_WIRE_PRODUCER_H
#define NECK_WIRE_PRODUCER_H
#include "neck_wire.h"
#include "neck_processing.h"
#include "neck_app.h"

typedef struct {
    uint32_t session_id,config_id;
    NeckWireC4 c4;
    NeckWireRelative relative;
    NeckWireStatus status;
} NeckWireSnapshot;

void neck_wire_producer_session(NeckWireSession *session,
    const NeckProcessingParameters *parameters);
/* D3 bring-up only: no registered world, relative outputs or valid angles. */
int neck_wire_producer_d3_diagnostic_session(NeckWireSession *session,
    const NeckProcessingParameters *parameters, uint8_t node_mask);
/* C4 tilt capability only; actual validity still comes from the quality gate. */
int neck_wire_producer_c4_session(NeckWireSession *session,
    const NeckProcessingParameters *parameters);
void neck_wire_producer_snapshot(NeckWireSnapshot *snapshot,
    const NeckProcessingOutput *output, const NeckAcquisitionFrame *last_input,
    uint8_t input_seen);
void neck_wire_producer_event(NeckWireEvent *event,
    const NeckAlarmEvent *source, uint64_t event_id);
/* estimated_finish_us includes UART serialization; invalidate a copied wire
 * message before encoding if delivery would outlive its shared-C deadline. */
int neck_wire_producer_expire(NeckWireMessage *message, uint64_t estimated_finish_us,
    uint32_t current_epoch);
#endif
