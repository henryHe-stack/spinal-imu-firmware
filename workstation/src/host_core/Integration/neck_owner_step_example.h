/* S09 optional integration example. NOT linked by the existing MCU build.
 * One algorithm owner only. No HAL/RTOS, allocation or hidden state. */
#ifndef NECK_OWNER_STEP_EXAMPLE_H
#define NECK_OWNER_STEP_EXAMPLE_H
#include "neck_processing.h"
enum { NECK_OWNER_POLL=1, NECK_OWNER_FRAME=2 };
/* Synchronous callback: copy anything retained before returning. The same
 * scratch is reused for the next output. Never re-enter/mutate the context,
 * wait on I/O, or keep a borrowed pointer for another task. */
typedef void (*NeckOwnerPublish)(const NeckProcessingOutput *output,
    uint8_t kind, NeckResult result, void *user);
/* Call AFTER bounded queue wait, using a newly read monotonic now_us.
 * frame==NULL means actual silence, not a fabricated missing-node packet.
 * Can publish TWO outputs: expired-old-state first, resumed frame second.
 * No auto-zero, commands, clock conversion, queues or configuration update.
 * Returns the frame result (quality rejection is still published), or OK for
 * poll-only. published_count is optional and is reset even on bad arguments.
 * Caller provides scratch: do not add another 1616-B output on the task stack. */
NeckResult neck_owner_step_example(NeckProcessingContext *context,
    const NeckAcquisitionFrame *frame, uint64_t now_us, NeckProcessingOutput *scratch,
    NeckOwnerPublish publish, void *user, unsigned *published_count);
#endif
