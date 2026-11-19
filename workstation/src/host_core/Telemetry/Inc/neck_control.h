#ifndef NECK_CONTROL_H
#define NECK_CONTROL_H
#include "neck_wire.h"
enum { NECK_CMD_QUERY=1, NECK_CMD_ZERO=2, NECK_CMD_CANCEL=3, NECK_CMD_STOP=4,
    NECK_CMD_RESTART=5, NECK_CMD_SET=6, NECK_CMD_SAVE=7, NECK_CMD_EVENT_ACK=8 };
enum { NECK_CMD_OK=0, NECK_CMD_IDENTITY=1, NECK_CMD_EXPIRED=2,
    NECK_CMD_REPLAY=3, NECK_CMD_CONFLICT=4, NECK_CMD_ARGUMENT=5,
    NECK_CMD_UNSUPPORTED=6, NECK_CMD_NOT_READY=7, NECK_CMD_STORAGE=8 };
typedef uint32_t (*NeckCommandHandler)(void *user, const NeckWireControl *command,
    uint32_t reply_args[4]);
typedef struct {
    uint32_t boot, session, config, high_water;
    NeckWireControl last_command, last_reply;
} NeckControl;
void neck_control_init(NeckControl *control, uint32_t boot, uint32_t session, uint32_t config);
/* One algorithm-task owner. Identical retry => cached reply; same id with
 * changed payload => conflict. Lower ids never re-execute. QUERY and EVENT_ACK
 * are idempotent and do not consume command high-water. Max age 2 seconds. */
int neck_control_dispatch(NeckControl *control, const NeckWireMessage *input,
    uint64_t now, NeckCommandHandler handler, void *user, NeckWireControl *reply);
#endif
