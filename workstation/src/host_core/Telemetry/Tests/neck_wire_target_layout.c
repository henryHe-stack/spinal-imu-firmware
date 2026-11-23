#include "neck_wire.h"
/* Read these constants in the ARM object, not the native host ABI. */
const uint32_t neck_wire_target_sizes[3]={
    (uint32_t)sizeof(NeckWireMessage),
    (uint32_t)sizeof(NeckWireParser),
    (uint32_t)sizeof(NeckWireReceiver)
};
typedef char frame_bound[(NECK_WIRE_MAX_FRAME==NECK_WIRE_HEADER_SIZE+NECK_WIRE_MAX_PAYLOAD+2)?1:-1];
typedef char relative_bound[(NECK_WIRE_RELATIVE_SIZE<=NECK_WIRE_MAX_PAYLOAD)?1:-1];
