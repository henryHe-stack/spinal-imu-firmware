#ifndef NECK_WIRE_HOST_H
#define NECK_WIRE_HOST_H
#include "neck_wire.h"
#ifdef _WIN32
#define NECK_HOST_EXPORT __declspec(dllexport)
#define NECK_HOST_CALL __cdecl
#else
#define NECK_HOST_EXPORT
#define NECK_HOST_CALL
#endif
/* ASCII integer-only canonical row; borrowed storage, callback must copy.
 * This bridge is PC-only. Do not link its allocation/file I/O into the MCU. */
typedef void (NECK_HOST_CALL *NeckHostCallback)(int result, int boundary,
    uint64_t arrival_us, const char *row, void *user);
int neck_wire_csv(const NeckWireMessage *message, char *output, size_t capacity);
NECK_HOST_EXPORT void *NECK_HOST_CALL neck_host_create(void);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_profile(void *handle,const double *rotations,
    const uint32_t *settings,int registered);
NECK_HOST_EXPORT void NECK_HOST_CALL neck_host_destroy(void *handle);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_feed(void *handle, const uint8_t *data,
    uint32_t length, uint64_t arrival_us, NeckHostCallback callback, void *user);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_gap(void *handle, uint64_t arrival_us,
    NeckHostCallback callback, void *user);
NECK_HOST_EXPORT uint32_t NECK_HOST_CALL neck_host_poll(void *handle, uint64_t now_us);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_stats(void *handle, uint64_t *output, uint32_t count);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_snapshot(void *handle, uint8_t *prefix,
    uint32_t capacity, uint32_t *prefix_length, uint8_t *session, uint32_t session_capacity,
    uint32_t *session_length, uint32_t *last_tx, uint64_t *last_emit);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_seed(void *handle, uint32_t last_tx, uint64_t last_emit);
NECK_HOST_EXPORT int NECK_HOST_CALL neck_host_command(uint32_t boot,uint32_t session,uint32_t config,
    uint64_t observed,uint32_t request,uint32_t opcode,const uint32_t *args,uint8_t *bytes,uint32_t capacity);
#endif
