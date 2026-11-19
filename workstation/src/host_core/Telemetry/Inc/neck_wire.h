/* D0 wire protocol v1. C99, integer-only, no HAL/RTOS/heap or struct dumping.
 * Explicit little endian fields; uint64 times are microseconds. Not F412 v1.
 * Codec/receiver are single-owner objects; callbacks must not re-enter them. */
#ifndef NECK_WIRE_H
#define NECK_WIRE_H
#include <stddef.h>
#include <stdint.h>
#include <limits.h>
#ifdef __cplusplus
extern "C" {
#endif

#define NECK_WIRE_VERSION 1u
#define NECK_WIRE_HEADER_SIZE 32u
#define NECK_WIRE_CRC_SIZE 2u
#define NECK_WIRE_MAX_PAYLOAD 256u
#define NECK_WIRE_MAX_FRAME 290u
#define NECK_WIRE_PROFILE_S09 1u
#define NECK_WIRE_INVALID_ANGLE INT32_MIN
#define NECK_WIRE_ANGLE_LIMIT 360000L /* mdeg, rounded by producer */
#define NECK_WIRE_UNKNOWN_CALIBRATION 255u
#define NECK_WIRE_SESSION_SIZE 72u
#define NECK_WIRE_C4_SIZE 136u
#define NECK_WIRE_RELATIVE_SIZE 212u
#define NECK_WIRE_EVENT_SIZE 56u
#define NECK_WIRE_STATUS_SIZE 168u
#define NECK_WIRE_RAW_FRAME_SIZE 120u
enum { NECK_WIRE_SESSION=1, NECK_WIRE_C4=2, NECK_WIRE_RELATIVE=3,
       NECK_WIRE_EVENT=4, NECK_WIRE_STATUS=5, NECK_WIRE_RAW_FRAME=6,
       NECK_WIRE_COMMAND=16, NECK_WIRE_REPLY=17 };
enum { NECK_WIRE_SIM=0, NECK_WIRE_HARDWARE=1 };
enum { NECK_WIRE_SINGLE_C4=1, NECK_WIRE_THREE_NODES=3 };
enum { NECK_WIRE_CAP_C4=1u, NECK_WIRE_CAP_RELATIVE=2u,
       NECK_WIRE_CAP_STATUS=4u, NECK_WIRE_CAP_EVENTS=8u,
       NECK_WIRE_CAP_CONTROL=16u, NECK_WIRE_CAP_EVENT_ACK=32u,
       NECK_WIRE_CAP_RAW_FRAME=64u };
/* COMMAND header identity targets the device; emit_time_us is the latest
 * device time observed by the host, never the host's unrelated clock. */
typedef struct {
    uint32_t request_id, opcode, result, args[4];
} NeckWireControl;
/* These bits describe which time values are known, NOT synchronization proof. */
enum { NECK_WIRE_FRAME_TIME=1u, NECK_WIRE_DECISION_TIME=2u,
       NECK_WIRE_DEADLINE_TIME=4u };
enum { NECK_WIRE_RAW_FLEX=1u, NECK_WIRE_RAW_LATERAL=2u,
       NECK_WIRE_FILTERED_FLEX=4u, NECK_WIRE_FILTERED_LATERAL=8u,
       NECK_WIRE_CORRECTED_FLEX=16u, NECK_WIRE_CORRECTED_LATERAL=32u };
enum { NECK_WIRE_PAIR_RAW=1u, NECK_WIRE_PAIR_CORRECTED=2u,
       NECK_WIRE_PAIR_SKEW=4u };
typedef enum { NECK_WIRE_OK=0, NECK_WIRE_BAD_ARGUMENT, NECK_WIRE_NO_SPACE,
    NECK_WIRE_BAD_LENGTH, NECK_WIRE_BAD_CRC, NECK_WIRE_BAD_VERSION,
    NECK_WIRE_BAD_TYPE, NECK_WIRE_BAD_PAYLOAD } NeckWireResult;

typedef struct {
    uint32_t tx_sequence, boot_id, session_id, config_id;
    uint64_t emit_time_us; /* monotonic device clock, always known */
    uint8_t type;
} NeckWireHeader;
typedef struct {
    uint32_t profile_id, firmware_id, world_frame_id;
    uint8_t source, mode, node_mask, reference_origin;
    uint32_t sample_period_us, quality_timeout_us, cutoff_millihz;
    uint64_t continuous_duration_us;
    int32_t thresholds_mdeg[4]; /* flex warn/danger, lateral warn/danger */
    uint32_t max_node_skew_us, c4_period_us, relative_period_us, status_period_us;
    uint32_t capabilities;
} NeckWireSession;
typedef struct {
    uint64_t warning_elapsed_us, danger_elapsed_us, extension_elapsed_us;
    uint8_t valid, instant_level, alarm_level;
    int8_t direction;
} NeckWireAxis;
typedef struct {
    uint64_t input_sequence, frame_time_us, decision_time_us, valid_until_us;
    uint32_t invalidation_epoch, reference_id, quality_flags, filter_flags, posture_flags;
    uint16_t valid_mask;
    uint8_t time_flags, zero_status;
    int32_t angles_mdeg[6]; /* raw signed proxy x2, LP absolute x2, corrected x2 */
    NeckWireAxis axes[2]; /* flexion, lateral; all three elapsed clocks preserved */
} NeckWireC4;
typedef struct {
    uint32_t flags;
    int32_t raw_mdeg[3], corrected_mdeg[3]; /* S06 flexion, lateral, axial */
    uint64_t sample_skew_us;
    uint8_t valid_mask, status;
} NeckWirePair;
typedef struct {
    uint64_t input_sequence, frame_time_us, decision_time_us, valid_until_us;
    uint32_t invalidation_epoch, reference_id, quality_flags, world_frame_id;
    uint8_t time_flags;
    NeckWirePair pairs[4]; /* C7->C0, C4->C0, C7->C4, WORLD->C7 */
} NeckWireRelative;
typedef struct {
    uint64_t event_id, input_sequence, frame_time_us, event_time_us;
    uint32_t reference_id, invalidation_epoch, events, posture_flags;
    uint8_t axis, previous_level, level, time_flags;
    int8_t direction;
} NeckWireEvent;
typedef struct {
    uint64_t accel_sample_time_us, orientation_sample_time_us;
    uint32_t quality_flags, filter_flags, reference_id;
    uint8_t status, accel_calibration, system_calibration, time_flags, zero_status;
} NeckWireNode;
typedef struct {
    uint64_t input_sequence, frame_time_us, decision_time_us;
    uint32_t invalidation_epoch, quality_flags, pending_reference_id;
    uint32_t input_drops, telemetry_drops, event_drops, uart_errors;
    uint8_t accel_usable_mask, orientation_usable_mask, quality_state, frame_accepted;
    uint8_t zero_request_state;
    NeckWireNode nodes[3];
} NeckWireStatus;
typedef struct {
    uint32_t timestamp_start_us, timestamp_end_us, timestamp_mid_us;
    int16_t quat_wxyz[4], accel_xyz[3], gyro_xyz[3];
    int8_t temperature_c;
    uint8_t calibration_status;
    uint16_t flags;
} NeckWireRawNode;
typedef struct {
    uint32_t sequence, sample_period_us;
    uint8_t node_mask, flags;
    uint16_t reserved;
    NeckWireRawNode nodes[3];
} NeckWireRawFrame;
typedef struct {
    NeckWireHeader header;
    union { NeckWireSession session; NeckWireC4 c4; NeckWireRelative relative;
            NeckWireEvent event; NeckWireStatus status; NeckWireRawFrame raw;
            NeckWireControl control; } body;
} NeckWireMessage;

uint16_t neck_wire_crc16(const uint8_t *data, size_t length);
size_t neck_wire_payload_size(uint8_t type);
/* Cleared validity bits force invalid sentinels/unknown times on the wire.
 * Invalid axis state must already be UNKNOWN with zero clocks. On error,
 * written=0 and the output buffer is untouched. */
NeckWireResult neck_wire_encode(const NeckWireMessage *message, uint8_t *output,
    size_t capacity, size_t *written);
/* Exact single frame only. Failure clears output; never publishes partial data. */
NeckWireResult neck_wire_decode(const uint8_t *frame, size_t length, NeckWireMessage *output);

typedef struct {
    uint64_t frames, crc_errors, length_errors, version_errors, type_errors;
    uint64_t payload_errors, discarded_bytes, partial_gaps;
} NeckWireParserStats;
typedef struct {
    uint8_t buffer[NECK_WIRE_MAX_FRAME];
    size_t used;
    NeckWireParserStats stats;
} NeckWireParser;
/* Message is borrowed only for the callback; copy it if it must outlive it.
 * A NULL callback permits count-only operation. */
typedef void (*NeckWireCallback)(const NeckWireMessage *message, void *user);
void neck_wire_parser_init(NeckWireParser *parser);
size_t neck_wire_parser_feed(NeckWireParser *parser, const uint8_t *bytes,
    size_t length, NeckWireCallback callback, void *user);
/* Caller declares an inter-byte idle gap. Reject unfinished outer candidate,
 * recover any complete following frames, then discard incomplete leftovers.
 * Not an implicit device quality timeout and not a UART idle interrupt API. */
size_t neck_wire_parser_gap(NeckWireParser *parser, NeckWireCallback callback, void *user);

enum { NECK_WIRE_HOST_LINK_STALE=1u, NECK_WIRE_HOST_C4_STALE=2u,
       NECK_WIRE_HOST_RELATIVE_STALE=4u, NECK_WIRE_HOST_CLOCK_FAULT=8u };
typedef enum { NECK_WIRE_RX_DATA=0, NECK_WIRE_RX_SESSION,
    NECK_WIRE_RX_UNREGISTERED, NECK_WIRE_RX_BOUNDARY_REQUIRED,
    NECK_WIRE_RX_DUPLICATE, NECK_WIRE_RX_OUT_OF_ORDER,
    NECK_WIRE_RX_CLOCK_FAULT, NECK_WIRE_RX_BAD_MESSAGE,
    NECK_WIRE_RX_UNSUPPORTED } NeckWireReceiveResult;
typedef struct {
    NeckWireSession session;
    uint32_t boot_id, session_id, config_id, last_tx_sequence;
    uint64_t last_emit_us, last_arrival_us, c4_arrival_us, relative_arrival_us;
    uint64_t link_timeout_us, pose_timeout_us, tx_gaps;
    uint8_t bound, c4_seen, relative_seen;
} NeckWireReceiver;
/* Host monotonic clock is separate from device time. Nonzero limits required.
 * Disconnect clears binding and pose freshness, retaining timeout settings.
 * New identity requires an explicit boundary/disconnect then SESSION. */
NeckWireResult neck_wire_receiver_init(NeckWireReceiver *receiver,
    uint64_t link_timeout_us, uint64_t pose_timeout_us);
void neck_wire_receiver_disconnect(NeckWireReceiver *receiver);
NeckWireReceiveResult neck_wire_receiver_accept(NeckWireReceiver *receiver,
    const NeckWireMessage *decoded_message, uint64_t host_arrival_us);
uint32_t neck_wire_receiver_poll(const NeckWireReceiver *receiver, uint64_t host_now_us);
#ifdef __cplusplus
}
#endif
#endif
