#ifndef NECK_APP_H
#define NECK_APP_H
#include <stdint.h>
#include "neck_processing.h"

#define NECK_INPUT_SIMULATION 1
#define NECK_INPUT_HARDWARE_DIAGNOSTIC 2
#define NECK_INPUT_HARDWARE_C4 3
#define NECK_INPUT_HARDWARE_THREE 4
#ifndef NECK_FULL_DEBUG_SNAPSHOTS
#define NECK_FULL_DEBUG_SNAPSHOTS 0 /* Keep pose/stats, omit redundant large mirrors. */
#endif
#ifndef NECK_INPUT_MODE
#define NECK_INPUT_MODE NECK_INPUT_SIMULATION
#endif
#if NECK_INPUT_MODE < 1 || NECK_INPUT_MODE > 4
#error "Unknown input mode."
#endif
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_C4 && NECK_HW_NODE_MASK == 7U
#error "C4 measurement supports only node mask 2."
#endif
#ifndef NECK_HW_NODE_MASK
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_THREE
#define NECK_HW_NODE_MASK 7U
#else
#define NECK_HW_NODE_MASK 2U /* C4 only; use 7U for C0/C4/C7. */
#endif
#endif
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_THREE && NECK_HW_NODE_MASK != 7U
#error "Three-node measurement requires mask 7."
#endif
#ifndef NECK_HW_EXTERNAL_CLOCK
#define NECK_HW_EXTERNAL_CLOCK 1U /* Circuit shows three 32.768 kHz crystals. */
#endif
#ifndef NECK_HW_FUSION_MODE
#define NECK_HW_FUSION_MODE 12U /* BNO055 NDOF; 8U is IMU relative mode. */
#endif
#if NECK_HW_NODE_MASK != 2U && NECK_HW_NODE_MASK != 7U
#error "D3 diagnostic node mask must be 2 (C4) or 7 (three nodes)."
#endif
#if NECK_HW_EXTERNAL_CLOCK > 1U || (NECK_HW_FUSION_MODE != 8U && NECK_HW_FUSION_MODE != 12U)
#error "Invalid D3 hardware diagnostic clock or fusion mode."
#endif
#ifndef NECK_UART_BAUD
#define NECK_UART_BAUD 115200U /* Engineering setting, not verified HC-05 setup. */
#endif
#define NECK_SAMPLE_PERIOD_MS 10U /* Target 100 Hz, NOT validated timing. */
#define NECK_TELEMETRY_DIVISOR 10U /* Target 10 Hz diagnostics. */
#define NECK_TELEMETRY_PERIOD_MS 50U /* C4 target 20 Hz; status/relative slower. */
#define NECK_SUPERVISION_PERIOD_MS 10U /* Bounded wait even with no input. */
#define NECK_SIM_SINE 0
#define NECK_SIM_SUSTAINED 1
#ifndef NECK_SIM_PROFILE
#define NECK_SIM_PROFILE NECK_SIM_SUSTAINED
#endif
#if NECK_SIM_PROFILE != NECK_SIM_SINE && NECK_SIM_PROFILE != NECK_SIM_SUSTAINED
#error "Unknown simulation profile."
#endif

typedef struct {
    int32_t raw_mdeg[3], corrected_mdeg[3];
    uint32_t valid_mask, flags;
    uint8_t status;
} NeckRelativeDiagnosticPair;
typedef struct {
    NeckRelativeDiagnosticPair pairs[NECK_RELATIVE_PAIR_COUNT];
    uint64_t sequence, frame_time_us, decision_time_us, valid_until_us;
    uint32_t reference_id, flags, quality_flags, invalidation_epoch;
    uint8_t source;
} NeckRelativeDiagnostic;

typedef struct {
    uint64_t sequence, frame_time_us, time_us;
    uint32_t reference_id, invalidation_epoch, events, posture_flags;
    uint8_t axis, previous_level, level, source;
    int8_t direction;
} NeckAlarmEvent;
typedef struct {
    uint32_t acquired_frames;
    uint32_t acquisition_queue_drops;
    uint32_t acquisition_overruns;
    uint32_t processed_frames;
    uint32_t algorithm_errors;
    uint32_t telemetry_queue_drops;
    uint32_t uart_errors;
    uint32_t fatal_error;
    uint32_t free_heap_bytes;
    uint32_t acquisition_stack_free_words;
    uint32_t algorithm_stack_free_words;
    uint32_t telemetry_stack_free_words;
    uint32_t zero_requests_started;
    uint32_t zero_cancel_commands;
    uint32_t zero_command_errors;
    uint32_t zero_node_commits;
    uint32_t filter_resets;
    uint32_t posture_errors;
    uint32_t alarm_event_queue_drops;
    uint32_t alarm_triggers;
    uint32_t alarm_clears;
    uint32_t alarm_level_changes;
    uint32_t relative_errors;
    uint32_t relative_zero_commits;
    uint32_t relative_zero_aborts;
    uint32_t relative_snapshots_overwritten;
    uint32_t quality_rejected_frames;
    uint32_t input_timeout_transitions;
    uint32_t gated_accel_nodes;
    uint32_t gated_orientation_nodes;
    uint32_t stale_snapshot_copies;
} NeckAppStats;
/* Debugger watch snapshots only, not an atomic inter-task publication API. */
extern volatile NeckFrame g_neck_latest_input;
extern volatile NeckPoseBatch g_neck_latest_pose;
#if NECK_FULL_DEBUG_SNAPSHOTS
extern volatile NeckPipelineOutput g_neck_latest_pipeline; /* C4 raw + filtered */
extern volatile NeckPostureOutput g_neck_latest_posture;
extern volatile NeckOrientationFrame g_neck_latest_orientation;
extern volatile NeckRelativeOutput g_neck_latest_relative;
#endif
extern volatile NeckAppStats g_neck_stats;
/* Debugger-only latch: write 1=request or 2=cancel; algorithm task clears it.
 * Production tasks must use the queue API below, not this watch variable. */
extern volatile uint32_t g_neck_zero_request;
/* Debugger-only simulation pause: nonzero stops generated frames, not RTOS.
 * Watch decision_time_us and timeout flags; not a sensor-control command. */
extern volatile uint32_t g_neck_simulation_pause;
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
#include "neck_sensor_acquisition.h"
/* Debugger watch only. Read windows are MCU observations, not sample times. */
extern volatile NeckSensorReadTrace g_neck_latest_sensor_trace;
/* Set 1 to request the same safe new-session transition as remote RESTART.
 * Acquisition stops, old queues/references retire, board hooks are reloaded.
 * Reconnect/remount still requires revalidating mount/time and a fresh zero. */
extern volatile uint32_t g_neck_hw_reprobe_request;
#endif
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_C4
extern volatile uint32_t g_neck_c4_mount_ready, g_neck_c4_time_rejected;
#endif
int neck_app_start(void);
/* Nonblocking task-context API, NOT ISR-safe. 1 means queued, not calibrated.
 * Three-node mode requests all nodes; C4 measurement requests only C4. */
int neck_app_request_zero(void);
int neck_app_cancel_zero(void);
#endif
