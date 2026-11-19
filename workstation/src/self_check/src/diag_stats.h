#ifndef DIAG_STATS_H
#define DIAG_STATS_H
#include <stddef.h>
#include <stdint.h>
#define DIAG_VERSION "0.1.0"
#define DIAG_NODES 3u
#define DIAG_PERIOD_US 10000u
#define DIAG_PERIOD_TOLERANCE_US 2000u
#define DIAG_NODE_SKEW_LIMIT_US 10000u

typedef struct {
    uint64_t start_us, end_us;
    double accel[3], quaternion[4];
    uint32_t result;
    uint8_t calibration_system, calibration_accel, observed;
} DiagNodeRead;
typedef struct {
    uint64_t start_us, end_us;
    DiagNodeRead nodes[DIAG_NODES];
} DiagCycle;
typedef struct {
    uint32_t attempts, ok, fail, invalid, calibration_pending;
    uint32_t read_max_us, last_result, last_norm_milli;
    uint8_t system_cal, accel_cal;
} DiagNodeStats;
typedef struct {
    uint64_t first_us, previous_start_us, last_end_us, period_sum_us;
    uint32_t cycles, periods, period_min_us, period_max_us;
    uint32_t period_outside, acquisition_overruns, cycle_max_us, skew_max_us;
    uint32_t clock_errors;
    uint8_t mask, started;
    DiagNodeStats nodes[DIAG_NODES];
} DiagStats;
typedef struct {
    DiagStats stats;
    uint32_t uart_errors, report_drops, free_heap_bytes;
    uint32_t acquire_stack_words, report_stack_words;
} DiagReport;

int diag_stats_init(DiagStats *s, uint8_t mask);
/* Reject malformed/reversed/overlapping observations before publishing counters. */
int diag_stats_add(DiagStats *s, const DiagCycle *cycle);
const char *diag_status(const DiagStats *s);
size_t diag_format_report(const DiagReport *report, char *out, size_t capacity);
size_t diag_format_header(uint8_t mask, uint8_t external_clock, uint8_t marker,
                          char *out, size_t capacity);
/* A timer snapshot is taken with interrupts masked: pending is sampled before after. */
uint64_t diag_timer_snapshot(uint64_t overflow_base, uint16_t before,
                             uint8_t pending, uint16_t after);
#endif
