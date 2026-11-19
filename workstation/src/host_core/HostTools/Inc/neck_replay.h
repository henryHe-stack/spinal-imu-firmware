/* S08 HOST ONLY: C99 file replay, not an MCU task or wire protocol.
 * Sensor payload faults are replayed; malformed text is a hard error.
 * Optional simulation truth is presentation-only, NEVER algorithm input.
 * Host caller must use LC_NUMERIC=C (CLI and tests do this explicitly). */
#ifndef NECK_REPLAY_H
#define NECK_REPLAY_H
#include "neck_processing.h"
#include <stdio.h>
#define NECK_REPLAY_ID "s08-typed-csv-shared-c-replay-v1"
#define NECK_REPLAY_INPUT_HEADER "NECK_REPLAY_INPUT_V1"
#define NECK_REPLAY_CONFIG_HEADER "NECK_REPLAY_CONFIG_V1"
#define NECK_REPLAY_TRUTH_HEADER "NECK_REPLAY_TRUTH_V1"
enum { NR_OK=0, NR_ARGUMENT=1, NR_FORMAT=2, NR_CONFIG=3, NR_IO=4, NR_EXISTS=5, NR_LIMIT=6 };
enum { NR_FRAME=1, NR_POLL=2, NR_ZERO=3, NR_CANCEL=4, NR_RESET=5 };
typedef struct { unsigned long line; char message[192]; } NeckReplayError;
typedef struct {
    uint32_t config_id, world_frame_id;
    uint64_t start_wall_us;
    double mounts[3][4];
    NeckProcessingParameters parameters;
} NeckReplayConfig;
typedef struct {
    NeckAcquisitionFrame frame;
    uint64_t now_us;
    uint8_t kind, origin;
} NeckReplayEvent;
typedef struct { uint64_t time_us; double euler_deg[3][3]; uint8_t source; } NeckReplayTruth;
typedef struct {
    uint64_t input_frames, poll_calls, outputs, poll_outputs, commands;
    uint64_t rejected_frames, triggers, clears, level_changes, timeout_edges;
    uint64_t node_rows, relative_rows, event_rows;
} NeckReplayStats;
void neck_replay_default_config(NeckReplayConfig *config);
int neck_replay_config_read(const char *path, NeckReplayConfig *config,
    uint64_t *byte_digest, NeckReplayError *error);
int neck_replay_config_write(FILE *file, const NeckReplayConfig *config);
int neck_replay_event_parse(const char *line, NeckReplayEvent *event, NeckReplayError *error);
int neck_replay_event_write(FILE *file, const NeckReplayEvent *event);
int neck_replay_truth_parse(const char *line, NeckReplayTruth *truth, NeckReplayError *error);
int neck_replay_truth_write(FILE *file, const NeckReplayTruth *truth);
/* Independent analytical ideal-gravity proxy from declared body Euler.
 * This is NOT physical truth or a reference for fusion accuracy. */
void neck_replay_ideal_proxy(double pitch, double roll, double output[2], uint8_t *mask);
/* New directory required; never overwrite a run. Errors retain partial output
 * with FAILED.txt and without COMPLETE.txt/index.html. No shell execution. */
int neck_replay_run(const char *input_path, const char *config_path,
    const char *truth_path, const char *new_directory, NeckReplayStats *stats, NeckReplayError *error);
/* Deterministic C fixtures: baseline, silence_faults, timing_faults, motion.
 * Writes inputs/config/truth ONLY; replay is a separate explicit operation. */
int neck_replay_generate(const char *scenario, const char *new_directory, NeckReplayError *error);
#endif
