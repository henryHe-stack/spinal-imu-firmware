#ifndef NECK_REPLAY_INTERNAL_H
#define NECK_REPLAY_INTERNAL_H
#include "neck_replay.h"
#define NR_LINE_MAX 4096u
#define NR_POINT_MAX 200000u
#define NR_ROW_MAX 1000000u
#define NR_FNV_OFFSET UINT64_C(14695981039346656037)
typedef struct {
    uint64_t time_us;
    double raw[2], filtered[2], corrected[2], relative[4][3];
    uint64_t timers[5];
    uint32_t raw_mask, filtered_mask, relative_masks[4], events, qflags, epoch;
    uint8_t instant[2], alarm[2], valid[2], accel_mask, quat_mask, source;
} NRPoint;
typedef struct { NRPoint *items; size_t count, capacity; } NRPoints;
typedef struct { NeckReplayTruth *items; size_t count, capacity; uint64_t digest; } NRTruths;
int nr_error(NeckReplayError *error, int code, const char *message);
int nr_read_line(FILE *file, char line[NR_LINE_MAX], unsigned long *number,
    uint64_t *digest, NeckReplayError *error); /* 1=line, 0=EOF, -1=error */
int nr_header(FILE *file, const char *expected, unsigned long *line,
    uint64_t *digest, NeckReplayError *error);
int nr_new_directory(const char *path, NeckReplayError *error);
int nr_path(char output[1024], const char *directory, const char *name, NeckReplayError *error);
FILE *nr_new_file(const char *directory, const char *name, NeckReplayError *error);
void nr_double(FILE *file, double value);
void nr_optional(FILE *file, double value, int valid);
int nr_close(FILE *file, NeckReplayError *error);
int nr_truths_read(const char *path, NRTruths *truth, NeckReplayError *error);
const NeckReplayTruth *nr_truth_at(const NRTruths *truth, uint64_t time, uint8_t source);
int nr_plots(const char *directory, const NRPoints *points, const NRTruths *truth,
    const NeckReplayConfig *config, NeckReplayError *error);
int nr_index(const char *directory, const NeckReplayStats *stats, int has_truth, NeckReplayError *error);
#endif
