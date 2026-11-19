#include "diag_stats.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static uint32_t bounded(uint64_t value)
{ return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value; }
static void increment(uint32_t *value)
{ if (*value != UINT32_MAX) ++*value; }
static void maximum(uint32_t *value, uint64_t candidate)
{ if (candidate > *value) *value = bounded(candidate); }

uint64_t diag_timer_snapshot(uint64_t base, uint16_t before,
                             uint8_t pending, uint16_t after)
{
    /* No overflow interrupt can run during the register snapshot. A pending
     * wrap uses the counter read AFTER UIF. A later wrap is deferred to next call. */
    return base + (pending ? 65536u + (uint64_t)after : (uint64_t)before);
}

int diag_stats_init(DiagStats *s, uint8_t mask)
{
    unsigned i;
    if (!s || (mask != 2u && mask != 7u)) return 0;
    memset(s, 0, sizeof(*s)); s->mask = mask; s->period_min_us = UINT32_MAX;
    for (i=0;i<DIAG_NODES;++i) {
        s->nodes[i].system_cal = 255u; s->nodes[i].accel_cal = 255u;
    }
    return 1;
}

static int observation_valid(const DiagNodeRead *n, double *norm2)
{
    unsigned i;
    *norm2=0;
    for (i=0;i<3u;++i) if (!isfinite(n->accel[i])) return 0;
    for (i=0;i<4u;++i) {
        if (!isfinite(n->quaternion[i])) return 0;
        *norm2 += n->quaternion[i]*n->quaternion[i];
    }
    /* Numerical plausibility only. Not magnetic/angle accuracy validation. */
    return isfinite(*norm2) && *norm2 >= 0.98 && *norm2 <= 1.02 &&
           n->calibration_system <= 3u && n->calibration_accel <= 3u;
}

int diag_stats_add(DiagStats *s, const DiagCycle *c)
{
    unsigned i;
    uint64_t previous_end, min_mid=UINT64_MAX, max_mid=0;
    if (!s || !c || (s->mask!=2u && s->mask!=7u)) return 0;
    if (s->cycles==UINT32_MAX) goto clock_error;
    if (c->end_us < c->start_us ||
        (s->started && (c->start_us <= s->previous_start_us || c->start_us < s->last_end_us)))
        goto clock_error;
    previous_end=c->start_us;
    for (i=0;i<DIAG_NODES;++i) if (s->mask & (1u<<i)) {
        const DiagNodeRead *n=&c->nodes[i];
        if (n->start_us < previous_end || n->end_us < n->start_us || n->end_us > c->end_us)
            goto clock_error;
        previous_end=n->end_us;
    }
    if (s->started) {
        uint64_t delta=c->start_us-s->previous_start_us;
        if (delta>UINT32_MAX || UINT64_MAX-s->period_sum_us < delta) goto clock_error;
        increment(&s->periods);s->period_sum_us+=delta;
        if (delta<s->period_min_us) s->period_min_us=(uint32_t)delta;
        maximum(&s->period_max_us,delta);
        if (delta < DIAG_PERIOD_US-DIAG_PERIOD_TOLERANCE_US ||
            delta > DIAG_PERIOD_US+DIAG_PERIOD_TOLERANCE_US) increment(&s->period_outside);
    } else { s->first_us=c->start_us; s->started=1u; }
    s->previous_start_us=c->start_us; s->last_end_us=c->end_us;
    increment(&s->cycles); maximum(&s->cycle_max_us,c->end_us-c->start_us);
    if (c->end_us-c->start_us >= DIAG_PERIOD_US) increment(&s->acquisition_overruns);
    for (i=0;i<DIAG_NODES;++i) if (s->mask & (1u<<i)) {
        const DiagNodeRead *n=&c->nodes[i];
        DiagNodeStats *o=&s->nodes[i];
        double norm2;
        uint64_t mid=n->start_us+(n->end_us-n->start_us)/2u;
        increment(&o->attempts); o->last_result=n->result;
        maximum(&o->read_max_us,n->end_us-n->start_us);
        if (n->result || !n->observed) {
            increment(&o->fail); o->system_cal=o->accel_cal=255u;
            o->last_norm_milli=0;
            continue;
        }
        increment(&o->ok);
        if (!observation_valid(n,&norm2)) {
            increment(&o->invalid); o->last_norm_milli=0;
        } else { o->last_norm_milli=(uint32_t)(norm2*1000.0+0.5); }
        o->system_cal=n->calibration_system; o->accel_cal=n->calibration_accel;
        if (o->system_cal!=3u || o->accel_cal!=3u) increment(&o->calibration_pending);
        if (mid<min_mid) min_mid=mid;
        if (mid>max_mid) max_mid=mid;
    }
    if (min_mid!=UINT64_MAX) maximum(&s->skew_max_us,max_mid-min_mid);
    return 1;
clock_error:
    increment(&s->clock_errors);
    return 0;
}

const char *diag_status(const DiagStats *s)
{
    unsigned i;
    if (!s) return "WAIT_DATA";
    if (s->clock_errors) return "CLOCK_ERROR";
    if (!s->started) return "WAIT_DATA";
    for (i=0;i<DIAG_NODES;++i) if (s->mask & (1u<<i))
        if (s->nodes[i].fail) return "READ_ERROR";
    for (i=0;i<DIAG_NODES;++i) if (s->mask & (1u<<i))
        if (s->nodes[i].invalid) return "DATA_ERROR";
    if (s->acquisition_overruns || s->period_outside || s->skew_max_us>DIAG_NODE_SKEW_LIMIT_US)
        return "TIMING_WARN";
    for (i=0;i<DIAG_NODES;++i) if ((s->mask & (1u<<i)) &&
        (s->nodes[i].system_cal!=3u || s->nodes[i].accel_cal!=3u)) return "CAL_PENDING";
    return "RUN_OK";
}
