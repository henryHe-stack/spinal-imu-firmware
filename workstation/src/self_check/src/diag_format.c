#include "diag_stats.h"
#include <stdarg.h>
#include <stdio.h>

static int append(char *out, size_t capacity, size_t *used, const char *format, ...)
{
    int n; va_list args;
    if (*used>=capacity) return 0;
    va_start(args,format); n=vsnprintf(out+*used,capacity-*used,format,args); va_end(args);
    if (n<0 || (size_t)n>=capacity-*used) return 0;
    *used+=(size_t)n; return 1;
}

size_t diag_format_header(uint8_t mask, uint8_t external_clock, uint8_t marker,
                          char *out, size_t cap)
{
    size_t used=0;
    if (!out || !cap) return 0;
    out[0]=0;
    if ((mask!=2u && mask!=7u) || external_clock>1u || marker>1u) return 0;
    if (!append(out,cap,&used,
        "IMU_SELF_CHECK version=%s source=HARDWARE mode=%s mask=%u period_us=%u "
        "tolerance_us=%u skew_limit_us=%u clock=HSI8 timer=TIM3 timer_hz=1000000 "
        "external_clock=%u marker=%s sample_time=UNKNOWN drdy=DISABLED\r\n",
        DIAG_VERSION,mask==2u?"C4":"THREE",mask,
        DIAG_PERIOD_US,DIAG_PERIOD_TOLERANCE_US,DIAG_NODE_SKEW_LIMIT_US,
        external_clock,marker?"PA8":"OFF")) { out[0]=0;return 0; }
    return used;
}

size_t diag_format_report(const DiagReport *r, char *out, size_t cap)
{
    const DiagStats *s;
    unsigned i;
    size_t used=0;
    if (!out || !cap) return 0;
    out[0]=0;
    if (!r) return 0;
    s=&r->stats;
    if (!append(out,cap,&used,
        "SUMMARY seq=%lu cycles=%lu periods=%lu period_avg_us=%lu period_min_us=%lu "
        "period_max_us=%lu period_outside=%lu cycle_max_us=%lu overrun=%lu "
        "skew_max_us=%lu clock_errors=%lu uart_errors=%lu report_drops=%lu "
        "heap_bytes=%lu acq_stack_words=%lu tx_stack_words=%lu state=%s\r\n",
        (unsigned long)s->cycles,(unsigned long)s->cycles,(unsigned long)s->periods,
        (unsigned long)(s->periods?s->period_sum_us/s->periods:0),
        (unsigned long)(s->periods?s->period_min_us:0),
        (unsigned long)s->period_max_us,(unsigned long)s->period_outside,
        (unsigned long)s->cycle_max_us,(unsigned long)s->acquisition_overruns,
        (unsigned long)s->skew_max_us,(unsigned long)s->clock_errors,
        (unsigned long)r->uart_errors,(unsigned long)r->report_drops,
        (unsigned long)r->free_heap_bytes,(unsigned long)r->acquire_stack_words,
        (unsigned long)r->report_stack_words,diag_status(s))) goto fail;
    for (i=0;i<DIAG_NODES;++i) if (s->mask & (1u<<i)) {
        const DiagNodeStats *n=&s->nodes[i];
        if (!append(out,cap,&used,
            "NODE seq=%lu id=%u attempts=%lu ok=%lu fail=%lu invalid=%lu cal_pending=%lu "
            "read_max_us=%lu result=%lu sys_cal=%u acc_cal=%u qnorm2_milli=%lu\r\n",
            (unsigned long)s->cycles,i,(unsigned long)n->attempts,
            (unsigned long)n->ok,(unsigned long)n->fail,(unsigned long)n->invalid,
            (unsigned long)n->calibration_pending,(unsigned long)n->read_max_us,
            (unsigned long)n->last_result,n->system_cal,n->accel_cal,
            (unsigned long)n->last_norm_milli)) goto fail;
    }
    if (!append(out,cap,&used,"END seq=%lu\r\n",(unsigned long)s->cycles)) goto fail;
    return used;
fail:
    out[0]=0; return 0;
}
