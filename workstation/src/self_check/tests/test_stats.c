#include "diag_stats.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static DiagCycle cycle(uint64_t start, uint8_t mask)
{
    DiagCycle c; unsigned i; uint64_t at=start;
    memset(&c,0,sizeof(c));c.start_us=start;
    for (i=0;i<3;++i) if (mask & (1u<<i)) {
        c.nodes[i].start_us=at; c.nodes[i].end_us=at+1000;at+=1200;
        c.nodes[i].quaternion[0]=1;c.nodes[i].accel[2]=9.81;
        c.nodes[i].calibration_system=c.nodes[i].calibration_accel=3;
        c.nodes[i].observed=1;
    }
    c.end_us=at;return c;
}
static void healthy_and_format(void)
{
    DiagReport r; DiagCycle c; unsigned i; char output[1500],small[20];
    memset(&r,0,sizeof(r));CHECK(diag_stats_init(&r.stats,7));
    CHECK(strcmp(diag_status(&r.stats),"WAIT_DATA")==0);
    for(i=0;i<6000;++i) { c=cycle((uint64_t)i*10000,7);CHECK(diag_stats_add(&r.stats,&c)); }
    CHECK(r.stats.cycles==6000);CHECK(r.stats.periods==5999);
    CHECK(r.stats.period_sum_us/r.stats.periods==10000);
    CHECK(r.stats.cycle_max_us==3600);CHECK(r.stats.skew_max_us==2400);
    CHECK(strcmp(diag_status(&r.stats),"RUN_OK")==0);
    CHECK(diag_format_report(&r,output,sizeof(output))>0);
    CHECK(strstr(output,"END seq=6000\r\n")!=NULL);
    CHECK(strstr(output,"NODE seq=6000 id=2 attempts=6000 ok=6000")!=NULL);
    CHECK(diag_format_report(&r,small,sizeof(small))==0);CHECK(small[0]==0);
    CHECK(diag_format_header(7,1,0,output,sizeof(output))>0);
    CHECK(strstr(output,"mode=THREE mask=7")!=NULL);
    CHECK(strstr(output,"sample_time=UNKNOWN drdy=DISABLED")!=NULL);
    CHECK(diag_format_header(2,0,1,output,sizeof(output))>0);
    CHECK(strstr(output,"mode=C4 mask=2")!=NULL);
    CHECK(diag_format_header(1,0,0,output,sizeof(output))==0);
    CHECK(diag_format_header(7,2,0,output,sizeof(output))==0);
    CHECK(diag_format_header(7,1,0,small,sizeof(small))==0 && !small[0]);
}
static void fault_cases(void)
{
    DiagStats s; DiagCycle c;unsigned i;
    CHECK(!diag_stats_init(NULL,7));CHECK(!diag_stats_init(&s,1));
    CHECK(diag_stats_init(&s,2));c=cycle(100,2);CHECK(diag_stats_add(&s,&c));
    CHECK(s.nodes[0].attempts==0 && s.nodes[1].ok==1);
    c=cycle(10100,2);c.nodes[1].result=2;c.nodes[1].observed=0;
    CHECK(diag_stats_add(&s,&c));CHECK(strcmp(diag_status(&s),"READ_ERROR")==0);
    CHECK(s.nodes[1].last_norm_milli==0 && s.nodes[1].system_cal==255);
    CHECK(diag_stats_init(&s,7));c=cycle(0,7);c.nodes[0].quaternion[0]=0;
    CHECK(diag_stats_add(&s,&c));CHECK(s.nodes[0].invalid==1);
    CHECK(strcmp(diag_status(&s),"DATA_ERROR")==0);
    CHECK(diag_stats_init(&s,7));c=cycle(0,7);c.nodes[2].calibration_system=1;
    CHECK(diag_stats_add(&s,&c));CHECK(strcmp(diag_status(&s),"CAL_PENDING")==0);
    c=cycle(10000,7);CHECK(diag_stats_add(&s,&c));CHECK(strcmp(diag_status(&s),"RUN_OK")==0);
    CHECK(s.nodes[2].calibration_pending==1);
    for(i=0;i<3;++i) {
        CHECK(diag_stats_init(&s,7));c=cycle(0,7);c.nodes[i].accel[0]=NAN;
        CHECK(diag_stats_add(&s,&c));CHECK(s.nodes[i].invalid==1);
    }
    CHECK(diag_stats_init(&s,7));c=cycle(0,7);CHECK(diag_stats_add(&s,&c));
    c=cycle(13000,7);CHECK(diag_stats_add(&s,&c));
    CHECK(s.period_outside==1);CHECK(strcmp(diag_status(&s),"TIMING_WARN")==0);
    CHECK(diag_stats_init(&s,7));c=cycle(0,7);c.end_us=10000;
    CHECK(diag_stats_add(&s,&c));CHECK(s.acquisition_overruns==1);
    c=cycle(5000,7);CHECK(!diag_stats_add(&s,&c));CHECK(s.clock_errors==1);
    CHECK(diag_stats_init(&s,7));c=cycle(0,7);c.nodes[1].start_us=500;
    CHECK(!diag_stats_add(&s,&c));CHECK(s.cycles==0);
    CHECK(strcmp(diag_status(&s),"CLOCK_ERROR")==0);
    CHECK(diag_stats_init(&s,7));c=cycle(UINT64_C(4294967290),7);
    CHECK(diag_stats_add(&s,&c));c=cycle(UINT64_C(4294977290),7);
    CHECK(diag_stats_add(&s,&c));CHECK(s.period_sum_us==10000);
    c=cycle(UINT64_C(8589944590),7);CHECK(!diag_stats_add(&s,&c));
    CHECK(s.cycles==2 && s.periods==1 && s.clock_errors==1);
    CHECK(diag_stats_init(&s,7));s.cycles=UINT32_MAX;
    c=cycle(0,7);CHECK(!diag_stats_add(&s,&c));CHECK(s.clock_errors==1);
}
static void timer_cases(void)
{
    CHECK(diag_timer_snapshot(0,65535,0,0)==65535);
    CHECK(diag_timer_snapshot(0,65535,1,1)==65537);
    CHECK(diag_timer_snapshot(65536,4,0,5)==65540);
    CHECK(diag_timer_snapshot(65536,65534,1,3)==131075);
}
int main(void)
{
    healthy_and_format();fault_cases();timer_cases();
    printf("PASS selfcheck statistics and timer snapshot: %u assertions\n",checks);
    return 0;
}
