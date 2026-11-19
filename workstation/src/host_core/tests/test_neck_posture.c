/* Native C fixtures and independent held-step oracle. No Python dependency.
 * Direct detector inputs are constructed filtered results, not real samples.
 */
#include "neck_posture.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, groups;
static const double pi=3.14159265358979323846264338327950288;
static const double gravity=9.80665;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
static void close_number(double a, double b)
{ CHECK(isfinite(a)); CHECK(fabs(a-b)<1e-10); }
static void fresh(NeckPostureContext *c)
{
    NeckPostureParameters p;
    neck_posture_default_parameters(&p);
    CHECK(neck_posture_init(c,1,NECK_NODE_C4,&p)==NECK_OK);
}
static NeckPipelineOutput input(uint64_t sequence, uint64_t time_us, double pitch, double side)
{
    NeckPipelineOutput x;
    NeckTiltEstimate *e=&x.filtered;
    memset(&x,0,sizeof(x));
    e->config_id=1; e->session_id=1; e->reference_id=1;
    e->sequence=sequence; e->frame_time_us=e->sample_time_us=time_us;
    e->node_id=NECK_NODE_C4; e->source=NECK_SOURCE_SIMULATION;
    e->status=e->pitch_status=e->lateral_status=NECK_COMPUTED;
    e->valid_mask=NECK_VALID_FLEXION|NECK_VALID_RIGHT_LATERAL|NECK_VALID_REFERENCE|
        NECK_VALID_SAMPLE_TIME|NECK_VALID_SAMPLE_AGE;
    e->flags=NECK_FLAG_QUALITY_NOT_EVALUATED;
    e->flexion_deg=pitch; e->right_lateral_deg=side;
    x.filter_flags=NECK_FILTER_UPDATED; x.alpha_valid=1;
    return x;
}
static NeckPostureOutput step(NeckPostureContext *c, NeckPipelineOutput *x)
{
    NeckPostureOutput o;
    memset(&o,0xa5,sizeof(o));
    CHECK(neck_posture_process(c,x,&o)==NECK_OK);
    return o;
}
static NeckPostureOutput tick(NeckPostureContext *c, uint64_t n, double pitch, double side)
{
    NeckPipelineOutput x=input(n,n*10000,pitch,side);
    return step(c,&x);
}
static void test_parameters(void)
{
    NeckPostureParameters p;
    NeckPostureContext c;
    NeckPostureOutput o;
    NeckPipelineOutput x=input(0,0,20,0);
    ++groups; neck_posture_default_parameters(&p);
    CHECK(p.continuous_duration_us==10000000 && p.max_gap_us==30000);
    CHECK(neck_posture_init(NULL,1,1,&p)==NECK_BAD_ARGUMENT);
    CHECK(neck_posture_init(&c,0,1,&p)==NECK_BAD_CONFIG);
    CHECK(neck_posture_init(&c,1,3,&p)==NECK_BAD_CONFIG);
    CHECK(neck_posture_init(&c,1,1,NULL)==NECK_BAD_CONFIG);
    p.flexion_warning_deg=NAN;
    CHECK(neck_posture_init(&c,1,1,&p)==NECK_BAD_CONFIG);
    CHECK(neck_posture_process(&c,&x,&o)==NECK_BAD_CONFIG);
    neck_posture_default_parameters(&p); p.lateral_danger_deg=5;
    CHECK(neck_posture_init(&c,1,1,&p)==NECK_BAD_CONFIG);
    neck_posture_default_parameters(&p); p.continuous_duration_us=0;
    CHECK(neck_posture_init(&c,1,1,&p)==NECK_BAD_CONFIG);
    fresh(&c); CHECK(neck_posture_process(&c,&x,NULL)==NECK_BAD_ARGUMENT);
    CHECK(neck_posture_process(NULL,&x,&o)==NECK_BAD_ARGUMENT);
    CHECK(o.flexion.instant_level==NECK_POSTURE_UNKNOWN);
    CHECK(neck_posture_process(&c,NULL,&o)==NECK_BAD_ARGUMENT);
}
static void test_boundaries(void)
{
    static const double flex[]={-1,-0.000000001,0,14.999999,15,29.999999,30,60};
    static const unsigned front[]={4,4,1,1,2,2,3,3};
    static const double sides[]={-10,-9.999999,-5.000001,-5,0,5,5.000001,9.999999,10};
    static const unsigned lateral[]={3,2,2,1,1,1,2,2,3};
    size_t i; ++groups;
    for(i=0;i<sizeof(flex)/sizeof(flex[0]);++i) {
        NeckPostureContext c; NeckPostureOutput o;
        fresh(&c); o=tick(&c,0,flex[i],0);
        CHECK(o.flexion.instant_level==front[i]);
        CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL && o.flexion.events==0);
    }
    for(i=0;i<sizeof(sides)/sizeof(sides[0]);++i) {
        NeckPostureContext c; NeckPostureOutput o;
        fresh(&c); o=tick(&c,0,0,sides[i]);
        CHECK(o.lateral.instant_level==lateral[i]);
        CHECK(o.lateral.alarm_level==NECK_POSTURE_NORMAL);
    }
}
static void test_ten_seconds_once(void)
{
    NeckPostureContext c; NeckPostureOutput o;
    uint64_t n; unsigned triggers=0;
    ++groups; fresh(&c);
    for(n=0;n<=1100;++n) {
        o=tick(&c,n,20,0);
        CHECK(o.flexion.warning_elapsed_us==n*10000);
        if(n<1000) CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL);
        else CHECK(o.flexion.alarm_level==NECK_POSTURE_WARNING);
        if(o.flexion.events&NECK_EVENT_TRIGGER) { ++triggers; CHECK(n==1000); }
    }
    CHECK(triggers==1);
    o=tick(&c,1101,0,0); CHECK(o.flexion.events==NECK_EVENT_CLEAR);
    o=tick(&c,1102,0,0); CHECK(o.flexion.events==0);
}
static void test_nested_upgrade_downgrade(void)
{
    NeckPostureContext c; NeckPostureOutput o;
    uint64_t n; ++groups; fresh(&c);
    for(n=0;n<=1900;++n) {
        o=tick(&c,n,n<900?20:40,0);
        if(n==999) CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL);
        if(n==1000) {
            CHECK(o.flexion.instant_level==NECK_POSTURE_DANGER);
            CHECK(o.flexion.alarm_level==NECK_POSTURE_WARNING);
            CHECK(o.flexion.events==NECK_EVENT_TRIGGER);
            CHECK(o.flexion.danger_elapsed_us==1000000);
        }
        if(n==1899) CHECK(o.flexion.alarm_level==NECK_POSTURE_WARNING);
    }
    CHECK(o.flexion.alarm_level==NECK_POSTURE_DANGER);
    CHECK(o.flexion.events==NECK_EVENT_LEVEL_CHANGE);
    o=tick(&c,1901,20,0);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_WARNING);
    CHECK(o.flexion.events==NECK_EVENT_LEVEL_CHANGE && o.flexion.danger_elapsed_us==0);
    o=tick(&c,1902,0,0); CHECK(o.flexion.events==NECK_EVENT_CLEAR);
    fresh(&c);
    for(n=0;n<=1000;++n) o=tick(&c,n,40,11);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_DANGER && o.flexion.events==NECK_EVENT_TRIGGER);
    CHECK(o.lateral.alarm_level==NECK_POSTURE_DANGER && o.lateral.events==NECK_EVENT_TRIGGER);
}
static void test_threshold_jitter_keeps_warning(void)
{
    NeckPostureContext c; NeckPostureOutput o; uint64_t n;
    ++groups; fresh(&c);
    for(n=0;n<=1000;++n) o=tick(&c,n,n%2?29.9:30.1,n%2?9.9:10.1);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_WARNING);
    CHECK(o.lateral.alarm_level==NECK_POSTURE_WARNING);
    CHECK(o.flexion.danger_elapsed_us==0 && o.lateral.danger_elapsed_us==0);
}
static void test_extension_and_direction(void)
{
    NeckPostureContext c; NeckPostureOutput o; uint64_t n;
    ++groups; fresh(&c);
    for(n=0;n<=1000;++n) o=tick(&c,n,-2,0);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_EXTENSION);
    CHECK(o.flexion.extension_elapsed_us==10000000 && o.flexion.events==NECK_EVENT_TRIGGER);
    o=tick(&c,1001,40,0);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL);
    CHECK(o.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_DIRECTION_RESET));
    for(n=1002;n<=2001;++n) o=tick(&c,n,40,0);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_DANGER);
    CHECK(o.flexion.events==NECK_EVENT_TRIGGER);
    fresh(&c);
    for(n=0;n<=1600;++n) {
        o=tick(&c,n,0,n<600?7:-7);
        if(n==600) CHECK(o.lateral.events==NECK_EVENT_DIRECTION_RESET);
        if(n==1000) CHECK(o.lateral.alarm_level==NECK_POSTURE_NORMAL);
    }
    CHECK(o.lateral.alarm_level==NECK_POSTURE_WARNING && o.lateral.direction==-1);
    CHECK(o.lateral.events==NECK_EVENT_TRIGGER);
    o=tick(&c,1601,0,7);
    CHECK(o.lateral.alarm_level==NECK_POSTURE_NORMAL);
    CHECK(o.lateral.events==(NECK_EVENT_CLEAR|NECK_EVENT_DIRECTION_RESET));
}
static void test_axes_independent(void)
{
    NeckPostureContext c; NeckPostureOutput o; uint64_t n;
    ++groups; fresh(&c);
    for(n=0;n<=1500;++n) {
        o=tick(&c,n,20,n<500?0:7);
        if(n==1000) {
            CHECK(o.flexion.events==NECK_EVENT_TRIGGER);
            CHECK(o.lateral.alarm_level==NECK_POSTURE_NORMAL);
        }
    }
    CHECK(o.lateral.events==NECK_EVENT_TRIGGER && o.flexion.events==0);
    fresh(&c);
    for(n=0;n<=1501;++n) {
        NeckPipelineOutput x=input(n,n*10000,n==500?NAN:20,7);
        if(n==500) { x.filtered.pitch_status=NECK_DIRECTION_UNDETERMINED; x.filtered.status=NECK_PARTIAL; }
        o=step(&c,&x);
        if(n==500) { CHECK(!o.flexion.valid && o.lateral.valid); CHECK(o.flags&NECK_POSTURE_INPUT_INVALID); }
        if(n==1000) { CHECK(o.lateral.events==NECK_EVENT_TRIGGER); CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL); }
    }
    CHECK(o.flexion.events==NECK_EVENT_TRIGGER && o.lateral.events==0);
}
static void test_invalid_and_recovery(void)
{
    NeckPostureContext c; NeckPostureOutput o;
    NeckPipelineOutput x; uint64_t n;
    ++groups; fresh(&c);
    for(n=0;n<=1000;++n) o=tick(&c,n,20,7);
    x=input(1001,10010000,20,7); x.filtered.status=NECK_NO_ACCEL; x.filtered.valid_mask&=~(uint32_t)(NECK_VALID_FLEXION|NECK_VALID_RIGHT_LATERAL);
    o=step(&c,&x);
    CHECK(o.flexion.alarm_level==NECK_POSTURE_UNKNOWN && !o.flexion.valid);
    CHECK(o.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    x.filtered.sequence=1002; x.filtered.frame_time_us=x.filtered.sample_time_us=10020000;
    o=step(&c,&x); CHECK(o.flexion.events==0 && o.lateral.events==0);
    for(n=1003;n<=2003;++n) o=tick(&c,n,20,7);
    CHECK(o.flexion.events==NECK_EVENT_TRIGGER);
    x=input(2004,20040000,20,7); x.filter_flags|=NECK_FILTER_WARMUP;
    o=step(&c,&x); CHECK(o.flags&NECK_POSTURE_WARMUP);
    CHECK(o.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    x=input(2005,20050000,20,7); x.filter_flags|=NECK_FILTER_RESET;
    o=step(&c,&x); CHECK(!o.flexion.valid && (o.flags&NECK_POSTURE_PIPELINE_RESET));
}
static void test_reference_reset(void)
{
    NeckPostureContext c; NeckPostureOutput o;
    NeckPipelineOutput x; uint64_t n;
    ++groups; fresh(&c);
    for(n=0;n<=1000;++n) o=tick(&c,n,20,0);
    x=input(1001,10010000,20,0); x.filtered.reference_id=2; x.filter_flags|=NECK_ZERO_COMMITTED;
    o=step(&c,&x);
    CHECK(o.flags&NECK_POSTURE_REFERENCE_CHANGED);
    CHECK(o.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    x=input(1002,10020000,20,0); x.filtered.reference_id=2;
    o=step(&c,&x); CHECK(o.flexion.warning_elapsed_us==0);
    x=input(1003,10030000,20,0); x.filtered.reference_id=0; x.filtered.valid_mask&=~(uint32_t)NECK_VALID_REFERENCE;
    o=step(&c,&x); CHECK(o.flags&NECK_POSTURE_NO_REFERENCE); CHECK(!o.flexion.valid);
}
static void test_gaps_and_unknown_time(void)
{
    NeckPostureContext c; NeckPostureOutput o; NeckPipelineOutput x;
    ++groups; fresh(&c); o=tick(&c,0,20,7);
    x=input(1,30000,20,7); o=step(&c,&x);
    CHECK(o.flexion.warning_elapsed_us==30000 && o.flexion.valid);
    x=input(2,60001,20,7); o=step(&c,&x);
    CHECK(!o.flexion.valid && (o.flags&NECK_POSTURE_STREAM_RESET));
    x=input(3,70001,20,7); o=step(&c,&x); CHECK(o.flexion.warning_elapsed_us==0);
    x=input(5,80001,20,7); o=step(&c,&x); CHECK(o.flags&NECK_POSTURE_STREAM_RESET);
    x=input(6,90001,20,7); x.filtered.valid_mask&=~(uint32_t)(NECK_VALID_SAMPLE_TIME|NECK_VALID_SAMPLE_AGE);
    o=step(&c,&x); CHECK(o.flags&NECK_POSTURE_FRAME_TIME_FALLBACK); CHECK(o.flags&NECK_POSTURE_STREAM_RESET);
    x.filtered.sequence=7; x.filtered.frame_time_us=100001; o=step(&c,&x);
    CHECK(o.flexion.valid && o.flexion.warning_elapsed_us==0 && !o.sample_time_known);
    x.filtered.sequence=8; x.filtered.frame_time_us=112001; o=step(&c,&x);
    CHECK(o.flexion.warning_elapsed_us==12000);
}
static void test_rejected_metadata(void)
{
    NeckPostureContext c; NeckPostureOutput o; NeckPipelineOutput x;
    ++groups; fresh(&c); o=tick(&c,0,20,0);
    x=input(0,0,20,0); CHECK(neck_posture_process(&c,&x,&o)==NECK_BAD_FRAME);
    CHECK(!o.flexion.valid && (o.flags&NECK_POSTURE_TIMING_REJECTED));
    x=input(1,10000,20,0); o=step(&c,&x); CHECK(o.flexion.warning_elapsed_us==0);
    x=input(2,20000,20,0); x.filtered.sample_time_us=10000; x.filtered.sample_age_us=10000;
    CHECK(neck_posture_process(&c,&x,&o)==NECK_BAD_FRAME);
    x=input(2,20000,20,0); x.filtered.config_id=2;
    CHECK(neck_posture_process(&c,&x,&o)==NECK_CONFIG_MISMATCH);
    x.filtered.config_id=1; x.filtered.session_id=2;
    CHECK(neck_posture_process(&c,&x,&o)==NECK_BAD_FRAME);
    neck_posture_reset(&c); o=step(&c,&x); CHECK(o.flexion.warning_elapsed_us==0);
    x=input(3,30000,20,0); x.filtered.session_id=2; x.filtered.source=NECK_SOURCE_HARDWARE;
    CHECK(neck_posture_process(&c,&x,&o)==NECK_BAD_FRAME);
    neck_posture_reset(&c); o=step(&c,&x); CHECK(o.flexion.valid);
    x=input(4,40000,20,0); x.filtered.session_id=2; x.filtered.source=NECK_SOURCE_HARDWARE;
    x.filtered.sample_time_us=40001;
    CHECK(neck_posture_process(&c,&x,&o)==NECK_BAD_FRAME);
    fresh(&c); x=input(0,50000,20,0); x.filtered.sample_time_us=10000; x.filtered.sample_age_us=40000;
    o=step(&c,&x); CHECK(!o.flexion.valid && (o.flags&NECK_POSTURE_INPUT_INVALID));
}
static void test_uint64_and_irregular_time(void)
{
    NeckPostureContext c; NeckPostureOutput o; NeckPipelineOutput x;
    const uint64_t base=(UINT64_C(1)<<63)+123;
    static const uint64_t dt[]={7000,12000,15000,9000,20000};
    uint64_t n, time_us=0;
    ++groups; fresh(&c);
    for(n=0;n<=1000;++n) {
        x=input(n,base+n*10000,20,0); o=step(&c,&x);
        CHECK(o.flexion.warning_elapsed_us==n*10000);
        if(n<1000) CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL);
    }
    CHECK(o.flexion.events==NECK_EVENT_TRIGGER);
    fresh(&c);
    for(n=0;time_us<10020000;++n) {
        x=input(n,time_us,20,0); o=step(&c,&x);
        CHECK(o.flexion.warning_elapsed_us==time_us);
        if(time_us<10000000) CHECK(o.flexion.alarm_level==NECK_POSTURE_NORMAL);
        else CHECK(o.flexion.alarm_level==NECK_POSTURE_WARNING);
        time_us+=dt[n%5];
    }
}
static NeckFrame acceleration_frame(uint64_t n, double pitch)
{
    NeckFrame f; unsigned i;
    double p=pitch*pi/180.0;
    memset(&f,0,sizeof(f)); f.config_id=f.session_id=1;
    f.sequence=n; f.frame_time_us=n*10000; f.source=NECK_SOURCE_SIMULATION;
    for(i=0;i<NECK_NODE_COUNT;++i) {
        f.nodes[i].accel_m_s2[0]=-gravity*sin(p);
        f.nodes[i].accel_m_s2[2]=gravity*cos(p);
        f.nodes[i].sample_time_us=f.frame_time_us;
        f.nodes[i].signal_valid_mask=NECK_SIGNAL_ACCEL;
        f.nodes[i].accel_present=f.nodes[i].sample_time_known=1;
        f.nodes[i].calibration_accel=NECK_CALIBRATION_UNKNOWN;
        f.nodes[i].status=NECK_NODE_PARTIAL;
    }
    return f;
}
static void pipeline_fresh(NeckPipelineContext *c)
{
    const double identity[4]={1,0,0,0};
    NeckPipelineParameters p;
    neck_pipeline_default_parameters(&p);
    CHECK(neck_pipeline_init(c,1,NECK_NODE_C4,identity,.001,&p)==NECK_OK);
}
static void test_filter_oracle(void)
{
    NeckPipelineContext c; NeckPipelineOutput o; NeckFrame f;
    unsigned n; ++groups; pipeline_fresh(&c);
    for(n=0;n<=300;++n) {
        double pole=exp(-pi*(double)n*.01);
        double oracle_x=-gravity*sin(pi/9)*(1-pole);
        double oracle_z=gravity*cos(pi/9)+(gravity-gravity*cos(pi/9))*pole;
        f=acceleration_frame(n,n?20:0);
        CHECK(neck_pipeline_process(&c,&f,&o)==NECK_OK);
        close_number(o.filtered.accel_body_m_s2[0],oracle_x);
        close_number(o.filtered.accel_body_m_s2[2],oracle_z);
        CHECK(!(o.filtered.valid_mask&NECK_VALID_FLEXION));
    }
}
static void test_zero_abort(void)
{
    NeckPipelineContext c; NeckPipelineOutput o; NeckFrame f;
    unsigned n; ++groups; pipeline_fresh(&c);
    CHECK(neck_pipeline_begin_zero(&c,1,NECK_REFERENCE_SIMULATION)==NECK_OK);
    for(n=0;n<=300;++n) {
        f=acceleration_frame(n,10); CHECK(neck_pipeline_process(&c,&f,&o)==NECK_OK);
        if(n<300) CHECK(!c.reference_set);
    }
    CHECK(o.zero.status==NECK_ZERO_SUCCEEDED && o.zero.sample_count==201);
    close_number(c.reference.pitch_deg,10); close_number(o.filtered.flexion_deg,0);
    CHECK(neck_pipeline_begin_zero(&c,2,NECK_REFERENCE_SIMULATION)==NECK_OK);
    f=acceleration_frame(301,10); CHECK(neck_pipeline_process(&c,&f,&o)==NECK_OK);
    f=acceleration_frame(303,20); CHECK(neck_pipeline_process(&c,&f,&o)==NECK_OK);
    CHECK(o.zero.status==NECK_ZERO_FAILED_INPUT && c.reference.reference_id==1);
    CHECK(o.filter_flags&NECK_FILTER_RESET);
}
static void test_pipeline_integration(const char *csv_path)
{
    NeckPipelineContext pipe; NeckPostureContext detector;
    NeckPipelineOutput filtered; NeckPostureOutput posture;
    FILE *stream=fopen(csv_path,"w");
    unsigned n, triggers=0, changes=0, clears=0;
    ++groups; CHECK(stream!=NULL); pipeline_fresh(&pipe); fresh(&detector);
    CHECK(neck_pipeline_begin_zero(&pipe,1,NECK_REFERENCE_SIMULATION)==NECK_OK);
    fprintf(stream,"sequence,time_us,filtered_angle_valid,filtered_flexion_deg,instant_level,alarm_level,events,warning_elapsed_us,danger_elapsed_us,zero_status\n");
    for(n=0;n<=4800;++n) {
        double pitch=n<500?10:(n<2000?35:(n<4000?50:10));
        NeckFrame f=acceleration_frame(n,pitch);
        CHECK(neck_pipeline_process(&pipe,&f,&filtered)==NECK_OK);
        CHECK(neck_posture_process(&detector,&filtered,&posture)==NECK_OK);
        if(posture.flexion.events&NECK_EVENT_TRIGGER) ++triggers;
        if(posture.flexion.events&NECK_EVENT_LEVEL_CHANGE) ++changes;
        if(posture.flexion.events&NECK_EVENT_CLEAR) ++clears;
        if(n<300) CHECK(posture.flexion.alarm_level==NECK_POSTURE_UNKNOWN);
        if(n==300) CHECK(filtered.zero.status==NECK_ZERO_SUCCEEDED && !posture.flexion.valid);
        if(n==1900) CHECK(posture.flexion.alarm_level==NECK_POSTURE_WARNING);
        if(n==3900) CHECK(posture.flexion.alarm_level==NECK_POSTURE_DANGER);
        fprintf(stream,"%u,%llu,%u,%.12g,%u,%u,%u,%llu,%llu,%u\n",n,
            (unsigned long long)posture.time_us,
            (unsigned)((filtered.filtered.valid_mask&NECK_VALID_FLEXION)!=0),
            filtered.filtered.flexion_deg,(unsigned)posture.flexion.instant_level,
            (unsigned)posture.flexion.alarm_level,(unsigned)posture.flexion.events,
            (unsigned long long)posture.flexion.warning_elapsed_us,
            (unsigned long long)posture.flexion.danger_elapsed_us,(unsigned)filtered.zero.status);
        if(posture.flexion.events) printf("integration event at %.2f s: mask=%u, level=%u\n",
            (double)n*.01,(unsigned)posture.flexion.events,(unsigned)posture.flexion.alarm_level);
    }
    CHECK(fclose(stream)==0);
    CHECK(triggers==1 && changes==2 && clears==1);
    CHECK(posture.flexion.alarm_level==NECK_POSTURE_NORMAL);
    printf("integration: 4801 frames, %u trigger, %u level changes, %u clear\n",triggers,changes,clears);
}
int main(int argc, char **argv)
{
    CHECK(argc==2);
    test_parameters(); test_boundaries(); test_ten_seconds_once();
    test_nested_upgrade_downgrade(); test_threshold_jitter_keeps_warning();
    test_extension_and_direction(); test_axes_independent(); test_invalid_and_recovery();
    test_reference_reset(); test_gaps_and_unknown_time(); test_rejected_metadata();
    test_uint64_and_irregular_time(); test_filter_oracle(); test_zero_abort(); test_pipeline_integration(argv[1]);
    printf("PASS: %u groups, %u C assertions; no Python runtime\n",groups,checks);
    return 0;
}
