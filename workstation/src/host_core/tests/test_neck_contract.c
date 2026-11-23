/* S09 integration-contract tests: native C, NOT an RTOS or driver emulator. */
#include "neck_owner_step_example.h"
#include "neck_clock.h"
#include "neck_simulator.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned groups,checks;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
typedef struct { NeckProcessingOutput outputs[2]; uint8_t kinds[2]; NeckResult results[2]; unsigned count; } Capture;
static void capture(const NeckProcessingOutput *o,uint8_t kind,NeckResult result,void *user)
{
    Capture *c=(Capture *)user; CHECK(c->count<2); c->outputs[c->count]=*o; c->kinds[c->count]=kind;
    c->results[c->count]=result; ++c->count;
}
static void parameters(NeckProcessingParameters *p)
{
    neck_processing_default_parameters(p); p->quality.recovery_us=0; p->pipeline.warmup_us=0;
    p->pipeline.zero_min_samples=3; p->pipeline.zero_duration_us=20000; p->posture.continuous_duration_us=100000;
}
static void fresh(NeckProcessingContext *c)
{
    NeckProcessingParameters p; parameters(&p);
    CHECK(neck_processing_init(c,1,1,neck_simulator_mounts(),0,&p)==NECK_OK);
}
static Capture step(NeckProcessingContext *c,const NeckAcquisitionFrame *f,uint64_t now)
{
    Capture cap; NeckProcessingOutput scratch; unsigned count; NeckResult r;
    memset(&cap,0,sizeof(cap));
    r=neck_owner_step_example(c,f,now,&scratch,capture,&cap,&count); CHECK(count==cap.count);
    CHECK(r==(f!=NULL?cap.results[cap.count-1]:NECK_OK)); return cap;
}
static NeckAcquisitionFrame frame(uint64_t sequence,uint64_t time,double pitch)
{
    NeckAcquisitionFrame f; const double rad=.01745329251994329577; neck_simulator_frame(&f,sequence,time);
    f.acceleration.nodes[1].accel_m_s2[0]=-9.80665*sin(pitch*rad); f.acceleration.nodes[1].accel_m_s2[1]=0;
    f.acceleration.nodes[1].accel_m_s2[2]=9.80665*cos(pitch*rad);
    f.orientation.nodes[1].sensor_to_world_wxyz[0]=cos(pitch*rad/2); f.orientation.nodes[1].sensor_to_world_wxyz[1]=0;
    f.orientation.nodes[1].sensor_to_world_wxyz[2]=sin(pitch*rad/2); f.orientation.nodes[1].sensor_to_world_wxyz[3]=0;
    return f;
}
static void references(NeckProcessingContext *c)
{
    unsigned i; const double zero[3]={12,10,5};
    for(i=0;i<3;++i) { NeckNeutralReference r; CHECK(neck_reference_init(&r,9,1,(uint8_t)i,0,zero[i],0)==NECK_OK); CHECK(neck_pipeline_set_reference(&c->pipelines[i],&r)==NECK_OK); }
}
static Capture active(NeckProcessingContext *c)
{
    unsigned n; Capture cap; fresh(c); references(c);
    for(n=0;n<=10;++n) { NeckAcquisitionFrame f=frame(n,(uint64_t)n*10000,35); cap=step(c,&f,(uint64_t)n*10000); }
    CHECK(cap.outputs[0].pose.posture.flexion.alarm_level==NECK_POSTURE_WARNING); CHECK(cap.outputs[0].pose.posture.flexion.events==NECK_EVENT_TRIGGER);
    return cap;
}
static void widths_and_defaults(void)
{
    NeckProcessingParameters p; ++groups; neck_processing_default_parameters(&p);
    CHECK(NECK_NODE_C0==0 && NECK_NODE_C4==1 && NECK_NODE_C7==2); CHECK(NECK_SOURCE_SIMULATION==0 && NECK_SOURCE_HARDWARE==1);
    CHECK(sizeof(((NeckFrame *)0)->sequence)==8); CHECK(sizeof(((NeckQualityOutput *)0)->decision_time_us)==8);
    CHECK(sizeof(((NeckPoseBatch *)0)->valid_until_us)==8); CHECK(sizeof(((NeckPoseBatch *)0)->invalidation_epoch)==4);
    CHECK(NECK_VALID_FLEXION==32 && NECK_VALID_RIGHT_LATERAL==64); CHECK(NECK_REL_RAW_EULER==2 && NECK_REL_CORRECTED_EULER==16);
    CHECK(NECK_POSTURE_EXTENSION==4 && NECK_POSTURE_UNKNOWN==0); CHECK((NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED)==10);
    CHECK(p.pipeline.cutoff_hz==.5 && p.pipeline.nominal_period_us==10000 && p.pipeline.warmup_us==1000000);
    CHECK(p.quality.recovery_us==100000 && p.quality.no_frame_timeout_us==30000);
    CHECK(p.posture.continuous_duration_us==10000000 && p.posture.flexion_warning_deg==15 && p.posture.flexion_danger_deg==30);
    CHECK(p.relative.stale_after_us==30000 && p.relative.max_node_skew_us==10000);
    printf("Host ABI only: frame=%u acquisition=%u context=%u output=%u pose=%u parameters=%u\n",
        (unsigned)sizeof(NeckFrame),(unsigned)sizeof(NeckAcquisitionFrame),(unsigned)sizeof(NeckProcessingContext),
        (unsigned)sizeof(NeckProcessingOutput),(unsigned)sizeof(NeckPoseBatch),(unsigned)sizeof(NeckProcessingParameters));
}
static void cold_and_copy(void)
{
    NeckProcessingParameters p; NeckProcessingContext c; NeckAcquisitionFrame f,before; Capture cap; unsigned n;
    ++groups; neck_processing_default_parameters(&p);
    CHECK(neck_processing_init(&c,1,1,neck_simulator_mounts(),0,&p)==NECK_OK);
    p.pipeline.cutoff_hz=1.5; p.quality.no_frame_timeout_us=50000;
    CHECK(c.pipelines[0].parameters.cutoff_hz==.5 && c.quality.parameters.no_frame_timeout_us==30000); /* init copies values */
    for(n=0;n<=110;++n) { neck_simulator_frame(&f,n,(uint64_t)n*10000); before=f; cap=step(&c,&f,(uint64_t)n*10000); CHECK(cap.count==1); CHECK(memcmp(&f,&before,sizeof(f))==0); }
    CHECK(cap.outputs[0].pose.quality.accel_usable_mask==7); CHECK(cap.outputs[0].pose.pending_reference_id==0);
    CHECK(cap.outputs[0].pose.posture.flexion.valid==0); CHECK(!(cap.outputs[0].c4.filtered.valid_mask&NECK_VALID_FLEXION));
    CHECK(!c.relative.reference_set && !c.pipelines[1].reference_set); CHECK(c.request_state==NECK_REQUEST_IDLE);
}
static void resume_order(void)
{
    NeckProcessingContext c; Capture cap; NeckAcquisitionFrame f; ++groups; (void)active(&c);
    f=frame(11,140001,35); cap=step(&c,&f,140001); CHECK(cap.count==2); CHECK(cap.kinds[0]==NECK_OWNER_POLL && cap.kinds[1]==NECK_OWNER_FRAME);
    CHECK(cap.outputs[0].pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    CHECK(cap.outputs[0].pose.quality.sequence==10); CHECK(cap.outputs[0].pose.quality.frame_time_us==100000);
    CHECK(cap.outputs[0].pose.quality.decision_time_us==140001); CHECK(cap.outputs[0].pose.posture.flexion.valid==0);
    CHECK(cap.outputs[1].pose.quality.sequence==11); CHECK(cap.outputs[1].pose.quality.frame_accepted);
    CHECK(cap.outputs[1].pose.posture.flexion.warning_elapsed_us==0); CHECK(!(cap.outputs[1].pose.posture.flexion.events&NECK_EVENT_TRIGGER));
    CHECK(c.pipelines[1].reference_set && c.pipelines[1].reference.reference_id==9);
    cap=step(&c,NULL,180002); CHECK(cap.count==1); CHECK(cap.kinds[0]==NECK_OWNER_POLL);
    cap=step(&c,NULL,190002); CHECK(cap.count==1); CHECK(cap.outputs[0].pose.posture.flexion.events==0);
}
static void reject_not_refresh(void)
{
    NeckProcessingContext c; Capture cap; NeckAcquisitionFrame f; ++groups; (void)active(&c);
    f=frame(11,120000,35); cap=step(&c,&f,110000); CHECK(cap.count==1); CHECK(cap.results[0]!=NECK_OK);
    CHECK(cap.outputs[0].pose.quality.flags&NECK_Q_FUTURE_FRAME); CHECK(cap.outputs[0].pose.posture.flexion.events==10);
    CHECK(c.quality.previous_frame_time_us==100000 && c.quality.previous_sequence==10);
    cap=step(&c,NULL,130001); CHECK(cap.count==1); CHECK(cap.outputs[0].pose.quality.flags&NECK_Q_TIMEOUT_EDGE);
    CHECK(cap.outputs[0].pose.posture.flexion.events==0);
}
static void zero_reset_and_copy(void)
{
    NeckProcessingContext c; Capture cap; NeckAcquisitionFrame f; NeckPoseBatch old; NeckProcessingOutput reset; unsigned n;
    ++groups; fresh(&c); CHECK(neck_processing_request_zero(&c,0,NECK_REFERENCE_SIMULATION)==NECK_OK);
    for(n=0;n<=3;++n) { neck_simulator_frame(&f,n,(uint64_t)n*10000); cap=step(&c,&f,(uint64_t)n*10000); }
    CHECK(c.request_state==NECK_REQUEST_IDLE && c.relative.reference_set); CHECK(cap.outputs[0].relative.reference_id==1);
    CHECK(cap.outputs[0].c4.filtered.valid_mask&NECK_VALID_FLEXION); old=cap.outputs[0].pose;
    CHECK(neck_processing_request_zero(&c,30000,0)==NECK_OK); neck_processing_cancel_zero(&c); CHECK(c.relative.reference_set);
    CHECK(neck_processing_reset_stream(&c,40000,&reset)==NECK_OK); CHECK(!c.relative.reference_set && !c.pipelines[1].reference_set);
    CHECK(neck_processing_expire_pose(&old,40000,c.invalidation_epoch)==1); CHECK(old.nodes[1].valid_mask==0);
    CHECK(old.posture.flexion.events==0); CHECK(!(old.posture.flags&NECK_POSTURE_INPUT_TIMEOUT));
    neck_simulator_frame(&f,0,50000); f.acceleration.session_id=f.orientation.session_id=2;
    cap=step(&c,&f,50000); CHECK(cap.results[0]==NECK_OK); CHECK(!cap.outputs[0].pose.posture.flexion.valid);
}
static void explicit_config_boundary(void)
{
    NeckProcessingContext c; NeckProcessingParameters p; NeckAcquisitionFrame f; Capture cap;
    ++groups; (void)active(&c); parameters(&p); p.pipeline.cutoff_hz=.8;
    /* This models a QUIESCED owner boundary, NOT an implemented RTOS hot swap.
     * The integrator must drain queues and serialize transport first. */
    CHECK(neck_processing_init(&c,2,2,neck_simulator_mounts(),200000,&p)==NECK_OK);
    CHECK(c.pipelines[1].parameters.cutoff_hz==.8 && c.request_state==NECK_REQUEST_IDLE && !c.pipelines[1].reference_set);
    f=frame(12,210000,35); cap=step(&c,&f,210000); CHECK(cap.count==1 && cap.results[0]!=NECK_OK); CHECK(!cap.outputs[0].pose.quality.frame_accepted);
    f=frame(0,220000,35); f.acceleration.config_id=f.orientation.config_id=2; f.acceleration.session_id=f.orientation.session_id=3;
    { unsigned i; for(i=0;i<3;++i) f.orientation.nodes[i].world_frame_id=2; }
    cap=step(&c,&f,220000); CHECK(cap.count==1 && cap.results[0]==NECK_OK); CHECK(!cap.outputs[0].pose.posture.flexion.valid);
    p.pipeline.cutoff_hz=0; CHECK(neck_processing_init(&c,3,2,neck_simulator_mounts(),230000,&p)==NECK_BAD_CONFIG);
    CHECK(!c.initialized); cap.count=0;
    { NeckProcessingOutput o; unsigned count=99; CHECK(neck_owner_step_example(&c,NULL,230000,&o,capture,&cap,&count)==NECK_BAD_CONFIG); CHECK(count==0 && cap.count==0); }
}
static void clock_and_unknown_time(void)
{
    NeckTickClock clock; uint64_t time; NeckProcessingContext c; NeckAcquisitionFrame f; Capture cap; unsigned i;
    ++groups; CHECK(neck_clock_init(&clock,UINT32_MAX-5,1000)==NECK_OK); CHECK(neck_clock_update(&clock,3,&time)==NECK_OK && time==9000);
    CHECK(neck_clock_update(&clock,3,&time)==NECK_OK && time==9000); CHECK(neck_clock_update(&clock,4,&time)==NECK_OK && time==10000);
    fresh(&c); f=frame(0,0,10); f.acceleration.source=f.orientation.source=NECK_SOURCE_HARDWARE;
    for(i=0;i<3;++i) f.acceleration.nodes[i].calibration_accel=f.orientation.nodes[i].calibration_system=3;
    f.acceleration.nodes[1].sample_time_known=f.orientation.nodes[1].sample_time_known=0;
    cap=step(&c,&f,0); CHECK(cap.results[0]==NECK_OK); CHECK(cap.outputs[0].pose.quality.accel_usable_mask==5);
    CHECK(cap.outputs[0].pose.quality.orientation_usable_mask==5); CHECK(cap.outputs[0].pose.quality.node_flags[1]&NECK_QN_ACCEL_TIME_UNKNOWN);
    CHECK(cap.outputs[0].pose.quality.flags&NECK_Q_ENGINEERING_ONLY); CHECK(cap.outputs[0].c4.filtered.valid_mask==0);
}
static void arguments(void)
{
    NeckProcessingContext c; NeckProcessingOutput o; Capture cap; unsigned count=99;
    ++groups; fresh(&c); memset(&cap,0,sizeof(cap));
    CHECK(neck_owner_step_example(NULL,NULL,0,&o,capture,&cap,&count)==NECK_BAD_ARGUMENT && count==0);
    CHECK(neck_owner_step_example(&c,NULL,0,NULL,capture,&cap,&count)==NECK_BAD_ARGUMENT && count==0);
    CHECK(neck_owner_step_example(&c,NULL,0,&o,NULL,&cap,&count)==NECK_BAD_ARGUMENT && count==0);
    CHECK(neck_owner_step_example(&c,NULL,0,&o,capture,&cap,NULL)==NECK_OK && cap.count==0); /* no implicit timeout/zero */
}
int main(void)
{
    widths_and_defaults(); cold_and_copy(); resume_order(); reject_not_refresh(); zero_reset_and_copy();
    explicit_config_boundary(); clock_and_unknown_time(); arguments();
    printf("PASS: %u S09 integration-contract groups, %u C assertions. No HAL/RTOS/board execution.\n",groups,checks); return 0;
}
