/* S07 native C tests execute the SAME processing functions as STM32 tasks.
 * Hardware-labelled fixtures only test policy: no physical measurement. */
#include "neck_processing.h"
#include "neck_clock.h"
#include "neck_simulator.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned groups,checks;
static const double rad=.01745329251994329577;
static const double mounts[3][4]={{1,0,0,0},{1,0,0,0},{1,0,0,0}};
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
static NeckAcquisitionFrame packet(uint64_t seq,uint64_t time,double pitch)
{
    NeckAcquisitionFrame f; unsigned i; memset(&f,0,sizeof(f));
    f.acceleration.config_id=f.orientation.config_id=1;
    f.acceleration.session_id=f.orientation.session_id=1;
    f.acceleration.sequence=f.orientation.sequence=seq;
    f.acceleration.frame_time_us=f.orientation.frame_time_us=time;
    for(i=0;i<3;++i) {
        NeckNodeSample *a=&f.acceleration.nodes[i]; NeckOrientationSample *q=&f.orientation.nodes[i];
        a->accel_m_s2[0]=-9.80665*sin(pitch*rad); a->accel_m_s2[2]=9.80665*cos(pitch*rad);
        a->sample_time_us=q->sample_time_us=time; a->accel_present=a->sample_time_known=1;
        a->signal_valid_mask=NECK_SIGNAL_ACCEL|NECK_SIGNAL_QUATERNION; a->calibration_accel=3;
        a->status=q->status=NECK_NODE_PARTIAL;
        q->sensor_to_world_wxyz[0]=cos(pitch*rad/2); q->sensor_to_world_wxyz[2]=sin(pitch*rad/2);
        q->quat_present=q->sample_time_known=1; q->calibration_system=3; q->world_frame_id=1;
    }
    return f;
}
static void quality_fresh(NeckQualityContext *c,uint64_t now,uint64_t hold)
{
    NeckQualityParameters p; neck_quality_default_parameters(&p); p.recovery_us=hold;
    CHECK(neck_quality_init(c,1,1,now,&p)==NECK_OK);
}
static NeckQualityOutput qstep(NeckQualityContext *c,NeckAcquisitionFrame *f,uint64_t now)
{
    NeckQualityOutput q; CHECK(neck_quality_process(c,&f->acceleration,&f->orientation,now,&q)==NECK_OK);
    CHECK(q.flags&NECK_Q_ENGINEERING_ONLY); return q;
}
static void fast_parameters(NeckProcessingParameters *p)
{
    neck_processing_default_parameters(p); p->quality.recovery_us=0; p->pipeline.warmup_us=0;
    p->pipeline.zero_duration_us=20000; p->pipeline.zero_min_samples=3;
    p->posture.continuous_duration_us=100000;
}
static void runtime_fresh(NeckProcessingContext *c,uint64_t now)
{
    NeckProcessingParameters p; fast_parameters(&p);
    CHECK(neck_processing_init(c,1,1,mounts,now,&p)==NECK_OK);
}
static void external_fixture_references(NeckProcessingContext *c)
{
    unsigned i; NeckRelativeReference r; memset(&r,0,sizeof(r));
    r.config_id=r.world_frame_id=1; r.reference_id=99; r.origin=NECK_REFERENCE_SIMULATION;
    for(i=0;i<3;++i) {
        NeckNeutralReference n; CHECK(neck_reference_init(&n,99,1,(uint8_t)i,0,10,0)==NECK_OK);
        CHECK(neck_pipeline_set_reference(&c->pipelines[i],&n)==NECK_OK);
        r.pair_wxyz[i][0]=1;
    }
    r.pair_wxyz[3][0]=cos(10*rad/2); r.pair_wxyz[3][2]=sin(10*rad/2);
    CHECK(neck_relative_set_reference(&c->relative,&r)==NECK_OK);
}
static NeckProcessingOutput step(NeckProcessingContext *c,uint64_t n,double pitch)
{
    NeckAcquisitionFrame f=packet(n,n*10000,pitch); NeckProcessingOutput o;
    CHECK(neck_processing_process(c,&f,n*10000,&o)==NECK_OK); return o;
}
static NeckProcessingOutput active_alarm(NeckProcessingContext *c)
{
    unsigned n; NeckProcessingOutput o; runtime_fresh(c,0); external_fixture_references(c);
    for(n=0;n<=10;++n) o=step(c,n,35);
    CHECK(o.pose.posture.flexion.alarm_level==NECK_POSTURE_WARNING);
    CHECK(o.pose.posture.flexion.events==NECK_EVENT_TRIGGER); return o;
}
static void all_invalid(const NeckProcessingOutput *o)
{
    unsigned i; CHECK(o->pose.posture.flexion.alarm_level==NECK_POSTURE_UNKNOWN);
    CHECK(!o->pose.posture.flexion.valid && !o->pose.posture.lateral.valid);
    CHECK(o->pose.posture.flexion.warning_elapsed_us==0);
    for(i=0;i<3;++i) CHECK(o->pose.nodes[i].valid_mask==0);
    for(i=0;i<4;++i) CHECK(o->relative.pairs[i].valid_mask==0);
}
static void test_parameters(void)
{
    NeckQualityParameters p; NeckQualityContext c; NeckQualityOutput q; NeckProcessingContext r;
    NeckProcessingParameters rp; NeckProcessingOutput o; NeckAcquisitionFrame f=packet(0,0,10);
    ++groups; neck_quality_default_parameters(&p); neck_quality_default_parameters(NULL);
    CHECK(p.recovery_us==100000 && p.no_frame_timeout_us==30000);
    CHECK(neck_quality_init(NULL,1,1,0,&p)==NECK_BAD_ARGUMENT);
    CHECK(neck_quality_init(&c,0,1,0,&p)==NECK_BAD_CONFIG);
    CHECK(neck_quality_init(&c,1,0,0,&p)==NECK_BAD_CONFIG);
    p.gravity_m_s2=NAN; CHECK(neck_quality_init(&c,1,1,0,&p)==NECK_BAD_CONFIG);
    CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,0,&q)==NECK_BAD_CONFIG);
    neck_quality_default_parameters(&p); p.norm_reentry_error_m_s2=p.norm_exit_error_m_s2;
    CHECK(neck_quality_init(&c,1,1,0,&p)==NECK_BAD_CONFIG);
    neck_quality_default_parameters(&p); p.minimum_accel_calibration=4;
    CHECK(neck_quality_init(&c,1,1,0,&p)==NECK_BAD_CONFIG);
    neck_quality_default_parameters(&p); p.max_frame_age_us=30001;
    CHECK(neck_quality_init(&c,1,1,0,&p)==NECK_BAD_CONFIG);
    quality_fresh(&c,0,0);
    CHECK(neck_quality_process(&c,NULL,&f.orientation,0,&q)==NECK_BAD_ARGUMENT);
    CHECK(q.flags&NECK_Q_FRAME_REJECTED);
    CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,0,NULL)==NECK_BAD_ARGUMENT);
    CHECK(neck_quality_process(NULL,&f.acceleration,&f.orientation,0,&q)==NECK_BAD_ARGUMENT);
    neck_processing_default_parameters(&rp); neck_processing_default_parameters(NULL);
    CHECK(neck_processing_init(NULL,1,1,mounts,0,&rp)==NECK_BAD_ARGUMENT);
    CHECK(neck_processing_init(&r,1,1,mounts,0,NULL)==NECK_BAD_CONFIG);
    CHECK(neck_processing_process(&r,&f,0,&o)==NECK_BAD_CONFIG);
    runtime_fresh(&r,0);
    CHECK(neck_processing_process(&r,NULL,0,&o)==NECK_BAD_ARGUMENT); all_invalid(&o);
    CHECK(neck_processing_request_zero(NULL,0,0)==NECK_BAD_ARGUMENT);
    CHECK(neck_processing_request_zero(&r,0,2)==NECK_BAD_REFERENCE);
    CHECK(neck_processing_expire_pose(NULL,0,0)==0);
    neck_processing_cancel_zero(NULL); neck_quality_reset_stream(NULL,0);
}
static void test_recovery_and_hysteresis(void)
{
    NeckQualityContext c; NeckQualityParameters p; NeckQualityOutput q; unsigned n;
    ++groups; neck_quality_default_parameters(&p); p.gravity_m_s2=10;
    p.norm_reentry_error_m_s2=.5; p.norm_exit_error_m_s2=1; p.recovery_us=20000;
    CHECK(neck_quality_init(&c,1,1,0,&p)==NECK_OK);
    for(n=0;n<=2;++n) { NeckAcquisitionFrame f=packet(n,n*10000,0);
        unsigned i; for(i=0;i<3;++i) f.acceleration.nodes[i].accel_m_s2[2]=10;
        q=qstep(&c,&f,n*10000); CHECK(q.accel_usable_mask==(n<2?0:7)); }
    for(n=3;n<=7;++n) {
        NeckAcquisitionFrame f=packet(n,n*10000,0); unsigned i;
        for(i=0;i<3;++i) f.acceleration.nodes[i].accel_m_s2[2]=10;
        f.acceleration.nodes[1].accel_m_s2[2]=n==3?11.1:(n==4?10.75:10.5);
        q=qstep(&c,&f,n*10000);
        CHECK(q.accel_usable_mask==(n<7?5:7)); CHECK(q.orientation_usable_mask==(n<7?5:7));
        if(n==4) CHECK(q.node_flags[1]&NECK_QN_NORM_REJECTED);
        if(n==5) CHECK(q.node_flags[1]&NECK_QN_ACCEL_RECOVERING);
    }
    { NeckAcquisitionFrame f=packet(8,80000,0); unsigned i;
      for(i=0;i<3;++i) f.acceleration.nodes[i].accel_m_s2[2]=11;
      q=qstep(&c,&f,80000); CHECK(q.accel_usable_mask==7); } /* Inclusive exit endpoint. */
}
static void test_numeric_and_metadata_faults(void)
{
    unsigned kind; ++groups;
    for(kind=0;kind<9;++kind) {
        NeckQualityContext c; NeckQualityOutput q; NeckAcquisitionFrame f=packet(0,0,0);
        quality_fresh(&c,0,0);
        if(kind==0) f.acceleration.nodes[1].accel_m_s2[0]=NAN;
        if(kind==1) f.acceleration.nodes[1].accel_m_s2[0]=INFINITY;
        if(kind==2) memset(f.acceleration.nodes[1].accel_m_s2,0,3*sizeof(double));
        if(kind==3) f.acceleration.nodes[1].status=NECK_NODE_FAULT;
        if(kind==4) f.acceleration.nodes[1].accel_present=0;
        if(kind==5) f.orientation.nodes[1].sensor_to_world_wxyz[0]=NAN;
        if(kind==6) memset(f.orientation.nodes[1].sensor_to_world_wxyz,0,4*sizeof(double));
        if(kind==7) f.orientation.nodes[1].world_frame_id=2;
        if(kind==8) f.orientation.nodes[1].calibration_system=4;
        q=qstep(&c,&f,0); CHECK(q.accel_usable_mask==(kind<5?5:7));
        CHECK(q.orientation_usable_mask==5 && q.state==NECK_Q_PARTIAL);
    }
}
static void test_time_endpoints_and_repetition(void)
{
    NeckQualityContext c; NeckQualityOutput q; NeckAcquisitionFrame f; unsigned i;
    ++groups; quality_fresh(&c,0,0); f=packet(0,0,0); q=qstep(&c,&f,30000);
    CHECK(q.accel_usable_mask==7 && q.frame_accepted);
    CHECK(neck_quality_poll(&c,30000,&q)==0);
    CHECK(neck_quality_poll(&c,30001,&q)==1 && (q.flags&NECK_Q_TIMEOUT_EDGE));
    CHECK(neck_quality_poll(&c,40000,&q)==1 && !(q.flags&NECK_Q_TIMEOUT_EDGE));
    quality_fresh(&c,0,0); f=packet(0,0,0);
    CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,30001,&q)==NECK_BAD_FRAME);
    CHECK((q.flags&NECK_Q_OLD_FRAME) && !c.stream_bound);
    quality_fresh(&c,0,0); f=packet(0,100000,0);
    for(i=0;i<3;++i) f.acceleration.nodes[i].sample_time_us=f.orientation.nodes[i].sample_time_us=70000;
    q=qstep(&c,&f,100000); CHECK(q.accel_usable_mask==7);
    quality_fresh(&c,0,0); f=packet(0,100000,0); f.acceleration.nodes[1].sample_time_us=69999;
    q=qstep(&c,&f,100000); CHECK(q.accel_usable_mask==5 && (q.node_flags[1]&NECK_QN_ACCEL_TIME_INVALID));
    quality_fresh(&c,0,0); f=packet(0,100000,0); q=qstep(&c,&f,100000);
    f=packet(1,110000,0); f.acceleration.nodes[1].sample_time_us=100000;
    q=qstep(&c,&f,110000); CHECK(q.accel_usable_mask==5 && (q.node_flags[1]&NECK_QN_ACCEL_NOT_NEW));
    f=packet(2,120000,0); f.orientation.nodes[1].sample_time_us=110000;
    q=qstep(&c,&f,120000); CHECK(q.accel_usable_mask==7 && q.orientation_usable_mask==5);
    quality_fresh(&c,0,0); f=packet(0,100000,0); f.acceleration.nodes[1].sample_time_known=0;
    q=qstep(&c,&f,100000); CHECK(q.accel_usable_mask==5 && (q.node_flags[1]&NECK_QN_ACCEL_TIME_UNKNOWN));
    quality_fresh(&c,0,0); f=packet(0,100000,0); f.orientation.nodes[1].sample_time_known=0;
    q=qstep(&c,&f,100000); CHECK(q.accel_usable_mask==7 && q.orientation_usable_mask==5);
    quality_fresh(&c,0,0); f=packet(0,100000,0); f.acceleration.nodes[1].sample_time_us=100001;
    q=qstep(&c,&f,100000); CHECK(q.node_flags[1]&NECK_QN_ACCEL_TIME_INVALID);
}
static void test_provenance_and_calibration(void)
{
    NeckQualityContext c; NeckQualityOutput q; NeckAcquisitionFrame f; unsigned i;
    ++groups; quality_fresh(&c,0,0); f=packet(0,0,0);
    for(i=0;i<3;++i) f.acceleration.nodes[i].calibration_accel=f.orientation.nodes[i].calibration_system=255;
    q=qstep(&c,&f,0); CHECK(q.accel_usable_mask==7 && (q.node_flags[0]&NECK_QN_ACCEL_CAL_UNKNOWN));
    quality_fresh(&c,0,0); f.acceleration.source=f.orientation.source=NECK_SOURCE_HARDWARE;
    q=qstep(&c,&f,0); CHECK(q.accel_usable_mask==0 && q.orientation_usable_mask==0);
    quality_fresh(&c,0,0); f=packet(0,0,0); f.acceleration.source=f.orientation.source=1;
    q=qstep(&c,&f,0); CHECK(q.accel_usable_mask==7);
    f=packet(1,10000,0); f.acceleration.source=f.orientation.source=0;
    CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,10000,&q)==NECK_BAD_FRAME);
    CHECK(q.flags&NECK_Q_STREAM_CHANGED);
    neck_quality_reset_stream(&c,10000); q=qstep(&c,&f,10000); CHECK(q.accel_usable_mask==7);
    f=packet(2,20000,0); f.orientation.sequence=1;
    CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,20000,&q)==NECK_BAD_FRAME);
    CHECK(q.flags&NECK_Q_METADATA_MISMATCH);
    f=packet(2,20000,0); f.acceleration.config_id=f.orientation.config_id=2;
    CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,20000,&q)==NECK_CONFIG_MISMATCH);
}
static void test_wall_timeout_active_alarm(void)
{
    NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c); unsigned n;
    ++groups; CHECK(neck_processing_poll(&c,130000,&o)==0);
    CHECK(neck_processing_poll(&c,130001,&o)==1); all_invalid(&o);
    CHECK(o.pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    CHECK(o.pose.posture.flags&NECK_POSTURE_INPUT_TIMEOUT);
    CHECK(o.pose.posture.time_us==130001 && o.pose.posture.frame_time_us==100000);
    CHECK(c.stats.timeout_transitions==1 && c.pipelines[1].reference_set && c.relative.reference_set);
    for(n=0;n<10;++n) {
        CHECK(neck_processing_poll(&c,140000+n*10000,&o)==1); all_invalid(&o);
        CHECK(o.pose.posture.flexion.events==0 && o.pose.posture.lateral.events==0);
    }
    CHECK(c.stats.timeout_transitions==1);
    for(n=24;n<=35;++n) {
        o=step(&c,n,35);
        if(n<35) CHECK(o.pose.posture.flexion.alarm_level!=NECK_POSTURE_WARNING);
    }
    CHECK(o.pose.posture.flexion.events==NECK_EVENT_TRIGGER);
}
static void test_rejected_input_and_cached_snapshot(void)
{
    NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c); NeckPoseBatch old=o.pose;
    NeckAcquisitionFrame f=packet(11,110000,35); unsigned n;
    ++groups; f.orientation.sequence=10;
    CHECK(neck_processing_process(&c,&f,110000,&o)==NECK_BAD_FRAME); all_invalid(&o);
    CHECK(o.pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    CHECK(neck_processing_expire_pose(&old,110000,o.pose.invalidation_epoch)==1);
    CHECK(old.quality.flags&NECK_Q_TRANSPORT_STALE);
    CHECK(old.posture.flexion.events==0 && !old.posture.flexion.valid);
    CHECK(!(old.posture.flags&NECK_POSTURE_INPUT_TIMEOUT));
    CHECK(!(old.nodes[1].flags&NECK_FLAG_INPUT_TIMEOUT)); /* Epoch fault is not time expiry. */
    CHECK(c.pipelines[1].reference_set); /* A metadata fault does not erase neutral. */
    for(n=12;n<=23;++n) {
        o=step(&c,n,35);
        if(n<23) CHECK(o.pose.posture.flexion.alarm_level!=NECK_POSTURE_WARNING);
    }
    CHECK(o.pose.posture.flexion.events==NECK_EVENT_TRIGGER);
    old=o.pose; CHECK(neck_processing_expire_pose(&old,260000,o.pose.invalidation_epoch)==0);
    CHECK(neck_processing_expire_pose(&old,260001,o.pose.invalidation_epoch)==1);
}
static void test_partial_runtime_faults(void)
{
    NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c);
    NeckAcquisitionFrame f=packet(11,110000,35);
    ++groups; f.orientation.nodes[1].quat_present=0;
    CHECK(neck_processing_process(&c,&f,110000,&o)==NECK_OK);
    CHECK(o.pose.posture.flexion.valid && o.pose.posture.flexion.alarm_level==NECK_POSTURE_WARNING);
    CHECK(!(o.relative.pairs[1].valid_mask&1) && !(o.relative.pairs[2].valid_mask&1));
    CHECK(o.relative.pairs[0].valid_mask&1); CHECK(o.relative.pairs[3].valid_mask&1);
    f=packet(12,120000,35); f.acceleration.nodes[1].accel_m_s2[0]=NAN;
    CHECK(neck_processing_process(&c,&f,120000,&o)==NECK_OK);
    CHECK(!o.pose.posture.flexion.valid && o.pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    CHECK(o.pose.nodes[0].valid_mask&NECK_VALID_FLEXION); CHECK(o.relative.pairs[0].valid_mask&1);
}
static void test_node_metadata_quarantine(void)
{
    unsigned kind; ++groups;
    for(kind=0;kind<9;++kind) {
        NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c);
        NeckAcquisitionFrame f=packet(11,110000,35);
        if(kind==0) f.acceleration.nodes[1].status=NECK_NODE_OK; /* Partial mask, incoherent. */
        if(kind==1) f.acceleration.nodes[1].calibration_accel=4;
        if(kind==2) f.acceleration.nodes[1].sample_time_known=2;
        if(kind==3) f.acceleration.nodes[1].accel_present=2;
        if(kind==4) f.acceleration.nodes[1].signal_valid_mask=128;
        if(kind==5) f.orientation.nodes[1].status=NECK_NODE_FAULT;
        if(kind==6) f.orientation.nodes[1].sensor_to_world_wxyz[0]=INFINITY;
        if(kind==7) f.orientation.nodes[1].sensor_to_world_wxyz[0]=1.1;
        if(kind==8) { f.acceleration.nodes[1].signal_valid_mask=7; } /* PARTIAL cannot claim all 3 signals. */
        CHECK(neck_processing_process(&c,&f,110000,&o)==NECK_OK);
        CHECK(o.pose.quality.accel_usable_mask==(kind>=5 && kind<=7?7:5));
        CHECK(o.pose.quality.orientation_usable_mask==5);
        CHECK(o.pose.nodes[0].valid_mask&NECK_VALID_FLEXION);
        CHECK(o.pose.nodes[2].valid_mask&NECK_VALID_FLEXION);
        CHECK(o.relative.pairs[0].valid_mask&NECK_REL_RAW_QUAT);
        CHECK(o.relative.pairs[3].valid_mask&NECK_REL_RAW_QUAT);
        CHECK(c.stats.pipeline_errors==0); /* Sanitized node does not break whole legacy frame. */
        if(kind>=5 && kind<=7) CHECK(o.pose.posture.flexion.alarm_level==NECK_POSTURE_WARNING);
        else {
            CHECK(o.pose.nodes[1].valid_mask==0);
            CHECK(o.pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
        }
    }
}
static void test_rejections_never_keep_alive(void)
{
    unsigned kind; ++groups;
    for(kind=0;kind<7;++kind) {
        NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c);
        NeckAcquisitionFrame f=packet(11,110000,35);
        NeckResult expected=NECK_BAD_FRAME;
        uint64_t now=110000;
        if(kind==0) f.acceleration.sequence=f.orientation.sequence=10;
        if(kind==1) f=packet(11,100000,35);
        if(kind==2) f=packet(11,110001,35);
        if(kind==3) now=150001;
        if(kind==4) { f.acceleration.config_id=f.orientation.config_id=2; expected=NECK_CONFIG_MISMATCH; }
        if(kind==5) f.acceleration.session_id=f.orientation.session_id=0;
        if(kind==6) f.acceleration.source=f.orientation.source=2;
        CHECK(neck_processing_process(&c,&f,now,&o)==expected); all_invalid(&o);
        CHECK(c.quality.previous_frame_time_us==100000 && c.quality.previous_sequence==10);
        CHECK(o.pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
        CHECK(neck_processing_poll(&c,160000,&o)==1);
        CHECK(o.pose.quality.flags&NECK_Q_TIMEOUT); CHECK(c.stats.timeout_transitions==1);
        CHECK(o.pose.posture.flexion.events==0);
    }
    {
        NeckQualityContext c; NeckQualityOutput q;
        NeckAcquisitionFrame f=packet(0,0,10); quality_fresh(&c,0,0);
        CHECK(neck_quality_process(&c,&f.acceleration,&f.orientation,0,&q)==NECK_OK);
        CHECK(neck_quality_process(&c,&f.acceleration,NULL,10000,&q)==NECK_BAD_ARGUMENT);
        CHECK(!c.nodes[0].accel_recovering && c.previous_frame_time_us==0);
        CHECK(neck_quality_poll(&c,30001,&q)==1 && (q.flags&NECK_Q_TIMEOUT_EDGE));
    }
}
static void test_downstream_validity_epoch(void)
{
    NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c); NeckPoseBatch old=o.pose;
    NeckAcquisitionFrame f=packet(11,115000,35); ++groups;
    f.orientation.nodes[0].sample_time_us=100001; /* New but pair skew >10 ms. */
    CHECK(neck_processing_process(&c,&f,115000,&o)==NECK_OK);
    CHECK(o.pose.quality.state==NECK_Q_SOFTWARE_READY);
    CHECK(o.relative.pairs[0].flags&NECK_REL_EXCESSIVE_SKEW);
    CHECK(!(o.relative.pairs[0].valid_mask&NECK_REL_RAW_QUAT));
    CHECK(o.relative.pairs[2].valid_mask&NECK_REL_RAW_QUAT);
    CHECK(neck_processing_expire_pose(&old,115000,o.pose.invalidation_epoch)==1);
    o=active_alarm(&c); old=o.pose; f=packet(11,110000,35);
    f.orientation.nodes[2].sensor_to_world_wxyz[0]=cos(90*rad/2);
    f.orientation.nodes[2].sensor_to_world_wxyz[2]=sin(90*rad/2);
    CHECK(neck_processing_process(&c,&f,110000,&o)==NECK_OK);
    CHECK(o.pose.quality.state==NECK_Q_SOFTWARE_READY);
    CHECK(o.relative.pairs[3].valid_mask&NECK_REL_RAW_QUAT);
    CHECK(!(o.relative.pairs[3].valid_mask&NECK_REL_RAW_EULER));
    CHECK(neck_processing_expire_pose(&old,110000,o.pose.invalidation_epoch)==1);
}
static void test_request_lifecycle(void)
{
    NeckProcessingContext c; NeckProcessingParameters p; NeckProcessingOutput o; unsigned n;
    ++groups; fast_parameters(&p); p.quality.recovery_us=20000;
    CHECK(neck_processing_init(&c,1,1,mounts,0,&p)==NECK_OK);
    CHECK(neck_processing_request_zero(&c,0,0)==NECK_OK);
    for(n=0;n<=5;++n) {
        o=step(&c,n,10);
        if(n<2) CHECK(c.request_state==NECK_REQUEST_WAIT_QUALITY && !c.pipelines[1].reference_set);
        if(n<4) CHECK(!c.relative.reference_set);
        if(n==4) CHECK(c.stats.node_commits==3 && c.stats.relative_commits==1 && o.relative.reference_id==0);
    }
    CHECK(o.relative.reference_id==1 && (o.relative.pairs[0].valid_mask&NECK_REL_CORRECTED_QUAT));
    CHECK(o.pose.posture.flexion.valid && c.pending_reference_id==0);
    CHECK(neck_processing_request_zero(&c,50000,0)==NECK_OK);
    neck_processing_cancel_zero(&c); CHECK(!c.pending_reference_id && c.relative.reference.reference_id==1);
    runtime_fresh(&c,0); CHECK(neck_processing_request_zero(&c,0,0)==NECK_OK);
    CHECK(neck_processing_poll(&c,40000,&o)==1 && c.request_state==NECK_REQUEST_WAIT_QUALITY);
    CHECK(neck_processing_poll(&c,10000000,&o)==1);
    CHECK(!c.pending_reference_id && o.pose.zero_status[1]==NECK_ZERO_TIMED_OUT);
    CHECK(c.stats.relative_aborts==1 && c.stats.timeout_transitions==1);
    runtime_fresh(&c,0); CHECK(neck_processing_request_zero(&c,0,0)==NECK_OK);
    o=step(&c,0,10); CHECK(c.request_state==NECK_REQUEST_COLLECTING);
    CHECK(neck_processing_poll(&c,30001,&o)==1 && !c.pending_reference_id);
    CHECK(o.pose.zero_status[1]==NECK_ZERO_FAILED_INPUT);
    runtime_fresh(&c,0); c.next_reference_id=UINT32_MAX;
    CHECK(neck_processing_request_zero(&c,0,0)==NECK_OK && c.pending_reference_id==UINT32_MAX);
    CHECK(neck_processing_request_zero(&c,0,0)==NECK_BAD_REFERENCE);
}
static void test_stream_change_clock_fault(void)
{
    NeckProcessingContext c; NeckProcessingOutput o=active_alarm(&c); NeckAcquisitionFrame f=packet(11,110000,35);
    ++groups; f.acceleration.source=f.orientation.source=1;
    CHECK(neck_processing_process(&c,&f,110000,&o)==NECK_BAD_FRAME);
    CHECK(!c.pipelines[1].reference_set && !c.relative.reference_set);
    CHECK(o.pose.posture.flexion.events==(NECK_EVENT_CLEAR|NECK_EVENT_INVALIDATED));
    CHECK(neck_processing_reset_stream(&c,110000,&o)==NECK_OK);
    f=packet(0,120000,35); f.acceleration.source=f.orientation.source=1;
    CHECK(neck_processing_process(&c,&f,120000,&o)==NECK_OK && !o.pose.posture.flexion.valid);
    CHECK(neck_processing_request_zero(&c,120000,0)==NECK_SIM_REFERENCE_ON_HARDWARE);
    o=active_alarm(&c); CHECK(neck_processing_poll(&c,99999,&o)==1);
    CHECK(o.pose.quality.flags&NECK_Q_CLOCK_FAULT); all_invalid(&o);
    f=packet(11,110000,35); CHECK(neck_processing_process(&c,&f,110000,&o)==NECK_BAD_FRAME);
    CHECK(neck_processing_reset_stream(&c,110000,&o)==NECK_OK);
    f=packet(0,120000,35); CHECK(neck_processing_process(&c,&f,120000,&o)==NECK_OK);
    CHECK(!c.pipelines[1].reference_set);
}
static void test_tick_wrap_large_time(void)
{
    NeckTickClock t; uint64_t now; NeckProcessingContext c; NeckProcessingOutput o; NeckAcquisitionFrame f;
    uint64_t base=(UINT64_C(1)<<63)+13; ++groups;
    CHECK(neck_clock_init(&t,UINT32_MAX-2,1000)==NECK_OK);
    CHECK(neck_clock_update(&t,1,&now)==NECK_OK && now==4000);
    CHECK(neck_clock_update(&t,1,&now)==NECK_OK && now==4000);
    t.elapsed_us=UINT64_MAX-999;
    CHECK(neck_clock_update(&t,2,&now)==NECK_BAD_FRAME && t.fault);
    CHECK(neck_clock_update(&t,2,&now)==NECK_BAD_FRAME);
    CHECK(neck_clock_init(NULL,0,1)==NECK_BAD_ARGUMENT);
    CHECK(neck_clock_init(&t,0,0)==NECK_BAD_CONFIG);
    runtime_fresh(&c,base); f=packet(0,base,35);
    CHECK(neck_processing_process(&c,&f,base,&o)==NECK_OK);
    CHECK(o.pose.valid_until_us==base+30000);
    CHECK(neck_processing_poll(&c,base+30000,&o)==0);
    CHECK(neck_processing_poll(&c,base+30001,&o)==1);
    runtime_fresh(&c,UINT64_MAX-15000); f=packet(0,UINT64_MAX-15000,35);
    CHECK(neck_processing_process(&c,&f,UINT64_MAX-15000,&o)==NECK_OK);
    CHECK(o.pose.valid_until_us==UINT64_MAX);
}
static void test_unobservable_linear_acceleration(void)
{
    NeckQualityContext c; NeckQualityParameters p; NeckQualityOutput q; NeckAcquisitionFrame f=packet(0,0,0);
    unsigned i; ++groups; neck_quality_default_parameters(&p); p.gravity_m_s2=10; p.recovery_us=0;
    CHECK(neck_quality_init(&c,1,1,0,&p)==NECK_OK);
    for(i=0;i<3;++i) { f.acceleration.nodes[i].accel_m_s2[0]=6; f.acceleration.nodes[i].accel_m_s2[2]=8; }
    q=qstep(&c,&f,0); CHECK(q.accel_usable_mask==7 && (q.flags&NECK_Q_ENGINEERING_ONLY));
    /* Norm alone cannot distinguish this vector from tilted gravity. This is
     * an explicit limitation test, NOT a claim that physical quality passed. */
}
static void test_default_end_to_end(const char *path)
{
    NeckProcessingParameters p; NeckProcessingContext c; NeckProcessingOutput o;
    FILE *csv=fopen(path,"w"); unsigned n,i,clears=0,triggers=0,unknown_polls=0;
    ++groups; CHECK(csv!=NULL); neck_processing_default_parameters(&p);
    CHECK(neck_processing_init(&c,1,1,neck_simulator_mounts(),0,&p)==NECK_OK);
    CHECK(neck_processing_request_zero(&c,0,0)==NECK_OK);
    fprintf(csv,"sequence,frame_us,decision_us,step,qflags,accel_mask,quat_mask,C4_flags,C4_valid,C4_alarm,C4_events,warning_us,reference_id,request_state,epoch\n");
    for(n=0;n<=6500;++n) {
        uint64_t now=(uint64_t)n*10000; NeckAcquisitionFrame f; int have=!(n>=3200 && n<3250);
        if(neck_processing_poll(&c,now,&o)) {
            ++unknown_polls;
            if(o.pose.posture.flexion.events&NECK_EVENT_CLEAR) ++clears;
            CHECK(!o.pose.posture.flexion.valid && o.relative.pairs[0].valid_mask==0);
            fprintf(csv,"%llu,%llu,%llu,POLL,%u,%u,%u,%u,%u,%u,%u,%llu,%u,%u,%u\n",
                (unsigned long long)o.pose.quality.sequence,(unsigned long long)o.pose.quality.frame_time_us,
                (unsigned long long)now,o.pose.quality.flags,(unsigned)o.pose.quality.accel_usable_mask,
                (unsigned)o.pose.quality.orientation_usable_mask,o.pose.posture.flags,(unsigned)o.pose.posture.flexion.valid,
                (unsigned)o.pose.posture.flexion.alarm_level,o.pose.posture.flexion.events,
                (unsigned long long)o.pose.posture.flexion.warning_elapsed_us,o.relative.reference_id,
                (unsigned)o.pose.zero_request_state,o.pose.invalidation_epoch);
        }
        if(!have) continue;
        neck_simulator_frame(&f,n,now);
        if(n==3600) f.acceleration.nodes[1].accel_m_s2[0]=NAN;
        if(n==3800) f.orientation.nodes[0].quat_present=0;
        CHECK(neck_processing_process(&c,&f,now,&o)==NECK_OK);
        if(o.pose.posture.flexion.events&NECK_EVENT_CLEAR) ++clears;
        if(o.pose.posture.flexion.events&NECK_EVENT_TRIGGER) ++triggers;
        if(n==310) CHECK(c.stats.relative_commits==1 && o.relative.reference_id==0);
        if(n==311) CHECK(o.relative.reference_id==1);
        if(n==3199) CHECK(o.pose.posture.flexion.alarm_level==NECK_POSTURE_DANGER);
        if(n==3250) CHECK(!o.pose.posture.flexion.valid && (o.pose.quality.flags&NECK_Q_RECOVERY_EDGE));
        if(n==3800) CHECK(o.pose.posture.flexion.valid && !(o.relative.pairs[0].valid_mask&1));
        for(i=0;i<3;++i) CHECK(o.pose.nodes[i].flags&NECK_FLAG_QUALITY_NOT_EVALUATED);
        CHECK(o.relative.flags&NECK_REL_THRESHOLDS_UNSET);
        fprintf(csv,"%llu,%llu,%llu,FRAME,%u,%u,%u,%u,%u,%u,%u,%llu,%u,%u,%u\n",
            (unsigned long long)o.pose.quality.sequence,(unsigned long long)o.pose.quality.frame_time_us,
            (unsigned long long)now,o.pose.quality.flags,(unsigned)o.pose.quality.accel_usable_mask,
            (unsigned)o.pose.quality.orientation_usable_mask,o.pose.posture.flags,(unsigned)o.pose.posture.flexion.valid,
            (unsigned)o.pose.posture.flexion.alarm_level,o.pose.posture.flexion.events,
            (unsigned long long)o.pose.posture.flexion.warning_elapsed_us,o.relative.reference_id,
            (unsigned)o.pose.zero_request_state,o.pose.invalidation_epoch);
    }
    CHECK(fclose(csv)==0); CHECK(c.stats.timeout_transitions==1);
    CHECK(c.stats.relative_commits==1 && c.stats.node_commits==3 && unknown_polls>0);
    CHECK(triggers==1 && clears==1);
    printf("S07 default integration: 6451 input frames, 0.50 s silence, %u UNKNOWN polls, %u trigger, %u invalidated clear\n",
        unknown_polls,triggers,clears);
}
int main(int argc,char **argv)
{
    CHECK(argc==2); test_parameters(); test_recovery_and_hysteresis(); test_numeric_and_metadata_faults();
    test_time_endpoints_and_repetition(); test_provenance_and_calibration(); test_wall_timeout_active_alarm();
    test_rejected_input_and_cached_snapshot(); test_partial_runtime_faults(); test_request_lifecycle();
    test_node_metadata_quarantine(); test_rejections_never_keep_alive(); test_downstream_validity_epoch();
    test_stream_change_clock_fault(); test_tick_wrap_large_time(); test_unobservable_linear_acceleration();
    test_default_end_to_end(argv[1]);
    printf("PASS: %u S07 groups, %u C assertions; no Python runtime\n",groups,checks); return 0;
}
