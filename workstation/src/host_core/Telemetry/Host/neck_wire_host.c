#include "neck_wire_host.h"
#include "neck_processing.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
typedef struct { char *text; size_t capacity,used; int failed; } Csv;
static void u(Csv *c,uint64_t v)
{
    int n; if(c->failed) return;
    n=snprintf(c->text+c->used,c->capacity-c->used,",%" PRIu64,v);
    if(n<0 || (size_t)n>=c->capacity-c->used) c->failed=1; else c->used+=(size_t)n;
}
static void s(Csv *c,int32_t v)
{
    int n; if(c->failed) return;
    n=snprintf(c->text+c->used,c->capacity-c->used,",%" PRId32,v);
    if(n<0 || (size_t)n>=c->capacity-c->used) c->failed=1; else c->used+=(size_t)n;
}
int neck_wire_csv(const NeckWireMessage *m,char *output,size_t capacity)
{
    Csv c; unsigned i,j; if(!m || !output || capacity<3) return 0;
    c.text=output; c.capacity=capacity; c.used=2; c.failed=0; memcpy(output,"W1",3);
    u(&c,m->header.type); u(&c,m->header.tx_sequence); u(&c,m->header.boot_id);
    u(&c,m->header.session_id); u(&c,m->header.config_id); u(&c,m->header.emit_time_us);
    switch(m->header.type) {
    case NECK_WIRE_COMMAND: case NECK_WIRE_REPLY:
        u(&c,m->body.control.request_id); u(&c,m->body.control.opcode); u(&c,m->body.control.result);
        for(i=0;i<4;++i) u(&c,m->body.control.args[i]);
        break;
    case NECK_WIRE_SESSION: {
        const NeckWireSession *p=&m->body.session;
        u(&c,p->profile_id); u(&c,p->firmware_id); u(&c,p->world_frame_id); u(&c,p->source);
        u(&c,p->mode); u(&c,p->node_mask); u(&c,p->reference_origin); u(&c,p->sample_period_us);
        u(&c,p->quality_timeout_us); u(&c,p->cutoff_millihz); u(&c,p->continuous_duration_us);
        for(i=0;i<4;++i) s(&c,p->thresholds_mdeg[i]);
        u(&c,p->max_node_skew_us); u(&c,p->c4_period_us); u(&c,p->relative_period_us);
        u(&c,p->status_period_us); u(&c,p->capabilities); break;
    }
    case NECK_WIRE_C4: {
        const NeckWireC4 *p=&m->body.c4;
        u(&c,p->input_sequence); u(&c,p->frame_time_us); u(&c,p->decision_time_us); u(&c,p->valid_until_us);
        u(&c,p->invalidation_epoch); u(&c,p->reference_id); u(&c,p->quality_flags); u(&c,p->filter_flags);
        u(&c,p->posture_flags); u(&c,p->valid_mask); u(&c,p->time_flags); u(&c,p->zero_status);
        for(i=0;i<6;++i) s(&c,p->angles_mdeg[i]);
        for(i=0;i<2;++i) { const NeckWireAxis *a=&p->axes[i]; u(&c,a->valid); u(&c,a->instant_level);
            u(&c,a->alarm_level); s(&c,a->direction); u(&c,a->warning_elapsed_us);
            u(&c,a->danger_elapsed_us); u(&c,a->extension_elapsed_us); } break;
    }
    case NECK_WIRE_RELATIVE: {
        const NeckWireRelative *p=&m->body.relative;
        u(&c,p->input_sequence); u(&c,p->frame_time_us); u(&c,p->decision_time_us); u(&c,p->valid_until_us);
        u(&c,p->invalidation_epoch); u(&c,p->reference_id); u(&c,p->quality_flags); u(&c,p->world_frame_id); u(&c,p->time_flags);
        for(i=0;i<4;++i) { const NeckWirePair *r=&p->pairs[i]; u(&c,r->valid_mask); u(&c,r->status); u(&c,r->flags);
            for(j=0;j<3;++j) s(&c,r->raw_mdeg[j]);
            for(j=0;j<3;++j) s(&c,r->corrected_mdeg[j]);
            u(&c,r->sample_skew_us); } break;
    }
    case NECK_WIRE_EVENT: {
        const NeckWireEvent *p=&m->body.event;
        u(&c,p->event_id); u(&c,p->input_sequence); u(&c,p->frame_time_us); u(&c,p->event_time_us);
        u(&c,p->reference_id); u(&c,p->invalidation_epoch); u(&c,p->events); u(&c,p->posture_flags);
        u(&c,p->axis); u(&c,p->previous_level); u(&c,p->level); s(&c,p->direction); u(&c,p->time_flags); break;
    }
    case NECK_WIRE_STATUS: {
        const NeckWireStatus *p=&m->body.status;
        u(&c,p->input_sequence); u(&c,p->frame_time_us); u(&c,p->decision_time_us);
        u(&c,p->invalidation_epoch); u(&c,p->quality_flags); u(&c,p->accel_usable_mask);
        u(&c,p->orientation_usable_mask); u(&c,p->quality_state); u(&c,p->frame_accepted);
        u(&c,p->pending_reference_id); u(&c,p->zero_request_state); u(&c,p->input_drops);
        u(&c,p->telemetry_drops); u(&c,p->event_drops); u(&c,p->uart_errors);
        for(i=0;i<3;++i) { const NeckWireNode *n=&p->nodes[i]; u(&c,n->accel_sample_time_us);
            u(&c,n->orientation_sample_time_us); u(&c,n->quality_flags); u(&c,n->filter_flags); u(&c,n->reference_id);
            u(&c,n->status); u(&c,n->accel_calibration); u(&c,n->system_calibration); u(&c,n->time_flags); u(&c,n->zero_status); } break;
    }
    default: return 0;
    }
    return c.failed?0:(int)c.used;
}
typedef struct { uint32_t boot,session,config; } Identity;
typedef struct {
    NeckWireParser parser; NeckWireReceiver receiver; NeckWireMessage session;
    NeckProcessingContext raw_core; NeckProcessingParameters raw_params;
    NeckAcquisitionFrame raw_input;
    uint64_t event_id,last_c4_us,last_rel_us,last_status_us;
    double rotations[6][4]; /* sensor->body, then local world->common world */
    uint32_t settings[6], last_zero_command;
    int has_profile,registered,raw_active,raw_had_output;
    Identity retired[16]; unsigned retired_count,retired_next;
    uint64_t accepted,rejected,arrival; int has_session;
    NeckHostCallback callback; void *user;
} Host;
int NECK_HOST_CALL neck_host_profile(void *handle,const double *q,const uint32_t *s,int registered)
{
    Host *h=(Host *)handle;
    unsigned i;
    if (!h || !q || !s || h->has_session || (registered!=0 && registered!=1)) return 0;
    for(i=0;i<6;++i) {
        const double *v=q+4*i;
        double norm=hypot(hypot(v[0],v[1]),hypot(v[2],v[3]));
        if(!isfinite(norm) || fabs(norm-1)>0.0001) return 0;
    }
    if(s[0]<100 || s[0]>20000 || s[1]<100 || s[1]>120000 ||
       !s[2] || s[2]>=s[3] || s[3]>90000 || !s[4] || s[4]>=s[5] || s[5]>90000) return 0;
    memcpy(h->rotations,q,sizeof(h->rotations));
    memcpy(h->settings,s,sizeof(h->settings));
    h->registered=registered; h->has_profile=1;
    return 1;
}
static void align_world(const double a[4],double q[4])
{
    double w=q[0],x=q[1],y=q[2],z=q[3];
    q[0]=a[0]*w-a[1]*x-a[2]*y-a[3]*z;
    q[1]=a[0]*x+a[1]*w+a[2]*z-a[3]*y;
    q[2]=a[0]*y-a[1]*z+a[2]*w+a[3]*x;
    q[3]=a[0]*z+a[1]*y-a[2]*x+a[3]*w;
}
static int identity_equal(const Identity *id,const NeckWireHeader *h)
{ return id->boot==h->boot_id && id->session==h->session_id && id->config==h->config_id; }
static void host_angle(int32_t *v,uint16_t *mask,uint16_t bit,double value,int valid)
{ if(valid && isfinite(value) && value>=-360.0 && value<=360.0) { *v=(int32_t)llround(value*1000.0); *mask|=bit; } }
static void host_axis(NeckWireAxis *w,const NeckPostureAxisOutput *a)
{
    w->valid=a->valid; w->instant_level=a->instant_level; w->alarm_level=a->alarm_level; w->direction=a->direction;
    w->warning_elapsed_us=a->warning_elapsed_us; w->danger_elapsed_us=a->danger_elapsed_us; w->extension_elapsed_us=a->extension_elapsed_us;
}
static void emit_synthetic(Host *h,const NeckWireMessage *raw_msg,const NeckProcessingOutput *o)
{
    NeckWireMessage synth; char row[2048]; unsigned i,j;
    uint64_t now=raw_msg->header.emit_time_us;
    const NeckWireSession *sess=&h->session.body.session;
    const NeckQualityOutput *q=&o->pose.quality;
    const NeckPostureAxisOutput *axes[2]={&o->pose.posture.flexion,&o->pose.posture.lateral};
    if(!h->callback) return;
    for(i=0;i<2;++i) if(axes[i]->events) {
        NeckWireEvent *e; memset(&synth,0,sizeof(synth));
        synth.header=raw_msg->header; synth.header.type=NECK_WIRE_EVENT; e=&synth.body.event;
        e->event_id=++h->event_id; e->input_sequence=o->pose.posture.sequence;
        e->frame_time_us=o->pose.posture.frame_time_us; e->event_time_us=o->pose.posture.time_us;
        e->reference_id=o->pose.posture.reference_id; e->invalidation_epoch=o->pose.invalidation_epoch;
        e->events=axes[i]->events; e->posture_flags=o->pose.posture.flags;
        e->axis=(uint8_t)i; e->previous_level=axes[i]->previous_alarm_level;
        e->level=axes[i]->alarm_level; e->direction=axes[i]->direction; e->time_flags=3;
        if(neck_wire_csv(&synth,row,sizeof(row))) h->callback(NECK_WIRE_RX_DATA,0,h->arrival,row,h->user);
    }
    if(!h->raw_had_output || now-h->last_c4_us>=sess->c4_period_us) {
        NeckWireC4 *c; const NeckTiltEstimate *raw=&o->c4.raw,*f=&o->c4.filtered;
        memset(&synth,0,sizeof(synth)); synth.header=raw_msg->header; synth.header.type=NECK_WIRE_C4; c=&synth.body.c4;
        c->input_sequence=q->sequence; c->frame_time_us=q->frame_time_us; c->decision_time_us=q->decision_time_us;
        c->valid_until_us=o->pose.valid_until_us; c->invalidation_epoch=o->pose.invalidation_epoch; c->reference_id=f->reference_id;
        c->quality_flags=q->flags; c->filter_flags=o->c4.filter_flags; c->posture_flags=o->pose.posture.flags;
        c->time_flags=(uint8_t)((q->flags&NECK_Q_NO_FRAME)?6:7); c->zero_status=o->c4.zero.status;
        host_angle(&c->angles_mdeg[0],&c->valid_mask,1,raw->pitch_raw_deg,raw->valid_mask&NECK_VALID_PITCH_RAW);
        host_angle(&c->angles_mdeg[1],&c->valid_mask,2,raw->lateral_raw_deg,raw->valid_mask&NECK_VALID_LATERAL_RAW);
        host_angle(&c->angles_mdeg[2],&c->valid_mask,4,f->pitch_raw_deg,f->valid_mask&NECK_VALID_PITCH_RAW);
        host_angle(&c->angles_mdeg[3],&c->valid_mask,8,f->lateral_raw_deg,f->valid_mask&NECK_VALID_LATERAL_RAW);
        host_angle(&c->angles_mdeg[4],&c->valid_mask,16,f->flexion_deg,f->valid_mask&NECK_VALID_FLEXION);
        host_angle(&c->angles_mdeg[5],&c->valid_mask,32,f->right_lateral_deg,f->valid_mask&NECK_VALID_RIGHT_LATERAL);
        host_axis(&c->axes[0],axes[0]); host_axis(&c->axes[1],axes[1]);
        if(!(c->valid_mask&16)) memset(&c->axes[0],0,sizeof(c->axes[0]));
        if(!(c->valid_mask&32)) memset(&c->axes[1],0,sizeof(c->axes[1]));
        if(neck_wire_csv(&synth,row,sizeof(row))) h->callback(NECK_WIRE_RX_DATA,0,h->arrival,row,h->user);
        h->last_c4_us=now;
    }
    if((sess->capabilities&NECK_WIRE_CAP_RELATIVE) && (!h->raw_had_output || now-h->last_rel_us>=sess->relative_period_us)) {
        NeckWireRelative *r; memset(&synth,0,sizeof(synth));
        synth.header=raw_msg->header; synth.header.type=NECK_WIRE_RELATIVE; r=&synth.body.relative;
        r->input_sequence=q->sequence; r->frame_time_us=q->frame_time_us; r->decision_time_us=q->decision_time_us;
        r->valid_until_us=o->pose.valid_until_us; r->invalidation_epoch=o->pose.invalidation_epoch;
        r->reference_id=o->relative.reference_id; r->quality_flags=q->flags; r->world_frame_id=o->relative.world_frame_id;
        r->time_flags=(uint8_t)((q->flags&NECK_Q_NO_FRAME)?6:7);
        for(i=0;i<4;++i) {
            const NeckRelativePairOutput *p=&o->relative.pairs[i]; NeckWirePair *w=&r->pairs[i];
            w->flags=p->flags; w->status=p->status; w->sample_skew_us=p->sample_skew_us;
            if(p->valid_mask&NECK_REL_KNOWN_SKEW) w->valid_mask|=4;
            for(j=0;j<3;++j) {
                w->raw_mdeg[j]=(int32_t)llround(p->raw_euler_deg[j]*1000.0);
                w->corrected_mdeg[j]=(int32_t)llround(p->corrected_euler_deg[j]*1000.0);
            }
            if(p->valid_mask&NECK_REL_RAW_EULER) w->valid_mask|=1;
            if(p->valid_mask&NECK_REL_CORRECTED_EULER) w->valid_mask|=2;
        }
        if(neck_wire_csv(&synth,row,sizeof(row))) h->callback(NECK_WIRE_RX_DATA,0,h->arrival,row,h->user);
        h->last_rel_us=now;
    }
    if((sess->capabilities&NECK_WIRE_CAP_STATUS) && (!h->raw_had_output || now-h->last_status_us>=sess->status_period_us)) {
        NeckWireStatus *s; memset(&synth,0,sizeof(synth));
        synth.header=raw_msg->header; synth.header.type=NECK_WIRE_STATUS; s=&synth.body.status;
        s->input_sequence=q->sequence; s->frame_time_us=q->frame_time_us; s->decision_time_us=q->decision_time_us;
        s->invalidation_epoch=o->pose.invalidation_epoch; s->quality_flags=q->flags;
        s->accel_usable_mask=q->accel_usable_mask; s->orientation_usable_mask=q->orientation_usable_mask;
        s->quality_state=q->state; s->frame_accepted=q->frame_accepted;
        s->pending_reference_id=o->pose.pending_reference_id; s->zero_request_state=o->pose.zero_request_state;
        for(i=0;i<3;++i) {
            NeckWireNode *n=&s->nodes[i]; const NeckNodeSample *a=&h->raw_input.acceleration.nodes[i];
            const NeckOrientationSample *b=&h->raw_input.orientation.nodes[i];
            n->accel_sample_time_us=a->sample_time_us; n->orientation_sample_time_us=b->sample_time_us;
            n->time_flags=(uint8_t)((a->sample_time_known?1:0)|(b->sample_time_known?2:0));
            n->quality_flags=q->node_flags[i]; n->filter_flags=o->pose.filter_flags[i];
            n->reference_id=o->pose.nodes[i].reference_id;
            n->status=(q->flags&NECK_Q_TIMEOUT)?NECK_NODE_STALE:a->status;
            n->accel_calibration=a->calibration_accel; n->system_calibration=b->calibration_system;
            n->zero_status=o->pose.zero_status[i];
        }
        if(neck_wire_csv(&synth,row,sizeof(row))) h->callback(NECK_WIRE_RX_DATA,0,h->arrival,row,h->user);
        h->last_status_us=now;
    }
    h->raw_had_output=1;
}
static void received(const NeckWireMessage *m,void *user)
{
    Host *h=(Host *)user; NeckWireReceiveResult result; int boundary=0; unsigned i;
    char row[2048]; result=neck_wire_receiver_accept(&h->receiver,m,h->arrival);
    if(result==NECK_WIRE_RX_BOUNDARY_REQUIRED) {
        int old=0,same=m->header.boot_id==h->receiver.boot_id && m->header.session_id==h->receiver.session_id &&
            m->header.config_id==h->receiver.config_id;
        for(i=0;i<h->retired_count;++i) if(identity_equal(&h->retired[i],&m->header)) old=1;
        if(!old && !same) {
            Identity *id=&h->retired[h->retired_next]; id->boot=h->receiver.boot_id;
            id->session=h->receiver.session_id; id->config=h->receiver.config_id;
            h->retired_next=(h->retired_next+1)%16; if(h->retired_count<16) ++h->retired_count;
            neck_wire_receiver_disconnect(&h->receiver); result=neck_wire_receiver_accept(&h->receiver,m,h->arrival); boundary=1;
        }
    }
    if(result==NECK_WIRE_RX_DATA || result==NECK_WIRE_RX_SESSION) ++h->accepted; else ++h->rejected;
    if(result==NECK_WIRE_RX_SESSION) {
        h->session=*m; h->has_session=1;
        if(m->body.session.capabilities & NECK_WIRE_CAP_RAW_FRAME) {
            if(!h->raw_active || boundary) {
                static const double kIdentityMounts[3][4]={{1,0,0,0},{1,0,0,0},{1,0,0,0}};
                const NeckWireSession *s=&m->body.session;
                neck_processing_default_parameters(&h->raw_params);
                h->raw_params.pipeline.nominal_period_us=s->sample_period_us;
                h->raw_params.pipeline.max_gap_us=s->quality_timeout_us;
                h->raw_params.pipeline.zero_min_samples=(s->sample_period_us>0 && s->sample_period_us<=250000u)?
                    (uint32_t)(1000000ULL/s->sample_period_us):4u;
                h->raw_params.pipeline.cutoff_hz=(double)s->cutoff_millihz/1000.0;
                h->raw_params.quality.max_frame_age_us=s->quality_timeout_us;
                h->raw_params.quality.max_sample_age_us=s->quality_timeout_us;
                h->raw_params.quality.max_gap_us=s->quality_timeout_us;
                h->raw_params.quality.no_frame_timeout_us=s->quality_timeout_us;
                h->raw_params.posture.continuous_duration_us=s->continuous_duration_us;
                h->raw_params.posture.max_gap_us=s->quality_timeout_us;
                h->raw_params.posture.flexion_warning_deg=(double)s->thresholds_mdeg[0]/1000.0;
                h->raw_params.posture.flexion_danger_deg=(double)s->thresholds_mdeg[1]/1000.0;
                h->raw_params.posture.lateral_warning_deg=(double)s->thresholds_mdeg[2]/1000.0;
                h->raw_params.posture.lateral_danger_deg=(double)s->thresholds_mdeg[3]/1000.0;
                h->raw_params.relative.stale_after_us=s->quality_timeout_us;
                h->raw_params.relative.max_node_skew_us=s->max_node_skew_us;
                if(h->has_profile) {
                    h->raw_params.pipeline.cutoff_hz=h->settings[0]/1000.0;
                    h->raw_params.posture.continuous_duration_us=(uint64_t)h->settings[1]*1000;
                    h->raw_params.posture.flexion_warning_deg=h->settings[2]/1000.0;
                    h->raw_params.posture.flexion_danger_deg=h->settings[3]/1000.0;
                    h->raw_params.posture.lateral_warning_deg=h->settings[4]/1000.0;
                    h->raw_params.posture.lateral_danger_deg=h->settings[5]/1000.0;
                }
                if(neck_processing_init(&h->raw_core,m->header.config_id,
                    s->world_frame_id?s->world_frame_id:1u,h->has_profile?(const double (*)[4])h->rotations:kIdentityMounts,
                    m->header.emit_time_us,&h->raw_params)==NECK_OK) {
                    h->raw_active=1; h->raw_had_output=0;
                    h->last_c4_us=h->last_rel_us=h->last_status_us=0;
                    h->last_zero_command=0;
                }
            }
        } else {
            h->raw_active=0;
        }
    }
    if(result==NECK_WIRE_RX_DATA && h->raw_active) {
        if(m->header.type==NECK_WIRE_REPLY && m->body.control.result==0) {
            if(m->body.control.opcode==2u || m->body.control.opcode==3u) {
                if(m->body.control.request_id<=h->last_zero_command) goto publish;
                h->last_zero_command=m->body.control.request_id;
            }
            if(m->body.control.opcode==2u) (void)neck_processing_request_zero(&h->raw_core,m->header.emit_time_us,NECK_REFERENCE_EXTERNAL);
            else if(m->body.control.opcode==3u) neck_processing_cancel_zero(&h->raw_core);
        } else if(m->header.type==NECK_WIRE_RAW_FRAME) {
            NeckProcessingOutput proc_out;
            uint64_t now_us=m->header.emit_time_us;
            memset(&h->raw_input,0,sizeof(h->raw_input));
            h->raw_input.acceleration.config_id=m->header.config_id;
            h->raw_input.acceleration.session_id=m->header.session_id;
            h->raw_input.acceleration.sequence=m->body.raw.sequence;
            h->raw_input.acceleration.frame_time_us=now_us;
            h->raw_input.acceleration.source=h->session.body.session.source;
            h->raw_input.orientation.config_id=m->header.config_id;
            h->raw_input.orientation.session_id=m->header.session_id;
            h->raw_input.orientation.sequence=m->body.raw.sequence;
            h->raw_input.orientation.frame_time_us=now_us;
            h->raw_input.orientation.source=h->session.body.session.source;
            for(i=0;i<3;++i) {
                const NeckWireRawNode *rn=&m->body.raw.nodes[i];
                NeckNodeSample *a=&h->raw_input.acceleration.nodes[i];
                NeckOrientationSample *b=&h->raw_input.orientation.nodes[i];
                int ok=((rn->flags&1u)!=0) && ((rn->flags&4u)==0);
                uint64_t sample_us;
                uint32_t age=(uint32_t)now_us-rn->timestamp_mid_us;
                uint32_t span=rn->timestamp_end_us-rn->timestamp_start_us;
                int trusted_window=age<=now_us && age<=h->raw_params.quality.max_sample_age_us &&
                    span<=h->raw_params.quality.max_sample_age_us &&
                    rn->timestamp_mid_us==rn->timestamp_start_us+span/2u;
                sample_us=age<=now_us?now_us-age:0;
                if(h->has_profile && !h->registered) ok=0;
                a->status=b->status=(uint8_t)(ok?NECK_NODE_OK:NECK_NODE_FAULT);
                a->signal_valid_mask=(uint8_t)(ok?(NECK_SIGNAL_QUATERNION|NECK_SIGNAL_ACCEL|NECK_SIGNAL_GYRO):0u);
                a->accel_present=(uint8_t)(ok?1u:0u);
                b->quat_present=(uint8_t)(ok?1u:0u);
                a->sample_time_us=b->sample_time_us=sample_us;
                a->sample_time_known=b->sample_time_known=(uint8_t)(trusted_window && !(rn->flags&2u));
                a->accel_m_s2[0]=(double)rn->accel_xyz[0]/100.0;
                a->accel_m_s2[1]=(double)rn->accel_xyz[1]/100.0;
                a->accel_m_s2[2]=(double)rn->accel_xyz[2]/100.0;
                a->calibration_accel=(uint8_t)((rn->calibration_status>>2)&0x03u);
                b->sensor_to_world_wxyz[0]=(double)rn->quat_wxyz[0]/16384.0;
                b->sensor_to_world_wxyz[1]=(double)rn->quat_wxyz[1]/16384.0;
                b->sensor_to_world_wxyz[2]=(double)rn->quat_wxyz[2]/16384.0;
                b->sensor_to_world_wxyz[3]=(double)rn->quat_wxyz[3]/16384.0;
                if(h->has_profile) align_world(h->rotations[3+i],b->sensor_to_world_wxyz);
                b->calibration_system=(uint8_t)((rn->calibration_status>>6)&0x03u);
                b->world_frame_id=h->session.body.session.world_frame_id?h->session.body.session.world_frame_id:1u;
            }
            if(neck_processing_process(&h->raw_core,&h->raw_input,now_us,&proc_out)==NECK_OK) {
                h->receiver.c4_seen=1; h->receiver.c4_arrival_us=h->arrival;
                h->receiver.relative_seen=1; h->receiver.relative_arrival_us=h->arrival;
                emit_synthetic(h,m,&proc_out);
            }
            return;
        }
    }
publish:
    if(h->callback && neck_wire_csv(m,row,sizeof(row))) h->callback(result,boundary,h->arrival,row,h->user);
}
void *NECK_HOST_CALL neck_host_create(void)
{
    Host *h=(Host *)calloc(1,sizeof(Host)); if(!h) return NULL;
    neck_wire_parser_init(&h->parser); (void)neck_wire_receiver_init(&h->receiver,500000,250000); return h;
}
void NECK_HOST_CALL neck_host_destroy(void *handle) { free(handle); }
int NECK_HOST_CALL neck_host_feed(void *handle,const uint8_t *data,uint32_t n,uint64_t arrival,NeckHostCallback cb,void *user)
{
    Host *h=(Host *)handle; if(!h || (!data && n) || n>4096) return -1;
    h->arrival=arrival; h->callback=cb; h->user=user;
    return (int)neck_wire_parser_feed(&h->parser,data,n,received,h);
}
int NECK_HOST_CALL neck_host_gap(void *handle,uint64_t arrival,NeckHostCallback cb,void *user)
{
    Host *h=(Host *)handle; if(!h) return -1; h->arrival=arrival; h->callback=cb; h->user=user;
    return (int)neck_wire_parser_gap(&h->parser,received,h);
}
uint32_t NECK_HOST_CALL neck_host_poll(void *handle,uint64_t now)
{ Host *h=(Host *)handle; return neck_wire_receiver_poll(h?&h->receiver:NULL,now); }
int NECK_HOST_CALL neck_host_stats(void *handle,uint64_t *o,uint32_t n)
{
    Host *h=(Host *)handle; NeckWireParserStats *p; if(!h || !o || n<12) return 0; p=&h->parser.stats;
    o[0]=p->frames; o[1]=p->crc_errors; o[2]=p->length_errors; o[3]=p->version_errors; o[4]=p->type_errors;
    o[5]=p->payload_errors; o[6]=p->discarded_bytes; o[7]=p->partial_gaps;
    o[8]=h->accepted; o[9]=h->rejected; o[10]=h->receiver.tx_gaps; o[11]=h->receiver.bound; return 12;
}
int NECK_HOST_CALL neck_host_snapshot(void *handle,uint8_t *prefix,uint32_t cap,uint32_t *n,
    uint8_t *session,uint32_t scap,uint32_t *sn,uint32_t *tx,uint64_t *emit)
{
    Host *h=(Host *)handle; size_t written=0;
    if(!h || !prefix || !n || !session || !sn || !tx || !emit || cap<h->parser.used) return 0;
    *sn=0; *n=(uint32_t)h->parser.used; memcpy(prefix,h->parser.buffer,*n);
    *tx=h->receiver.last_tx_sequence; *emit=h->receiver.last_emit_us;
    if(h->has_session) {
        if(neck_wire_encode(&h->session,session,scap,&written)!=NECK_WIRE_OK) return 0;
        *sn=(uint32_t)written;
    }
    return 1;
}
int NECK_HOST_CALL neck_host_seed(void *handle,uint32_t tx,uint64_t emit)
{
    Host *h=(Host *)handle; if(!h || !h->receiver.bound || emit<h->receiver.last_emit_us) return 0;
    h->receiver.last_tx_sequence=tx; h->receiver.last_emit_us=emit; h->receiver.tx_gaps=0; return 1;
}
int NECK_HOST_CALL neck_host_command(uint32_t boot,uint32_t session,uint32_t config,uint64_t observed,
    uint32_t request,uint32_t opcode,const uint32_t *args,uint8_t *bytes,uint32_t capacity)
{
    NeckWireMessage m; size_t written=0;
    if(!args) return 0;
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_COMMAND; m.header.boot_id=boot;
    m.header.session_id=session; m.header.config_id=config; m.header.emit_time_us=observed;
    m.body.control.request_id=request; m.body.control.opcode=opcode;
    memcpy(m.body.control.args,args,sizeof(m.body.control.args));
    return neck_wire_encode(&m,bytes,capacity,&written)==NECK_WIRE_OK?(int)written:0;
}
