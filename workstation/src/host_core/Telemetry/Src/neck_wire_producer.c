#include "neck_wire_producer.h"
#include <math.h>
#include <string.h>

static int32_t milli(double degrees)
{
    double scaled=degrees*1000.0;
    return (int32_t)(scaled+(scaled>=0.0?0.5:-0.5));
}
static void angle(int32_t *dest, uint16_t *mask, uint16_t bit, double degrees, int valid)
{
    if(valid && isfinite(degrees) && degrees>=-360.0 && degrees<=360.0) {
        *dest=milli(degrees); *mask=(uint16_t)(*mask|bit);
    }
}
static void axis(NeckWireAxis *dest, const NeckPostureAxisOutput *source)
{
    if(!source->valid) return;
    dest->valid=source->valid; dest->instant_level=source->instant_level;
    dest->alarm_level=source->alarm_level; dest->direction=source->direction;
    dest->warning_elapsed_us=source->warning_elapsed_us;
    dest->danger_elapsed_us=source->danger_elapsed_us;
    dest->extension_elapsed_us=source->extension_elapsed_us;
}
void neck_wire_producer_session(NeckWireSession *s, const NeckProcessingParameters *p)
{
    if(s==NULL || p==NULL) return;
    memset(s,0,sizeof(*s));
    s->profile_id=NECK_WIRE_PROFILE_S09; s->firmware_id=0xd2000001u;
    s->world_frame_id=1; s->source=NECK_WIRE_SIM; s->mode=NECK_WIRE_THREE_NODES;
    s->node_mask=7; s->reference_origin=0;
    s->sample_period_us=(uint32_t)p->pipeline.nominal_period_us;
    s->quality_timeout_us=(uint32_t)p->quality.no_frame_timeout_us;
    s->cutoff_millihz=(uint32_t)milli(p->pipeline.cutoff_hz);
    s->continuous_duration_us=p->posture.continuous_duration_us;
    s->thresholds_mdeg[0]=milli(p->posture.flexion_warning_deg);
    s->thresholds_mdeg[1]=milli(p->posture.flexion_danger_deg);
    s->thresholds_mdeg[2]=milli(p->posture.lateral_warning_deg);
    s->thresholds_mdeg[3]=milli(p->posture.lateral_danger_deg);
    s->max_node_skew_us=(uint32_t)p->relative.max_node_skew_us;
    /* One data frame per 50 ms snapshot: C4 and RELATIVE alternate.
     * STATUS replaces a slot every 500 ms instead of making a stale burst. */
    s->c4_period_us=100000; s->relative_period_us=100000;
    s->status_period_us=500000; s->capabilities=15;
}
int neck_wire_producer_d3_diagnostic_session(NeckWireSession *s,
    const NeckProcessingParameters *p, uint8_t node_mask)
{
    if(s==NULL) return 0;
    memset(s,0,sizeof(*s));
    if(p==NULL || (node_mask!=2u && node_mask!=7u)) return 0;
    neck_wire_producer_session(s,p);
    s->firmware_id=0xd3000002u;
    s->source=NECK_WIRE_HARDWARE;
    s->mode=node_mask==2u?NECK_WIRE_SINGLE_C4:NECK_WIRE_THREE_NODES;
    s->node_mask=node_mask;
    s->reference_origin=255u;
    s->world_frame_id=0u;
    s->capabilities=NECK_WIRE_CAP_C4|NECK_WIRE_CAP_STATUS;
    s->relative_period_us=0u;
    return 1;
}
int neck_wire_producer_c4_session(NeckWireSession *s, const NeckProcessingParameters *p)
{
    if(!neck_wire_producer_d3_diagnostic_session(s,p,2u)) return 0;
    s->firmware_id=0xc4000001u;
    s->reference_origin=NECK_REFERENCE_EXTERNAL;
    s->capabilities|=NECK_WIRE_CAP_EVENTS;
    s->c4_period_us=NECK_TELEMETRY_PERIOD_MS*1000u;
    return 1;
}
void neck_wire_producer_snapshot(NeckWireSnapshot *dest,
    const NeckProcessingOutput *o, const NeckAcquisitionFrame *input, uint8_t input_seen)
{
    unsigned i,j; const NeckQualityOutput *q; NeckWireC4 *c; NeckWireRelative *r;
    NeckWireStatus *s; const NeckTiltEstimate *raw,*filtered;
    if(dest==NULL || o==NULL) return;
    memset(dest,0,sizeof(*dest)); q=&o->pose.quality;
    dest->session_id=q->session_id; dest->config_id=q->config_id;
    c=&dest->c4; raw=&o->c4.raw; filtered=&o->c4.filtered;
    c->input_sequence=q->sequence; c->frame_time_us=q->frame_time_us;
    c->decision_time_us=q->decision_time_us; c->valid_until_us=o->pose.valid_until_us;
    c->invalidation_epoch=o->pose.invalidation_epoch; c->reference_id=filtered->reference_id;
    c->quality_flags=q->flags; c->filter_flags=o->c4.filter_flags;
    c->posture_flags=o->pose.posture.flags;
    c->time_flags=(uint8_t)((q->flags&NECK_Q_NO_FRAME)?6u:7u);
    c->zero_status=o->c4.zero.status;
    angle(&c->angles_mdeg[0],&c->valid_mask,NECK_WIRE_RAW_FLEX,
        raw->pitch_raw_deg,raw->valid_mask&NECK_VALID_PITCH_RAW);
    angle(&c->angles_mdeg[1],&c->valid_mask,NECK_WIRE_RAW_LATERAL,
        raw->lateral_raw_deg,raw->valid_mask&NECK_VALID_LATERAL_RAW);
    angle(&c->angles_mdeg[2],&c->valid_mask,NECK_WIRE_FILTERED_FLEX,
        filtered->pitch_raw_deg,filtered->valid_mask&NECK_VALID_PITCH_RAW);
    angle(&c->angles_mdeg[3],&c->valid_mask,NECK_WIRE_FILTERED_LATERAL,
        filtered->lateral_raw_deg,filtered->valid_mask&NECK_VALID_LATERAL_RAW);
    angle(&c->angles_mdeg[4],&c->valid_mask,NECK_WIRE_CORRECTED_FLEX,
        filtered->flexion_deg,filtered->valid_mask&NECK_VALID_FLEXION);
    angle(&c->angles_mdeg[5],&c->valid_mask,NECK_WIRE_CORRECTED_LATERAL,
        filtered->right_lateral_deg,filtered->valid_mask&NECK_VALID_RIGHT_LATERAL);
    if(c->valid_mask&NECK_WIRE_CORRECTED_FLEX) axis(&c->axes[0],&o->pose.posture.flexion);
    if(c->valid_mask&NECK_WIRE_CORRECTED_LATERAL) axis(&c->axes[1],&o->pose.posture.lateral);
    if(q->flags&(NECK_Q_TIMEOUT|NECK_Q_TRANSPORT_STALE)) {
        c->valid_mask=0; memset(c->axes,0,sizeof(c->axes));
    }
    r=&dest->relative;
    r->input_sequence=q->sequence; r->frame_time_us=q->frame_time_us;
    r->decision_time_us=q->decision_time_us; r->valid_until_us=o->pose.valid_until_us;
    r->invalidation_epoch=o->pose.invalidation_epoch; r->reference_id=o->relative.reference_id;
    r->quality_flags=q->flags; r->world_frame_id=o->relative.world_frame_id;
    if(r->world_frame_id==0) r->world_frame_id=1;
    r->time_flags=(uint8_t)((q->flags&NECK_Q_NO_FRAME)?6u:7u);
    for(i=0;i<4;++i) {
        const NeckRelativePairOutput *from=&o->relative.pairs[i]; NeckWirePair *to=&r->pairs[i];
        to->flags=from->flags; to->status=from->status;
        if(to->status==NECK_REL_CORRECTED && r->reference_id==0) to->status=NECK_REL_RAW_ONLY;
        if(from->valid_mask&NECK_REL_KNOWN_SKEW) {
            to->valid_mask|=NECK_WIRE_PAIR_SKEW; to->sample_skew_us=from->sample_skew_us;
        }
        for(j=0;j<3;++j) {
            if((from->valid_mask&NECK_REL_RAW_EULER) && isfinite(from->raw_euler_deg[j]) &&
               from->raw_euler_deg[j]>=-360.0 && from->raw_euler_deg[j]<=360.0)
                to->raw_mdeg[j]=milli(from->raw_euler_deg[j]);
            else break;
        }
        if(j==3 && to->status) to->valid_mask|=NECK_WIRE_PAIR_RAW;
        for(j=0;j<3;++j) {
            if((from->valid_mask&NECK_REL_CORRECTED_EULER) && isfinite(from->corrected_euler_deg[j]) &&
               from->corrected_euler_deg[j]>=-360.0 && from->corrected_euler_deg[j]<=360.0)
                to->corrected_mdeg[j]=milli(from->corrected_euler_deg[j]);
            else break;
        }
        if(j==3 && to->status==NECK_REL_CORRECTED && r->reference_id)
            to->valid_mask|=NECK_WIRE_PAIR_CORRECTED;
        if(q->flags&(NECK_Q_TIMEOUT|NECK_Q_TRANSPORT_STALE)) to->valid_mask&=NECK_WIRE_PAIR_SKEW;
    }
    s=&dest->status;
    s->input_sequence=q->sequence; s->frame_time_us=q->frame_time_us;
    s->decision_time_us=q->decision_time_us; s->invalidation_epoch=o->pose.invalidation_epoch;
    s->quality_flags=q->flags; s->pending_reference_id=o->pose.pending_reference_id;
    s->zero_request_state=o->pose.zero_request_state;
    s->accel_usable_mask=q->accel_usable_mask;
    s->orientation_usable_mask=q->orientation_usable_mask;
    s->quality_state=q->state; s->frame_accepted=q->frame_accepted;
    if(q->flags&(NECK_Q_TIMEOUT|NECK_Q_TRANSPORT_STALE)) {
        s->accel_usable_mask=0; s->orientation_usable_mask=0;
        s->quality_state=0; s->frame_accepted=0;
    }
    for(i=0;i<3;++i) {
        NeckWireNode *n=&s->nodes[i];
        n->quality_flags=q->node_flags[i]; n->filter_flags=o->pose.filter_flags[i];
        n->reference_id=o->pose.nodes[i].reference_id; n->zero_status=o->pose.zero_status[i];
        n->accel_calibration=NECK_WIRE_UNKNOWN_CALIBRATION;
        n->system_calibration=NECK_WIRE_UNKNOWN_CALIBRATION;
        n->status=(q->flags&NECK_Q_TIMEOUT)?NECK_NODE_STALE:NECK_NODE_MISSING;
        if(input_seen && input!=NULL) {
            const NeckNodeSample *a=&input->acceleration.nodes[i];
            const NeckOrientationSample *b=&input->orientation.nodes[i];
            n->status=(q->flags&NECK_Q_TIMEOUT)?NECK_NODE_STALE:a->status;
            n->accel_calibration=a->calibration_accel;
            n->system_calibration=b->calibration_system;
            if(a->sample_time_known) { n->time_flags|=1; n->accel_sample_time_us=a->sample_time_us; }
            if(b->sample_time_known) { n->time_flags|=2; n->orientation_sample_time_us=b->sample_time_us; }
        }
    }
}
int neck_wire_producer_expire(NeckWireMessage *m, uint64_t finish, uint32_t epoch)
{
    int stale=0; unsigned i;
    if(m==NULL) return 0;
    if(m->header.type==NECK_WIRE_C4) {
        NeckWireC4 *c=&m->body.c4;
        stale=finish<c->decision_time_us || finish>c->valid_until_us || epoch!=c->invalidation_epoch;
        if(stale) {
            c->valid_mask=0; memset(c->axes,0,sizeof(c->axes));
            c->quality_flags|=NECK_Q_TRANSPORT_STALE;
        }
    } else if(m->header.type==NECK_WIRE_RELATIVE) {
        NeckWireRelative *r=&m->body.relative;
        stale=finish<r->decision_time_us || finish>r->valid_until_us || epoch!=r->invalidation_epoch;
        if(stale) {
            r->quality_flags|=NECK_Q_TRANSPORT_STALE;
            for(i=0;i<4;++i) r->pairs[i].valid_mask&=NECK_WIRE_PAIR_SKEW;
        }
    } else if(m->header.type==NECK_WIRE_STATUS) {
        NeckWireStatus *s=&m->body.status;
        stale=finish<s->decision_time_us || epoch!=s->invalidation_epoch;
        if(stale) {
            s->quality_flags|=NECK_Q_TRANSPORT_STALE;
            s->accel_usable_mask=0; s->orientation_usable_mask=0;
            s->quality_state=0; s->frame_accepted=0;
        }
    }
    return stale;
}
void neck_wire_producer_event(NeckWireEvent *dest,
    const NeckAlarmEvent *source, uint64_t event_id)
{
    if(dest==NULL || source==NULL) return;
    memset(dest,0,sizeof(*dest));
    dest->event_id=event_id; dest->input_sequence=source->sequence;
    dest->frame_time_us=source->frame_time_us; dest->event_time_us=source->time_us;
    dest->reference_id=source->reference_id;
    dest->invalidation_epoch=source->invalidation_epoch;
    dest->events=source->events; dest->posture_flags=source->posture_flags;
    dest->axis=source->axis; dest->previous_level=source->previous_level;
    dest->level=source->level; dest->direction=source->direction;
    dest->time_flags=(uint8_t)(source->time_us>=source->frame_time_us?3U:2U);
}
