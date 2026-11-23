#include "neck_processing.h"
#include <stddef.h>
#include <string.h>
static uint64_t deadline(uint64_t time, uint64_t span)
{ return span>UINT64_MAX-time?UINT64_MAX:time+span; }
static int selected(const NeckProcessingContext *c, unsigned node)
{ return !c->single_c4 || node==NECK_NODE_C4; }
static int zero_quality(const NeckProcessingContext *c, const NeckQualityOutput *q)
{ return c->single_c4 ? (q->accel_usable_mask & (1u<<NECK_NODE_C4))!=0 :
    q->accel_usable_mask==7 && q->orientation_usable_mask==7; }
void neck_processing_default_parameters(NeckProcessingParameters *p)
{
    if(p==NULL) return;
    neck_quality_default_parameters(&p->quality); neck_pipeline_default_parameters(&p->pipeline);
    neck_posture_default_parameters(&p->posture); neck_relative_default_parameters(&p->relative);
}
NeckResult neck_processing_init(NeckProcessingContext *c, uint32_t config, uint32_t world,
    const double mounts[NECK_NODE_COUNT][4], uint64_t now, const NeckProcessingParameters *p)
{
    unsigned i;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    memset(c,0,sizeof(*c));
    if(p==NULL || mounts==NULL) return NECK_BAD_CONFIG;
    if(neck_quality_init(&c->quality,config,world,now,&p->quality)!=NECK_OK ||
        neck_relative_init(&c->relative,config,world,mounts,&p->relative)!=NECK_OK ||
        neck_posture_init(&c->posture,config,NECK_NODE_C4,&p->posture)!=NECK_OK) return NECK_BAD_CONFIG;
    for(i=0;i<3;++i) if(neck_pipeline_init(&c->pipelines[i],config,(uint8_t)i,
        mounts[i],p->relative.quaternion_norm_tolerance,&p->pipeline)!=NECK_OK) return NECK_BAD_CONFIG;
    c->next_reference_id=1; c->initialized=1; return NECK_OK;
}
NeckResult neck_processing_init_single_c4(NeckProcessingContext *c, uint32_t config,
    const double mount[4], uint64_t now, const NeckProcessingParameters *p)
{
    double mounts[3][4]={{1,0,0,0},{1,0,0,0},{1,0,0,0}};
    NeckResult result;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(mount==NULL) { memset(c,0,sizeof(*c)); return NECK_BAD_CONFIG; }
    memcpy(mounts[NECK_NODE_C4],mount,sizeof(mounts[0]));
    /* Internal token only; no world registration is exported. */
    result=neck_processing_init(c,config,1,(const double (*)[4])mounts,now,p);
    if(result==NECK_OK) c->single_c4=1;
    return result;
}
static void request_fields(const NeckProcessingContext *c, NeckProcessingOutput *o)
{
    unsigned i;
    o->pose.pending_reference_id=c->pending_reference_id; o->pose.zero_request_state=c->request_state;
    o->pose.invalidation_epoch=c->invalidation_epoch;
    for(i=0;i<3;++i) o->pose.zero_status[i]=c->pipelines[i].zero_status;
}
static void abort_request(NeckProcessingContext *c, uint8_t status)
{
    unsigned i;
    if(!c->pending_reference_id) return;
    for(i=0;i<3;++i) if(selected(c,i)) {
        neck_pipeline_invalidate(&c->pipelines[i],status);
        if(c->request_state==NECK_REQUEST_WAIT_QUALITY) c->pipelines[i].zero_status=status;
    }
    c->pending_reference_id=0; c->request_state=NECK_REQUEST_IDLE;
    if(!c->single_c4) ++c->stats.relative_aborts;
}
static void expire_request(NeckProcessingContext *c, uint64_t now)
{
    if(c->pending_reference_id && (now<c->request_wall_us ||
        now-c->request_wall_us>=c->pipelines[c->single_c4?NECK_NODE_C4:0].parameters.zero_timeout_us))
        abort_request(c,NECK_ZERO_TIMED_OUT);
}
NeckResult neck_processing_request_zero(NeckProcessingContext *c, uint64_t now, uint8_t origin)
{
    unsigned i; uint32_t id;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(origin>NECK_REFERENCE_EXTERNAL || !c->next_reference_id || c->quality.clock_fault ||
        now<c->quality.previous_wall_us) { ++c->stats.command_errors; return NECK_BAD_REFERENCE; }
    if(c->quality.stream_bound && origin==NECK_REFERENCE_SIMULATION && c->quality.source!=NECK_SOURCE_SIMULATION) {
        ++c->stats.command_errors; return NECK_SIM_REFERENCE_ON_HARDWARE;
    }
    for(i=0;i<3;++i) if(selected(c,i)) { neck_pipeline_cancel_zero(&c->pipelines[i]); c->pipelines[i].zero_status=NECK_ZERO_IDLE; }
    id=c->next_reference_id; c->next_reference_id=id==UINT32_MAX?0:id+1;
    c->pending_reference_id=id; c->request_state=NECK_REQUEST_WAIT_QUALITY;
    c->request_wall_us=now; c->request_origin=origin; ++c->stats.zero_requests;
    return NECK_OK;
}
void neck_processing_cancel_zero(NeckProcessingContext *c)
{
    unsigned i;
    if(c==NULL || !c->initialized) return;
    for(i=0;i<3;++i) if(selected(c,i)) {
        neck_pipeline_cancel_zero(&c->pipelines[i]);
        if(c->request_state==NECK_REQUEST_WAIT_QUALITY) c->pipelines[i].zero_status=NECK_ZERO_CANCELED;
    }
    c->pending_reference_id=0; c->request_state=NECK_REQUEST_IDLE; ++c->stats.zero_cancels;
}
static void invalid_output(NeckProcessingContext *c, uint64_t now, uint8_t expired,
    uint8_t clear_reference, NeckProcessingOutput *o)
{
    static const uint8_t refs[4]={2,1,2,3},targets[4]={0,0,1,2};
    NeckQualityOutput quality=o->pose.quality;
    uint32_t tilt_flags=NECK_FLAG_QUALITY_NOT_EVALUATED|NECK_FLAG_QUALITY_REJECTED;
    uint32_t rel_flags=NECK_REL_QUALITY_NOT_EVALUATED|NECK_REL_THRESHOLDS_UNSET|NECK_REL_QUALITY_REJECTED;
    uint32_t posture_flags=NECK_POSTURE_QUALITY_REJECTED;
    unsigned i;
    if(expired) { tilt_flags|=NECK_FLAG_INPUT_TIMEOUT; rel_flags|=NECK_REL_INPUT_TIMEOUT;
        posture_flags|=NECK_POSTURE_INPUT_TIMEOUT; }
    if(c->request_state==NECK_REQUEST_COLLECTING) abort_request(c,NECK_ZERO_FAILED_INPUT);
    if(clear_reference) {
        abort_request(c,NECK_ZERO_FAILED_INPUT);
        for(i=0;i<3;++i) c->pipelines[i].reference_set=0;
        c->relative.reference_set=0;
    }
    if(c->previous_accel_mask || c->previous_quat_mask || (quality.flags&NECK_Q_TIMEOUT_EDGE))
        ++c->invalidation_epoch;
    c->previous_accel_mask=c->previous_quat_mask=0;
    memset(c->previous_tilt_masks,0,sizeof(c->previous_tilt_masks));
    memset(c->previous_relative_masks,0,sizeof(c->previous_relative_masks));
    memset(o,0,sizeof(*o)); o->pose.quality=quality;
    o->pose.valid_until_us=deadline(now,c->quality.parameters.max_frame_age_us);
    (void)neck_posture_invalidate(&c->posture,now,posture_flags,&o->pose.posture);
    for(i=0;i<3;++i) {
        NeckTiltEstimate *e=&o->pose.nodes[i];
        neck_pipeline_invalidate(&c->pipelines[i],NECK_ZERO_FAILED_INPUT);
        e->config_id=c->quality.config_id; e->session_id=quality.session_id; e->source=quality.source;
        e->sequence=quality.sequence; e->frame_time_us=quality.frame_time_us; e->node_id=(uint8_t)i;
        e->reference_id=c->pipelines[i].reference_set?c->pipelines[i].reference.reference_id:0;
        e->flags=tilt_flags; e->status=e->pitch_status=e->lateral_status=NECK_INVALID;
        o->pose.filter_flags[i]=NECK_FILTER_DATA_INVALID|NECK_FILTER_RESET;
    }
    o->c4.raw=o->c4.filtered=o->pose.nodes[NECK_NODE_C4];
    o->c4.filter_flags=o->pose.filter_flags[NECK_NODE_C4];
    o->c4.zero.status=c->pipelines[NECK_NODE_C4].zero_status;
    o->relative.config_id=c->quality.config_id; o->relative.session_id=quality.session_id;
    o->relative.source=quality.source; o->relative.sequence=quality.sequence;
    o->relative.frame_time_us=quality.frame_time_us;
    o->relative.world_frame_id=c->single_c4?0:c->quality.world_frame_id;
    o->relative.reference_id=c->relative.reference_set?c->relative.reference.reference_id:0;
    o->relative.flags=rel_flags;
    for(i=0;i<4;++i) {
        o->relative.pairs[i].flags=rel_flags; o->relative.pairs[i].reference_node=refs[i];
        o->relative.pairs[i].target_node=targets[i];
    }
    if(quality.flags&NECK_Q_TIMEOUT_EDGE) o->step_flags|=NECK_STEP_TIMEOUT_EDGE;
    request_fields(c,o);
}
int neck_processing_poll(NeckProcessingContext *c, uint64_t now, NeckProcessingOutput *o)
{
    NeckQualityOutput q;
    if(c==NULL || o==NULL || !c->initialized) return 0;
    expire_request(c,now);
    if(!neck_quality_poll(&c->quality,now,&q)) return 0;
    o->pose.quality=q;
    invalid_output(c,now,(uint8_t)((q.flags&NECK_Q_TIMEOUT)!=0),
        (uint8_t)((q.flags&NECK_Q_CLOCK_FAULT)!=0),o);
    if(q.flags&NECK_Q_TIMEOUT_EDGE) ++c->stats.timeout_transitions;
    return 1;
}
static void begin_candidates(NeckProcessingContext *c, const NeckQualityOutput *q)
{
    unsigned i;
    if(c->request_state!=NECK_REQUEST_WAIT_QUALITY || !zero_quality(c,q)) return;
    if(c->request_origin==NECK_REFERENCE_SIMULATION && q->source!=NECK_SOURCE_SIMULATION) {
        ++c->stats.command_errors; abort_request(c,NECK_ZERO_FAILED_INPUT); return;
    }
    for(i=0;i<3;++i) if(selected(c,i) && neck_pipeline_begin_zero(&c->pipelines[i],c->pending_reference_id,
        c->request_origin)!=NECK_OK) { ++c->stats.command_errors; abort_request(c,NECK_ZERO_FAILED_INPUT); return; }
    c->request_state=NECK_REQUEST_COLLECTING;
}
NeckResult neck_processing_process(NeckProcessingContext *c, const NeckAcquisitionFrame *input,
    uint64_t now, NeckProcessingOutput *o)
{
    NeckAcquisitionFrame gated;
    NeckResult result; unsigned n,i; static const uint8_t order[3]={0,2,1}; int all_zero=1;
    if(o==NULL) return NECK_BAD_ARGUMENT;
    memset(o,0,sizeof(*o));
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    expire_request(c,now);
    if(input && c->single_c4) {
        gated=*input;
        for(i=0;i<3;++i) if(!selected(c,i)) {
            memset(&gated.acceleration.nodes[i],0,sizeof(gated.acceleration.nodes[i]));
            gated.acceleration.nodes[i].status=NECK_NODE_MISSING;
            gated.acceleration.nodes[i].calibration_accel=NECK_CALIBRATION_UNKNOWN;
            memset(&gated.orientation.nodes[i],0,sizeof(gated.orientation.nodes[i]));
            gated.orientation.nodes[i].status=NECK_NODE_MISSING;
            gated.orientation.nodes[i].calibration_system=NECK_CALIBRATION_UNKNOWN;
        }
        gated.orientation.nodes[NECK_NODE_C4].world_frame_id=0;
        input=&gated;
    }
    result=neck_quality_process(&c->quality,input?&input->acceleration:NULL,
        input?&input->orientation:NULL,now,&o->pose.quality);
    if(result!=NECK_OK) {
        o->pose.quality.flags|=NECK_Q_FRAME_REJECTED;
        ++c->stats.rejected_frames;
        invalid_output(c,now,(uint8_t)((o->pose.quality.flags&NECK_Q_TIMEOUT)!=0),
            (uint8_t)((o->pose.quality.flags&(NECK_Q_STREAM_CHANGED|NECK_Q_CLOCK_FAULT))!=0),o);
        o->step_flags|=NECK_STEP_REJECTED;
        if(o->pose.quality.flags&NECK_Q_TIMEOUT_EDGE) ++c->stats.timeout_transitions;
        return result;
    }
    o->step_flags=NECK_STEP_FRAME; gated=*input;
    if((c->previous_accel_mask&~o->pose.quality.accel_usable_mask) ||
        (c->previous_quat_mask&~o->pose.quality.orientation_usable_mask)) ++c->invalidation_epoch;
    c->previous_accel_mask=o->pose.quality.accel_usable_mask;
    c->previous_quat_mask=o->pose.quality.orientation_usable_mask;
    o->pose.valid_until_us=deadline(input->acceleration.frame_time_us,c->quality.parameters.max_frame_age_us);
    if(c->request_state==NECK_REQUEST_COLLECTING &&
        (!zero_quality(c,&o->pose.quality) ||
        (o->pose.quality.flags&NECK_Q_STREAM_GAP))) abort_request(c,NECK_ZERO_FAILED_INPUT);
    begin_candidates(c,&o->pose.quality);
    for(i=0;i<3;++i) {
        uint64_t expires;
        if(o->pose.quality.accel_usable_mask&(1u<<i)) {
            expires=deadline(input->acceleration.nodes[i].sample_time_us,c->quality.parameters.max_sample_age_us);
            if(expires<o->pose.valid_until_us) o->pose.valid_until_us=expires;
        }
        if(o->pose.quality.orientation_usable_mask&(1u<<i)) {
            expires=deadline(input->orientation.nodes[i].sample_time_us,c->quality.parameters.max_sample_age_us);
            if(expires<o->pose.valid_until_us) o->pose.valid_until_us=expires;
        }
        if(!(o->pose.quality.accel_usable_mask&(1u<<i))) {
            /* Legacy tilt validates every node's metadata. Convert only the
             * rejected node to a coherent empty record, so one bad node cannot
             * poison otherwise eligible nodes. Reasons remain in quality. */
            memset(&gated.acceleration.nodes[i],0,sizeof(gated.acceleration.nodes[i]));
            gated.acceleration.nodes[i].status=NECK_NODE_MISSING;
            gated.acceleration.nodes[i].calibration_accel=NECK_CALIBRATION_UNKNOWN;
        }
        if(!(o->pose.quality.orientation_usable_mask&(1u<<i))) gated.orientation.nodes[i].quat_present=0;
    }
    /* Reuse c4 scratch; process C4 last so diagnostics retain the correct node. */
    for(n=0;n<3;++n) {
        i=order[n];
        if(neck_pipeline_process(&c->pipelines[i],&gated.acceleration,&o->c4)!=NECK_OK) ++c->stats.pipeline_errors;
        if(!(o->pose.quality.accel_usable_mask&(1u<<i))) {
            o->c4.raw.valid_mask=o->c4.filtered.valid_mask=0;
            o->c4.raw.flags|=NECK_FLAG_QUALITY_REJECTED; o->c4.filtered.flags|=NECK_FLAG_QUALITY_REJECTED;
            o->c4.raw.status=o->c4.filtered.status=NECK_INVALID;
        }
        o->pose.nodes[i]=o->c4.filtered; o->pose.filter_flags[i]=o->c4.filter_flags;
        if(o->c4.filter_flags&NECK_ZERO_COMMITTED) { ++c->stats.node_commits; ++c->invalidation_epoch; }
        if(o->c4.filter_flags&NECK_FILTER_RESET) ++c->stats.filter_resets;
        if(selected(c,i) && (c->pipelines[i].zero_status!=NECK_ZERO_SUCCEEDED ||
            c->pipelines[i].reference.reference_id!=c->pending_reference_id)) all_zero=0;
    }
    if(neck_posture_process(&c->posture,&o->c4,&o->pose.posture)!=NECK_OK) ++c->stats.posture_errors;
    if(!(o->pose.quality.accel_usable_mask&(1u<<NECK_NODE_C4))) o->pose.posture.flags|=NECK_POSTURE_QUALITY_REJECTED;
    if((o->pose.posture.flexion.events|o->pose.posture.lateral.events)&NECK_EVENT_INVALIDATED)
        ++c->invalidation_epoch;
    if(!c->single_c4) {
        if(neck_relative_process(&c->relative,&gated.orientation,&o->relative)!=NECK_OK) ++c->stats.relative_errors;
    } else {
        static const uint8_t refs[4]={2,1,2,3}, targets[4]={0,0,1,2};
        o->relative.config_id=input->acceleration.config_id;
        o->relative.session_id=input->acceleration.session_id;
        o->relative.source=input->acceleration.source;
        o->relative.sequence=input->acceleration.sequence;
        o->relative.frame_time_us=input->acceleration.frame_time_us;
        o->relative.flags=NECK_REL_QUALITY_REJECTED;
        for(i=0;i<4;++i) {
            o->relative.pairs[i].reference_node=refs[i];
            o->relative.pairs[i].target_node=targets[i];
        }
    }
    for(i=0;i<4;++i) {
        NeckRelativePairOutput *pair=&o->relative.pairs[i];
        if(!(o->pose.quality.orientation_usable_mask&(1u<<pair->target_node)) ||
            (pair->reference_node!=NECK_WORLD_REFERENCE &&
            !(o->pose.quality.orientation_usable_mask&(1u<<pair->reference_node))))
            pair->flags|=NECK_REL_QUALITY_REJECTED;
    }
    for(i=0;i<3;++i) {
        if(c->previous_tilt_masks[i]&~o->pose.nodes[i].valid_mask) ++c->invalidation_epoch;
        c->previous_tilt_masks[i]=o->pose.nodes[i].valid_mask;
    }
    for(i=0;i<4;++i) {
        if(c->previous_relative_masks[i]&~o->relative.pairs[i].valid_mask) ++c->invalidation_epoch;
        c->previous_relative_masks[i]=o->relative.pairs[i].valid_mask;
    }
    if(c->request_state==NECK_REQUEST_COLLECTING) {
        for(i=0;i<3;++i) if(selected(c,i) && c->pipelines[i].zero_status>=NECK_ZERO_FAILED_INPUT) {
            abort_request(c,NECK_ZERO_FAILED_INPUT); break;
        }
        if(c->pending_reference_id && all_zero && (c->single_c4 || neck_relative_capture_reference(&c->relative,&o->relative,
            c->pending_reference_id,c->request_origin)==NECK_OK)) {
            c->pending_reference_id=0; c->request_state=NECK_REQUEST_IDLE;
            if(!c->single_c4) ++c->stats.relative_commits;
            ++c->invalidation_epoch;
        }
    }
    request_fields(c,o); return NECK_OK;
}
NeckResult neck_processing_reset_stream(NeckProcessingContext *c, uint64_t now, NeckProcessingOutput *o)
{
    unsigned i;
    if(c==NULL || o==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    memset(o,0,sizeof(*o)); o->pose.quality.flags=NECK_Q_RESET|NECK_Q_ENGINEERING_ONLY;
    o->pose.quality.decision_time_us=now;
    invalid_output(c,now,0,1,o);
    for(i=0;i<3;++i) neck_pipeline_reset_stream(&c->pipelines[i],1);
    neck_relative_reset_stream(&c->relative,1); neck_posture_reset(&c->posture);
    neck_quality_reset_stream(&c->quality,now); ++c->invalidation_epoch; request_fields(c,o); return NECK_OK;
}
int neck_processing_expire_pose(NeckPoseBatch *pose, uint64_t now, uint32_t epoch)
{
    unsigned i;
    uint32_t tilt_timeout, posture_timeout;
    if(pose==NULL) return 0;
    if(now>=pose->quality.decision_time_us && now<=pose->valid_until_us &&
        epoch==pose->invalidation_epoch) return 0;
    /* An epoch mismatch may be a new reference or partial-node fault, not
     * input silence. Keep TRANSPORT_STALE distinct from real time expiry. */
    tilt_timeout=now>pose->valid_until_us?NECK_FLAG_INPUT_TIMEOUT:0;
    posture_timeout=now>pose->valid_until_us?NECK_POSTURE_INPUT_TIMEOUT:0;
    for(i=0;i<3;++i) {
        pose->nodes[i].valid_mask=0;
        pose->nodes[i].status=pose->nodes[i].pitch_status=pose->nodes[i].lateral_status=NECK_INVALID;
        pose->nodes[i].flags|=NECK_FLAG_QUALITY_REJECTED|tilt_timeout;
    }
    memset(&pose->posture.flexion,0,sizeof(pose->posture.flexion));
    memset(&pose->posture.lateral,0,sizeof(pose->posture.lateral));
    pose->posture.flags|=NECK_POSTURE_INPUT_INVALID|NECK_POSTURE_QUALITY_REJECTED|posture_timeout;
    pose->posture.time_us=now; pose->posture.sample_time_known=0;
    pose->quality.flags|=NECK_Q_TRANSPORT_STALE;
    pose->quality.accel_usable_mask=pose->quality.orientation_usable_mask=0;
    pose->quality.state=NECK_Q_UNKNOWN; pose->quality.frame_accepted=0;
    pose->quality.decision_time_us=now; return 1;
}
