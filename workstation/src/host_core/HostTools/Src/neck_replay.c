/* S08 host I/O. Algorithm decisions are made only by neck_processing.c.
 * NO implicit POLL, ZERO, resampling, truth correction or gap filling. */
#include "neck_replay_internal.h"
#include <stdlib.h>
#include <string.h>

#define U(x) ((unsigned)(x))
#define L(x) ((unsigned long long)(x))
typedef struct { FILE *frames,*nodes,*relative,*events,*commands; } Outputs;
static void cell(FILE *f,double x,uint32_t mask,uint32_t bit)
{ fputc(',',f); nr_optional(f,x,(mask&bit)!=0); }
static void headers(Outputs *f)
{
    fputs("mode,output_id,step,sequence,frame_time_us,decision_time_us,source,config_id,session_id,result,step_flags,quality_flags,quality_state,frame_accepted,accel_mask,quat_mask,node_flags_C0,node_flags_C4,node_flags_C7,filter_flags_C0,filter_flags_C4,filter_flags_C7,zero_C0,zero_C4,zero_C7,zero_request,pending_reference,epoch,valid_until_us,c4_alpha,posture_flags,posture_reference,flex_valid,flex_instant,flex_alarm,flex_direction,flex_warning_us,flex_danger_us,flex_extension_us,flex_events,lateral_valid,lateral_instant,lateral_alarm,lateral_direction,lateral_warning_us,lateral_danger_us,lateral_events\n",f->frames);
    fputs("mode,output_id,step,sequence,frame_time_us,decision_time_us,source,node,raw_mask,raw_flags,raw_status,raw_pitch_deg,raw_lateral_deg,raw_norm_m_s2,filtered_mask,filtered_flags,filtered_status,filtered_absolute_pitch_deg,filtered_absolute_lateral_deg,corrected_flexion_deg,corrected_right_lateral_deg,filtered_norm_m_s2,sample_time_us,sample_age_us,reference_id,node_quality_flags,filter_flags,zero_status,epoch\n",f->nodes);
    fputs("mode,output_id,step,sequence,frame_time_us,decision_time_us,source,pair,reference_node,target_node,reference_id,valid_mask,flags,status,skew_us,raw_w,raw_x,raw_y,raw_z,raw_flexion_deg,raw_lateral_deg,raw_axial_deg,raw_rotation_deg,corrected_w,corrected_x,corrected_y,corrected_z,corrected_flexion_deg,corrected_lateral_deg,corrected_axial_deg,corrected_rotation_deg,epoch\n",f->relative);
    fputs("mode,output_id,decision_time_us,source,axis,event_mask,previous_alarm,new_alarm,instant_level,valid,direction,reference_id,posture_flags,quality_flags,epoch\n",f->events);
    fputs("mode,input_row,now_us,command,origin,result,request_state,pending_reference,epoch\n",f->commands);
}
static void event_row(FILE *f,const NeckProcessingOutput *o,const NeckPostureAxisOutput *a,
    const char *axis,uint64_t id,NeckReplayStats *s)
{
    if(!a->events) return;
    fprintf(f,"OFFLINE_REPLAY,%llu,%llu,%u,%s,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u\n",
        L(id),L(o->pose.quality.decision_time_us),U(o->pose.quality.source),axis,U(a->events),
        U(a->previous_alarm_level),U(a->alarm_level),U(a->instant_level),U(a->valid),
        (int)a->direction,U(o->pose.posture.reference_id),U(o->pose.posture.flags),
        U(o->pose.quality.flags),U(o->pose.invalidation_epoch));
    ++s->event_rows;
    if(a->events&NECK_EVENT_TRIGGER) ++s->triggers;
    if(a->events&NECK_EVENT_CLEAR) ++s->clears;
    if(a->events&NECK_EVENT_LEVEL_CHANGE) ++s->level_changes;
}
static int point_add(NRPoints *p,const NeckProcessingOutput *o,NeckReplayError *e)
{
    NRPoint *x; unsigned i,j;
    if(p->count==NR_POINT_MAX) return nr_error(e,NR_LIMIT,"Output point limit reached.");
    if(p->count==p->capacity) {
        size_t cap=p->capacity?p->capacity*2:1024; NRPoint *items;
        if(cap>NR_POINT_MAX) cap=NR_POINT_MAX;
        items=(NRPoint *)realloc(p->items,cap*sizeof(*items));
        if(items==NULL) return nr_error(e,NR_IO,"Plot allocation failed (host only).");
        p->items=items; p->capacity=cap;
    }
    x=&p->items[p->count++]; memset(x,0,sizeof(*x));
    x->time_us=o->pose.quality.decision_time_us; x->source=o->pose.quality.source;
    x->raw_mask=o->c4.raw.valid_mask; x->filtered_mask=o->c4.filtered.valid_mask;
    x->raw[0]=o->c4.raw.pitch_raw_deg; x->raw[1]=o->c4.raw.lateral_raw_deg;
    x->filtered[0]=o->c4.filtered.pitch_raw_deg; x->filtered[1]=o->c4.filtered.lateral_raw_deg;
    x->corrected[0]=o->c4.filtered.flexion_deg; x->corrected[1]=o->c4.filtered.right_lateral_deg;
    x->instant[0]=o->pose.posture.flexion.instant_level; x->instant[1]=o->pose.posture.lateral.instant_level;
    x->alarm[0]=o->pose.posture.flexion.alarm_level; x->alarm[1]=o->pose.posture.lateral.alarm_level;
    x->valid[0]=o->pose.posture.flexion.valid; x->valid[1]=o->pose.posture.lateral.valid;
    x->timers[0]=o->pose.posture.flexion.warning_elapsed_us;
    x->timers[1]=o->pose.posture.flexion.danger_elapsed_us;
    x->timers[2]=o->pose.posture.flexion.extension_elapsed_us;
    x->timers[3]=o->pose.posture.lateral.warning_elapsed_us;
    x->timers[4]=o->pose.posture.lateral.danger_elapsed_us;
    x->events=o->pose.posture.flexion.events|o->pose.posture.lateral.events;
    x->qflags=o->pose.quality.flags; x->epoch=o->pose.invalidation_epoch;
    x->accel_mask=o->pose.quality.accel_usable_mask; x->quat_mask=o->pose.quality.orientation_usable_mask;
    for(i=0;i<4;++i) {
        x->relative_masks[i]=o->relative.pairs[i].valid_mask;
        for(j=0;j<3;++j) x->relative[i][j]=o->relative.pairs[i].corrected_euler_deg[j];
    }
    return NR_OK;
}
static int output_row(Outputs *f,NRPoints *p,NeckProcessingContext *c,
    const NeckReplayEvent *ev,const NeckProcessingOutput *o,NeckResult result,
    const char *step,NeckReplayStats *s,NeckReplayError *e)
{
    const NeckQualityOutput *q=&o->pose.quality; const NeckPostureOutput *post=&o->pose.posture;
    const NeckPostureAxisOutput *a=&post->flexion,*b=&post->lateral;
    NeckFrame gated; unsigned i,j; uint64_t id=s->outputs;
    int added=point_add(p,o,e); if(added!=NR_OK) return added;
    fprintf(f->frames,"OFFLINE_REPLAY,%llu,%s,%llu,%llu,%llu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        L(id),step,L(q->sequence),L(q->frame_time_us),L(q->decision_time_us),U(q->source),U(q->config_id),
        U(q->session_id),U(result),U(o->step_flags),U(q->flags),U(q->state),U(q->frame_accepted),U(q->accel_usable_mask),U(q->orientation_usable_mask));
    for(i=0;i<3;++i) fprintf(f->frames,",%u",U(q->node_flags[i]));
    for(i=0;i<3;++i) fprintf(f->frames,",%u",U(o->pose.filter_flags[i]));
    for(i=0;i<3;++i) fprintf(f->frames,",%u",U(o->pose.zero_status[i]));
    fprintf(f->frames,",%u,%u,%u,%llu,",U(o->pose.zero_request_state),U(o->pose.pending_reference_id),U(o->pose.invalidation_epoch),L(o->pose.valid_until_us));
    nr_optional(f->frames,o->c4.alpha,o->c4.alpha_valid);
    fprintf(f->frames,",%u,%u,%u,%u,%u,%d,%llu,%llu,%llu,%u,%u,%u,%u,%d,%llu,%llu,%u\n",
        U(post->flags),U(post->reference_id),U(a->valid),U(a->instant_level),U(a->alarm_level),(int)a->direction,
        L(a->warning_elapsed_us),L(a->danger_elapsed_us),L(a->extension_elapsed_us),U(a->events),
        U(b->valid),U(b->instant_level),U(b->alarm_level),(int)b->direction,L(b->warning_elapsed_us),L(b->danger_elapsed_us),U(b->events));
    memset(&gated,0,sizeof(gated));
    if(ev->kind==NR_FRAME && q->frame_accepted) {
        gated=ev->frame.acceleration;
        for(i=0;i<3;++i) if(!(q->accel_usable_mask&(1u<<i))) {
            memset(&gated.nodes[i],0,sizeof(gated.nodes[i])); gated.nodes[i].status=NECK_NODE_MISSING;
            gated.nodes[i].calibration_accel=NECK_CALIBRATION_UNKNOWN;
        }
    }
    for(i=0;i<3;++i) {
        const NeckTiltEstimate *filtered=&o->pose.nodes[i]; NeckTiltEstimate raw;
        memset(&raw,0,sizeof(raw)); raw.status=NECK_INVALID;
        if(i==NECK_NODE_C4) raw=o->c4.raw;
        else if(ev->kind==NR_FRAME && q->frame_accepted && (q->accel_usable_mask&(1u<<i)))
            (void)neck_tilt_estimate(&c->pipelines[i].tilt,&gated,NULL,&raw); /* diagnostic only */
        fprintf(f->nodes,"OFFLINE_REPLAY,%llu,%s,%llu,%llu,%llu,%u,%u,%u,%u,%u",L(id),step,L(q->sequence),L(q->frame_time_us),L(q->decision_time_us),U(q->source),i,U(raw.valid_mask),U(raw.flags),U(raw.status));
        cell(f->nodes,raw.pitch_raw_deg,raw.valid_mask,NECK_VALID_PITCH_RAW);
        cell(f->nodes,raw.lateral_raw_deg,raw.valid_mask,NECK_VALID_LATERAL_RAW);
        cell(f->nodes,raw.accel_norm_m_s2,raw.valid_mask,NECK_VALID_NORM);
        fprintf(f->nodes,",%u,%u,%u",U(filtered->valid_mask),U(filtered->flags),U(filtered->status));
        cell(f->nodes,filtered->pitch_raw_deg,filtered->valid_mask,NECK_VALID_PITCH_RAW);
        cell(f->nodes,filtered->lateral_raw_deg,filtered->valid_mask,NECK_VALID_LATERAL_RAW);
        cell(f->nodes,filtered->flexion_deg,filtered->valid_mask,NECK_VALID_FLEXION);
        cell(f->nodes,filtered->right_lateral_deg,filtered->valid_mask,NECK_VALID_RIGHT_LATERAL);
        cell(f->nodes,filtered->accel_norm_m_s2,filtered->valid_mask,NECK_VALID_NORM);
        fputc(',',f->nodes); if(filtered->valid_mask&NECK_VALID_SAMPLE_TIME) fprintf(f->nodes,"%llu",L(filtered->sample_time_us));
        fputc(',',f->nodes); if(filtered->valid_mask&NECK_VALID_SAMPLE_AGE) fprintf(f->nodes,"%llu",L(filtered->sample_age_us));
        fprintf(f->nodes,",%u,%u,%u,%u,%u\n",U(filtered->reference_id),U(q->node_flags[i]),U(o->pose.filter_flags[i]),U(o->pose.zero_status[i]),U(o->pose.invalidation_epoch));
        ++s->node_rows;
    }
    for(i=0;i<4;++i) {
        const NeckRelativePairOutput *r=&o->relative.pairs[i];
        fprintf(f->relative,"OFFLINE_REPLAY,%llu,%s,%llu,%llu,%llu,%u,%u,%u,%u,%u,%u,%u,%u,",L(id),step,L(q->sequence),L(q->frame_time_us),L(q->decision_time_us),U(q->source),i,U(r->reference_node),U(r->target_node),U(o->relative.reference_id),U(r->valid_mask),U(r->flags),U(r->status));
        if(r->valid_mask&NECK_REL_KNOWN_SKEW) fprintf(f->relative,"%llu",L(r->sample_skew_us));
        for(j=0;j<4;++j) cell(f->relative,r->raw_wxyz[j],r->valid_mask,NECK_REL_RAW_QUAT);
        for(j=0;j<3;++j) cell(f->relative,r->raw_euler_deg[j],r->valid_mask,NECK_REL_RAW_EULER);
        cell(f->relative,r->raw_rotation_deg,r->valid_mask,NECK_REL_RAW_ANGLE);
        for(j=0;j<4;++j) cell(f->relative,r->corrected_wxyz[j],r->valid_mask,NECK_REL_CORRECTED_QUAT);
        for(j=0;j<3;++j) cell(f->relative,r->corrected_euler_deg[j],r->valid_mask,NECK_REL_CORRECTED_EULER);
        cell(f->relative,r->corrected_rotation_deg,r->valid_mask,NECK_REL_CORRECTED_ANGLE);
        fprintf(f->relative,",%u\n",U(o->pose.invalidation_epoch)); ++s->relative_rows;
    }
    event_row(f->events,o,a,"flexion",id,s); event_row(f->events,o,b,"lateral",id,s);
    if(o->step_flags&NECK_STEP_REJECTED) ++s->rejected_frames;
    if(q->flags&NECK_Q_TIMEOUT_EDGE) ++s->timeout_edges;
    ++s->outputs;
    return ferror(f->frames)||ferror(f->nodes)||ferror(f->relative)||ferror(f->events)?nr_error(e,NR_IO,"CSV output failed."):NR_OK;
}
static int comparison(const char *dir,const NRPoints *points,const NRTruths *truth,NeckReplayError *e)
{
    FILE *f; size_t i; unsigned j;
    if(!truth->count) return NR_OK;
    f=nr_new_file(dir,"comparison.csv",e); if(f==NULL) return NR_IO;
    fputs("mode,output_id,decision_time_us,source,construction_joined,body_pitch_deg,body_roll_deg,body_yaw_deg,ideal_proxy_mask,ideal_absolute_pitch_deg,ideal_absolute_lateral_deg,raw_mask,raw_absolute_pitch_deg,raw_absolute_lateral_deg,filtered_mask,filtered_absolute_pitch_deg,filtered_absolute_lateral_deg,corrected_flexion_deg,corrected_right_lateral_deg\n",f);
    for(i=0;i<points->count;++i) {
        const NRPoint *p=&points->items[i]; const NeckReplayTruth *t=nr_truth_at(truth,p->time_us,p->source);
        double proxy[2]={0,0}; uint8_t mask=0;
        fprintf(f,"OFFLINE_REPLAY,%llu,%llu,%u,%u",L(i),L(p->time_us),U(p->source),t!=NULL?1u:0u);
        for(j=0;j<3;++j) { fputc(',',f); if(t!=NULL) nr_double(f,t->euler_deg[1][j]); }
        if(t!=NULL) neck_replay_ideal_proxy(t->euler_deg[1][0],t->euler_deg[1][1],proxy,&mask);
        fprintf(f,",%u",U(mask)); cell(f,proxy[0],mask,1); cell(f,proxy[1],mask,2);
        fprintf(f,",%u",U(p->raw_mask)); cell(f,p->raw[0],p->raw_mask,NECK_VALID_PITCH_RAW); cell(f,p->raw[1],p->raw_mask,NECK_VALID_LATERAL_RAW);
        fprintf(f,",%u",U(p->filtered_mask)); cell(f,p->filtered[0],p->filtered_mask,NECK_VALID_PITCH_RAW); cell(f,p->filtered[1],p->filtered_mask,NECK_VALID_LATERAL_RAW);
        cell(f,p->corrected[0],p->filtered_mask,NECK_VALID_FLEXION); cell(f,p->corrected[1],p->filtered_mask,NECK_VALID_RIGHT_LATERAL); fputc('\n',f);
    }
    return nr_close(f,e);
}
static int metadata(const char *dir,const NeckReplayConfig *cfg,const NeckReplayStats *s,
    uint64_t input_hash,uint64_t config_hash,const NRTruths *t,NeckReplayError *e)
{
    FILE *f=nr_new_file(dir,"metadata.txt",e); if(f==NULL) return NR_IO;
    fprintf(f,"mode=OFFLINE_REPLAY\nreplay=%s\ntilt=%s\npipeline=%s\nposture=%s\nrelative=%s\nquality=%s\nphysical_quality=NOT_EVALUATED\ntruth=SIMULATION_CONSTRUCTION_ONLY_NOT_PHYSICAL_REFERENCE\nfingerprints=FNV1A64_ACTUAL_BYTES_NON_CRYPTOGRAPHIC\ninput_fnv1a64=%016llx\nconfig_fnv1a64=%016llx\ntruth_fnv1a64=%016llx\ntruth_rows=%llu\nstart_wall_us=%llu\ninput_frames=%llu\npoll_calls=%llu\noutputs=%llu\npoll_outputs=%llu\ncommands=%llu\nrejected_frames=%llu\ntriggers=%llu\nclears=%llu\nlevel_changes=%llu\ntimeout_edges=%llu\nnode_rows=%llu\nrelative_rows=%llu\nevent_rows=%llu\ninvalid_numeric_cells=BLANK_NOT_ZERO\nimplicit_commands=NONE\nimplicit_poll=NONE\nresampling=NONE\nchart_gap_limit_us=%llu\nchart_time=RELATIVE_TO_MIN_RECORDED_DECISION_US\n",
        NECK_REPLAY_ID,NECK_ALGORITHM_ID,NECK_PIPELINE_ALGORITHM_ID,NECK_POSTURE_ALGORITHM_ID,NECK_RELATIVE_ALGORITHM_ID,NECK_QUALITY_ALGORITHM_ID,
        L(input_hash),L(config_hash),L(t->digest),L(t->count),L(cfg->start_wall_us),L(s->input_frames),L(s->poll_calls),L(s->outputs),L(s->poll_outputs),L(s->commands),L(s->rejected_frames),L(s->triggers),L(s->clears),L(s->level_changes),L(s->timeout_edges),L(s->node_rows),L(s->relative_rows),L(s->event_rows),L(cfg->parameters.pipeline.max_gap_us));
    return nr_close(f,e);
}
static int close_outputs(Outputs *f,NeckReplayError *e)
{
    int bad=0;
    if(nr_close(f->frames,e)!=NR_OK) bad=1;
    if(nr_close(f->nodes,e)!=NR_OK) bad=1;
    if(nr_close(f->relative,e)!=NR_OK) bad=1;
    if(nr_close(f->events,e)!=NR_OK) bad=1;
    if(nr_close(f->commands,e)!=NR_OK) bad=1;
    memset(f,0,sizeof(*f)); return bad?NR_IO:NR_OK;
}
int neck_replay_run(const char *inputs,const char *config_path,const char *truth_path,const char *dir,
    NeckReplayStats *s,NeckReplayError *e)
{
    NeckReplayConfig cfg; NeckProcessingContext c; NeckProcessingOutput o; Outputs files;
    NRPoints points; NRTruths truth; FILE *in=NULL,*f; NeckReplayEvent ev;
    char line[NR_LINE_MAX],path[1024]; unsigned long row=0; uint64_t input_hash=NR_FNV_OFFSET,config_hash=0;
    int status,r=0,has_index=0,has_complete=0;
    if(e!=NULL) memset(e,0,sizeof(*e));
    if(s==NULL || inputs==NULL || config_path==NULL) return nr_error(e,NR_ARGUMENT,"Missing replay arguments.");
    memset(s,0,sizeof(*s)); memset(&files,0,sizeof(files)); memset(&points,0,sizeof(points)); memset(&truth,0,sizeof(truth));
    status=nr_new_directory(dir,e); if(status!=NR_OK) return status;
    status=neck_replay_config_read(config_path,&cfg,&config_hash,e); if(status!=NR_OK) goto done;
    status=nr_truths_read(truth_path,&truth,e); if(status!=NR_OK) goto done;
    if(e!=NULL) e->line=0;
    if(neck_processing_init(&c,cfg.config_id,cfg.world_frame_id,(const double (*)[4])cfg.mounts,cfg.start_wall_us,&cfg.parameters)!=NECK_OK) {
        status=nr_error(e,NR_CONFIG,"Processing initialization failed."); goto done;
    }
    in=fopen(inputs,"rb"); if(in==NULL) { status=nr_error(e,NR_IO,"Cannot open input events."); goto done; }
    status=nr_header(in,NECK_REPLAY_INPUT_HEADER,&row,&input_hash,e); if(status!=NR_OK) goto done;
    files.frames=nr_new_file(dir,"frames.csv",e); files.nodes=nr_new_file(dir,"nodes.csv",e);
    files.relative=nr_new_file(dir,"relative.csv",e); files.events=nr_new_file(dir,"events.csv",e); files.commands=nr_new_file(dir,"commands.csv",e);
    if(!files.frames || !files.nodes || !files.relative || !files.events || !files.commands) { status=NR_IO; goto done; }
    headers(&files);
    while((r=nr_read_line(in,line,&row,&input_hash,e))==1) {
        NeckResult result=NECK_OK;
        if(row>NR_ROW_MAX+1u) { status=nr_error(e,NR_LIMIT,"Input event row limit reached."); break; }
        status=neck_replay_event_parse(line,&ev,e); if(status!=NR_OK) break;
        if(ev.kind==NR_FRAME) {
            ++s->input_frames; result=neck_processing_process(&c,&ev.frame,ev.now_us,&o);
            status=output_row(&files,&points,&c,&ev,&o,result,"FRAME",s,e);
        } else if(ev.kind==NR_POLL) {
            ++s->poll_calls;
            if(neck_processing_poll(&c,ev.now_us,&o)) {
                ++s->poll_outputs; status=output_row(&files,&points,&c,&ev,&o,NECK_OK,"POLL",s,e);
            }
        } else {
            const char *name=ev.kind==NR_ZERO?"ZERO":(ev.kind==NR_CANCEL?"CANCEL":"RESET");
            ++s->commands;
            if(ev.kind==NR_ZERO) result=neck_processing_request_zero(&c,ev.now_us,ev.origin);
            else if(ev.kind==NR_CANCEL) neck_processing_cancel_zero(&c);
            else { result=neck_processing_reset_stream(&c,ev.now_us,&o); status=output_row(&files,&points,&c,&ev,&o,result,"RESET",s,e); }
            fprintf(files.commands,"OFFLINE_REPLAY,%lu,%llu,%s,",row,L(ev.now_us),name);
            if(ev.kind==NR_ZERO) fprintf(files.commands,"%u",U(ev.origin));
            fprintf(files.commands,",%u,%u,%u,%u\n",U(result),U(c.request_state),U(c.pending_reference_id),U(c.invalidation_epoch));
        }
        if(status!=NR_OK) break;
    }
    if(status==NR_OK && r<0) status=nr_error(e,ferror(in)?NR_IO:NR_FORMAT,"Input line read failed.");
    if(nr_close(in,status==NR_OK?e:NULL)!=NR_OK && status==NR_OK) status=NR_IO;
    in=NULL;
    if(close_outputs(&files,status==NR_OK?e:NULL)!=NR_OK && status==NR_OK) status=NR_IO;
    if(status!=NR_OK) goto done;
    f=nr_new_file(dir,"config_used.csv",e); if(f==NULL) { status=NR_IO; goto done; }
    status=neck_replay_config_write(f,&cfg); if(nr_close(f,e)!=NR_OK) status=NR_IO;
    if(status!=NR_OK) goto done;
    status=comparison(dir,&points,&truth,e); if(status!=NR_OK) goto done;
    status=metadata(dir,&cfg,s,input_hash,config_hash,&truth,e); if(status!=NR_OK) goto done;
    status=nr_plots(dir,&points,&truth,&cfg,e); if(status!=NR_OK) goto done;
    status=nr_index(dir,s,truth.count!=0,e); if(status!=NR_OK) goto done;
    has_index=1;
    f=nr_new_file(dir,"COMPLETE.txt",e); if(f==NULL) { status=NR_IO; goto done; }
    has_complete=1;
    fputs("OFFLINE_REPLAY_COMPLETE\nCheck metadata.txt, config_used.csv and validity masks. NOT hardware validation.\n",f);
    status=nr_close(f,e);
done:
    if(in!=NULL) (void)nr_close(in,NULL);
    (void)close_outputs(&files,NULL); free(points.items); free(truth.items);
    if(status!=NR_OK) {
        /* Only our own new run's presentation/marker; partial CSVs are retained. */
        if(has_index && nr_path(path,dir,"index.html",NULL)==NR_OK) (void)remove(path);
        if(has_complete && nr_path(path,dir,"COMPLETE.txt",NULL)==NR_OK) (void)remove(path);
        f=nr_new_file(dir,"FAILED.txt",NULL);
        if(f!=NULL) { fprintf(f,"status=%d\nline=%lu\nmessage=%s\nPARTIAL OUTPUT: DO NOT USE AS A COMPLETED RUN\n",status,e?e->line:0,e?e->message:"Replay failed."); (void)nr_close(f,NULL); }
    }
    return status;
}
