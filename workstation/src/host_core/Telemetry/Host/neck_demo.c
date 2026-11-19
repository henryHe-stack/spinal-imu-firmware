/* PC simulator uses the actual S03-S07 C owner, then the D0 wire codec.
 * Host scheduling/commands are NOT an implementation of MCU UART/FreeRTOS. */
#include "neck_simulator.h"
#include "neck_wire_host.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>
#include <windows.h>
typedef struct {
    NeckProcessingContext core; NeckProcessingParameters parameters;
    NeckAcquisitionFrame input; NeckWireMessage message; FILE *output;
    uint32_t tx,boot,input_drops,telemetry_drops,event_drops;
    uint64_t event_id,last_c4,last_rel,last_status,last_session;
    int single,input_off,tx_off,c4_fault,quit,script,had_output,single_zero_pending;
    char command[128]; unsigned command_length;
} Demo;
static void stamp(Demo *d,uint8_t type,uint64_t now)
{
    memset(&d->message,0,sizeof(d->message)); d->message.header.type=type;
    d->message.header.tx_sequence=d->tx++; d->message.header.boot_id=d->boot;
    d->message.header.session_id=1; d->message.header.config_id=1; d->message.header.emit_time_us=now;
}
static int transmit(Demo *d,int tx_off)
{
    uint8_t bytes[NECK_WIRE_MAX_FRAME]; size_t n=0;
    NeckWireResult r=neck_wire_encode(&d->message,bytes,sizeof(bytes),&n);
    if(r!=NECK_WIRE_OK) { fprintf(stderr,"ENCODE_ERROR type=%u code=%d\n",d->message.header.type,r); return 0; }
    if(tx_off) { ++d->telemetry_drops; if(d->message.header.type==NECK_WIRE_EVENT) ++d->event_drops; return 1; }
    if(fwrite(bytes,1,n,d->output)!=n || fflush(d->output)) { fprintf(stderr,"OUTPUT_ERROR\n"); return 0; }
    return 1;
}
static int session(Demo *d,uint64_t now,int tx_off)
{
    NeckWireSession *s; NeckProcessingParameters *p=&d->parameters; stamp(d,NECK_WIRE_SESSION,now); s=&d->message.body.session;
    s->profile_id=1; s->firmware_id=0xd1000001u; s->world_frame_id=1; s->source=0;
    s->mode=d->single?1:3; s->node_mask=d->single?2:7; s->reference_origin=0;
    s->sample_period_us=(uint32_t)p->pipeline.nominal_period_us; s->quality_timeout_us=(uint32_t)p->quality.no_frame_timeout_us;
    s->cutoff_millihz=(uint32_t)llround(p->pipeline.cutoff_hz*1000); s->continuous_duration_us=p->posture.continuous_duration_us;
    s->thresholds_mdeg[0]=(int32_t)llround(p->posture.flexion_warning_deg*1000);
    s->thresholds_mdeg[1]=(int32_t)llround(p->posture.flexion_danger_deg*1000);
    s->thresholds_mdeg[2]=(int32_t)llround(p->posture.lateral_warning_deg*1000);
    s->thresholds_mdeg[3]=(int32_t)llround(p->posture.lateral_danger_deg*1000);
    s->max_node_skew_us=(uint32_t)p->relative.max_node_skew_us; s->c4_period_us=50000;
    s->relative_period_us=d->single?0:100000; s->status_period_us=500000; s->capabilities=d->single?13:15;
    d->last_session=now; return transmit(d,tx_off);
}
static void angle(int32_t *v,uint16_t *mask,uint16_t bit,double value,int valid)
{ if(valid && isfinite(value) && value>=-360 && value<=360) { *v=(int32_t)llround(value*1000); *mask|=bit; } }
static void axis(NeckWireAxis *w,const NeckPostureAxisOutput *a)
{
    w->valid=a->valid; w->instant_level=a->instant_level; w->alarm_level=a->alarm_level; w->direction=a->direction;
    w->warning_elapsed_us=a->warning_elapsed_us; w->danger_elapsed_us=a->danger_elapsed_us; w->extension_elapsed_us=a->extension_elapsed_us;
}
static int output(Demo *d,const NeckProcessingOutput *o,uint64_t now,int tx_off,int force)
{
    unsigned i,j; const NeckQualityOutput *q=&o->pose.quality; const NeckPostureAxisOutput *axes[2]={&o->pose.posture.flexion,&o->pose.posture.lateral};
    for(i=0;i<2;++i) if(axes[i]->events) {
        NeckWireEvent *e; stamp(d,NECK_WIRE_EVENT,now); e=&d->message.body.event;
        e->event_id=++d->event_id; e->input_sequence=o->pose.posture.sequence; e->frame_time_us=o->pose.posture.frame_time_us;
        e->event_time_us=o->pose.posture.time_us; e->reference_id=o->pose.posture.reference_id;
        e->invalidation_epoch=o->pose.invalidation_epoch; e->events=axes[i]->events; e->posture_flags=o->pose.posture.flags;
        e->axis=(uint8_t)i; e->previous_level=axes[i]->previous_alarm_level; e->level=axes[i]->alarm_level; e->direction=axes[i]->direction;
        e->time_flags=3; if(!transmit(d,tx_off)) return 0;
    }
    if(force || !d->had_output || now-d->last_c4>=50000) {
        NeckWireC4 *c; const NeckTiltEstimate *raw=&o->c4.raw,*f=&o->c4.filtered;
        stamp(d,NECK_WIRE_C4,now); c=&d->message.body.c4;
        c->input_sequence=q->sequence; c->frame_time_us=q->frame_time_us; c->decision_time_us=q->decision_time_us;
        c->valid_until_us=o->pose.valid_until_us; c->invalidation_epoch=o->pose.invalidation_epoch; c->reference_id=f->reference_id;
        c->quality_flags=q->flags; c->filter_flags=o->c4.filter_flags; c->posture_flags=o->pose.posture.flags;
        c->time_flags=(uint8_t)((q->flags&NECK_Q_NO_FRAME)?6:7); c->zero_status=o->c4.zero.status;
        angle(&c->angles_mdeg[0],&c->valid_mask,1,raw->pitch_raw_deg,raw->valid_mask&NECK_VALID_PITCH_RAW);
        angle(&c->angles_mdeg[1],&c->valid_mask,2,raw->lateral_raw_deg,raw->valid_mask&NECK_VALID_LATERAL_RAW);
        angle(&c->angles_mdeg[2],&c->valid_mask,4,f->pitch_raw_deg,f->valid_mask&NECK_VALID_PITCH_RAW);
        angle(&c->angles_mdeg[3],&c->valid_mask,8,f->lateral_raw_deg,f->valid_mask&NECK_VALID_LATERAL_RAW);
        angle(&c->angles_mdeg[4],&c->valid_mask,16,f->flexion_deg,f->valid_mask&NECK_VALID_FLEXION);
        angle(&c->angles_mdeg[5],&c->valid_mask,32,f->right_lateral_deg,f->valid_mask&NECK_VALID_RIGHT_LATERAL);
        axis(&c->axes[0],axes[0]); axis(&c->axes[1],axes[1]);
        if(!(c->valid_mask&16)) memset(&c->axes[0],0,sizeof(c->axes[0]));
        if(!(c->valid_mask&32)) memset(&c->axes[1],0,sizeof(c->axes[1]));
        if(!transmit(d,tx_off)) return 0;
        d->last_c4=now;
    }
    if(!d->single && (force || !d->had_output || now-d->last_rel>=100000)) {
        NeckWireRelative *r; stamp(d,NECK_WIRE_RELATIVE,now); r=&d->message.body.relative;
        r->input_sequence=q->sequence; r->frame_time_us=q->frame_time_us; r->decision_time_us=q->decision_time_us;
        r->valid_until_us=o->pose.valid_until_us; r->invalidation_epoch=o->pose.invalidation_epoch;
        r->reference_id=o->relative.reference_id; r->quality_flags=q->flags; r->world_frame_id=o->relative.world_frame_id;
        r->time_flags=(uint8_t)((q->flags&NECK_Q_NO_FRAME)?6:7);
        for(i=0;i<4;++i) {
            const NeckRelativePairOutput *p=&o->relative.pairs[i]; NeckWirePair *w=&r->pairs[i];
            w->flags=p->flags; w->status=p->status; w->sample_skew_us=p->sample_skew_us;
            if(p->valid_mask&NECK_REL_KNOWN_SKEW) w->valid_mask|=4;
            for(j=0;j<3;++j) { w->raw_mdeg[j]=(int32_t)llround(p->raw_euler_deg[j]*1000); w->corrected_mdeg[j]=(int32_t)llround(p->corrected_euler_deg[j]*1000); }
            if(p->valid_mask&NECK_REL_RAW_EULER) w->valid_mask|=1;
            if(p->valid_mask&NECK_REL_CORRECTED_EULER) w->valid_mask|=2;
        }
        if(!transmit(d,tx_off)) return 0;
        d->last_rel=now;
    }
    if(force || !d->had_output || now-d->last_status>=500000) {
        NeckWireStatus *s; stamp(d,NECK_WIRE_STATUS,now); s=&d->message.body.status;
        s->input_sequence=q->sequence; s->frame_time_us=q->frame_time_us; s->decision_time_us=q->decision_time_us;
        s->invalidation_epoch=o->pose.invalidation_epoch; s->quality_flags=q->flags; s->accel_usable_mask=q->accel_usable_mask;
        s->orientation_usable_mask=q->orientation_usable_mask; s->quality_state=q->state; s->frame_accepted=q->frame_accepted;
        s->pending_reference_id=o->pose.pending_reference_id; s->zero_request_state=o->pose.zero_request_state;
        s->input_drops=d->input_drops; s->telemetry_drops=d->telemetry_drops; s->event_drops=d->event_drops;
        for(i=0;i<3;++i) {
            NeckWireNode *n=&s->nodes[i]; const NeckNodeSample *a=&d->input.acceleration.nodes[i];
            const NeckOrientationSample *b=&d->input.orientation.nodes[i];
            n->accel_sample_time_us=a->sample_time_us; n->orientation_sample_time_us=b->sample_time_us;
            n->time_flags=(uint8_t)((a->sample_time_known?1:0)|(b->sample_time_known?2:0));
            n->quality_flags=q->node_flags[i]; n->filter_flags=o->pose.filter_flags[i]; n->reference_id=o->pose.nodes[i].reference_id;
            n->status=(q->flags&NECK_Q_TIMEOUT)?NECK_NODE_STALE:a->status;
            n->accel_calibration=a->calibration_accel; n->system_calibration=b->calibration_system; n->zero_status=o->pose.zero_status[i];
        }
        if(!transmit(d,tx_off)) return 0;
        d->last_status=now;
    }
    d->had_output=1; return 1;
}
static void zero(Demo *d,uint64_t now)
{
    NeckResult r;
    if(d->single) { d->single_zero_pending=1; r=NECK_OK; }
    else r=neck_processing_request_zero(&d->core,now,NECK_REFERENCE_SIMULATION);
    fprintf(stderr,"SIM_ZERO accepted=%d time_us=%" PRIu64 " (accepted is not committed)\n",r==NECK_OK,now); fflush(stderr);
}
static void commands(Demo *d,uint64_t now)
{
    HANDLE pipe=GetStdHandle(STD_INPUT_HANDLE); DWORD available=0,n; char ch;
    if(!PeekNamedPipe(pipe,NULL,0,NULL,&available,NULL)) return;
    while(available--) {
        if(!ReadFile(pipe,&ch,1,&n,NULL) || n!=1) break;
        if(ch=='\n') {
            d->command[d->command_length]='\0';
            if(!strcmp(d->command,"ZERO")) zero(d,now);
            else if(!strcmp(d->command,"CANCEL")) { if(d->single) { d->single_zero_pending=0; neck_pipeline_cancel_zero(&d->core.pipelines[1]); } else neck_processing_cancel_zero(&d->core); }
            else if(!strcmp(d->command,"INPUT_OFF")) d->input_off=1;
            else if(!strcmp(d->command,"INPUT_ON")) d->input_off=0;
            else if(!strcmp(d->command,"TX_OFF")) d->tx_off=1;
            else if(!strcmp(d->command,"TX_ON")) d->tx_off=0;
            else if(!strcmp(d->command,"FAULT_C4")) d->c4_fault=1;
            else if(!strcmp(d->command,"RESTORE_C4")) d->c4_fault=0;
            else if(!strcmp(d->command,"QUIT")) d->quit=1;
            fprintf(stderr,"SIM_COMMAND %s time_us=%" PRIu64 "\n",d->command,now); fflush(stderr); d->command_length=0;
        } else if(ch!='\r' && d->command_length<sizeof(d->command)-1) d->command[d->command_length++]=ch;
    }
}
static void absent(NeckAcquisitionFrame *f,unsigned i)
{
    memset(&f->acceleration.nodes[i],0,sizeof(NeckNodeSample)); memset(&f->orientation.nodes[i],0,sizeof(NeckOrientationSample));
    f->acceleration.nodes[i].status=f->orientation.nodes[i].status=NECK_NODE_MISSING;
    f->acceleration.nodes[i].calibration_accel=f->orientation.nodes[i].calibration_system=255;
    f->orientation.nodes[i].world_frame_id=1;
}
static int run(Demo *d,int realtime,uint64_t duration)
{
    LARGE_INTEGER frequency,start,current; uint64_t step=0,last=UINT64_MAX; int first=1;
    HANDLE pacing=NULL;
    if(realtime) {
        /* Process-owned high-resolution wait; do not relax the shared C quality
         * gate or manufacture missed 100 Hz samples when Windows is late. */
        pacing=CreateWaitableTimerExW(NULL,NULL,0x00000002u,TIMER_ALL_ACCESS);
        if(!pacing) { fprintf(stderr,"HIGH_RESOLUTION_TIMER_UNAVAILABLE\n"); return 0; }
    }
    QueryPerformanceFrequency(&frequency); QueryPerformanceCounter(&start);
    if(d->script) zero(d,0); /* explicit scripted request, NOT algorithm auto-zero */
    while(!d->quit) {
        uint64_t now,ticks,sample; int off,fault,tx_off; NeckProcessingOutput result;
        if(realtime) {
            QueryPerformanceCounter(&current); ticks=(uint64_t)(current.QuadPart-start.QuadPart);
            now=(ticks/(uint64_t)frequency.QuadPart)*1000000+(ticks%(uint64_t)frequency.QuadPart)*1000000/(uint64_t)frequency.QuadPart;
            commands(d,now); step=now/10000;
            if(last==step) {
                LARGE_INTEGER due; uint64_t wait_us=(last+1)*10000-now;
                if(wait_us>1000) wait_us=1000;
                due.QuadPart=-(LONGLONG)(wait_us*10);
                if(!SetWaitableTimer(pacing,&due,0,NULL,NULL,FALSE)) { CloseHandle(pacing); return 0; }
                (void)WaitForSingleObject(pacing,20); continue;
            }
            if(last!=UINT64_MAX && step>last+1) d->input_drops+=(uint32_t)(step-last-1);
        } else now=step*10000;
        if(duration && now>=duration) break;
        sample=step*10000; last=step;
        off=d->input_off || (d->script && now>=34000000 && now<35000000);
        fault=d->c4_fault || (d->script && now>=45000000 && now<47000000);
        tx_off=d->tx_off || (d->script && now>=55000000 && now<56000000);
        if(first || now-d->last_session>=1000000) { if(!session(d,now,tx_off)) return 0; first=0; }
        if(neck_processing_poll(&d->core,now,&result) && !output(d,&result,now,tx_off,(result.step_flags&NECK_STEP_TIMEOUT_EDGE)!=0)) return 0;
        if(!off) {
            neck_simulator_frame(&d->input,step,sample);
            if(d->single) { absent(&d->input,0); absent(&d->input,2); }
            if(fault) absent(&d->input,1);
            if(neck_processing_process(&d->core,&d->input,now,&result)!=NECK_OK) { fprintf(stderr,"CORE_REJECTED\n"); return 0; }
            if(d->single_zero_pending && (result.pose.quality.accel_usable_mask&2)) {
                if(neck_pipeline_begin_zero(&d->core.pipelines[1],d->core.next_reference_id++,NECK_REFERENCE_SIMULATION)!=NECK_OK) return 0;
                d->single_zero_pending=0;
            }
            if(!output(d,&result,now,tx_off,0)) return 0;
        }
        if(!realtime) ++step;
    }
    fprintf(stderr,"SIM_END zero_commits=%" PRIu32 " relative_commits=%" PRIu32 " timeouts=%" PRIu32 "\n",
        d->core.stats.node_commits,d->core.stats.relative_commits,d->core.stats.timeout_transitions);
    if(pacing) CloseHandle(pacing);
    return 1;
}
static void dump(const NeckWireMessage *m,void *unused)
{ char row[2048]; (void)unused; if(neck_wire_csv(m,row,sizeof(row))) puts(row); }
int main(int argc,char **argv)
{
    Demo d; uint64_t duration=65000000; int realtime=0,i,fd=-1,ok; const char *generate=NULL,*dump_path=NULL;
    memset(&d,0,sizeof(d)); d.boot=101; d.output=stdout;
    for(i=1;i<argc;++i) {
        if(!strcmp(argv[i],"--realtime")) realtime=1;
        else if(!strcmp(argv[i],"--script")) d.script=1;
        else if(!strcmp(argv[i],"--mode") && i+1<argc) { const char *mode=argv[++i]; if(!strcmp(mode,"single")) d.single=1; else if(strcmp(mode,"three")) return 2; }
        else if(!strcmp(argv[i],"--boot") && i+1<argc) d.boot=(uint32_t)strtoul(argv[++i],NULL,10);
        else if(!strcmp(argv[i],"--duration") && i+1<argc) { char *end; double seconds=strtod(argv[++i],&end);
            if(*end || !isfinite(seconds) || seconds<0 || seconds>86400) return 2;
            duration=(uint64_t)llround(seconds*1000000); }
        else if(!strcmp(argv[i],"--generate") && i+1<argc) generate=argv[++i];
        else if(!strcmp(argv[i],"--dump") && i+1<argc) dump_path=argv[++i];
        else { fprintf(stderr,"--realtime --mode single|three --script --boot ID --duration seconds --generate new.bin | --dump file.bin\n"); return 2; }
    }
    if(dump_path) {
        NeckWireParser parser; uint8_t bytes[4096]; size_t n; FILE *f=fopen(dump_path,"rb"); if(!f) return 3;
        neck_wire_parser_init(&parser); while((n=fread(bytes,1,sizeof(bytes),f))!=0) (void)neck_wire_parser_feed(&parser,bytes,n,dump,NULL);
        ok=!ferror(f); (void)neck_wire_parser_gap(&parser,dump,NULL); fclose(f); return ok?0:3;
    }
    if(!d.boot || (!realtime && duration==0)) return 2;
    if(generate) { fd=_open(generate,_O_CREAT|_O_EXCL|_O_WRONLY|_O_BINARY,_S_IREAD|_S_IWRITE); if(fd<0) return 3;
        d.output=_fdopen(fd,"wb"); if(!d.output) { _close(fd); return 3; } }
    else if(_setmode(_fileno(stdout),_O_BINARY)<0) return 3;
    neck_processing_default_parameters(&d.parameters);
    if(neck_processing_init(&d.core,1,1,neck_simulator_mounts(),0,&d.parameters)!=NECK_OK) return 4;
    ok=run(&d,realtime,duration); if(generate && fclose(d.output)) ok=0; return ok?0:4;
}
