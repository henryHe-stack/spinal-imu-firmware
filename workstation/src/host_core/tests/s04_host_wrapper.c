/* Host-only bridge. Allocation here is not part of STM32 production code. */
#include "neck_pipeline.h"
#include <stdlib.h>
#include <string.h>
typedef struct { uint64_t meta[22]; double values[22]; } HostOutput;
void *s04_create(const double mount[4], unsigned int node)
{
    NeckPipelineParameters p;
    NeckPipelineContext *c=(NeckPipelineContext *)malloc(sizeof(*c));
    if(c==NULL) return NULL;
    neck_pipeline_default_parameters(&p);
    if(neck_pipeline_init(c,1,(uint8_t)node,mount,0.001,&p)!=NECK_OK) { free(c); return NULL; }
    return c;
}
void s04_destroy(void *handle) { free(handle); }
int s04_zero(void *handle, unsigned int id, unsigned int origin)
{ return (int)neck_pipeline_begin_zero((NeckPipelineContext *)handle,id,(uint8_t)origin); }
void s04_cancel(void *handle) { neck_pipeline_cancel_zero((NeckPipelineContext *)handle); }
void s04_reset(void *handle, unsigned int clear)
{ neck_pipeline_reset_stream((NeckPipelineContext *)handle,(uint8_t)clear); }
int s04_reference(void *handle, unsigned int id, unsigned int origin, double pitch, double side)
{
    NeckPipelineContext *c=(NeckPipelineContext *)handle;
    NeckNeutralReference r;
    NeckResult result=neck_reference_init(&r,id,c->tilt.config_id,c->tilt.node_id,
        (uint8_t)origin,pitch,side);
    return result==NECK_OK?(int)neck_pipeline_set_reference(c,&r):(int)result;
}
static void numbers(const NeckTiltEstimate *e, double *v)
{
    unsigned int i;
    for(i=0;i<3;++i) v[i]=e->accel_body_m_s2[i];
    v[3]=e->accel_norm_m_s2; v[4]=e->total_tilt_deg;
    v[5]=e->pitch_raw_deg; v[6]=e->lateral_raw_deg;
}
int s04_step(void *handle, unsigned int config, unsigned int session,
    uint64_t sequence, uint64_t frame_time, uint64_t sample_time,
    unsigned int known, unsigned int source, unsigned int status,
    unsigned int mask, unsigned int calibration, const double accel[3], HostOutput *out)
{
    NeckPipelineContext *c=(NeckPipelineContext *)handle;
    NeckFrame frame;
    NeckNodeSample *node;
    NeckPipelineOutput result;
    NeckResult code;
    unsigned int i;
    memset(&frame,0,sizeof(frame)); memset(out,0,sizeof(*out));
    frame.config_id=config; frame.session_id=session; frame.sequence=sequence;
    frame.frame_time_us=frame_time; frame.source=(uint8_t)source;
    for(i=0;i<NECK_NODE_COUNT;++i) {
        frame.nodes[i].status=NECK_NODE_MISSING;
        frame.nodes[i].calibration_accel=NECK_CALIBRATION_UNKNOWN;
    }
    node=&frame.nodes[c->tilt.node_id];
    node->status=(uint8_t)status; node->signal_valid_mask=(uint8_t)mask;
    node->accel_present=(mask&NECK_SIGNAL_ACCEL)!=0;
    node->sample_time_known=(uint8_t)known; node->sample_time_us=sample_time;
    node->calibration_accel=(uint8_t)calibration;
    memcpy(node->accel_m_s2,accel,sizeof(node->accel_m_s2));
    code=neck_pipeline_process(c,&frame,&result);
    out->meta[0]=code; out->meta[1]=result.filter_flags; out->meta[2]=result.alpha_valid;
    out->meta[3]=result.raw.valid_mask; out->meta[4]=result.filtered.valid_mask;
    out->meta[5]=result.raw.status; out->meta[6]=result.raw.pitch_status; out->meta[7]=result.raw.lateral_status;
    out->meta[8]=result.filtered.status; out->meta[9]=result.filtered.pitch_status; out->meta[10]=result.filtered.lateral_status;
    out->meta[11]=result.raw.flags; out->meta[12]=result.filtered.flags;
    out->meta[13]=result.zero.status; out->meta[14]=result.zero.sample_count;
    out->meta[15]=result.zero.window_elapsed_us; out->meta[16]=result.filtered.reference_id;
    out->meta[17]=result.filtered.sample_time_us; out->meta[18]=result.filtered.sample_age_us;
    out->meta[19]=result.filtered.sequence; out->meta[20]=result.filtered.frame_time_us;
    out->meta[21]=c->reference_set;
    numbers(&result.raw,out->values); numbers(&result.filtered,out->values+7);
    out->values[14]=result.filtered.flexion_deg; out->values[15]=result.filtered.right_lateral_deg;
    out->values[16]=result.alpha;
    out->values[17]=result.zero.pitch_span_deg; out->values[18]=result.zero.lateral_span_deg;
    out->values[19]=result.zero.norm_span_m_s2;
    out->values[20]=c->reference.pitch_deg; out->values[21]=c->reference.lateral_deg;
    return (int)code;
}
