#include "neck_pipeline.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static const double two_pi = 6.28318530717958647693;
/* 按采样间隔计算一阶低通权重；expm1 可减小间隔很短时的相减误差。 */
static double filter_alpha(double hz, uint64_t dt_us)
{ return -expm1(-two_pi * hz * ((double)dt_us * 1.0e-6)); }
static int active_zero(const NeckPipelineContext *c)
{ return c->zero_status==NECK_ZERO_COLLECTING || c->zero_status==NECK_ZERO_WAITING_WARMUP; }
static void clear_window(NeckPipelineContext *c)
{
    c->count=0; c->window_start_us=0; c->window_last_us=0;
    memset(c->mean,0,sizeof(c->mean));
    c->min_pitch=c->max_pitch=c->min_lateral=c->max_lateral=0.0;
    c->min_norm=c->max_norm=0.0;
}
static void reset_filter(NeckPipelineContext *c)
{ c->filter_initialized=0; c->reset_pending=1; }
static void abort_zero(NeckPipelineContext *c)
{ if(active_zero(c)) c->zero_status=NECK_ZERO_FAILED_INPUT; }
void neck_pipeline_invalidate(NeckPipelineContext *c, uint8_t status)
{
    if(c==NULL || !c->initialized) return;
    reset_filter(c);
    if(active_zero(c)) c->zero_status=status==NECK_ZERO_TIMED_OUT?NECK_ZERO_TIMED_OUT:NECK_ZERO_FAILED_INPUT;
    clear_window(c);
}
static void progress(const NeckPipelineContext *c, NeckZeroProgress *p)
{
    memset(p,0,sizeof(*p)); p->status=c->zero_status; p->sample_count=c->count;
    if(c->count) {
        p->window_elapsed_us=c->window_last_us-c->window_start_us;
        p->pitch_span_deg=c->max_pitch-c->min_pitch;
        p->lateral_span_deg=c->max_lateral-c->min_lateral;
        p->norm_span_m_s2=c->max_norm-c->min_norm;
    }
}
void neck_pipeline_default_parameters(NeckPipelineParameters *p)
{
    if(p==NULL) return;
    memset(p,0,sizeof(*p));
    p->cutoff_hz=0.5; p->nominal_period_us=10000; p->max_gap_us=30000;
    p->warmup_us=1000000; p->zero_duration_us=2000000; p->zero_min_samples=100;
    p->zero_timeout_us=10000000; p->gravity_m_s2=9.80665;
    p->gravity_tolerance_m_s2=0.8; p->norm_span_m_s2=0.2; p->angle_span_deg=0.5;
}
static int parameters_valid(const NeckPipelineParameters *p)
{
    return p!=NULL && isfinite(p->cutoff_hz) && p->cutoff_hz>0.0 &&
        p->nominal_period_us>0 && p->cutoff_hz<500000.0/(double)p->nominal_period_us &&
        p->max_gap_us>=p->nominal_period_us && p->zero_duration_us>0 &&
        p->zero_min_samples>=2 && p->zero_timeout_us>=p->zero_duration_us &&
        isfinite(p->gravity_m_s2) && p->gravity_m_s2>0.0 &&
        isfinite(p->gravity_tolerance_m_s2) && p->gravity_tolerance_m_s2>0.0 &&
        isfinite(p->norm_span_m_s2) && p->norm_span_m_s2>0.0 &&
        isfinite(p->angle_span_deg) && p->angle_span_deg>0.0;
}
NeckResult neck_pipeline_init(NeckPipelineContext *c, uint32_t config_id,
    uint8_t node_id, const double mount[4], double tolerance,
    const NeckPipelineParameters *p)
{
    NeckResult result;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    memset(c,0,sizeof(*c));
    if(!parameters_valid(p)) return NECK_BAD_CONFIG;
    result=neck_tilt_init(&c->tilt,config_id,node_id,mount,tolerance);
    if(result!=NECK_OK) return result;
    c->parameters=*p; c->nominal_alpha=filter_alpha(p->cutoff_hz,p->nominal_period_us);
    c->initialized=1;
    return NECK_OK;
}
void neck_pipeline_reset_stream(NeckPipelineContext *c, uint8_t clear_reference)
{
    if(c==NULL || !c->initialized) return;
    reset_filter(c); c->stream_bound=0; c->zero_status=NECK_ZERO_IDLE;
    clear_window(c);
    if(clear_reference) c->reference_set=0;
}
NeckResult neck_pipeline_set_reference(NeckPipelineContext *c,
    const NeckNeutralReference *ref)
{
    NeckNeutralReference checked;
    NeckResult result;
    if(c==NULL || ref==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    result=neck_reference_init(&checked,ref->reference_id,ref->config_id,ref->node_id,
        ref->origin,ref->pitch_deg,ref->lateral_deg);
    if(result!=NECK_OK || ref->config_id!=c->tilt.config_id || ref->node_id!=c->tilt.node_id)
        return NECK_BAD_REFERENCE;
    if(c->stream_bound && ref->origin==NECK_REFERENCE_SIMULATION && c->source!=NECK_SOURCE_SIMULATION)
        return NECK_SIM_REFERENCE_ON_HARDWARE;
    c->reference=checked; c->reference_set=1;
    return NECK_OK;
}
NeckResult neck_pipeline_begin_zero(NeckPipelineContext *c, uint32_t reference_id, uint8_t origin)
{
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(reference_id==0 || origin>NECK_REFERENCE_EXTERNAL) return NECK_BAD_REFERENCE;
    if(c->stream_bound && origin==NECK_REFERENCE_SIMULATION && c->source!=NECK_SOURCE_SIMULATION)
        return NECK_SIM_REFERENCE_ON_HARDWARE;
    c->pending_reference_id=reference_id; c->pending_origin=origin;
    c->request_time_set=0; c->zero_status=NECK_ZERO_COLLECTING; clear_window(c);
    return NECK_OK;
}
void neck_pipeline_cancel_zero(NeckPipelineContext *c)
{ if(c!=NULL && c->initialized && active_zero(c)) c->zero_status=NECK_ZERO_CANCELED; }
static NeckResult reject(NeckPipelineContext *c, NeckPipelineOutput *o,
    NeckResult reason, uint8_t clear_reference)
{
    reset_filter(c); abort_zero(c); if(clear_reference) c->reference_set=0;
    memset(o,0,sizeof(*o));
    o->raw.status=o->raw.pitch_status=o->raw.lateral_status=NECK_INVALID;
    o->filtered.status=o->filtered.pitch_status=o->filtered.lateral_status=NECK_INVALID;
    o->raw.flags=o->filtered.flags=NECK_FLAG_INPUT_REJECTED;
    o->filter_flags=NECK_FILTER_RESET|NECK_FILTER_TIMING_REJECTED;
    progress(c,&o->zero);
    return reason;
}
static void filtered_estimate(const NeckPipelineContext *c, NeckPipelineOutput *o,
    uint8_t vector_valid)
{
    NeckRawTilt raw;
    NeckTiltEstimate *e=&o->filtered;
    *e=o->raw;
    e->reference_id=c->reference_set?c->reference.reference_id:0;
    if(c->reference_set) e->valid_mask|=NECK_VALID_REFERENCE;
    else e->valid_mask&=~(uint32_t)NECK_VALID_REFERENCE;
    if(!vector_valid) {
        e->valid_mask&=NECK_VALID_SAMPLE_TIME|NECK_VALID_SAMPLE_AGE|NECK_VALID_REFERENCE;
        memset(e->accel_body_m_s2,0,sizeof(e->accel_body_m_s2));
        e->accel_norm_m_s2=e->total_tilt_deg=e->pitch_raw_deg=e->lateral_raw_deg=0.0;
        e->flexion_deg=e->right_lateral_deg=0.0;
        return;
    }
    e->flags &= NECK_FLAG_QUALITY_NOT_EVALUATED|NECK_FLAG_SAMPLE_TIME_UNKNOWN|
        NECK_FLAG_CALIBRATION_UNKNOWN|NECK_FLAG_CALIBRATION_PENDING;
    e->valid_mask &= NECK_VALID_SAMPLE_TIME|NECK_VALID_SAMPLE_AGE|NECK_VALID_REFERENCE;
    memcpy(e->accel_body_m_s2,c->filtered_vector,sizeof(e->accel_body_m_s2));
    e->valid_mask|=NECK_VALID_BODY;
    (void)neck_tilt_raw(c->filtered_vector,&raw);
    e->status=raw.status; e->pitch_status=raw.pitch_status; e->lateral_status=raw.lateral_status;
    e->valid_mask|=raw.valid_mask; e->flags|=raw.flags;
    e->accel_norm_m_s2=raw.accel_norm_m_s2; e->total_tilt_deg=raw.total_tilt_deg;
    e->pitch_raw_deg=raw.pitch_raw_deg; e->lateral_raw_deg=raw.lateral_raw_deg;
    e->flexion_deg=e->right_lateral_deg=0.0;
    if(raw.status==NECK_INVALID) return;
    if(!c->reference_set) {
        e->status=NECK_UNCALIBRATED; e->flags|=NECK_FLAG_NEUTRAL_UNSET;
        if(raw.valid_mask&NECK_VALID_PITCH_RAW) e->pitch_status=NECK_UNCALIBRATED;
        if(raw.valid_mask&NECK_VALID_LATERAL_RAW) e->lateral_status=NECK_UNCALIBRATED;
    } else {
        if(raw.valid_mask&NECK_VALID_PITCH_RAW) {
            e->flexion_deg=raw.pitch_raw_deg-c->reference.pitch_deg;
            e->valid_mask|=NECK_VALID_FLEXION;
        }
        if(raw.valid_mask&NECK_VALID_LATERAL_RAW) {
            e->right_lateral_deg=raw.lateral_raw_deg-c->reference.lateral_deg;
            e->valid_mask|=NECK_VALID_RIGHT_LATERAL;
        }
    }
}
static uint32_t collect_zero(NeckPipelineContext *c, const NeckTiltEstimate *raw,
    uint64_t time_us, uint8_t warmup)
{
    const NeckPipelineParameters *p=&c->parameters;
    uint32_t flags=0;
    unsigned int i;
    double pitch=raw->pitch_raw_deg, lateral=raw->lateral_raw_deg, norm=raw->accel_norm_m_s2;
    NeckRawTilt neutral;
    if(!active_zero(c)) return 0;
    /* 以调零请求后的首个有效样本开始计时。预热和窗口重启都计入本次请求的超时时间。 */
    if(!c->request_time_set) { c->request_time_set=1; c->request_start_us=time_us; }
    if(time_us-c->request_start_us>p->zero_timeout_us) { c->zero_status=NECK_ZERO_TIMED_OUT; return 0; }
    if(warmup) { c->zero_status=NECK_ZERO_WAITING_WARMUP; return NECK_ZERO_WAIT_WARMUP; }
    c->zero_status=NECK_ZERO_COLLECTING;
    if(!(raw->valid_mask&NECK_VALID_PITCH_RAW) || !(raw->valid_mask&NECK_VALID_LATERAL_RAW)) {
        c->zero_status=NECK_ZERO_FAILED_AMBIGUOUS; return 0;
    }
    /* 模长偏离重力过多时丢弃当前样本；仅窗口波动超限时，用当前样本重新开始。 */
    if(fabs(norm-p->gravity_m_s2)>p->gravity_tolerance_m_s2) {
        clear_window(c); return NECK_ZERO_WINDOW_RESTART;
    }
    if(c->count && (fmax(c->max_pitch,pitch)-fmin(c->min_pitch,pitch)>p->angle_span_deg ||
        fmax(c->max_lateral,lateral)-fmin(c->min_lateral,lateral)>p->angle_span_deg ||
        fmax(c->max_norm,norm)-fmin(c->min_norm,norm)>p->norm_span_m_s2)) {
        clear_window(c); flags|=NECK_ZERO_WINDOW_RESTART;
    }
    if(c->count==UINT32_MAX) { c->zero_status=NECK_ZERO_FAILED_INPUT; return flags; }
    if(c->count==0) {
        c->window_start_us=time_us; c->min_pitch=c->max_pitch=pitch;
        c->min_lateral=c->max_lateral=lateral; c->min_norm=c->max_norm=norm;
    }
    ++c->count; c->window_last_us=time_us;
    c->min_pitch=fmin(c->min_pitch,pitch); c->max_pitch=fmax(c->max_pitch,pitch);
    c->min_lateral=fmin(c->min_lateral,lateral); c->max_lateral=fmax(c->max_lateral,lateral);
    c->min_norm=fmin(c->min_norm,norm); c->max_norm=fmax(c->max_norm,norm);
    for(i=0;i<3;++i) c->mean[i]+=(raw->accel_body_m_s2[i]-c->mean[i])/(double)c->count;
    /* 时长和样本数都达标后，从原始加速度的向量均值计算中立位。 */
    if(time_us-c->window_start_us>=p->zero_duration_us && c->count>=p->zero_min_samples) {
        (void)neck_tilt_raw(c->mean,&neutral);
        if(!(neutral.valid_mask&NECK_VALID_PITCH_RAW) || !(neutral.valid_mask&NECK_VALID_LATERAL_RAW))
            c->zero_status=NECK_ZERO_FAILED_AMBIGUOUS;
        else {
            (void)neck_reference_init(&c->reference,c->pending_reference_id,c->tilt.config_id,
                c->tilt.node_id,c->pending_origin,neutral.pitch_raw_deg,neutral.lateral_raw_deg);
            /* 用同一个窗口均值更新滤波向量，使调零成功这一帧的校正角也从零开始。 */
            c->reference_set=1; memcpy(c->filtered_vector,c->mean,sizeof(c->mean));
            c->zero_status=NECK_ZERO_SUCCEEDED; flags|=NECK_ZERO_COMMITTED;
        }
    }
    return flags;
}
NeckResult neck_pipeline_process(NeckPipelineContext *c, const NeckFrame *frame,
    NeckPipelineOutput *o)
{
    const NeckNodeSample *node;
    NeckResult result;
    uint64_t time_us, dt;
    uint8_t known, gap=0, basis_changed=0, warmup;
    uint32_t flags=0;
    unsigned int i;
    if(o==NULL) return NECK_BAD_ARGUMENT;
    memset(o,0,sizeof(*o));
    o->raw.status=o->raw.pitch_status=o->raw.lateral_status=NECK_INVALID;
    o->filtered.status=o->filtered.pitch_status=o->filtered.lateral_status=NECK_INVALID;
    if(c==NULL || frame==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(frame->config_id!=c->tilt.config_id) return reject(c,o,NECK_CONFIG_MISMATCH,0);
    if(c->stream_bound) {
        if(c->session_id!=frame->session_id || c->source!=frame->source)
            return reject(c,o,NECK_BAD_FRAME,1);
        if(frame->sequence<=c->previous_sequence || frame->frame_time_us<=c->previous_frame_time_us)
            return reject(c,o,NECK_BAD_FRAME,0);
        gap=frame->sequence!=c->previous_sequence+1U ||
            frame->frame_time_us-c->previous_frame_time_us>c->parameters.max_gap_us;
    }
    if((c->reference_set && c->reference.origin==NECK_REFERENCE_SIMULATION && frame->source!=NECK_SOURCE_SIMULATION) ||
       (active_zero(c) && c->pending_origin==NECK_REFERENCE_SIMULATION && frame->source!=NECK_SOURCE_SIMULATION))
        return reject(c,o,NECK_SIM_REFERENCE_ON_HARDWARE,1);
    result=neck_tilt_estimate(&c->tilt,frame,NULL,&o->raw);
    if(result!=NECK_OK) return reject(c,o,result,0);
    node=&frame->nodes[c->tilt.node_id]; known=node->sample_time_known;
    time_us=known?node->sample_time_us:frame->frame_time_us;
    if(!known) flags|=NECK_FILTER_FRAME_TIME_FALLBACK;
    if(c->filter_initialized) {
        basis_changed=known!=c->time_basis_known;
        if(!basis_changed && time_us<=c->previous_time_us) return reject(c,o,NECK_BAD_FRAME,0);
        if(!basis_changed && time_us-c->previous_time_us>c->parameters.max_gap_us) gap=1;
    }
    /* 遇到数据断档或采样时间与帧时间切换时，重新初始化滤波并终止正在进行的调零。 */
    if(gap || basis_changed) {
        reset_filter(c); abort_zero(c); flags|=NECK_FILTER_RESET;
        if(basis_changed) flags|=NECK_FILTER_TIME_BASIS_CHANGED;
    }
    c->stream_bound=1; c->source=frame->source; c->session_id=frame->session_id;
    c->previous_sequence=frame->sequence; c->previous_frame_time_us=frame->frame_time_us;
    if(o->raw.status==NECK_INVALID || o->raw.status==NECK_NO_ACCEL) {
        reset_filter(c); abort_zero(c); filtered_estimate(c,o,0);
        o->filter_flags=flags|NECK_FILTER_DATA_INVALID|NECK_FILTER_RESET;
        progress(c,&o->zero); return NECK_OK;
    }
    /* 首个有效样本直接作为滤波初值；后续按实际时间间隔更新三个加速度分量。 */
    o->alpha_valid=1;
    if(!c->filter_initialized) {
        memcpy(c->filtered_vector,o->raw.accel_body_m_s2,sizeof(c->filtered_vector));
        c->filter_start_us=time_us; c->filter_initialized=1; o->alpha=1.0;
        flags|=NECK_FILTER_SEEDED;
        if(c->reset_pending) flags|=NECK_FILTER_RESET;
        c->reset_pending=0;
    } else {
        dt=time_us-c->previous_time_us;
        o->alpha=dt==c->parameters.nominal_period_us?c->nominal_alpha:filter_alpha(c->parameters.cutoff_hz,dt);
        for(i=0;i<3;++i)
            c->filtered_vector[i]=(1.0-o->alpha)*c->filtered_vector[i]+o->alpha*o->raw.accel_body_m_s2[i];
        flags|=NECK_FILTER_UPDATED;
    }
    c->previous_time_us=time_us; c->time_basis_known=known;
    warmup=time_us-c->filter_start_us<c->parameters.warmup_us;
    if(warmup) flags|=NECK_FILTER_WARMUP;
    flags|=collect_zero(c,&o->raw,time_us,warmup);
    filtered_estimate(c,o,1);
    if(o->filtered.status==NECK_INVALID) {
        reset_filter(c); abort_zero(c); flags|=NECK_FILTER_DATA_INVALID|NECK_FILTER_RESET;
    }
    o->filter_flags=flags; progress(c,&o->zero);
    return NECK_OK;
}
