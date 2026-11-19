#include "neck_posture.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static int abnormal(uint8_t level)
{
    return level==NECK_POSTURE_WARNING || level==NECK_POSTURE_DANGER ||
        level==NECK_POSTURE_EXTENSION;
}
static void clear_clocks(NeckPostureAxisState *s)
{
    s->warning_running=s->danger_running=s->extension_running=0;
    s->warning_start_us=s->danger_start_us=s->extension_start_us=0;
}
static void invalidate_axis(NeckPostureAxisState *s, NeckPostureAxisOutput *o)
{
    memset(o,0,sizeof(*o));
    o->previous_alarm_level=s->alarm_level;
    if(s->known) o->events|=NECK_EVENT_INVALIDATED;
    if(abnormal(s->alarm_level)) o->events|=NECK_EVENT_CLEAR;
    clear_clocks(s); s->known=0; s->direction=0;
    s->alarm_level=NECK_POSTURE_UNKNOWN;
}
static void invalidate(NeckPostureContext *c, NeckPostureOutput *o)
{
    invalidate_axis(&c->flexion,&o->flexion);
    invalidate_axis(&c->lateral,&o->lateral);
}
NeckResult neck_posture_invalidate(NeckPostureContext *c, uint64_t now,
    uint32_t reasons, NeckPostureOutput *o)
{
    if(o==NULL) return NECK_BAD_ARGUMENT;
    memset(o,0,sizeof(*o));
    o->flags=NECK_POSTURE_QUALITY_NOT_EVALUATED|NECK_POSTURE_INPUT_INVALID|reasons;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    o->config_id=c->config_id; o->session_id=c->session_id;
    o->reference_id=c->reference_id; o->source=c->source; o->node_id=c->node_id;
    o->sequence=c->previous_sequence; o->frame_time_us=c->previous_frame_time_us;
    o->time_us=now; invalidate(c,o); return NECK_OK;
}
void neck_posture_default_parameters(NeckPostureParameters *p)
{
    if(p==NULL) return;
    p->flexion_warning_deg=15.0; p->flexion_danger_deg=30.0;
    p->lateral_warning_deg=5.0; p->lateral_danger_deg=10.0;
    p->continuous_duration_us=10000000; p->max_gap_us=30000;
}
static int parameters_valid(const NeckPostureParameters *p)
{
    return p!=NULL && isfinite(p->flexion_warning_deg) &&
        isfinite(p->flexion_danger_deg) && p->flexion_warning_deg>0.0 &&
        p->flexion_danger_deg>p->flexion_warning_deg &&
        isfinite(p->lateral_warning_deg) && isfinite(p->lateral_danger_deg) &&
        p->lateral_warning_deg>0.0 && p->lateral_danger_deg>p->lateral_warning_deg &&
        p->continuous_duration_us>0 && p->max_gap_us>0;
}
NeckResult neck_posture_init(NeckPostureContext *c, uint32_t config_id,
    uint8_t node_id, const NeckPostureParameters *p)
{
    if(c==NULL) return NECK_BAD_ARGUMENT;
    memset(c,0,sizeof(*c));
    if(!parameters_valid(p) || config_id==0 || node_id>=NECK_NODE_COUNT)
        return NECK_BAD_CONFIG;
    c->parameters=*p; c->config_id=config_id; c->node_id=node_id;
    c->initialized=1;
    return NECK_OK;
}
void neck_posture_reset(NeckPostureContext *c)
{
    if(c==NULL || !c->initialized) return;
    memset(&c->flexion,0,sizeof(c->flexion));
    memset(&c->lateral,0,sizeof(c->lateral));
    c->stream_bound=0;
}
static uint64_t timer(uint8_t condition, uint8_t *running,
    uint64_t *start_us, uint64_t time_us)
{
    if(!condition) { *running=0; *start_us=0; return 0; }
    if(!*running) { *running=1; *start_us=time_us; }
    return time_us-*start_us;
}
static void axis_step(NeckPostureAxisState *s, NeckPostureAxisOutput *o,
    const NeckPostureParameters *p, double angle, uint64_t time_us, uint8_t lateral)
{
    uint8_t warning, danger, extension=0, next=NECK_POSTURE_NORMAL;
    uint8_t old=s->alarm_level;
    int8_t direction=0;
    memset(o,0,sizeof(*o)); o->valid=1; o->previous_alarm_level=old;
    if(lateral) {
        double magnitude=fabs(angle);
        /* 侧屈按角度绝对值判断：恰好等于警告阈值仍属正常，恰好等于危险阈值已进入危险范围。 */
        warning=magnitude>p->lateral_warning_deg;
        danger=magnitude>=p->lateral_danger_deg;
        if(warning) direction=angle>0.0?1:-1;
    } else {
        extension=angle<0.0;
        warning=angle>=p->flexion_warning_deg;
        danger=angle>=p->flexion_danger_deg;
        if(extension) direction=-1;
        else if(warning) direction=1;
    }
    o->instant_level=extension?NECK_POSTURE_EXTENSION:
        (danger?NECK_POSTURE_DANGER:(warning?NECK_POSTURE_WARNING:NECK_POSTURE_NORMAL));
    /* 左右侧屈换向或屈伸反转时重新计时，两个方向的持续时间不能相加。 */
    if(direction!=0 && s->direction!=0 && direction!=s->direction) {
        clear_clocks(s); o->events|=NECK_EVENT_DIRECTION_RESET;
    }
    /* 达到危险阈值时，警告和危险分别累计各自连续满足条件的时间。 */
    o->warning_elapsed_us=timer(warning,&s->warning_running,&s->warning_start_us,time_us);
    o->danger_elapsed_us=timer(danger,&s->danger_running,&s->danger_start_us,time_us);
    o->extension_elapsed_us=timer(extension,&s->extension_running,&s->extension_start_us,time_us);
    if(danger && o->danger_elapsed_us>=p->continuous_duration_us) next=NECK_POSTURE_DANGER;
    else if(warning && o->warning_elapsed_us>=p->continuous_duration_us) next=NECK_POSTURE_WARNING;
    else if(extension && o->extension_elapsed_us>=p->continuous_duration_us) next=NECK_POSTURE_EXTENSION;
    if(abnormal(old)) {
        if(!abnormal(next)) o->events|=NECK_EVENT_CLEAR;
        else if(old!=next) o->events|=NECK_EVENT_LEVEL_CHANGE;
    } else if(abnormal(next)) o->events|=NECK_EVENT_TRIGGER;
    s->known=1; s->direction=direction; s->alarm_level=next;
    o->direction=direction; o->alarm_level=next;
}
static NeckResult reject(NeckPostureContext *c, NeckPostureOutput *o, NeckResult reason)
{
    o->flags|=NECK_POSTURE_INPUT_INVALID|NECK_POSTURE_TIMING_REJECTED;
    invalidate(c,o);
    return reason;
}
NeckResult neck_posture_process(NeckPostureContext *c,
    const NeckPipelineOutput *input, NeckPostureOutput *o)
{
    const NeckTiltEstimate *e;
    uint64_t time_us;
    uint8_t known, boundary=0;
    uint32_t flags=NECK_POSTURE_QUALITY_NOT_EVALUATED;
    if(o==NULL) return NECK_BAD_ARGUMENT;
    memset(o,0,sizeof(*o)); o->flags=flags;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(input==NULL) return reject(c,o,NECK_BAD_ARGUMENT);
    e=&input->filtered;
    known=(e->valid_mask&NECK_VALID_SAMPLE_TIME)!=0;
    time_us=known?e->sample_time_us:e->frame_time_us;
    o->config_id=e->config_id; o->session_id=e->session_id; o->reference_id=e->reference_id;
    o->sequence=e->sequence; o->frame_time_us=e->frame_time_us; o->time_us=time_us;
    o->source=e->source; o->node_id=e->node_id; o->sample_time_known=known;
    if(!known) o->flags|=NECK_POSTURE_FRAME_TIME_FALLBACK;
    if(e->config_id!=c->config_id) return reject(c,o,NECK_CONFIG_MISMATCH);
    if(e->node_id!=c->node_id || e->session_id==0 || e->source>NECK_SOURCE_HARDWARE ||
        (known && e->sample_time_us>e->frame_time_us) ||
        (known && (e->valid_mask&NECK_VALID_SAMPLE_AGE) &&
            e->sample_age_us!=e->frame_time_us-e->sample_time_us))
        return reject(c,o,NECK_BAD_FRAME);
    if(c->stream_bound) {
        if(e->session_id!=c->session_id || e->source!=c->source)
            return reject(c,o,NECK_BAD_FRAME);
        if(e->sequence<=c->previous_sequence || e->frame_time_us<=c->previous_frame_time_us ||
            (known==c->time_basis_known && time_us<=c->previous_time_us))
            return reject(c,o,NECK_BAD_FRAME);
        if(e->sequence!=c->previous_sequence+1U ||
            e->frame_time_us-c->previous_frame_time_us>c->parameters.max_gap_us ||
            known!=c->time_basis_known ||
            (known==c->time_basis_known && time_us-c->previous_time_us>c->parameters.max_gap_us)) {
            boundary=1; o->flags|=NECK_POSTURE_STREAM_RESET;
        }
        if(e->reference_id!=c->reference_id) {
            boundary=1; o->flags|=NECK_POSTURE_REFERENCE_CHANGED;
        }
    }
    c->stream_bound=1; c->session_id=e->session_id; c->source=e->source;
    c->reference_id=e->reference_id; c->previous_sequence=e->sequence;
    c->previous_frame_time_us=e->frame_time_us; c->previous_time_us=time_us;
    c->time_basis_known=known;
    if(input->filter_flags&(NECK_FILTER_RESET|NECK_ZERO_COMMITTED|NECK_FILTER_TIME_BASIS_CHANGED)) {
        boundary=1; o->flags|=NECK_POSTURE_PIPELINE_RESET;
    }
    if(input->filter_flags&NECK_FILTER_WARMUP) o->flags|=NECK_POSTURE_WARMUP;
    if(!(e->valid_mask&NECK_VALID_REFERENCE) || e->reference_id==0)
        o->flags|=NECK_POSTURE_NO_REFERENCE;
    if((e->status!=NECK_COMPUTED && e->status!=NECK_PARTIAL) ||
        (e->flags&NECK_FLAG_INPUT_REJECTED) ||
        (input->filter_flags&(NECK_FILTER_DATA_INVALID|NECK_FILTER_TIMING_REJECTED)) ||
        (known && (e->valid_mask&NECK_VALID_SAMPLE_AGE) && e->sample_age_us>c->parameters.max_gap_us))
        o->flags|=NECK_POSTURE_INPUT_INVALID;
    /* 数据中断或中立位变化会打断持续计时；预热、缺少参考或输入无效时也清除两轴状态。 */
    if(boundary || (o->flags&(NECK_POSTURE_WARMUP|NECK_POSTURE_NO_REFERENCE|NECK_POSTURE_INPUT_INVALID))) {
        invalidate(c,o); return NECK_OK;
    }
    /* 通过整帧检查后，两轴独立判断。一轴角度无效时，另一轴仍可继续计时。 */
    if((e->valid_mask&NECK_VALID_FLEXION) && isfinite(e->flexion_deg) && e->pitch_status==NECK_COMPUTED)
        axis_step(&c->flexion,&o->flexion,&c->parameters,e->flexion_deg,time_us,0);
    else { invalidate_axis(&c->flexion,&o->flexion); o->flags|=NECK_POSTURE_INPUT_INVALID; }
    if((e->valid_mask&NECK_VALID_RIGHT_LATERAL) && isfinite(e->right_lateral_deg) && e->lateral_status==NECK_COMPUTED)
        axis_step(&c->lateral,&o->lateral,&c->parameters,e->right_lateral_deg,time_us,1);
    else { invalidate_axis(&c->lateral,&o->lateral); o->flags|=NECK_POSTURE_INPUT_INVALID; }
    return NECK_OK;
}
