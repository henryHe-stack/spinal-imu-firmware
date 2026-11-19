#include "neck_quality.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
void neck_quality_default_parameters(NeckQualityParameters *p)
{
    if(p==NULL) return;
    memset(p,0,sizeof(*p));
    p->gravity_m_s2=9.80665; p->norm_reentry_error_m_s2=.5; p->norm_exit_error_m_s2=.8;
    p->quaternion_norm_tolerance=.001;
    p->max_frame_age_us=p->max_sample_age_us=p->max_gap_us=p->no_frame_timeout_us=30000;
    p->recovery_us=100000;
    p->minimum_accel_calibration=p->minimum_system_calibration=3;
    p->allow_simulation_uncalibrated=1;
}
static int valid_parameters(const NeckQualityParameters *p)
{
    return p!=NULL && isfinite(p->gravity_m_s2) && p->gravity_m_s2>0 &&
        isfinite(p->norm_reentry_error_m_s2) && p->norm_reentry_error_m_s2>=0 &&
        isfinite(p->norm_exit_error_m_s2) && p->norm_exit_error_m_s2>p->norm_reentry_error_m_s2 &&
        p->norm_exit_error_m_s2<p->gravity_m_s2 &&
        isfinite(p->quaternion_norm_tolerance) && p->quaternion_norm_tolerance>=0 &&
        p->quaternion_norm_tolerance<1 && p->max_frame_age_us>0 && p->max_sample_age_us>0 &&
        p->max_gap_us>0 && p->no_frame_timeout_us>0 &&
        p->max_frame_age_us<=p->no_frame_timeout_us &&
        p->minimum_accel_calibration<=3 && p->minimum_system_calibration<=3 &&
        p->allow_simulation_uncalibrated<=1;
}
NeckResult neck_quality_init(NeckQualityContext *c, uint32_t config,
    uint32_t world, uint64_t now, const NeckQualityParameters *p)
{
    if(c==NULL) return NECK_BAD_ARGUMENT;
    memset(c,0,sizeof(*c));
    if(!valid_parameters(p) || config==0 || world==0) return NECK_BAD_CONFIG;
    c->parameters=*p; c->config_id=config; c->world_frame_id=world;
    c->previous_wall_us=c->start_wall_us=now; c->initialized=1;
    return NECK_OK;
}
void neck_quality_reset_stream(NeckQualityContext *c, uint64_t now)
{
    if(c==NULL || !c->initialized) return;
    memset(c->nodes,0,sizeof(c->nodes)); c->stream_bound=0;
    c->timeout_latched=c->clock_fault=0; c->previous_wall_us=c->start_wall_us=now;
    c->previous_sequence=c->previous_frame_time_us=0; c->session_id=0; c->source=0;
}
static void clear_recovery(NeckQualityContext *c)
{
    unsigned i;
    for(i=0;i<3;++i) c->nodes[i].accel_recovering=c->nodes[i].quat_recovering=0;
}
static void empty(const NeckQualityContext *c, uint64_t now, NeckQualityOutput *o)
{
    memset(o,0,sizeof(*o)); o->flags=NECK_Q_ENGINEERING_ONLY; o->decision_time_us=now;
    if(c!=NULL) {
        o->config_id=c->config_id; o->session_id=c->session_id; o->source=c->source;
        if(c->stream_bound) { o->sequence=c->previous_sequence; o->frame_time_us=c->previous_frame_time_us; }
        else o->flags|=NECK_Q_NO_FRAME;
    }
}
static int wall(NeckQualityContext *c, uint64_t now)
{
    if(now<c->previous_wall_us) c->clock_fault=1;
    if(!c->clock_fault) c->previous_wall_us=now;
    return !c->clock_fault;
}
static uint32_t timeout(NeckQualityContext *c, uint64_t now)
{
    /* 收到首帧前从启动或重置时刻计算超时，之后从最近一次接受的帧时间计算。 */
    uint64_t last=c->stream_bound?c->previous_frame_time_us:c->start_wall_us;
    uint32_t flags=0;
    if(c->clock_fault) return NECK_Q_CLOCK_FAULT;
    if(now>=last && now-last>c->parameters.no_frame_timeout_us) {
        flags=NECK_Q_TIMEOUT;
        if(!c->timeout_latched) { flags|=NECK_Q_TIMEOUT_EDGE; clear_recovery(c); }
        c->timeout_latched=1;
    }
    return flags;
}
int neck_quality_poll(NeckQualityContext *c, uint64_t now, NeckQualityOutput *o)
{
    if(o==NULL) return 1;
    empty(c,now,o);
    if(c==NULL || !c->initialized) { o->flags|=NECK_Q_FRAME_REJECTED; return 1; }
    (void)wall(c,now); o->flags|=timeout(c,now);
    return (o->flags&(NECK_Q_TIMEOUT|NECK_Q_CLOCK_FAULT))!=0;
}
static NeckResult reject(NeckQualityContext *c, NeckQualityOutput *o,
    uint32_t flags, uint64_t now, NeckResult result)
{
    o->flags|=NECK_Q_FRAME_REJECTED|flags|timeout(c,now); clear_recovery(c);
    return result;
}
static int recovery(uint8_t good, uint8_t *running, uint64_t *since,
    uint64_t now, uint64_t hold)
{
    if(!good) { *running=0; return 0; }
    if(!*running) { *running=1; *since=now; }
    return now-*since>=hold;
}
static int calibration(uint8_t level, uint8_t minimum, uint8_t permissive,
    uint32_t unknown_flag, uint32_t pending_flag, uint32_t *flags)
{
    if(level==255) { *flags|=unknown_flag; return permissive; }
    if(level>3) { *flags|=pending_flag; return 0; }
    if(level<3) *flags|=pending_flag;
    return permissive || level>=minimum;
}
static int sample_time(uint8_t known, uint64_t sample, uint64_t frame,
    uint64_t now, uint64_t limit, uint64_t *previous, uint8_t *seen,
    uint32_t unknown_flag, uint32_t invalid_flag, uint32_t repeat_flag, uint32_t *flags)
{
    /* 每路信号分别检查时间戳：样本须足够新，且采样时刻须晚于该路上次通过时间检查的样本。 */
    if(known!=1) { *flags|=unknown_flag; return 0; }
    if(sample>frame || sample>now || now-sample>limit) { *flags|=invalid_flag; return 0; }
    if(*seen && sample<=*previous) { *flags|=repeat_flag; return 0; }
    *previous=sample; *seen=1; return 1;
}
static void node(NeckQualityContext *c, const NeckFrame *a, const NeckOrientationFrame *q,
    unsigned i, uint64_t now, NeckQualityOutput *o)
{
    const NeckNodeSample *acc=&a->nodes[i]; const NeckOrientationSample *quat=&q->nodes[i];
    NeckQualityNodeState *s=&c->nodes[i]; const NeckQualityParameters *p=&c->parameters;
    uint32_t *f=&o->node_flags[i]; unsigned j; int agood=1,qgood=1,aready,qready;
    uint8_t permissive=a->source==NECK_SOURCE_SIMULATION && p->allow_simulation_uncalibrated;
    double norm,error;
    if(acc->accel_present!=1 || !(acc->signal_valid_mask&NECK_SIGNAL_ACCEL)) {
        *f|=NECK_QN_ACCEL_MISSING; agood=0;
    }
    if(acc->status>NECK_NODE_PARTIAL || acc->accel_present>1 ||
        !!(acc->signal_valid_mask&NECK_SIGNAL_ACCEL)!=!!acc->accel_present ||
        (acc->status==NECK_NODE_OK && acc->signal_valid_mask!=7) ||
        (acc->status==NECK_NODE_PARTIAL && (acc->signal_valid_mask==0 || acc->signal_valid_mask==7)) ||
        (acc->signal_valid_mask&~(NECK_SIGNAL_ACCEL|NECK_SIGNAL_QUATERNION|NECK_SIGNAL_GYRO))) {
        *f|=NECK_QN_ACCEL_METADATA; agood=0;
    }
    for(j=0;j<3;++j) if(!isfinite(acc->accel_m_s2[j])) { *f|=NECK_QN_ACCEL_NONFINITE; agood=0; }
    norm=hypot(hypot(acc->accel_m_s2[0],acc->accel_m_s2[1]),acc->accel_m_s2[2]);
    if(!isfinite(norm)) { *f|=NECK_QN_ACCEL_NONFINITE; s->norm_rejected=1; agood=0; }
    else {
        error=fabs(norm-p->gravity_m_s2);
        /* 模长误差超过较宽阈值时拒绝加速度，回到较窄阈值以内（含边界）才解除拒绝。 */
        if(error>p->norm_exit_error_m_s2) s->norm_rejected=1;
        else if(s->norm_rejected && error<=p->norm_reentry_error_m_s2) s->norm_rejected=0;
        if(s->norm_rejected) { *f|=NECK_QN_NORM_REJECTED; agood=0; }
    }
    if(!sample_time(acc->sample_time_known,acc->sample_time_us,a->frame_time_us,now,
        p->max_sample_age_us,&s->last_accel_time_us,&s->accel_seen,
        NECK_QN_ACCEL_TIME_UNKNOWN,NECK_QN_ACCEL_TIME_INVALID,NECK_QN_ACCEL_NOT_NEW,f)) agood=0;
    if(!calibration(acc->calibration_accel,p->minimum_accel_calibration,permissive,
        NECK_QN_ACCEL_CAL_UNKNOWN,NECK_QN_ACCEL_CAL_PENDING,f)) agood=0;
    if(quat->quat_present!=1) { *f|=NECK_QN_QUAT_MISSING; qgood=0; }
    if(quat->status>NECK_NODE_PARTIAL || quat->quat_present>1 ||
        !!(acc->signal_valid_mask&NECK_SIGNAL_QUATERNION)!=!!quat->quat_present) {
        *f|=NECK_QN_QUAT_METADATA; qgood=0;
    }
    if(quat->world_frame_id!=c->world_frame_id) { *f|=NECK_QN_WORLD_MISMATCH; qgood=0; }
    norm=hypot(hypot(quat->sensor_to_world_wxyz[0],quat->sensor_to_world_wxyz[1]),
        hypot(quat->sensor_to_world_wxyz[2],quat->sensor_to_world_wxyz[3]));
    for(j=0;j<4;++j) if(!isfinite(quat->sensor_to_world_wxyz[j])) qgood=0;
    if(!isfinite(norm) || norm==0 || fabs(norm-1)>p->quaternion_norm_tolerance || !qgood) {
        /* Metadata absence is recorded separately; inspect numbers only when
         * present, but never approve malformed numeric members. */
        if(quat->quat_present==1 && (!isfinite(norm) || norm==0 ||
            fabs(norm-1)>p->quaternion_norm_tolerance)) *f|=NECK_QN_QUAT_INVALID;
        qgood=0;
    }
    if(!sample_time(quat->sample_time_known,quat->sample_time_us,q->frame_time_us,now,
        p->max_sample_age_us,&s->last_quat_time_us,&s->quat_seen,
        NECK_QN_QUAT_TIME_UNKNOWN,NECK_QN_QUAT_TIME_INVALID,NECK_QN_QUAT_NOT_NEW,f)) qgood=0;
    if(!calibration(quat->calibration_system,p->minimum_system_calibration,permissive,
        NECK_QN_QUAT_CAL_UNKNOWN,NECK_QN_QUAT_CAL_PENDING,f)) qgood=0;
    /* 加速度连续合格满恢复时长后才可用；姿态还要求同期的四元数也连续合格。 */
    aready=recovery((uint8_t)agood,&s->accel_recovering,&s->accel_good_since_us,now,p->recovery_us);
    qready=recovery((uint8_t)(qgood && agood),&s->quat_recovering,&s->quat_good_since_us,now,p->recovery_us);
    if(agood && !aready) *f|=NECK_QN_ACCEL_RECOVERING;
    if(qgood && agood && !qready) *f|=NECK_QN_QUAT_RECOVERING;
    if(!aready) *f|=NECK_QN_QUAT_REQUIRES_ACCEL;
    if(aready) o->accel_usable_mask|=(uint8_t)(1u<<i);
    if(aready && qready) o->orientation_usable_mask|=(uint8_t)(1u<<i);
}
NeckResult neck_quality_process(NeckQualityContext *c, const NeckFrame *a,
    const NeckOrientationFrame *q, uint64_t now, NeckQualityOutput *o)
{
    unsigned i;
    if(o==NULL) return NECK_BAD_ARGUMENT;
    empty(c,now,o);
    if(c==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(a==NULL || q==NULL) {
        (void)wall(c,now);
        return reject(c,o,NECK_Q_METADATA_MISMATCH,now,NECK_BAD_ARGUMENT);
    }
    o->sequence=a->sequence; o->frame_time_us=a->frame_time_us;
    o->config_id=a->config_id; o->session_id=a->session_id; o->source=a->source;
    if(!wall(c,now)) return reject(c,o,NECK_Q_CLOCK_FAULT,now,NECK_BAD_FRAME);
    /* 两路输入须属于同一帧；被拒绝的帧不会刷新超时计时的起点。 */
    if(a->config_id!=q->config_id || a->session_id!=q->session_id || a->source!=q->source ||
        a->sequence!=q->sequence || a->frame_time_us!=q->frame_time_us)
        return reject(c,o,NECK_Q_METADATA_MISMATCH,now,NECK_BAD_FRAME);
    if(a->config_id!=c->config_id) return reject(c,o,0,now,NECK_CONFIG_MISMATCH);
    if(a->session_id==0 || a->source>NECK_SOURCE_HARDWARE)
        return reject(c,o,NECK_Q_METADATA_MISMATCH,now,NECK_BAD_FRAME);
    if(c->stream_bound && (a->session_id!=c->session_id || a->source!=c->source))
        return reject(c,o,NECK_Q_STREAM_CHANGED,now,NECK_BAD_FRAME);
    if(a->frame_time_us>now) return reject(c,o,NECK_Q_FUTURE_FRAME,now,NECK_BAD_FRAME);
    if(now-a->frame_time_us>c->parameters.max_frame_age_us)
        return reject(c,o,NECK_Q_OLD_FRAME,now,NECK_BAD_FRAME);
    if(c->stream_bound && a->sequence<=c->previous_sequence)
        return reject(c,o,NECK_Q_BAD_SEQUENCE,now,NECK_BAD_FRAME);
    if(c->stream_bound && a->frame_time_us<=c->previous_frame_time_us)
        return reject(c,o,NECK_Q_BAD_FRAME_TIME,now,NECK_BAD_FRAME);
    if(c->stream_bound && (a->sequence!=c->previous_sequence+1u ||
        a->frame_time_us-c->previous_frame_time_us>c->parameters.max_gap_us)) {
        o->flags|=NECK_Q_STREAM_GAP; clear_recovery(c);
    }
    /* 超时后首次接受新帧时，重新累计各节点的恢复时长。此时信号可能仍在恢复中。 */
    if(c->timeout_latched) { o->flags|=NECK_Q_RECOVERY_EDGE; clear_recovery(c); }
    c->timeout_latched=0; c->stream_bound=1; c->source=a->source; c->session_id=a->session_id;
    c->previous_sequence=a->sequence; c->previous_frame_time_us=a->frame_time_us;
    o->flags&=~(uint32_t)NECK_Q_NO_FRAME; o->frame_accepted=1;
    for(i=0;i<3;++i) node(c,a,q,i,now,o);
    o->state=o->accel_usable_mask==7 && o->orientation_usable_mask==7?NECK_Q_SOFTWARE_READY:
        (o->accel_usable_mask || o->orientation_usable_mask?NECK_Q_PARTIAL:NECK_Q_UNKNOWN);
    return NECK_OK;
}
