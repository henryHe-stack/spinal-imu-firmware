#include "neck_relative.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static const double degrees=57.2957795130823208768;
/* 每项按“参考系 → 目标节点”配对：整体 C7→C0、上段 C4→C0、下段 C7→C4、基部 世界→C7。 */
static const uint8_t references[4]={NECK_NODE_C7,NECK_NODE_C4,NECK_NODE_C7,NECK_WORLD_REFERENCE};
static const uint8_t targets[4]={NECK_NODE_C0,NECK_NODE_C0,NECK_NODE_C4,NECK_NODE_C7};

static int unit_quaternion(const double input[4], double tolerance, double output[4])
{
    double norm;
    unsigned i;
    for(i=0;i<4;++i) if(!isfinite(input[i])) return 0;
    norm=hypot(hypot(input[0],input[1]),hypot(input[2],input[3]));
    if(!isfinite(norm) || norm==0.0 || fabs(norm-1.0)>tolerance) return 0;
    for(i=0;i<4;++i) output[i]=input[i]/norm;
    return 1;
}
/* q 和 -q 表示同一旋转；让首个非零分量为正，使输出符号一致。 */
static void canonical(double q[4])
{
    unsigned i,j;
    for(i=0;i<4;++i) {
        if(q[i]>0.0) break;
        if(q[i]<0.0) { for(j=0;j<4;++j) q[j]=-q[j]; break; }
    }
    for(i=0;i<4;++i) if(q[i]==0.0) q[i]=0.0; /* canonical positive zero */
}
static void multiply(const double a[4], const double b[4], double out[4])
{
    double q[4];
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    q[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
    /* Products of checked unit inputs can only have rounding-sized drift. */
    (void)unit_quaternion(q,0.001,out);
    canonical(out);
}
static void conjugate(const double q[4], double out[4])
{ out[0]=q[0]; out[1]=-q[1]; out[2]=-q[2]; out[3]=-q[3]; }
static uint8_t euler(const double q[4], double min_cos, double out[3])
{
    double w=q[0],x=q[1],y=q[2],z=q[3];
    double r00=1.0-2.0*(y*y+z*z), r10=2.0*(x*y+w*z);
    double r20=2.0*(x*z-w*y), r21=2.0*(y*z+w*x), r22=1.0-2.0*(x*x+y*y);
    double cosine=hypot(r00,r10);
    /* 接近万向节锁时跳过欧拉角换算，调用方仍可使用四元数和旋转角。 */
    if(cosine<=min_cos) return 0;
    out[0]=atan2(-r20,cosine)*degrees;
    out[1]=atan2(r21,r22)*degrees;
    out[2]=atan2(r10,r00)*degrees;
    return 1;
}
static double rotation_angle(const double q[4])
{ return 2.0*atan2(hypot(hypot(q[1],q[2]),q[3]),fabs(q[0]))*degrees; }
void neck_relative_default_parameters(NeckRelativeParameters *p)
{
    if(p==NULL) return;
    p->quaternion_norm_tolerance=0.001; p->gimbal_cos_min=1.0e-6;
    p->stale_after_us=30000; p->max_node_skew_us=10000;
}
static int parameters_valid(const NeckRelativeParameters *p)
{
    return p!=NULL && isfinite(p->quaternion_norm_tolerance) &&
        p->quaternion_norm_tolerance>=0.0 && p->quaternion_norm_tolerance<1.0 &&
        isfinite(p->gimbal_cos_min) && p->gimbal_cos_min>0.0 && p->gimbal_cos_min<1.0 &&
        p->stale_after_us>0;
}
NeckResult neck_relative_init(NeckRelativeContext *c, uint32_t config_id,
    uint32_t world_id, const double mounts[NECK_NODE_COUNT][4], const NeckRelativeParameters *p)
{
    unsigned i;
    if(c==NULL) return NECK_BAD_ARGUMENT;
    memset(c,0,sizeof(*c));
    if(!parameters_valid(p) || mounts==NULL || config_id==0 || world_id==0) return NECK_BAD_CONFIG;
    for(i=0;i<NECK_NODE_COUNT;++i) {
        double mount[4];
        if(!unit_quaternion(mounts[i],p->quaternion_norm_tolerance,mount)) return NECK_BAD_CONFIG;
        conjugate(mount,c->inverse_mount_wxyz[i]);
    }
    c->parameters=*p; c->config_id=config_id; c->world_frame_id=world_id; c->initialized=1;
    return NECK_OK;
}
void neck_relative_reset_stream(NeckRelativeContext *c, uint8_t clear_reference)
{
    if(c==NULL || !c->initialized) return;
    c->stream_bound=0;
    if(clear_reference) c->reference_set=0;
}
static void empty_output(const NeckRelativeContext *c, const NeckOrientationFrame *f,
    NeckRelativeOutput *o, uint32_t flags)
{
    unsigned i;
    memset(o,0,sizeof(*o)); o->flags=NECK_REL_QUALITY_NOT_EVALUATED|NECK_REL_THRESHOLDS_UNSET|flags;
    if(c!=NULL) { o->world_frame_id=c->world_frame_id; o->reference_id=c->reference_set?c->reference.reference_id:0; }
    if(f!=NULL) {
        o->config_id=f->config_id; o->session_id=f->session_id; o->sequence=f->sequence;
        o->frame_time_us=f->frame_time_us; o->source=f->source;
    }
    for(i=0;i<4;++i) {
        o->pairs[i].reference_node=references[i]; o->pairs[i].target_node=targets[i];
        o->pairs[i].flags=o->flags;
    }
}
static NeckResult reject(NeckRelativeContext *c, const NeckOrientationFrame *f,
    NeckRelativeOutput *o, NeckResult reason, uint8_t clear_reference)
{
    if(clear_reference) c->reference_set=0;
    empty_output(c,f,o,NECK_REL_INPUT_REJECTED);
    return reason;
}
static uint32_t node_pose(const NeckRelativeContext *c, const NeckOrientationFrame *f,
    unsigned id, double pose[4], uint8_t *valid)
{
    const NeckOrientationSample *n=&f->nodes[id];
    double sensor[4];
    uint32_t flags=0;
    *valid=0;
    if(n->status>NECK_NODE_PARTIAL || n->quat_present!=1)
        flags|=NECK_REL_NODE_UNAVAILABLE;
    if(n->world_frame_id!=c->world_frame_id) flags|=NECK_REL_WORLD_MISMATCH;
    if(n->sample_time_known!=1) flags|=NECK_REL_TIME_UNKNOWN;
    else if(n->sample_time_us>f->frame_time_us || f->frame_time_us-n->sample_time_us>c->parameters.stale_after_us)
        flags|=NECK_REL_STALE;
    if(n->calibration_system==NECK_CALIBRATION_UNKNOWN) flags|=NECK_REL_CALIBRATION_UNKNOWN;
    else if(n->calibration_system>3) flags|=NECK_REL_NODE_UNAVAILABLE;
    else if(n->calibration_system<3) flags|=NECK_REL_CALIBRATION_PENDING;
    if(flags&(NECK_REL_NODE_UNAVAILABLE|NECK_REL_WORLD_MISMATCH|NECK_REL_TIME_UNKNOWN|NECK_REL_STALE)) return flags;
    if(!unit_quaternion(n->sensor_to_world_wxyz,c->parameters.quaternion_norm_tolerance,sensor))
        return flags|NECK_REL_INVALID_QUATERNION;
    /* 用安装旋转的逆抵消传感器安装方向，得到局部人体坐标系在世界坐标系中的姿态。 */
    multiply(sensor,c->inverse_mount_wxyz[id],pose);
    *valid=1;
    return flags|NECK_REL_NORMALIZED;
}
NeckResult neck_relative_process(NeckRelativeContext *c, const NeckOrientationFrame *f,
    NeckRelativeOutput *o)
{
    double poses[NECK_NODE_COUNT][4];
    uint32_t flags[NECK_NODE_COUNT];
    uint8_t valid[NECK_NODE_COUNT];
    unsigned i;
    if(o==NULL) return NECK_BAD_ARGUMENT;
    empty_output(NULL,NULL,o,NECK_REL_INPUT_REJECTED);
    if(c==NULL || f==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(f->config_id!=c->config_id) return reject(c,f,o,NECK_CONFIG_MISMATCH,0);
    if(f->session_id==0 || f->source>NECK_SOURCE_HARDWARE) return reject(c,f,o,NECK_BAD_FRAME,0);
    if(c->stream_bound && (f->session_id!=c->session_id || f->source!=c->source))
        return reject(c,f,o,NECK_BAD_FRAME,1);
    if(c->reference_set && c->reference.origin==NECK_REFERENCE_SIMULATION && f->source!=NECK_SOURCE_SIMULATION)
        return reject(c,f,o,NECK_SIM_REFERENCE_ON_HARDWARE,1);
    if(c->stream_bound && (f->sequence<=c->previous_sequence || f->frame_time_us<=c->previous_frame_time_us))
        return reject(c,f,o,NECK_BAD_FRAME,0);
    empty_output(c,f,o,0);
    if(c->stream_bound && (f->sequence!=c->previous_sequence+1U ||
        f->frame_time_us-c->previous_frame_time_us>c->parameters.stale_after_us)) o->flags|=NECK_REL_STREAM_GAP;
    c->stream_bound=1; c->source=f->source; c->session_id=f->session_id;
    c->previous_sequence=f->sequence; c->previous_frame_time_us=f->frame_time_us;
    for(i=0;i<NECK_NODE_COUNT;++i) flags[i]=node_pose(c,f,i,poses[i],&valid[i]);
    for(i=0;i<4;++i) {
        NeckRelativePairOutput *p=&o->pairs[i];
        unsigned target=targets[i], ref=references[i];
        p->flags=o->flags|flags[target];
        if(ref!=NECK_WORLD_REFERENCE) p->flags|=flags[ref];
        if(!valid[target] || (ref!=NECK_WORLD_REFERENCE && !valid[ref])) continue;
        p->valid_mask|=NECK_REL_KNOWN_SKEW;
        if(ref!=NECK_WORLD_REFERENCE) {
            uint64_t a=f->nodes[ref].sample_time_us,b=f->nodes[target].sample_time_us;
            double inverse[4];
            /* 同帧中的节点可能在不同时刻采样。时间差超限时跳过当前配对，其他配对继续计算。 */
            p->sample_skew_us=a>b?a-b:b-a;
            if(p->sample_skew_us>c->parameters.max_node_skew_us) { p->flags|=NECK_REL_EXCESSIVE_SKEW; continue; }
            /* 在参考节点坐标系中表示目标姿态，消去两节点共有的世界旋转。 */
            conjugate(poses[ref],inverse); multiply(inverse,poses[target],p->raw_wxyz);
        } else { memcpy(p->raw_wxyz,poses[target],sizeof(p->raw_wxyz)); canonical(p->raw_wxyz); }
        p->raw_rotation_deg=rotation_angle(p->raw_wxyz);
        p->valid_mask|=NECK_REL_RAW_QUAT|NECK_REL_RAW_ANGLE;
        if(euler(p->raw_wxyz,c->parameters.gimbal_cos_min,p->raw_euler_deg)) p->valid_mask|=NECK_REL_RAW_EULER;
        else p->flags|=NECK_REL_RAW_GIMBAL_LOCK;
        p->status=NECK_REL_RAW_ONLY;
        if(!c->reference_set) { p->flags|=NECK_REL_NEUTRAL_UNSET; continue; }
        {
            double inverse[4];
            /* 左乘中立位相对旋转的逆，得到相对于中立位的旋转。 */
            conjugate(c->reference.pair_wxyz[i],inverse);
            multiply(inverse,p->raw_wxyz,p->corrected_wxyz);
        }
        p->corrected_rotation_deg=rotation_angle(p->corrected_wxyz);
        p->valid_mask|=NECK_REL_CORRECTED_QUAT|NECK_REL_CORRECTED_ANGLE;
        if(euler(p->corrected_wxyz,c->parameters.gimbal_cos_min,p->corrected_euler_deg)) p->valid_mask|=NECK_REL_CORRECTED_EULER;
        else p->flags|=NECK_REL_CORRECTED_GIMBAL_LOCK;
        p->status=NECK_REL_CORRECTED;
    }
    return NECK_OK;
}
NeckResult neck_relative_set_reference(NeckRelativeContext *c, const NeckRelativeReference *r)
{
    NeckRelativeReference checked;
    unsigned i;
    if(c==NULL || r==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(r->reference_id==0 || r->config_id!=c->config_id || r->world_frame_id!=c->world_frame_id ||
        r->origin>NECK_REFERENCE_EXTERNAL) return NECK_BAD_REFERENCE;
    if(c->stream_bound && r->origin==NECK_REFERENCE_SIMULATION && c->source!=NECK_SOURCE_SIMULATION)
        return NECK_SIM_REFERENCE_ON_HARDWARE;
    memset(&checked,0,sizeof(checked));
    checked.reference_id=r->reference_id; checked.config_id=r->config_id;
    checked.world_frame_id=r->world_frame_id; checked.origin=r->origin;
    for(i=0;i<4;++i) {
        if(!unit_quaternion(r->pair_wxyz[i],c->parameters.quaternion_norm_tolerance,checked.pair_wxyz[i]))
            return NECK_BAD_REFERENCE;
        canonical(checked.pair_wxyz[i]);
    }
    c->reference=checked; c->reference_set=1;
    return NECK_OK;
}
NeckResult neck_relative_capture_reference(NeckRelativeContext *c, const NeckRelativeOutput *o,
    uint32_t reference_id, uint8_t origin)
{
    NeckRelativeReference reference;
    unsigned i;
    if(c==NULL || o==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    if(!c->stream_bound || o->config_id!=c->config_id || o->world_frame_id!=c->world_frame_id ||
        o->session_id!=c->session_id || o->source!=c->source ||
        o->sequence!=c->previous_sequence || o->frame_time_us!=c->previous_frame_time_us ||
        (o->flags&NECK_REL_INPUT_REJECTED)) return NECK_BAD_REFERENCE;
    memset(&reference,0,sizeof(reference));
    reference.reference_id=reference_id; reference.config_id=c->config_id;
    reference.world_frame_id=c->world_frame_id; reference.origin=origin;
    for(i=0;i<4;++i) {
        if(!(o->pairs[i].valid_mask&NECK_REL_RAW_QUAT)) return NECK_BAD_REFERENCE;
        memcpy(reference.pair_wxyz[i],o->pairs[i].raw_wxyz,sizeof(reference.pair_wxyz[i]));
    }
    return neck_relative_set_reference(c,&reference);
}
