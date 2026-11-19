#include "neck_measurement_port.h"
#include <math.h>
#include <string.h>
static int unit(const double *q)
{
    double n=0; unsigned i;
    for(i=0;i<4;++i) { if(!isfinite(q[i])) return 0; n+=q[i]*q[i]; }
    return fabs(sqrt(n)-1.0)<=0.001;
}
static void multiply(const double *a,const double *b,double *q)
{
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    q[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
int neck_measurement_profile_valid(const NeckMeasurementProfile *p,uint32_t config)
{
    unsigned i;
    if(!p || !config || p->config_id!=config || !p->world_id || p->node_mask!=7) return 0;
    for(i=0;i<3;++i) if(!unit(p->sensor_to_body[i]) || !unit(p->local_to_common_world[i])) return 0;
    return 1;
}
uint8_t neck_measurement_apply(const NeckMeasurementProfile *p,NeckAcquisitionFrame *f,
    const NeckSensorReadTrace *trace,const NeckMeasurementTimes *t)
{
    unsigned i; uint8_t approved=0;
    if(!f) return 0;
    for(i=0;i<3;++i) {
        f->acceleration.nodes[i].sample_time_known=0; f->acceleration.nodes[i].sample_time_us=0;
        f->orientation.nodes[i].sample_time_known=0; f->orientation.nodes[i].sample_time_us=0;
        f->orientation.nodes[i].world_frame_id=0;
    }
    if(!trace || !t || !neck_measurement_profile_valid(p,f->acceleration.config_id) ||
        trace->selected_mask!=7 || (t->accel_mask|t->quat_mask)&~7u ||
        f->acceleration.source!=NECK_SOURCE_HARDWARE || f->orientation.source!=NECK_SOURCE_HARDWARE ||
        !t->session_id || t->config_id!=f->acceleration.config_id || t->config_id!=f->orientation.config_id ||
        t->session_id!=f->acceleration.session_id || t->session_id!=f->orientation.session_id ||
        t->sequence!=f->acceleration.sequence || t->sequence!=f->orientation.sequence ||
        f->acceleration.frame_time_us!=f->orientation.frame_time_us) return 0;
    for(i=0;i<3;++i) {
        NeckNodeSample *a=&f->acceleration.nodes[i]; NeckOrientationSample *q=&f->orientation.nodes[i];
        double transformed[4];
        if(!(trace->observed_mask&(1u<<i)) || trace->result[i]!=NECK_SENSOR_OK ||
            t->read_start_us[i]!=trace->read_start_us[i] || t->read_end_us[i]!=trace->read_end_us[i] ||
            t->read_start_us[i]>t->read_end_us[i] || t->read_end_us[i]>f->acceleration.frame_time_us) continue;
        if((t->accel_mask&(1u<<i)) && a->accel_present && t->accel_time_us[i]<=t->read_start_us[i]) {
            a->sample_time_known=1; a->sample_time_us=t->accel_time_us[i]; approved|=(uint8_t)(1u<<i);
        }
        if((t->quat_mask&(1u<<i)) && q->quat_present && t->quat_time_us[i]<=t->read_start_us[i] &&
            unit(q->sensor_to_world_wxyz)) {
            multiply(p->local_to_common_world[i],q->sensor_to_world_wxyz,transformed);
            memcpy(q->sensor_to_world_wxyz,transformed,sizeof(transformed));
            q->sample_time_known=1; q->sample_time_us=t->quat_time_us[i]; q->world_frame_id=p->world_id;
        }
    }
    return approved;
}
__attribute__((weak)) int neck_board_three_profile(uint32_t config,NeckMeasurementProfile *profile)
{ (void)config; (void)profile; return 0; }
__attribute__((weak)) int neck_board_three_times(const NeckAcquisitionFrame *frame,
    const NeckSensorReadTrace *trace,NeckMeasurementTimes *times)
{ (void)frame; (void)trace; (void)times; return 0; }
