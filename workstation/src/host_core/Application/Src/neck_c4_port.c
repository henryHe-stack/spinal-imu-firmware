#include "neck_c4_port.h"
#include <stddef.h>

int neck_c4_apply_time(NeckAcquisitionFrame *f, const NeckSensorReadTrace *t,
    const NeckC4TimeEvidence *e)
{
    NeckNodeSample *a;
    if(f==NULL) return 0;
    a=&f->acceleration.nodes[NECK_NODE_C4];
    a->sample_time_known=0; a->sample_time_us=0;
    f->orientation.nodes[NECK_NODE_C4].sample_time_known=0;
    f->orientation.nodes[NECK_NODE_C4].sample_time_us=0;
    if(t==NULL || e==NULL || t->selected_mask!=NECK_SENSOR_C4_ONLY ||
        t->observed_mask!=NECK_SENSOR_C4_ONLY || t->result[NECK_NODE_C4]!=NECK_SENSOR_OK ||
        f->acceleration.source!=NECK_SOURCE_HARDWARE || f->orientation.source!=NECK_SOURCE_HARDWARE ||
        !a->accel_present || !(a->signal_valid_mask&NECK_SIGNAL_ACCEL) ||
        a->status!=NECK_NODE_PARTIAL ||
        !e->config_id || !e->session_id ||
        e->config_id!=f->acceleration.config_id || e->config_id!=f->orientation.config_id ||
        e->session_id!=f->acceleration.session_id || e->session_id!=f->orientation.session_id ||
        e->sequence!=f->acceleration.sequence || e->sequence!=f->orientation.sequence ||
        f->acceleration.frame_time_us!=f->orientation.frame_time_us ||
        e->read_start_us!=t->read_start_us[NECK_NODE_C4] ||
        e->read_end_us!=t->read_end_us[NECK_NODE_C4] ||
        e->read_start_us>e->read_end_us || e->read_end_us>f->acceleration.frame_time_us ||
        e->sample_time_us>e->read_start_us) return 0;
    /* Port must prove this sample belongs to this read and remained coherent
     * throughout it. Bounds alone cannot prove that claim. Age/monotonicity
     * and calibration are still checked by the shared quality engine. */
    a->sample_time_us=e->sample_time_us; a->sample_time_known=1;
    return 1;
}

__attribute__((weak)) int neck_c4_board_mount(double mount[4])
{ (void)mount; return 0; }
__attribute__((weak)) int neck_c4_board_time(const NeckAcquisitionFrame *frame,
    const NeckSensorReadTrace *trace, NeckC4TimeEvidence *evidence)
{ (void)frame; (void)trace; (void)evidence; return 0; }
