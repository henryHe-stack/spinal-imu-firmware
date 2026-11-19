#include "neck_wire.h"
#include <string.h>

static const uint8_t sync_bytes[4] = {0x4e,0x43,0x4b,0x31}; /* NCK1 */
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v)
{ unsigned i; for(i=0;i<4;++i) p[i]=(uint8_t)(v>>(8*i)); }
static void put64(uint8_t *p, uint64_t v)
{ unsigned i; for(i=0;i<8;++i) p[i]=(uint8_t)(v>>(8*i)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static uint32_t get32(const uint8_t *p)
{ unsigned i; uint32_t v=0; for(i=0;i<4;++i) v|=(uint32_t)p[i]<<(8*i); return v; }
static uint64_t get64(const uint8_t *p)
{ unsigned i; uint64_t v=0; for(i=0;i<8;++i) v|=(uint64_t)p[i]<<(8*i); return v; }
static int32_t geti32(const uint8_t *p)
{ uint32_t v=get32(p); return v<=INT32_MAX?(int32_t)v:-1-(int32_t)(UINT32_MAX-v); }
static int8_t geti8(uint8_t v) { return v<=INT8_MAX?(int8_t)v:(int8_t)(-1-(int)(UINT8_MAX-v)); }
uint16_t neck_wire_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc=0xffffu; size_t i; unsigned bit;
    if(data==NULL && length!=0) return 0;
    for(i=0;i<length;++i) {
        crc^=(uint16_t)((uint16_t)data[i]<<8);
        for(bit=0;bit<8;++bit) crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);
    }
    return crc;
}
size_t neck_wire_payload_size(uint8_t type)
{
    switch(type) {
    case NECK_WIRE_SESSION: return NECK_WIRE_SESSION_SIZE;
    case NECK_WIRE_C4: return NECK_WIRE_C4_SIZE;
    case NECK_WIRE_RELATIVE: return NECK_WIRE_RELATIVE_SIZE;
    case NECK_WIRE_EVENT: return NECK_WIRE_EVENT_SIZE;
    case NECK_WIRE_STATUS: return NECK_WIRE_STATUS_SIZE;
    case NECK_WIRE_RAW_FRAME: return NECK_WIRE_RAW_FRAME_SIZE;
    case NECK_WIRE_COMMAND: case NECK_WIRE_REPLY: return 28;
    default: return 0;
    }
}
static int angle_valid(int32_t angle, unsigned valid, int canonical)
{
    if(!valid) return !canonical || angle==NECK_WIRE_INVALID_ANGLE;
    return angle>=-NECK_WIRE_ANGLE_LIMIT && angle<=NECK_WIRE_ANGLE_LIMIT;
}
static int time_valid(uint64_t time, unsigned known, int canonical)
{ return known || !canonical || time==0; }
static int axis_valid(const NeckWireAxis *a, unsigned axis)
{
    if(a->valid>1 || a->instant_level>4 || a->alarm_level>4 || a->direction < -1 || a->direction > 1) return 0;
    if(!a->valid) return a->instant_level==0 && a->alarm_level==0 && a->direction==0 &&
        a->warning_elapsed_us==0 && a->danger_elapsed_us==0 && a->extension_elapsed_us==0;
    return a->instant_level!=0 && a->alarm_level!=0 &&
        (axis==0 || (a->instant_level!=4 && a->alarm_level!=4 && a->extension_elapsed_us==0));
}
static int session_valid(const NeckWireSession *s)
{
    unsigned i; uint32_t cap=s->capabilities;
    if(s->profile_id!=NECK_WIRE_PROFILE_S09 || !s->firmware_id || s->source>1 ||
       (s->mode!=NECK_WIRE_SINGLE_C4 && s->mode!=NECK_WIRE_THREE_NODES) ||
       s->node_mask!=(s->mode==NECK_WIRE_SINGLE_C4?2u:7u) ||
       (s->reference_origin!=0 && s->reference_origin!=1 && s->reference_origin!=255) ||
       (s->source==NECK_WIRE_HARDWARE && s->reference_origin==0) ||
       (cap&~127u) || ((cap&NECK_WIRE_CAP_EVENT_ACK) &&
       (cap&(NECK_WIRE_CAP_CONTROL|NECK_WIRE_CAP_EVENTS))!=(NECK_WIRE_CAP_CONTROL|NECK_WIRE_CAP_EVENTS)) ||
       !(cap&NECK_WIRE_CAP_C4) || !s->sample_period_us ||
       !s->quality_timeout_us || !s->cutoff_millihz || !s->continuous_duration_us ||
       !s->c4_period_us) return 0;
    for(i=0;i<4;++i) if(s->thresholds_mdeg[i]<=0 || s->thresholds_mdeg[i]>NECK_WIRE_ANGLE_LIMIT) return 0;
    if(s->thresholds_mdeg[1]<s->thresholds_mdeg[0] || s->thresholds_mdeg[3]<s->thresholds_mdeg[2]) return 0;
    if(cap&NECK_WIRE_CAP_RELATIVE) {
        if(s->mode!=NECK_WIRE_THREE_NODES || !s->relative_period_us || !s->world_frame_id) return 0;
    } else if(s->relative_period_us) return 0;
    return (cap&NECK_WIRE_CAP_STATUS)?s->status_period_us!=0:s->status_period_us==0;
}
static int times_valid(uint64_t frame, uint64_t decision, uint64_t deadline, uint8_t flags, int canonical)
{
    if(flags&~7u || !time_valid(frame,flags&1u,canonical) ||
       !time_valid(decision,flags&2u,canonical) || !time_valid(deadline,flags&4u,canonical)) return 0;
    if((flags&3u)==3u && decision<frame) return 0;
    return 1; /* An already-expired deadline is legal in an invalidated snapshot. */
}
static int body_valid(const NeckWireMessage *m, int canonical)
{
    unsigned i,j;
    switch(m->header.type) {
    case NECK_WIRE_COMMAND: case NECK_WIRE_REPLY:
        return m->body.control.request_id!=0 && m->body.control.opcode!=0 &&
            (m->header.type==NECK_WIRE_REPLY || m->body.control.result==0);
    case NECK_WIRE_SESSION: return session_valid(&m->body.session);
    case NECK_WIRE_C4: {
        const NeckWireC4 *c=&m->body.c4;
        if(c->valid_mask&~63u || c->zero_status>7 ||
           !times_valid(c->frame_time_us,c->decision_time_us,c->valid_until_us,c->time_flags,canonical)) return 0;
        if((c->valid_mask&48u) && !c->reference_id) return 0;
        if(c->valid_mask && (c->time_flags!=7 || m->header.emit_time_us<c->decision_time_us ||
            m->header.emit_time_us>c->valid_until_us)) return 0;
        if((c->quality_flags&(128u|16384u)) && (c->valid_mask || c->axes[0].valid || c->axes[1].valid)) return 0;
        for(i=0;i<6;++i) if(!angle_valid(c->angles_mdeg[i],c->valid_mask&(1u<<i),canonical)) return 0;
        for(i=0;i<2;++i) if(!axis_valid(&c->axes[i],i) ||
            (c->axes[i].valid && !(c->valid_mask&(16u<<i)))) return 0;
        return 1;
    }
    case NECK_WIRE_RELATIVE: {
        const NeckWireRelative *r=&m->body.relative;
        if(!r->world_frame_id || !times_valid(r->frame_time_us,r->decision_time_us,r->valid_until_us,r->time_flags,canonical)) return 0;
        for(i=0;i<4;++i) {
            const NeckWirePair *p=&r->pairs[i];
            if(p->valid_mask&~7u || p->status>2 ||
               (!p->status && (p->valid_mask&3u)) || ((p->valid_mask&2u) && (p->status!=2 || !r->reference_id)) ||
               (p->status==2 && !r->reference_id) ||
               ((r->quality_flags&(128u|16384u)) && (p->valid_mask&3u)) ||
               !time_valid(p->sample_skew_us,p->valid_mask&4u,canonical)) return 0;
            if((p->valid_mask&3u) && (r->time_flags!=7 || m->header.emit_time_us<r->decision_time_us ||
                m->header.emit_time_us>r->valid_until_us)) return 0;
            for(j=0;j<3;++j) if(!angle_valid(p->raw_mdeg[j],p->valid_mask&1u,canonical) ||
                !angle_valid(p->corrected_mdeg[j],p->valid_mask&2u,canonical)) return 0;
        }
        return 1;
    }
    case NECK_WIRE_EVENT: {
        const NeckWireEvent *e=&m->body.event;
        return e->event_id!=0 && e->events!=0 && !(e->events&~31u) && e->axis<=1 &&
            e->previous_level<=4 && e->level<=4 && e->direction>=-1 && e->direction<=1 &&
            !(e->time_flags&~3u) && time_valid(e->frame_time_us,e->time_flags&1u,canonical) &&
            time_valid(e->event_time_us,e->time_flags&2u,canonical) &&
            ((e->time_flags&3u)!=3u || e->event_time_us>=e->frame_time_us);
    }
    case NECK_WIRE_STATUS: {
        const NeckWireStatus *s=&m->body.status;
        if(s->accel_usable_mask&~7u || s->orientation_usable_mask&~7u || s->quality_state>2 ||
           s->frame_accepted>1 || s->zero_request_state>2 || s->decision_time_us<s->frame_time_us) return 0;
        if((s->quality_flags&(128u|16384u)) && (s->accel_usable_mask || s->orientation_usable_mask ||
            s->quality_state || s->frame_accepted)) return 0;
        for(i=0;i<3;++i) {
            const NeckWireNode *n=&s->nodes[i];
            if(n->status>4 || n->zero_status>7 || n->time_flags&~3u ||
               (n->accel_calibration>3 && n->accel_calibration!=255) ||
               (n->system_calibration>3 && n->system_calibration!=255) ||
               !time_valid(n->accel_sample_time_us,n->time_flags&1u,canonical) ||
               !time_valid(n->orientation_sample_time_us,n->time_flags&2u,canonical)) return 0;
        }
        return 1;
    }
    case NECK_WIRE_RAW_FRAME: {
        const NeckWireRawFrame *raw=&m->body.raw;
        if(!raw->sample_period_us || raw->sample_period_us>1000000u ||
           !raw->node_mask || (raw->node_mask&~7u) || raw->reserved!=0) return 0;
        return 1;
    }
    default: return 0;
    }
}
static void write_session(uint8_t *p, const NeckWireSession *s)
{
    unsigned i; put32(p,s->profile_id); put32(p+4,s->firmware_id); put32(p+8,s->world_frame_id);
    p[12]=s->source; p[13]=s->mode; p[14]=s->node_mask; p[15]=s->reference_origin;
    put32(p+16,s->sample_period_us); put32(p+20,s->quality_timeout_us); put32(p+24,s->cutoff_millihz);
    put64(p+28,s->continuous_duration_us);
    for(i=0;i<4;++i) put32(p+36+4*i,(uint32_t)s->thresholds_mdeg[i]);
    put32(p+52,s->max_node_skew_us); put32(p+56,s->c4_period_us); put32(p+60,s->relative_period_us);
    put32(p+64,s->status_period_us); put32(p+68,s->capabilities);
}
static void read_session(const uint8_t *p, NeckWireSession *s)
{
    unsigned i; s->profile_id=get32(p); s->firmware_id=get32(p+4); s->world_frame_id=get32(p+8);
    s->source=p[12]; s->mode=p[13]; s->node_mask=p[14]; s->reference_origin=p[15];
    s->sample_period_us=get32(p+16); s->quality_timeout_us=get32(p+20); s->cutoff_millihz=get32(p+24);
    s->continuous_duration_us=get64(p+28);
    for(i=0;i<4;++i) s->thresholds_mdeg[i]=geti32(p+36+4*i);
    s->max_node_skew_us=get32(p+52); s->c4_period_us=get32(p+56); s->relative_period_us=get32(p+60);
    s->status_period_us=get32(p+64); s->capabilities=get32(p+68);
}
static uint64_t wire_time(uint64_t value, unsigned known) { return known?value:0; }
static void write_axis(uint8_t *p, const NeckWireAxis *a)
{
    p[0]=a->valid; p[1]=a->instant_level; p[2]=a->alarm_level; p[3]=(uint8_t)a->direction;
    put64(p+4,a->warning_elapsed_us); put64(p+12,a->danger_elapsed_us); put64(p+20,a->extension_elapsed_us);
}
static void read_axis(const uint8_t *p, NeckWireAxis *a)
{
    a->valid=p[0]; a->instant_level=p[1]; a->alarm_level=p[2]; a->direction=geti8(p[3]);
    a->warning_elapsed_us=get64(p+4); a->danger_elapsed_us=get64(p+12); a->extension_elapsed_us=get64(p+20);
}
static void write_body(uint8_t *p, const NeckWireMessage *m)
{
    unsigned i,j;
    switch(m->header.type) {
    case NECK_WIRE_COMMAND: case NECK_WIRE_REPLY:
        put32(p,m->body.control.request_id); put32(p+4,m->body.control.opcode);
        put32(p+8,m->body.control.result);
        for(i=0;i<4;++i) put32(p+12+4*i,m->body.control.args[i]);
        break;
    case NECK_WIRE_SESSION: write_session(p,&m->body.session); break;
    case NECK_WIRE_C4: {
        const NeckWireC4 *c=&m->body.c4;
        put64(p,c->input_sequence); put64(p+8,wire_time(c->frame_time_us,c->time_flags&1u));
        put64(p+16,wire_time(c->decision_time_us,c->time_flags&2u)); put64(p+24,wire_time(c->valid_until_us,c->time_flags&4u));
        put32(p+32,c->invalidation_epoch); put32(p+36,c->reference_id); put32(p+40,c->quality_flags);
        put32(p+44,c->filter_flags); put32(p+48,c->posture_flags); put16(p+52,c->valid_mask);
        p[54]=c->time_flags; p[55]=c->zero_status;
        for(i=0;i<6;++i) put32(p+56+4*i,(uint32_t)((c->valid_mask&(1u<<i))?c->angles_mdeg[i]:NECK_WIRE_INVALID_ANGLE));
        for(i=0;i<2;++i) write_axis(p+80+28*i,&c->axes[i]);
        break;
    }
    case NECK_WIRE_RELATIVE: {
        const NeckWireRelative *r=&m->body.relative;
        put64(p,r->input_sequence); put64(p+8,wire_time(r->frame_time_us,r->time_flags&1u));
        put64(p+16,wire_time(r->decision_time_us,r->time_flags&2u)); put64(p+24,wire_time(r->valid_until_us,r->time_flags&4u));
        put32(p+32,r->invalidation_epoch); put32(p+36,r->reference_id); put32(p+40,r->quality_flags);
        put32(p+44,r->world_frame_id); p[48]=r->time_flags;
        for(i=0;i<4;++i) {
            uint8_t *q=p+52+40*i; const NeckWirePair *pair=&r->pairs[i];
            q[0]=pair->valid_mask; q[1]=pair->status; put32(q+4,pair->flags);
            for(j=0;j<3;++j) {
                put32(q+8+4*j,(uint32_t)((pair->valid_mask&1u)?pair->raw_mdeg[j]:NECK_WIRE_INVALID_ANGLE));
                put32(q+20+4*j,(uint32_t)((pair->valid_mask&2u)?pair->corrected_mdeg[j]:NECK_WIRE_INVALID_ANGLE));
            }
            put64(q+32,wire_time(pair->sample_skew_us,pair->valid_mask&4u));
        } break;
    }
    case NECK_WIRE_EVENT: {
        const NeckWireEvent *e=&m->body.event;
        put64(p,e->event_id); put64(p+8,e->input_sequence); put64(p+16,wire_time(e->frame_time_us,e->time_flags&1u));
        put64(p+24,wire_time(e->event_time_us,e->time_flags&2u)); put32(p+32,e->reference_id);
        put32(p+36,e->invalidation_epoch); put32(p+40,e->events); put32(p+44,e->posture_flags);
        p[48]=e->axis; p[49]=e->previous_level; p[50]=e->level; p[51]=(uint8_t)e->direction; p[52]=e->time_flags; break;
    }
    case NECK_WIRE_STATUS: {
        const NeckWireStatus *s=&m->body.status;
        put64(p,s->input_sequence); put64(p+8,s->frame_time_us); put64(p+16,s->decision_time_us);
        put32(p+24,s->invalidation_epoch); put32(p+28,s->quality_flags);
        p[32]=s->accel_usable_mask; p[33]=s->orientation_usable_mask; p[34]=s->quality_state; p[35]=s->frame_accepted;
        put32(p+36,s->pending_reference_id); p[40]=s->zero_request_state;
        put32(p+44,s->input_drops); put32(p+48,s->telemetry_drops); put32(p+52,s->event_drops); put32(p+56,s->uart_errors);
        for(i=0;i<3;++i) {
            uint8_t *q=p+60+36*i; const NeckWireNode *n=&s->nodes[i];
            put64(q,wire_time(n->accel_sample_time_us,n->time_flags&1u));
            put64(q+8,wire_time(n->orientation_sample_time_us,n->time_flags&2u));
            put32(q+16,n->quality_flags); put32(q+20,n->filter_flags); put32(q+24,n->reference_id);
            q[28]=n->status; q[29]=n->accel_calibration; q[30]=n->system_calibration; q[31]=n->time_flags; q[32]=n->zero_status;
        } break;
    }
    case NECK_WIRE_RAW_FRAME: {
        const NeckWireRawFrame *raw=&m->body.raw;
        put32(p,raw->sequence); put32(p+4,raw->sample_period_us);
        p[8]=raw->node_mask; p[9]=raw->flags; put16(p+10,raw->reserved);
        for(i=0;i<3;++i) {
            uint8_t *q=p+12+36*i; const NeckWireRawNode *n=&raw->nodes[i];
            put32(q,n->timestamp_start_us); put32(q+4,n->timestamp_end_us); put32(q+8,n->timestamp_mid_us);
            for(j=0;j<4;++j) put16(q+12+2*j,(uint16_t)n->quat_wxyz[j]);
            for(j=0;j<3;++j) put16(q+20+2*j,(uint16_t)n->accel_xyz[j]);
            for(j=0;j<3;++j) put16(q+26+2*j,(uint16_t)n->gyro_xyz[j]);
            q[32]=(uint8_t)n->temperature_c; q[33]=n->calibration_status; put16(q+34,n->flags);
        } break;
    }
    default: break;
    }
}
static int read_body(const uint8_t *p, NeckWireMessage *m)
{
    unsigned i,j;
    switch(m->header.type) {
    case NECK_WIRE_COMMAND: case NECK_WIRE_REPLY:
        m->body.control.request_id=get32(p); m->body.control.opcode=get32(p+4);
        m->body.control.result=get32(p+8);
        for(i=0;i<4;++i) m->body.control.args[i]=get32(p+12+4*i);
        break;
    case NECK_WIRE_SESSION: read_session(p,&m->body.session); break;
    case NECK_WIRE_C4: {
        NeckWireC4 *c=&m->body.c4;
        c->input_sequence=get64(p); c->frame_time_us=get64(p+8); c->decision_time_us=get64(p+16); c->valid_until_us=get64(p+24);
        c->invalidation_epoch=get32(p+32); c->reference_id=get32(p+36); c->quality_flags=get32(p+40);
        c->filter_flags=get32(p+44); c->posture_flags=get32(p+48); c->valid_mask=get16(p+52); c->time_flags=p[54]; c->zero_status=p[55];
        for(i=0;i<6;++i) c->angles_mdeg[i]=geti32(p+56+4*i);
        for(i=0;i<2;++i) read_axis(p+80+28*i,&c->axes[i]);
        break;
    }
    case NECK_WIRE_RELATIVE: {
        NeckWireRelative *r=&m->body.relative;
        r->input_sequence=get64(p); r->frame_time_us=get64(p+8); r->decision_time_us=get64(p+16); r->valid_until_us=get64(p+24);
        r->invalidation_epoch=get32(p+32); r->reference_id=get32(p+36); r->quality_flags=get32(p+40);
        r->world_frame_id=get32(p+44); r->time_flags=p[48]; if(p[49] || p[50] || p[51]) return 0;
        for(i=0;i<4;++i) {
            const uint8_t *q=p+52+40*i; NeckWirePair *pair=&r->pairs[i];
            pair->valid_mask=q[0]; pair->status=q[1]; pair->flags=get32(q+4); if(q[2] || q[3]) return 0;
            for(j=0;j<3;++j) { pair->raw_mdeg[j]=geti32(q+8+4*j); pair->corrected_mdeg[j]=geti32(q+20+4*j); }
            pair->sample_skew_us=get64(q+32);
        } break;
    }
    case NECK_WIRE_EVENT: {
        NeckWireEvent *e=&m->body.event;
        e->event_id=get64(p); e->input_sequence=get64(p+8); e->frame_time_us=get64(p+16); e->event_time_us=get64(p+24);
        e->reference_id=get32(p+32); e->invalidation_epoch=get32(p+36); e->events=get32(p+40); e->posture_flags=get32(p+44);
        e->axis=p[48]; e->previous_level=p[49]; e->level=p[50]; e->direction=geti8(p[51]); e->time_flags=p[52];
        if(p[53] || p[54] || p[55]) return 0;
        break;
    }
    case NECK_WIRE_STATUS: {
        NeckWireStatus *s=&m->body.status;
        s->input_sequence=get64(p); s->frame_time_us=get64(p+8); s->decision_time_us=get64(p+16);
        s->invalidation_epoch=get32(p+24); s->quality_flags=get32(p+28);
        s->accel_usable_mask=p[32]; s->orientation_usable_mask=p[33]; s->quality_state=p[34]; s->frame_accepted=p[35];
        s->pending_reference_id=get32(p+36); s->zero_request_state=p[40]; if(p[41] || p[42] || p[43]) return 0;
        s->input_drops=get32(p+44); s->telemetry_drops=get32(p+48); s->event_drops=get32(p+52); s->uart_errors=get32(p+56);
        for(i=0;i<3;++i) {
            const uint8_t *q=p+60+36*i; NeckWireNode *n=&s->nodes[i];
            n->accel_sample_time_us=get64(q); n->orientation_sample_time_us=get64(q+8);
            n->quality_flags=get32(q+16); n->filter_flags=get32(q+20); n->reference_id=get32(q+24);
            n->status=q[28]; n->accel_calibration=q[29]; n->system_calibration=q[30]; n->time_flags=q[31]; n->zero_status=q[32];
            if(q[33] || q[34] || q[35]) return 0;
        } break;
    }
    case NECK_WIRE_RAW_FRAME: {
        NeckWireRawFrame *raw=&m->body.raw;
        raw->sequence=get32(p); raw->sample_period_us=get32(p+4);
        raw->node_mask=p[8]; raw->flags=p[9]; raw->reserved=get16(p+10);
        for(i=0;i<3;++i) {
            const uint8_t *q=p+12+36*i; NeckWireRawNode *n=&raw->nodes[i];
            n->timestamp_start_us=get32(q); n->timestamp_end_us=get32(q+4); n->timestamp_mid_us=get32(q+8);
            for(j=0;j<4;++j) n->quat_wxyz[j]=(int16_t)get16(q+12+2*j);
            for(j=0;j<3;++j) n->accel_xyz[j]=(int16_t)get16(q+20+2*j);
            for(j=0;j<3;++j) n->gyro_xyz[j]=(int16_t)get16(q+26+2*j);
            n->temperature_c=geti8(q[32]); n->calibration_status=q[33]; n->flags=get16(q+34);
        } break;
    }
    default: return 0;
    }
    return 1;
}
NeckWireResult neck_wire_encode(const NeckWireMessage *m, uint8_t *output, size_t capacity, size_t *written)
{
    size_t payload,total;
    if(written) *written=0;
    if(!m || !output || !written) return NECK_WIRE_BAD_ARGUMENT;
    payload=neck_wire_payload_size(m->header.type); if(!payload) return NECK_WIRE_BAD_TYPE;
    total=NECK_WIRE_HEADER_SIZE+payload+NECK_WIRE_CRC_SIZE;
    if(!m->header.boot_id || !m->header.session_id || !m->header.config_id || !body_valid(m,0)) return NECK_WIRE_BAD_PAYLOAD;
    if(capacity<total) return NECK_WIRE_NO_SPACE;
    memset(output,0,total); memcpy(output,sync_bytes,4); output[4]=NECK_WIRE_VERSION; output[5]=m->header.type;
    put16(output+6,(uint16_t)payload); put32(output+8,m->header.tx_sequence); put32(output+12,m->header.boot_id);
    put32(output+16,m->header.session_id); put32(output+20,m->header.config_id); put64(output+24,m->header.emit_time_us);
    write_body(output+NECK_WIRE_HEADER_SIZE,m);
    put16(output+total-2,neck_wire_crc16(output+4,total-6)); *written=total; return NECK_WIRE_OK;
}
NeckWireResult neck_wire_decode(const uint8_t *frame, size_t length, NeckWireMessage *output)
{
    NeckWireResult result=NECK_WIRE_OK; size_t payload;
    if(!output) return NECK_WIRE_BAD_ARGUMENT;
    memset(output,0,sizeof(*output)); if(!frame) return NECK_WIRE_BAD_ARGUMENT;
    if(length<NECK_WIRE_HEADER_SIZE+2 || memcmp(frame,sync_bytes,4)) return NECK_WIRE_BAD_LENGTH;
    payload=get16(frame+6);
    if(payload>NECK_WIRE_MAX_PAYLOAD || length!=NECK_WIRE_HEADER_SIZE+payload+2) return NECK_WIRE_BAD_LENGTH;
    if(get16(frame+length-2)!=neck_wire_crc16(frame+4,length-6)) return NECK_WIRE_BAD_CRC;
    if(frame[4]!=NECK_WIRE_VERSION) return NECK_WIRE_BAD_VERSION;
    if(!neck_wire_payload_size(frame[5])) return NECK_WIRE_BAD_TYPE;
    if(payload!=neck_wire_payload_size(frame[5])) return NECK_WIRE_BAD_LENGTH;
    output->header.type=frame[5]; output->header.tx_sequence=get32(frame+8); output->header.boot_id=get32(frame+12);
    output->header.session_id=get32(frame+16); output->header.config_id=get32(frame+20); output->header.emit_time_us=get64(frame+24);
    if(!output->header.boot_id || !output->header.session_id || !output->header.config_id ||
       !read_body(frame+NECK_WIRE_HEADER_SIZE,output) || !body_valid(output,1)) result=NECK_WIRE_BAD_PAYLOAD;
    if(result!=NECK_WIRE_OK) memset(output,0,sizeof(*output));
    return result;
}
void neck_wire_parser_init(NeckWireParser *p) { if(p) memset(p,0,sizeof(*p)); }
static void drop_bytes(NeckWireParser *p, size_t length, int discarded)
{
    if(discarded) p->stats.discarded_bytes+=length;
    p->used-=length; memmove(p->buffer,p->buffer+length,p->used);
}
static size_t drain(NeckWireParser *p, NeckWireCallback callback, void *user)
{
    size_t count=0;
    while(p->used>=4) {
        size_t payload,total; NeckWireResult result; NeckWireMessage message;
        if(memcmp(p->buffer,sync_bytes,4)) { drop_bytes(p,1,1); continue; }
        if(p->used<8) break;
        payload=get16(p->buffer+6);
        if(payload>NECK_WIRE_MAX_PAYLOAD || (p->buffer[4]==NECK_WIRE_VERSION &&
           neck_wire_payload_size(p->buffer[5]) && payload!=neck_wire_payload_size(p->buffer[5]))) {
            ++p->stats.length_errors; drop_bytes(p,1,1); continue;
        }
        total=NECK_WIRE_HEADER_SIZE+payload+2; if(p->used<total) break;
        result=neck_wire_decode(p->buffer,total,&message);
        if(result==NECK_WIRE_OK) {
            ++p->stats.frames; ++count; if(callback) callback(&message,user); drop_bytes(p,total,0);
        } else if(result==NECK_WIRE_BAD_CRC) { ++p->stats.crc_errors; drop_bytes(p,1,1); }
        else {
            if(result==NECK_WIRE_BAD_VERSION) ++p->stats.version_errors;
            else if(result==NECK_WIRE_BAD_TYPE) ++p->stats.type_errors;
            else if(result==NECK_WIRE_BAD_LENGTH) ++p->stats.length_errors;
            else ++p->stats.payload_errors;
            /* A CRC-valid unsupported envelope is consumed whole, not searched
             * for fake nested frames inside its payload. */
            drop_bytes(p,total,1);
        }
    }
    return count;
}
size_t neck_wire_parser_feed(NeckWireParser *p, const uint8_t *bytes, size_t length, NeckWireCallback callback, void *user)
{
    size_t i,count=0;
    if(!p || (!bytes && length)) return 0;
    for(i=0;i<length;++i) {
        if(p->used==NECK_WIRE_MAX_FRAME) { ++p->stats.length_errors; drop_bytes(p,1,1); }
        p->buffer[p->used++]=bytes[i]; count+=drain(p,callback,user);
    }
    return count;
}
size_t neck_wire_parser_gap(NeckWireParser *p, NeckWireCallback callback, void *user)
{
    size_t count=0;
    if(!p || !p->used) return 0;
    ++p->stats.partial_gaps; drop_bytes(p,1,1);
    while(p->used) {
        count+=drain(p,callback,user);
        if(p->used) drop_bytes(p,1,1);
    }
    return count;
}
NeckWireResult neck_wire_receiver_init(NeckWireReceiver *r, uint64_t link_timeout, uint64_t pose_timeout)
{
    if(!r) return NECK_WIRE_BAD_ARGUMENT;
    memset(r,0,sizeof(*r)); if(!link_timeout || !pose_timeout) return NECK_WIRE_BAD_ARGUMENT;
    r->link_timeout_us=link_timeout; r->pose_timeout_us=pose_timeout; return NECK_WIRE_OK;
}
void neck_wire_receiver_disconnect(NeckWireReceiver *r)
{
    uint64_t link,pose; if(!r) return; link=r->link_timeout_us; pose=r->pose_timeout_us;
    memset(r,0,sizeof(*r)); r->link_timeout_us=link; r->pose_timeout_us=pose;
}
static int same_session(const NeckWireSession *a, const NeckWireSession *b)
{ uint8_t x[NECK_WIRE_SESSION_SIZE],y[NECK_WIRE_SESSION_SIZE]; write_session(x,a); write_session(y,b); return memcmp(x,y,sizeof(x))==0; }
NeckWireReceiveResult neck_wire_receiver_accept(NeckWireReceiver *r, const NeckWireMessage *m, uint64_t host_time)
{
    uint32_t delta=0,cap=0;
    if(m && m->header.type==NECK_WIRE_COMMAND) return NECK_WIRE_RX_UNSUPPORTED;
    if(!r || !m || !r->link_timeout_us || !r->pose_timeout_us || !m->header.boot_id ||
       !m->header.session_id || !m->header.config_id || !body_valid(m,1)) return NECK_WIRE_RX_BAD_MESSAGE;
    if(!r->bound && m->header.type!=NECK_WIRE_SESSION) return NECK_WIRE_RX_UNREGISTERED;
    if(r->bound) {
        if(m->header.boot_id!=r->boot_id || m->header.session_id!=r->session_id || m->header.config_id!=r->config_id)
            return m->header.type==NECK_WIRE_SESSION?NECK_WIRE_RX_BOUNDARY_REQUIRED:NECK_WIRE_RX_UNREGISTERED;
        if(m->header.type==NECK_WIRE_SESSION && !same_session(&m->body.session,&r->session)) return NECK_WIRE_RX_BOUNDARY_REQUIRED;
        delta=m->header.tx_sequence-r->last_tx_sequence;
        if(!delta) return NECK_WIRE_RX_DUPLICATE;
        if(delta>=UINT32_C(0x80000000)) return NECK_WIRE_RX_OUT_OF_ORDER;
        if(host_time<r->last_arrival_us || m->header.emit_time_us<r->last_emit_us) return NECK_WIRE_RX_CLOCK_FAULT;
    }
    switch(m->header.type) {
    case NECK_WIRE_C4: cap=NECK_WIRE_CAP_C4; break;
    case NECK_WIRE_RELATIVE: cap=NECK_WIRE_CAP_RELATIVE; break;
    case NECK_WIRE_EVENT: cap=NECK_WIRE_CAP_EVENTS; break;
    case NECK_WIRE_STATUS: cap=NECK_WIRE_CAP_STATUS; break;
    case NECK_WIRE_RAW_FRAME: cap=NECK_WIRE_CAP_RAW_FRAME; break;
    case NECK_WIRE_REPLY: cap=NECK_WIRE_CAP_CONTROL; break;
    default: break;
    }
    if(r->bound && cap && !(r->session.capabilities&cap)) return NECK_WIRE_RX_UNSUPPORTED;
    if(r->bound && m->header.type==NECK_WIRE_RELATIVE && m->body.relative.world_frame_id!=r->session.world_frame_id)
        return NECK_WIRE_RX_UNREGISTERED;
    if(r->bound && m->header.type==NECK_WIRE_STATUS &&
       ((m->body.status.accel_usable_mask|m->body.status.orientation_usable_mask)&~r->session.node_mask)) return NECK_WIRE_RX_BAD_MESSAGE;
    if(m->header.type==NECK_WIRE_SESSION) {
        r->session=m->body.session; r->boot_id=m->header.boot_id; r->session_id=m->header.session_id;
        r->config_id=m->header.config_id; r->bound=1;
    }
    if(delta>1) r->tx_gaps+=(uint64_t)delta-1;
    r->last_tx_sequence=m->header.tx_sequence; r->last_emit_us=m->header.emit_time_us; r->last_arrival_us=host_time;
    if(m->header.type==NECK_WIRE_C4) { r->c4_seen=1; r->c4_arrival_us=host_time; }
    if(m->header.type==NECK_WIRE_RELATIVE) { r->relative_seen=1; r->relative_arrival_us=host_time; }
    return m->header.type==NECK_WIRE_SESSION?NECK_WIRE_RX_SESSION:NECK_WIRE_RX_DATA;
}
uint32_t neck_wire_receiver_poll(const NeckWireReceiver *r, uint64_t now)
{
    uint32_t flags=0;
    if(!r || !r->bound || !r->link_timeout_us || !r->pose_timeout_us)
        return NECK_WIRE_HOST_LINK_STALE|NECK_WIRE_HOST_C4_STALE|NECK_WIRE_HOST_RELATIVE_STALE;
    if(now<r->last_arrival_us) return NECK_WIRE_HOST_CLOCK_FAULT|NECK_WIRE_HOST_LINK_STALE|
        NECK_WIRE_HOST_C4_STALE|NECK_WIRE_HOST_RELATIVE_STALE;
    if(now-r->last_arrival_us>r->link_timeout_us) flags|=NECK_WIRE_HOST_LINK_STALE;
    if(!r->c4_seen || now-r->c4_arrival_us>r->pose_timeout_us) flags|=NECK_WIRE_HOST_C4_STALE;
    if(!(r->session.capabilities&NECK_WIRE_CAP_RELATIVE) || !r->relative_seen ||
       now-r->relative_arrival_us>r->pose_timeout_us) flags|=NECK_WIRE_HOST_RELATIVE_STALE;
    return flags;
}
