/* Protocol fixtures are synthetic values, NOT outputs of real sensors. */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include "neck_wire.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif
static unsigned groups,checks;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
typedef struct { unsigned count; NeckWireMessage last; } Capture;
static void capture(const NeckWireMessage *m, void *user)
{ Capture *c=(Capture *)user; ++c->count; c->last=*m; }
static NeckWireMessage example(uint8_t type)
{
    NeckWireMessage m; unsigned i,j; memset(&m,0,sizeof(m));
    m.header.type=type; m.header.tx_sequence=type; m.header.boot_id=11; m.header.session_id=22;
    m.header.config_id=33; m.header.emit_time_us=1002000;
    switch(type) {
    case NECK_WIRE_SESSION: {
        NeckWireSession *s=&m.body.session;
        s->profile_id=1; s->firmware_id=0xd0000001u; s->world_frame_id=44; s->mode=3; s->node_mask=7;
        s->sample_period_us=10000; s->quality_timeout_us=30000; s->cutoff_millihz=500;
        s->continuous_duration_us=10000000; s->thresholds_mdeg[0]=15000; s->thresholds_mdeg[1]=30000;
        s->thresholds_mdeg[2]=5000; s->thresholds_mdeg[3]=10000;
        s->max_node_skew_us=20000; s->c4_period_us=50000; s->relative_period_us=100000;
        s->status_period_us=500000; s->capabilities=15; break;
    }
    case NECK_WIRE_C4: {
        NeckWireC4 *c=&m.body.c4;
        c->input_sequence=100; c->frame_time_us=1000000; c->decision_time_us=1001000; c->valid_until_us=1031000;
        c->invalidation_epoch=1; c->reference_id=9; c->valid_mask=63; c->time_flags=7; c->zero_status=3;
        for(i=0;i<6;++i) c->angles_mdeg[i]=(int32_t)i*1000-2000;
        for(i=0;i<2;++i) { c->axes[i].valid=1; c->axes[i].instant_level=1; c->axes[i].alarm_level=1; }
        break;
    }
    case NECK_WIRE_RELATIVE: {
        NeckWireRelative *r=&m.body.relative;
        r->input_sequence=100; r->frame_time_us=1000000; r->decision_time_us=1001000; r->valid_until_us=1031000;
        r->invalidation_epoch=1; r->reference_id=9; r->world_frame_id=44; r->time_flags=7;
        for(i=0;i<4;++i) { r->pairs[i].valid_mask=7; r->pairs[i].status=2; r->pairs[i].sample_skew_us=123;
            for(j=0;j<3;++j) { r->pairs[i].raw_mdeg[j]=(int32_t)(i*1000+j); r->pairs[i].corrected_mdeg[j]=-(int32_t)(i*100+j); } }
        break;
    }
    case NECK_WIRE_EVENT: {
        NeckWireEvent *e=&m.body.event;
        e->event_id=7; e->input_sequence=100; e->frame_time_us=1000000; e->event_time_us=1001000;
        e->reference_id=9; e->invalidation_epoch=1; e->events=10; e->posture_flags=1024;
        e->previous_level=2; e->level=0; e->time_flags=3; break;
    }
    case NECK_WIRE_STATUS: {
        NeckWireStatus *s=&m.body.status;
        s->input_sequence=100; s->frame_time_us=1000000; s->decision_time_us=1001000;
        s->invalidation_epoch=1; s->accel_usable_mask=7; s->orientation_usable_mask=7;
        s->quality_state=1; s->frame_accepted=1; s->input_drops=2; s->event_drops=3;
        for(i=0;i<3;++i) { s->nodes[i].accel_sample_time_us=999999+i; s->nodes[i].orientation_sample_time_us=999998+i;
            s->nodes[i].time_flags=3; s->nodes[i].accel_calibration=3; s->nodes[i].system_calibration=255;
            s->nodes[i].reference_id=9; s->nodes[i].zero_status=3; }
        break;
    }
    default: break;
    }
    return m;
}
static size_t encode(const NeckWireMessage *m, uint8_t *bytes)
{ size_t n=0; CHECK(neck_wire_encode(m,bytes,NECK_WIRE_MAX_FRAME,&n)==NECK_WIRE_OK); CHECK(n==neck_wire_payload_size(m->header.type)+34); return n; }
static NeckWireMessage normalized(NeckWireMessage m)
{ uint8_t b[NECK_WIRE_MAX_FRAME]; size_t n=encode(&m,b); CHECK(neck_wire_decode(b,n,&m)==NECK_WIRE_OK); return m; }
static void repair_crc(uint8_t *b, size_t n)
{ uint16_t crc=neck_wire_crc16(b+4,n-6); b[n-2]=(uint8_t)crc; b[n-1]=(uint8_t)(crc>>8); }
static void crc_and_layout(void)
{
    NeckWireMessage m=example(NECK_WIRE_EVENT),d; uint8_t b[NECK_WIRE_MAX_FRAME]; size_t n;
    static const uint8_t golden[90]={
        0x4e,0x43,0x4b,0x31,0x01,0x04,0x38,0x00,0x04,0x00,0x00,0x00,0x0b,0x00,0x00,0x00,
        0x16,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x10,0x4a,0x0f,0x00,0x00,0x00,0x00,0x00,
        0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x00,0x28,0x46,0x0f,0x00,0x00,0x00,0x00,0x00,
        0x09,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,0x00,0x04,0x00,0x00,
        0x00,0x02,0x00,0x00,0x03,0x00,0x00,0x00,0xfd,0x5a};
    ++groups; CHECK(neck_wire_crc16((const uint8_t *)"123456789",9)==0x29b1);
    CHECK(neck_wire_crc16(NULL,0)==0xffff); n=encode(&m,b); CHECK(n==90);
    CHECK(memcmp(b,"NCK1",4)==0 && b[4]==1 && b[5]==4 && b[6]==56 && b[7]==0);
    CHECK(b[8]==4 && b[12]==11 && b[16]==22 && b[20]==33);
    CHECK(b[24]==0x10 && b[25]==0x4a && b[26]==0x0f); /* 1,002,000 little endian */
    CHECK(memcmp(b,golden,n)==0);
    CHECK(b[32]==7 && b[40]==100 && b[64]==9 && b[72]==10 && b[77]==4);
    CHECK(b[80]==0 && b[81]==2 && b[82]==0 && b[83]==0 && b[84]==3);
    CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_OK); CHECK(d.body.event.events==10 && d.body.event.level==0);
}
static void all_roundtrips(void)
{
    uint8_t b[NECK_WIRE_MAX_FRAME],again[NECK_WIRE_MAX_FRAME]; unsigned type; size_t n,k; NeckWireMessage m,d;
    ++groups;
    for(type=1;type<=5;++type) {
        m=example((uint8_t)type); n=encode(&m,b); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_OK);
        k=encode(&d,again); CHECK(n==k && memcmp(b,again,n)==0);
        CHECK(d.header.type==type && d.header.boot_id==11 && d.header.emit_time_us==1002000);
        /* Independent byte offsets for the PC implementation, not only a
         * codec roundtrip which could conceal matching layout mistakes. */
        if(type==NECK_WIRE_SESSION) {
            CHECK(b[44]==0 && b[45]==3 && b[46]==7);
            CHECK(b[56]==0xf4 && b[57]==1 && b[68]==0x98 && b[69]==0x3a && b[100]==15);
            CHECK(d.body.session.thresholds_mdeg[3]==10000 && d.body.session.continuous_duration_us==10000000);
        } else if(type==NECK_WIRE_C4) {
            CHECK(b[84]==63 && b[86]==7 && b[87]==3 && b[104]==0xd0 && b[105]==7);
            CHECK(b[96]==0 && b[97]==0 && b[112]==1 && b[140]==1);
            CHECK(d.body.c4.angles_mdeg[4]==2000 && d.body.c4.axes[1].valid==1);
        } else if(type==NECK_WIRE_RELATIVE) {
            CHECK(b[80]==7 && b[204]==7 && b[205]==2 && b[212]==0xb8 && b[213]==0x0b);
            CHECK(b[224]==0xd4 && b[225]==0xfe && b[226]==0xff && b[236]==123);
            CHECK(d.body.relative.pairs[3].corrected_mdeg[0]==-300 && d.body.relative.pairs[3].sample_skew_us==123);
        } else if(type==NECK_WIRE_STATUS) {
            CHECK(b[64]==7 && b[65]==7 && b[66]==1 && b[67]==1 && b[76]==2);
            CHECK(b[164]==0x41 && b[165]==0x42 && b[194]==255 && b[195]==3);
            CHECK(d.body.status.nodes[2].accel_sample_time_us==1000001 && d.body.status.nodes[2].system_calibration==255);
        }
    }
}
static void int64_and_signed(void)
{
    NeckWireMessage m=example(NECK_WIRE_C4),d; uint8_t b[NECK_WIRE_MAX_FRAME]; uint64_t t=UINT64_C(9007199254740993); size_t n;
    ++groups; m.header.emit_time_us=t+2; m.body.c4.input_sequence=UINT64_MAX;
    m.body.c4.frame_time_us=t; m.body.c4.decision_time_us=t+1; m.body.c4.valid_until_us=t+30001;
    m.body.c4.angles_mdeg[0]=-360000; m.body.c4.angles_mdeg[1]=360000; m.body.c4.angles_mdeg[2]=0;
    m.body.c4.axes[0].direction=-1; m.body.c4.axes[0].warning_elapsed_us=UINT64_MAX;
    n=encode(&m,b); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_OK);
    CHECK(d.body.c4.frame_time_us==t && d.body.c4.input_sequence==UINT64_MAX);
    CHECK(d.body.c4.axes[0].direction==-1 && d.body.c4.axes[0].warning_elapsed_us==UINT64_MAX);
    CHECK(d.body.c4.angles_mdeg[0]==-360000 && d.body.c4.angles_mdeg[1]==360000 && d.body.c4.angles_mdeg[2]==0);
}
static void invalid_is_not_zero(void)
{
    NeckWireMessage m=example(NECK_WIRE_C4),d; uint8_t b[NECK_WIRE_MAX_FRAME]; size_t n;
    ++groups; m.body.c4.valid_mask=0; memset(m.body.c4.axes,0,sizeof(m.body.c4.axes));
    m.body.c4.angles_mdeg[0]=INT32_MAX; m.body.c4.quality_flags=128; m.body.c4.time_flags=0;
    n=encode(&m,b); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_OK);
    CHECK(d.body.c4.angles_mdeg[0]==INT32_MIN && d.body.c4.angles_mdeg[5]==INT32_MIN);
    CHECK(d.body.c4.frame_time_us==0 && d.body.c4.decision_time_us==0 && d.body.c4.valid_until_us==0);
    CHECK(d.body.c4.axes[0].instant_level==0 && d.body.c4.axes[0].alarm_level==0);
    memset(b+32+56,0,4); repair_crc(b,n); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_BAD_PAYLOAD); CHECK(d.header.type==0);
    m=example(NECK_WIRE_RELATIVE); m.body.relative.pairs[0].valid_mask=0; m.body.relative.pairs[0].status=0;
    n=encode(&m,b); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_OK);
    CHECK(d.body.relative.pairs[0].raw_mdeg[0]==INT32_MIN && d.body.relative.pairs[0].sample_skew_us==0);
    m=example(NECK_WIRE_STATUS); m.body.status.nodes[0].time_flags=0;
    n=encode(&m,b); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_OK && d.body.status.nodes[0].accel_sample_time_us==0);
}
static void argument_and_payload_errors(void)
{
    NeckWireMessage m=example(NECK_WIRE_C4),d; uint8_t b[NECK_WIRE_MAX_FRAME]; size_t n=99;
    ++groups; memset(b,0xa5,sizeof(b));
    CHECK(neck_wire_encode(&m,b,169,&n)==NECK_WIRE_NO_SPACE && n==0 && b[0]==0xa5);
    CHECK(neck_wire_encode(NULL,b,sizeof(b),&n)==NECK_WIRE_BAD_ARGUMENT && n==0);
    CHECK(neck_wire_encode(&m,NULL,sizeof(b),&n)==NECK_WIRE_BAD_ARGUMENT);
    CHECK(neck_wire_encode(&m,b,sizeof(b),NULL)==NECK_WIRE_BAD_ARGUMENT);
    CHECK(neck_wire_decode(NULL,0,&d)==NECK_WIRE_BAD_ARGUMENT && d.header.type==0);
    CHECK(neck_wire_decode(b,0,NULL)==NECK_WIRE_BAD_ARGUMENT);
    m.body.c4.angles_mdeg[0]=INT32_MIN; CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD && b[0]==0xa5);
    m=example(NECK_WIRE_C4); m.body.c4.reference_id=0; CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_C4); m.body.c4.axes[0].valid=0; CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_C4); m.body.c4.quality_flags=128; CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_C4); m.header.emit_time_us=m.body.c4.valid_until_us+1;
    CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_C4); m.body.c4.time_flags=3;
    CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_SESSION); m.body.session.source=1; CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_SESSION); m.body.session.profile_id=2; CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(99); CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_TYPE);
    m=example(NECK_WIRE_STATUS); m.body.status.nodes[0].system_calibration=4;
    CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
    m=example(NECK_WIRE_STATUS); m.body.status.quality_flags=128;
    CHECK(neck_wire_encode(&m,b,sizeof(b),&n)==NECK_WIRE_BAD_PAYLOAD);
}
static void exact_decode_errors(void)
{
    NeckWireMessage m=example(NECK_WIRE_EVENT),d; uint8_t b[NECK_WIRE_MAX_FRAME]; size_t n=encode(&m,b);
    ++groups; CHECK(neck_wire_decode(b,n-1,&d)==NECK_WIRE_BAD_LENGTH);
    CHECK(neck_wire_decode(b,n+1,&d)==NECK_WIRE_BAD_LENGTH); b[40]^=1;
    CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_BAD_CRC); b[40]^=1;
    b[4]=2; repair_crc(b,n); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_BAD_VERSION);
    b[4]=1; b[5]=99; repair_crc(b,n); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_BAD_TYPE);
    b[5]=4; b[87]=1; repair_crc(b,n); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_BAD_PAYLOAD && d.header.type==0);
    b[87]=0; b[12]=0; repair_crc(b,n); CHECK(neck_wire_decode(b,n,&d)==NECK_WIRE_BAD_PAYLOAD);
}
static void stream_chunks(void)
{
    NeckWireMessage m=example(NECK_WIRE_RELATIVE); uint8_t b[NECK_WIRE_MAX_FRAME]; size_t n=encode(&m,b),split;
    ++groups;
    for(split=0;split<=n;++split) {
        NeckWireParser p; Capture c={0}; neck_wire_parser_init(&p);
        CHECK(neck_wire_parser_feed(&p,b,split,capture,&c)==(split==n?1u:0u));
        (void)neck_wire_parser_feed(&p,b+split,n-split,capture,&c);
        CHECK(c.count==1 && c.last.header.type==3 && p.used==0 && p.stats.frames==1);
    }
    { NeckWireParser p; Capture c={0}; size_t i; neck_wire_parser_init(&p);
      for(i=0;i<n;++i) (void)neck_wire_parser_feed(&p,b+i,1,capture,&c);
      CHECK(c.count==1 && p.used==0); }
}
static void stream_noise_and_glue(void)
{
    NeckWireParser p; Capture c={0}; uint8_t stream[1024]; size_t n=0; unsigned type;
    const uint8_t noise[]={0x4e,0x4e,0x43,0x4b,0x00,0x31,0xff}; ++groups;
    memcpy(stream,noise,sizeof(noise)); n=sizeof(noise);
    for(type=1;type<=5;++type) { NeckWireMessage m=example((uint8_t)type); n+=encode(&m,stream+n); }
    neck_wire_parser_init(&p); CHECK(neck_wire_parser_feed(&p,stream,n,capture,&c)==5);
    CHECK(c.count==5 && c.last.header.type==5 && p.used==0 && p.stats.discarded_bytes==sizeof(noise));
    CHECK(neck_wire_parser_feed(&p,NULL,1,capture,&c)==0 && p.used==0);
}
static void stream_all_single_bit_errors(void)
{
    NeckWireMessage m=example(NECK_WIRE_EVENT); uint8_t good[NECK_WIRE_MAX_FRAME],bad[NECK_WIRE_MAX_FRAME];
    size_t n=encode(&m,good),i; unsigned bit; ++groups;
    for(i=0;i<n;++i) for(bit=0;bit<8;++bit) {
        NeckWireParser p; Capture c={0}; neck_wire_parser_init(&p); memcpy(bad,good,n); bad[i]^=(uint8_t)(1u<<bit);
        (void)neck_wire_parser_feed(&p,bad,n,capture,&c); (void)neck_wire_parser_feed(&p,good,n,capture,&c);
        (void)neck_wire_parser_gap(&p,capture,&c);
        CHECK(c.count==1 && c.last.body.event.event_id==7 && p.used==0);
    }
}
static void stream_length_and_idle(void)
{
    NeckWireMessage m=example(NECK_WIRE_C4),e=example(NECK_WIRE_EVENT); uint8_t b[NECK_WIRE_MAX_FRAME],good[NECK_WIRE_MAX_FRAME];
    NeckWireParser p; Capture c={0}; size_t n=encode(&m,b),k=encode(&e,good); ++groups;
    neck_wire_parser_init(&p); b[6]=0xff; b[7]=0xff;
    (void)neck_wire_parser_feed(&p,b,n,capture,&c); (void)neck_wire_parser_feed(&p,good,k,capture,&c);
    CHECK(c.count==1 && p.stats.length_errors==1); c.count=0; neck_wire_parser_init(&p);
    n=encode(&m,b); (void)neck_wire_parser_feed(&p,b,40,capture,&c);
    (void)neck_wire_parser_feed(&p,good,k,capture,&c); CHECK(c.count==0 && p.used==130);
    CHECK(neck_wire_parser_gap(&p,capture,&c)==1 && c.count==1 && p.used==0 && p.stats.partial_gaps==1);
    CHECK(neck_wire_parser_gap(&p,capture,&c)==0);
    (void)neck_wire_parser_feed(&p,b,3,NULL,NULL); CHECK(neck_wire_parser_gap(&p,NULL,NULL)==0 && p.used==0);
}
static void stream_unsupported_and_reserved(void)
{
    NeckWireMessage m=example(NECK_WIRE_EVENT); uint8_t inner[NECK_WIRE_MAX_FRAME],outer[NECK_WIRE_MAX_FRAME];
    size_t n=encode(&m,inner),total=n+34; NeckWireParser p; Capture c={0}; ++groups;
    memset(outer,0,total); memcpy(outer,inner,32); outer[5]=99; outer[6]=(uint8_t)n; outer[7]=0;
    memcpy(outer+32,inner,n); repair_crc(outer,total); neck_wire_parser_init(&p);
    CHECK(neck_wire_parser_feed(&p,outer,total,capture,&c)==0 && p.stats.type_errors==1 && c.count==0);
    outer[4]=2; repair_crc(outer,total); CHECK(neck_wire_parser_feed(&p,outer,total,capture,&c)==0 && p.stats.version_errors==1);
    CHECK(neck_wire_parser_feed(&p,inner,n,capture,&c)==1 && c.count==1);
    inner[87]=1; repair_crc(inner,n); CHECK(neck_wire_parser_feed(&p,inner,n,capture,&c)==0 && p.stats.payload_errors==1);
}
static void deterministic_noise(void)
{
    NeckWireParser p; Capture c={0}; uint8_t noise[997],b[NECK_WIRE_MAX_FRAME]; uint32_t rng=0x51932u;
    NeckWireMessage m=example(NECK_WIRE_EVENT); size_t n=encode(&m,b); unsigned round,i; ++groups; neck_wire_parser_init(&p);
    for(round=0;round<200;++round) {
        for(i=0;i<sizeof(noise);++i) { rng=rng*1664525u+1013904223u; noise[i]=(uint8_t)(rng>>24); }
        (void)neck_wire_parser_feed(&p,noise,sizeof(noise),capture,&c); (void)neck_wire_parser_feed(&p,b,n,capture,&c);
        CHECK(p.used<=NECK_WIRE_MAX_FRAME && c.count==round+1);
    }
    (void)neck_wire_parser_gap(&p,capture,&c); CHECK(c.count==200 && p.stats.frames==200);
}
static void receiver_registration_and_order(void)
{
    NeckWireReceiver r; NeckWireMessage s=normalized(example(NECK_WIRE_SESSION)),c=normalized(example(NECK_WIRE_C4));
    ++groups; CHECK(neck_wire_receiver_init(&r,500000,250000)==NECK_WIRE_OK);
    CHECK(neck_wire_receiver_accept(&r,&c,0)==NECK_WIRE_RX_UNREGISTERED && !r.bound);
    CHECK(neck_wire_receiver_accept(&r,&s,0)==NECK_WIRE_RX_SESSION && r.bound);
    CHECK(neck_wire_receiver_accept(&r,&c,1000)==NECK_WIRE_RX_DATA);
    CHECK(neck_wire_receiver_accept(&r,&c,2000)==NECK_WIRE_RX_DUPLICATE && r.last_arrival_us==1000);
    c.header.tx_sequence=1; CHECK(neck_wire_receiver_accept(&r,&c,2000)==NECK_WIRE_RX_OUT_OF_ORDER);
    c.header.tx_sequence=5; CHECK(neck_wire_receiver_accept(&r,&c,2000)==NECK_WIRE_RX_DATA && r.tx_gaps==2);
    c.header.tx_sequence=6; c.header.boot_id=12;
    CHECK(neck_wire_receiver_accept(&r,&c,3000)==NECK_WIRE_RX_UNREGISTERED && r.last_tx_sequence==5);
    s.header.tx_sequence=6; s.header.boot_id=12;
    CHECK(neck_wire_receiver_accept(&r,&s,3000)==NECK_WIRE_RX_BOUNDARY_REQUIRED && r.boot_id==11);
    neck_wire_receiver_disconnect(&r); CHECK(!r.bound && !r.c4_seen && r.pose_timeout_us==250000);
    CHECK(neck_wire_receiver_accept(&r,&s,3000)==NECK_WIRE_RX_SESSION && r.boot_id==12);
    s.body.session.cutoff_millihz=600; s.header.tx_sequence=7;
    CHECK(neck_wire_receiver_accept(&r,&s,4000)==NECK_WIRE_RX_BOUNDARY_REQUIRED);
}
static void receiver_rollover_and_clock(void)
{
    NeckWireReceiver r; NeckWireMessage s=normalized(example(NECK_WIRE_SESSION)),c=normalized(example(NECK_WIRE_C4));
    ++groups; CHECK(neck_wire_receiver_init(&r,1,1)==NECK_WIRE_OK); s.header.tx_sequence=UINT32_MAX;
    CHECK(neck_wire_receiver_accept(&r,&s,10)==NECK_WIRE_RX_SESSION); c.header.tx_sequence=0;
    CHECK(neck_wire_receiver_accept(&r,&c,11)==NECK_WIRE_RX_DATA && r.tx_gaps==0);
    c.header.tx_sequence=1; CHECK(neck_wire_receiver_accept(&r,&c,9)==NECK_WIRE_RX_CLOCK_FAULT && r.last_tx_sequence==0);
    c.header.emit_time_us=1001999;
    CHECK(neck_wire_receiver_accept(&r,&c,12)==NECK_WIRE_RX_CLOCK_FAULT);
    CHECK(neck_wire_receiver_poll(&r,10)&NECK_WIRE_HOST_CLOCK_FAULT);
    CHECK(neck_wire_receiver_init(&r,0,1)==NECK_WIRE_BAD_ARGUMENT && !r.bound);
}
static void receiver_timeout_and_status_heartbeats(void)
{
    NeckWireReceiver r; NeckWireMessage s=normalized(example(NECK_WIRE_SESSION)),c=normalized(example(NECK_WIRE_C4));
    NeckWireMessage rel=normalized(example(NECK_WIRE_RELATIVE)),status=normalized(example(NECK_WIRE_STATUS));
    uint32_t flags; ++groups; CHECK(neck_wire_receiver_init(&r,500000,250000)==NECK_WIRE_OK);
    CHECK(neck_wire_receiver_accept(&r,&s,0)==NECK_WIRE_RX_SESSION);
    CHECK(neck_wire_receiver_poll(&r,0)==(NECK_WIRE_HOST_C4_STALE|NECK_WIRE_HOST_RELATIVE_STALE));
    CHECK(neck_wire_receiver_accept(&r,&c,0)==NECK_WIRE_RX_DATA);
    CHECK(neck_wire_receiver_accept(&r,&rel,0)==NECK_WIRE_RX_DATA);
    CHECK(neck_wire_receiver_poll(&r,250000)==0);
    CHECK(neck_wire_receiver_accept(&r,&status,250001)==NECK_WIRE_RX_DATA);
    flags=neck_wire_receiver_poll(&r,250001);
    CHECK(flags==(NECK_WIRE_HOST_C4_STALE|NECK_WIRE_HOST_RELATIVE_STALE));
    CHECK(neck_wire_receiver_poll(&r,750001)==flags);
    CHECK(neck_wire_receiver_poll(&r,750002)==(flags|NECK_WIRE_HOST_LINK_STALE));
    CHECK(neck_wire_receiver_poll(&r,250000)&NECK_WIRE_HOST_CLOCK_FAULT);
}
static void receiver_single_node_and_world(void)
{
    NeckWireReceiver r; NeckWireMessage s=example(NECK_WIRE_SESSION),rel=normalized(example(NECK_WIRE_RELATIVE));
    NeckWireMessage status=normalized(example(NECK_WIRE_STATUS)); ++groups;
    s.body.session.mode=1; s.body.session.node_mask=2; s.body.session.capabilities=13; s.body.session.relative_period_us=0; s=normalized(s);
    CHECK(neck_wire_receiver_init(&r,500000,250000)==NECK_WIRE_OK);
    CHECK(neck_wire_receiver_accept(&r,&s,0)==NECK_WIRE_RX_SESSION);
    CHECK(neck_wire_receiver_accept(&r,&rel,1)==NECK_WIRE_RX_UNSUPPORTED);
    CHECK(neck_wire_receiver_accept(&r,&status,1)==NECK_WIRE_RX_BAD_MESSAGE);
    status.body.status.accel_usable_mask=status.body.status.orientation_usable_mask=2;
    CHECK(neck_wire_receiver_accept(&r,&status,1)==NECK_WIRE_RX_DATA);
    neck_wire_receiver_disconnect(&r); s=normalized(example(NECK_WIRE_SESSION));
    CHECK(neck_wire_receiver_accept(&r,&s,2)==NECK_WIRE_RX_SESSION);
    rel.body.relative.world_frame_id=45;
    CHECK(neck_wire_receiver_accept(&r,&rel,3)==NECK_WIRE_RX_UNREGISTERED && r.last_tx_sequence==1);
}
static void fixture_file(const char *directory, const char *name, const uint8_t *data, size_t size)
{
    char path[512]; int fd; int written; int length=snprintf(path,sizeof(path),"%s/%s",directory,name);
    CHECK(length>0 && (size_t)length<sizeof(path));
#ifdef _WIN32
    fd=_open(path,_O_CREAT|_O_EXCL|_O_BINARY|_O_WRONLY,_S_IREAD|_S_IWRITE);
    CHECK(fd>=0); written=_write(fd,data,(unsigned)size); CHECK(written>=0 && (size_t)written==size); CHECK(_close(fd)==0);
#else
    fd=open(path,O_CREAT|O_EXCL|O_WRONLY,0600); CHECK(fd>=0);
    written=(int)write(fd,data,size); CHECK(written>=0 && (size_t)written==size); CHECK(close(fd)==0);
#endif
    printf("FIXTURE %s %lu bytes\n",name,(unsigned long)size);
}
static void fixtures(const char *directory)
{
    uint8_t all[2048],b[NECK_WIRE_MAX_FRAME],tail[NECK_WIRE_MAX_FRAME],fault[2048]; size_t n=0,k,t; unsigned type;
    static const char *const names[]={"session.bin","c4.bin","relative.bin","event.bin","status.bin"};
    for(type=1;type<=5;++type) { NeckWireMessage m=example((uint8_t)type); k=encode(&m,b);
        fixture_file(directory,names[type-1],b,k); memcpy(all+n,b,k); n+=k; }
    fixture_file(directory,"normal_all.bin",all,n);
    { NeckWireMessage c=example(NECK_WIRE_C4),e=example(NECK_WIRE_EVENT); k=encode(&c,b); t=encode(&e,tail);
      memcpy(fault,b,k); fault[88]^=1; memcpy(fault+k,tail,t); fixture_file(directory,"bad_crc_then_ok.bin",fault,k+t);
      memcpy(fault,b,k); fault[6]=0xff; fault[7]=0xff; memcpy(fault+k,tail,t); fixture_file(directory,"bad_length_then_ok.bin",fault,k+t);
      memcpy(fault,b,k); fault[4]=2; repair_crc(fault,k); memcpy(fault+k,tail,t); fixture_file(directory,"unknown_version_then_ok.bin",fault,k+t);
      memcpy(fault,b,40); memcpy(fault+40,tail,t); fixture_file(directory,"truncated_then_ok.bin",fault,40+t);
      c.body.c4.valid_mask=0; c.body.c4.quality_flags=128; c.body.c4.invalidation_epoch=2; memset(c.body.c4.axes,0,sizeof(c.body.c4.axes));
      k=encode(&c,b); fixture_file(directory,"invalid_c4.bin",b,k); }
    { NeckWireMessage s=example(NECK_WIRE_SESSION),c=example(NECK_WIRE_C4),status=example(NECK_WIRE_STATUS);
      s.body.session.mode=1; s.body.session.node_mask=2; s.body.session.capabilities=13; s.body.session.relative_period_us=0;
      n=encode(&s,all); fixture_file(directory,"single_session.bin",all,n); n+=encode(&c,all+n);
      status.body.status.accel_usable_mask=status.body.status.orientation_usable_mask=2;
      for(type=0;type<3;type+=2) { memset(&status.body.status.nodes[type],0,sizeof(NeckWireNode));
          status.body.status.nodes[type].status=2; status.body.status.nodes[type].accel_calibration=255;
          status.body.status.nodes[type].system_calibration=255; }
      status.header.tx_sequence=3; n+=encode(&status,all+n); fixture_file(directory,"single_node.bin",all,n); }
    { NeckWireMessage s=example(NECK_WIRE_SESSION),c=example(NECK_WIRE_C4),r=example(NECK_WIRE_RELATIVE);
      NeckWireMessage e=example(NECK_WIRE_EVENT),status=example(NECK_WIRE_STATUS); unsigned j;
      s.header.emit_time_us=c.header.emit_time_us=r.header.emit_time_us=20002000;
      c.body.c4.input_sequence=r.body.relative.input_sequence=2000;
      c.body.c4.frame_time_us=r.body.relative.frame_time_us=20000000;
      c.body.c4.decision_time_us=r.body.relative.decision_time_us=20001000;
      c.body.c4.valid_until_us=r.body.relative.valid_until_us=20031000;
      c.body.c4.angles_mdeg[4]=20000; c.body.c4.axes[0].instant_level=2;
      c.body.c4.axes[0].alarm_level=2; c.body.c4.axes[0].direction=1; c.body.c4.axes[0].warning_elapsed_us=10000000;
      n=encode(&s,all); n+=encode(&c,all+n); n+=encode(&r,all+n);
      c.header.emit_time_us=r.header.emit_time_us=e.header.emit_time_us=status.header.emit_time_us=20040001;
      c.header.tx_sequence=4; r.header.tx_sequence=5; e.header.tx_sequence=6; status.header.tx_sequence=7;
      c.body.c4.decision_time_us=r.body.relative.decision_time_us=20040001;
      c.body.c4.invalidation_epoch=r.body.relative.invalidation_epoch=2;
      c.body.c4.quality_flags=r.body.relative.quality_flags=128;
      c.body.c4.valid_mask=0; c.body.c4.posture_flags=1024; memset(c.body.c4.axes,0,sizeof(c.body.c4.axes));
      for(type=0;type<4;++type) { r.body.relative.pairs[type].valid_mask=0;
          r.body.relative.pairs[type].status=0; r.body.relative.pairs[type].flags=131072; }
      e.body.event.frame_time_us=20000000; e.body.event.event_time_us=20040001;
      e.body.event.input_sequence=2000; e.body.event.invalidation_epoch=2;
      status.body.status.input_sequence=2000; status.body.status.frame_time_us=20000000;
      status.body.status.decision_time_us=20040001; status.body.status.invalidation_epoch=2;
      status.body.status.quality_flags=128; status.body.status.accel_usable_mask=0;
      status.body.status.orientation_usable_mask=0; status.body.status.quality_state=0; status.body.status.frame_accepted=0;
      for(j=0;j<3;++j) { status.body.status.nodes[j].status=4; status.body.status.nodes[j].quality_flags=1040;
          status.body.status.nodes[j].accel_sample_time_us=status.body.status.nodes[j].orientation_sample_time_us=20000000; }
      n+=encode(&c,all+n); n+=encode(&r,all+n); n+=encode(&e,all+n); n+=encode(&status,all+n);
      fixture_file(directory,"quality_timeout_all.bin",all,n); }
}
int main(int argc, char **argv)
{
    if(argc>2) { fprintf(stderr,"Usage: test_neck_wire [new_fixture_directory]\n"); return 2; }
    crc_and_layout(); all_roundtrips(); int64_and_signed(); invalid_is_not_zero(); argument_and_payload_errors();
    exact_decode_errors(); stream_chunks(); stream_noise_and_glue(); stream_all_single_bit_errors();
    stream_length_and_idle(); stream_unsupported_and_reserved(); deterministic_noise();
    receiver_registration_and_order(); receiver_rollover_and_clock(); receiver_timeout_and_status_heartbeats();
    receiver_single_node_and_world();
    if(argc==2) fixtures(argv[1]);
    printf("D0 wire tests: %u groups, %u checks passed.\n",groups,checks);
    printf("Native sizes: Message=%lu Parser=%lu Receiver=%lu bytes.\n",
        (unsigned long)sizeof(NeckWireMessage),(unsigned long)sizeof(NeckWireParser),(unsigned long)sizeof(NeckWireReceiver));
    return 0;
}
