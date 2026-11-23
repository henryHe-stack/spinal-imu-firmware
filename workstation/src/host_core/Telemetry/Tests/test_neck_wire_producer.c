#include "neck_wire_producer.h"
#include "neck_simulator.h"
#include <stdio.h>
#include <string.h>

static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)

static int roundtrip(uint8_t type, const NeckWireSnapshot *s, uint64_t now)
{
    NeckWireMessage message, decoded;
    uint8_t data[NECK_WIRE_MAX_FRAME]; size_t used=0;
    memset(&message,0,sizeof(message));
    message.header.type=type; message.header.boot_id=101; message.header.session_id=1;
    message.header.config_id=1; message.header.emit_time_us=now;
    if(type==NECK_WIRE_C4) message.body.c4=s->c4;
    else if(type==NECK_WIRE_RELATIVE) message.body.relative=s->relative;
    else if(type==NECK_WIRE_STATUS) message.body.status=s->status;
    CHECK(neck_wire_encode(&message,data,sizeof(data),&used)==NECK_WIRE_OK);
    CHECK(used==NECK_WIRE_HEADER_SIZE+neck_wire_payload_size(type)+NECK_WIRE_CRC_SIZE);
    CHECK(neck_wire_decode(data,used,&decoded)==NECK_WIRE_OK);
    if(type==NECK_WIRE_C4) CHECK(decoded.body.c4.valid_mask==s->c4.valid_mask);
    if(type==NECK_WIRE_RELATIVE) CHECK(decoded.body.relative.reference_id==s->relative.reference_id);
    if(type==NECK_WIRE_STATUS) CHECK(decoded.body.status.quality_flags==s->status.quality_flags);
    return 0;
}
int main(void)
{
    NeckProcessingContext core; NeckProcessingParameters p;
    NeckAcquisitionFrame input; NeckProcessingOutput output;
    NeckWireSnapshot snapshot; NeckWireSession session;
    NeckWireMessage m,decoded; uint8_t bytes[NECK_WIRE_MAX_FRAME];
    NeckAlarmEvent source_event;
    uint64_t time; size_t used=0; unsigned i,c4_valid=0,relative_valid=0,corrected=0;
    neck_processing_default_parameters(&p);
    neck_wire_producer_session(&session,&p);
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_SESSION;
    m.header.boot_id=101; m.header.session_id=1; m.header.config_id=1;
    m.body.session=session;
    CHECK(neck_wire_encode(&m,bytes,sizeof(bytes),&used)==NECK_WIRE_OK);
    CHECK(neck_wire_decode(bytes,used,&decoded)==NECK_WIRE_OK);
    CHECK(decoded.body.session.capabilities==15);
    CHECK(decoded.body.session.c4_period_us==100000);
    memset(&source_event,0,sizeof(source_event));
    source_event.sequence=123; source_event.frame_time_us=5000000;
    source_event.time_us=5000010; source_event.invalidation_epoch=7;
    source_event.reference_id=3; source_event.events=NECK_EVENT_TRIGGER;
    source_event.axis=0; source_event.level=NECK_POSTURE_WARNING;
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_EVENT;
    m.header.boot_id=101; m.header.session_id=1; m.header.config_id=1;
    m.header.emit_time_us=5000020;
    neck_wire_producer_event(&m.body.event,&source_event,9);
    CHECK(m.body.event.event_id==9 && m.body.event.invalidation_epoch==7);
    CHECK(neck_wire_encode(&m,bytes,sizeof(bytes),&used)==NECK_WIRE_OK);
    CHECK(neck_wire_decode(bytes,used,&decoded)==NECK_WIRE_OK);
    CHECK(decoded.body.event.events==NECK_EVENT_TRIGGER);
    CHECK(neck_processing_init(&core,1,1,neck_simulator_mounts(),0,&p)==NECK_OK);
    CHECK(neck_processing_request_zero(&core,0,NECK_REFERENCE_SIMULATION)==NECK_OK);
    for(i=0;i<700;++i) {
        time=(uint64_t)i*10000U;
        neck_simulator_frame(&input,i,time);
        CHECK(neck_processing_process(&core,&input,time,&output)==NECK_OK);
        if(i%5) continue;
        neck_wire_producer_snapshot(&snapshot,&output,&input,1);
        if(snapshot.c4.valid_mask) ++c4_valid;
        if(snapshot.relative.pairs[0].valid_mask&NECK_WIRE_PAIR_RAW) ++relative_valid;
        if(snapshot.relative.pairs[0].valid_mask&NECK_WIRE_PAIR_CORRECTED) ++corrected;
        CHECK(roundtrip(NECK_WIRE_C4,&snapshot,time)==0);
        if(i%10==0) CHECK(roundtrip(NECK_WIRE_RELATIVE,&snapshot,time)==0);
        if(i%50==0) CHECK(roundtrip(NECK_WIRE_STATUS,&snapshot,time)==0);
    }
    CHECK(c4_valid>10); CHECK(relative_valid>10); CHECK(corrected>10);
    CHECK(snapshot.status.nodes[0].time_flags==3);
    CHECK(snapshot.status.nodes[1].status<=NECK_NODE_STALE);
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_C4;
    m.header.boot_id=101; m.header.session_id=1; m.header.config_id=1;
    m.header.emit_time_us=time; m.body.c4=snapshot.c4;
    CHECK(neck_wire_producer_expire(&m,snapshot.c4.valid_until_us+1,
        snapshot.c4.invalidation_epoch)==1);
    CHECK(m.body.c4.valid_mask==0 && m.body.c4.axes[0].valid==0);
    CHECK(m.body.c4.quality_flags&NECK_Q_TRANSPORT_STALE);
    CHECK(neck_wire_encode(&m,bytes,sizeof(bytes),&used)==NECK_WIRE_OK);
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_RELATIVE;
    m.header.boot_id=101; m.header.session_id=1; m.header.config_id=1;
    m.header.emit_time_us=time; m.body.relative=snapshot.relative;
    CHECK(neck_wire_producer_expire(&m,time+1,snapshot.relative.invalidation_epoch+1)==1);
    CHECK((m.body.relative.pairs[0].valid_mask&3)==0);
    CHECK(neck_wire_encode(&m,bytes,sizeof(bytes),&used)==NECK_WIRE_OK);
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_STATUS;
    m.header.boot_id=101; m.header.session_id=1; m.header.config_id=1;
    m.header.emit_time_us=time; m.body.status=snapshot.status;
    CHECK(neck_wire_producer_expire(&m,time+1,snapshot.status.invalidation_epoch+1)==1);
    CHECK(m.body.status.accel_usable_mask==0 && m.body.status.frame_accepted==0);
    CHECK(neck_wire_encode(&m,bytes,sizeof(bytes),&used)==NECK_WIRE_OK);
    time+=500000;
    CHECK(neck_processing_poll(&core,time,&output)==1);
    neck_wire_producer_snapshot(&snapshot,&output,NULL,0);
    CHECK(snapshot.c4.valid_mask==0);
    CHECK(snapshot.status.quality_flags&NECK_Q_TIMEOUT);
    CHECK(snapshot.status.nodes[0].accel_calibration==NECK_WIRE_UNKNOWN_CALIBRATION);
    CHECK(roundtrip(NECK_WIRE_C4,&snapshot,time)==0);
    CHECK(roundtrip(NECK_WIRE_RELATIVE,&snapshot,time)==0);
    CHECK(roundtrip(NECK_WIRE_STATUS,&snapshot,time)==0);
    printf("PASS D2 producer %u checks, C4=%u relative=%u corrected=%u\n",checks,c4_valid,relative_valid,corrected);
    return 0;
}
