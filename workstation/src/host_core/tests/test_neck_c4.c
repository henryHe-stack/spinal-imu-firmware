#include "neck_c4_port.h"
#include "neck_wire_producer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const double identity[4]={1,0,0,0};
static NeckProcessingContext context;
static NeckProcessingParameters parameters;
static NeckProcessingOutput output;
static NeckAcquisitionFrame frame;
static NeckSensorReadTrace trace;
static NeckC4TimeEvidence evidence;
static uint64_t sequence;
static void setup(void)
{
    neck_processing_default_parameters(&parameters);
    assert(neck_processing_init_single_c4(&context,1,identity,0,&parameters)==NECK_OK);
    sequence=0;
}
static void sample(uint64_t now, double pitch)
{
    unsigned i;
    memset(&frame,0,sizeof(frame)); memset(&trace,0,sizeof(trace));
    frame.acceleration.config_id=frame.orientation.config_id=1;
    frame.acceleration.session_id=frame.orientation.session_id=1;
    frame.acceleration.sequence=frame.orientation.sequence=sequence++;
    frame.acceleration.frame_time_us=frame.orientation.frame_time_us=now;
    frame.acceleration.source=frame.orientation.source=NECK_SOURCE_HARDWARE;
    for(i=0;i<3;++i) {
        frame.acceleration.nodes[i].status=frame.orientation.nodes[i].status=NECK_NODE_MISSING;
        frame.acceleration.nodes[i].calibration_accel=NECK_CALIBRATION_UNKNOWN;
        frame.orientation.nodes[i].calibration_system=NECK_CALIBRATION_UNKNOWN;
    }
    frame.acceleration.nodes[1].status=frame.orientation.nodes[1].status=NECK_NODE_PARTIAL;
    frame.acceleration.nodes[1].accel_present=1;
    frame.acceleration.nodes[1].signal_valid_mask=3;
    frame.acceleration.nodes[1].calibration_accel=3;
    frame.acceleration.nodes[1].accel_m_s2[0]=-9.81*sin(pitch*0.017453292519943295);
    frame.acceleration.nodes[1].accel_m_s2[2]=9.81*cos(pitch*0.017453292519943295);
    frame.orientation.nodes[1].quat_present=1;
    frame.orientation.nodes[1].sensor_to_world_wxyz[0]=1;
    trace.selected_mask=trace.observed_mask=2;
    trace.result[1]=NECK_SENSOR_OK;
    trace.read_start_us[1]=now-1000; trace.read_end_us[1]=now-500;
    evidence.config_id=evidence.session_id=1; evidence.sequence=frame.acceleration.sequence;
    evidence.read_start_us=trace.read_start_us[1]; evidence.read_end_us=trace.read_end_us[1];
    evidence.sample_time_us=now-2000;
}
static void step(uint64_t now, double pitch)
{
    sample(now,pitch);
    assert(neck_c4_apply_time(&frame,&trace,&evidence));
    assert(neck_processing_process(&context,&frame,now,&output)==NECK_OK);
    assert(output.pose.quality.orientation_usable_mask==0);
    assert(output.relative.world_frame_id==0);
    assert(output.pose.nodes[0].valid_mask==0 && output.pose.nodes[2].valid_mask==0);
}
static void test_time_contract(void)
{
    double mount[4];
    sample(10000,0);
    assert(!neck_c4_board_mount(mount));
    assert(!neck_c4_board_time(&frame,&trace,&evidence));
    assert(neck_c4_apply_time(&frame,&trace,&evidence));
    assert(frame.acceleration.nodes[1].sample_time_known);
    assert(!frame.orientation.nodes[1].sample_time_known);
    ++evidence.sequence;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    assert(!frame.acceleration.nodes[1].sample_time_known);
    --evidence.sequence; ++evidence.session_id;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    --evidence.session_id; ++evidence.config_id;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    --evidence.config_id; ++evidence.read_end_us;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    --evidence.read_end_us; evidence.sample_time_us=evidence.read_start_us+1;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    evidence.sample_time_us=8000; trace.observed_mask=0;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    trace.observed_mask=2; frame.acceleration.source=NECK_SOURCE_SIMULATION;
    assert(!neck_c4_apply_time(&frame,&trace,&evidence));
    assert(!neck_c4_apply_time(NULL,&trace,&evidence));
    assert(!neck_c4_apply_time(&frame,NULL,&evidence));
    assert(!neck_c4_apply_time(&frame,&trace,NULL));
    puts("PASS timing evidence binding / default unavailable hooks");
}
static void test_zero_and_output(void)
{
    uint64_t now;
    NeckWireSnapshot snapshot;
    setup();
    assert(neck_processing_request_zero(&context,0,NECK_REFERENCE_EXTERNAL)==NECK_OK);
    for(now=10000;now<=4000000;now+=10000) step(now,0);
    assert(context.request_state==NECK_REQUEST_IDLE);
    assert(context.pipelines[1].zero_status==NECK_ZERO_SUCCEEDED);
    assert(!context.pipelines[0].reference_set && !context.pipelines[2].reference_set);
    assert(context.stats.node_commits==1 && context.stats.relative_commits==0);
    assert(context.stats.relative_aborts==0 && context.stats.relative_errors==0);
    assert(output.c4.filtered.valid_mask&NECK_VALID_FLEXION);
    assert(fabs(output.c4.filtered.flexion_deg)<1e-8);
    for(;now<=6000000;now+=10000) step(now,20);
    assert(fabs(output.c4.filtered.flexion_deg-20)<0.05);
    neck_wire_producer_snapshot(&snapshot,&output,&frame,1);
    assert(snapshot.c4.valid_mask&NECK_WIRE_CORRECTED_FLEX);
    assert(snapshot.c4.angles_mdeg[4]>=19950 && snapshot.c4.angles_mdeg[4]<=20050);
    assert(snapshot.status.accel_usable_mask==2 && snapshot.status.orientation_usable_mask==0);
    assert(neck_processing_request_zero(&context,now,NECK_REFERENCE_SIMULATION)==NECK_SIM_REFERENCE_ON_HARDWARE);
    assert(neck_processing_poll(&context,now+40000,&output));
    assert(!output.c4.filtered.valid_mask && output.pose.quality.accel_usable_mask==0);
    assert(neck_processing_reset_stream(&context,now+50000,&output)==NECK_OK);
    assert(context.single_c4 && !context.pipelines[1].reference_set);
    puts("PASS C4-only external zero / 20-degree tilt / D0 snapshot / silence / reset");
}
static void test_quality_and_commands(void)
{
    uint64_t now;
    setup();
    assert(neck_processing_request_zero(&context,0,NECK_REFERENCE_EXTERNAL)==NECK_OK);
    for(now=10000;now<=300000;now+=10000) {
        sample(now,0);
        assert(neck_processing_process(&context,&frame,now,&output)==NECK_OK);
        assert(!output.c4.raw.valid_mask);
        assert(context.request_state==NECK_REQUEST_WAIT_QUALITY);
        assert(!(output.pose.quality.node_flags[1]&NECK_QN_ACCEL_METADATA));
    }
    neck_processing_cancel_zero(&context);
    assert(context.pipelines[1].zero_status==NECK_ZERO_CANCELED);
    assert(context.pipelines[0].zero_status==NECK_ZERO_IDLE);
    assert(neck_processing_request_zero(&context,now,NECK_REFERENCE_EXTERNAL)==NECK_OK);
    for(;now<=600000;now+=10000) step(now,0);
    assert(context.request_state==NECK_REQUEST_COLLECTING);
    sample(now,0); assert(neck_c4_apply_time(&frame,&trace,&evidence));
    frame.acceleration.nodes[1].calibration_accel=0;
    assert(neck_processing_process(&context,&frame,now,&output)==NECK_OK);
    assert(!output.c4.raw.valid_mask && context.request_state==NECK_REQUEST_IDLE);
    assert(context.pipelines[1].zero_status==NECK_ZERO_FAILED_INPUT);
    now+=10000;
    for(;now<=900000;now+=10000) step(now,0);
    assert(output.c4.raw.valid_mask&NECK_VALID_PITCH_RAW);
    sample(now,0); assert(neck_c4_apply_time(&frame,&trace,&evidence));
    frame.acceleration.nodes[1].sample_time_us=now-12000; /* duplicate previous sample */
    assert(neck_processing_process(&context,&frame,now,&output)==NECK_OK);
    assert(!output.c4.raw.valid_mask);
    assert(neck_processing_request_zero(&context,now,NECK_REFERENCE_EXTERNAL)==NECK_OK);
    assert(neck_processing_poll(&context,now+parameters.pipeline.zero_timeout_us,&output));
    assert(context.pipelines[1].zero_status==NECK_ZERO_TIMED_OUT);
    assert(context.request_state==NECK_REQUEST_IDLE);
    puts("PASS unknown time / calibration / duplicate / cancel / timeout / recovery");
}
static void test_node_scope(void)
{
    uint64_t now;
    setup();
    for(now=10000;now<=200000;now+=10000) {
        sample(now,0); assert(neck_c4_apply_time(&frame,&trace,&evidence));
        frame.acceleration.nodes[0]=frame.acceleration.nodes[1];
        frame.acceleration.nodes[2]=frame.acceleration.nodes[1];
        frame.orientation.nodes[1].world_frame_id=1;
        frame.orientation.nodes[1].sample_time_known=1;
        frame.orientation.nodes[1].sample_time_us=now-2000;
        frame.orientation.nodes[1].calibration_system=3;
        assert(neck_processing_process(&context,&frame,now,&output)==NECK_OK);
        assert(!output.pose.nodes[0].valid_mask && !output.pose.nodes[2].valid_mask);
        assert(output.pose.quality.orientation_usable_mask==0);
    }
    assert(output.pose.quality.accel_usable_mask==2);
    puts("PASS C0/C7 excluded / no accidental common-world activation");
}
static void test_hardware_session(void)
{
    NeckWireMessage message, decoded;
    NeckWireReceiver receiver;
    uint8_t bytes[NECK_WIRE_MAX_FRAME];
    size_t length=0;
    memset(&message,0,sizeof(message));
    neck_processing_default_parameters(&parameters);
    message.header.type=NECK_WIRE_SESSION;
    message.header.boot_id=message.header.session_id=message.header.config_id=1;
    assert(neck_wire_producer_c4_session(&message.body.session,&parameters));
    assert(neck_wire_encode(&message,bytes,sizeof(bytes),&length)==NECK_WIRE_OK);
    assert(neck_wire_decode(bytes,length,&decoded)==NECK_WIRE_OK);
    assert(neck_wire_receiver_init(&receiver,500000,300000)==NECK_WIRE_OK);
    assert(neck_wire_receiver_accept(&receiver,&decoded,1000)==NECK_WIRE_RX_SESSION);
    assert(receiver.session.world_frame_id==0);
    assert(receiver.session.source==NECK_WIRE_HARDWARE);
    assert(receiver.session.mode==NECK_WIRE_SINGLE_C4 && receiver.session.node_mask==2);
    assert(receiver.session.capabilities==(NECK_WIRE_CAP_C4|NECK_WIRE_CAP_STATUS|NECK_WIRE_CAP_EVENTS));
    assert(receiver.session.reference_origin==NECK_REFERENCE_EXTERNAL);
    assert(receiver.session.relative_period_us==0);
    assert(!neck_wire_producer_c4_session(NULL,&parameters));
    assert(!neck_wire_producer_c4_session(&message.body.session,NULL));
    puts("PASS hardware C4 SESSION encode / decode / native receiver capability");
}
int main(void)
{
    test_time_contract(); test_zero_and_output(); test_quality_and_commands(); test_node_scope();
    test_hardware_session();
    puts("C4 measurement host tests passed (synthetic evidence, no board validation).");
    return 0;
}
