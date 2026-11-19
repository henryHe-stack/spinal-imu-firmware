#include "neck_control.h"
#include "neck_settings.h"
#include "neck_measurement_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint8_t flash[2048];
static int budget=-1,erase_fail;
static int io_read(void *u,uint32_t offset,uint8_t *p,size_t n)
{ (void)u; if(offset>2048 || n>2048-offset) return 0; memcpy(p,flash+offset,n); return 1; }
static int io_write(void *u,uint32_t offset,const uint8_t *p,size_t n)
{
    size_t i; (void)u;
    if(offset>2048 || n>2048-offset || (offset&1) || (n&1)) return 0;
    for(i=0;i<n;i+=2) {
        if(budget==0) return 0;
        if(budget>0) --budget;
        if((flash[offset+i]&p[i])!=p[i] || (flash[offset+i+1]&p[i+1])!=p[i+1]) return 0;
        flash[offset+i]&=p[i]; flash[offset+i+1]&=p[i+1];
    }
    return 1;
}
static int io_erase(void *u,uint32_t page)
{ (void)u; if(page>1) return 0; memset(flash+page*1024,255,erase_fail?(size_t)erase_fail:1024); return !erase_fail; }
static NeckSettingsIo io={NULL,io_read,io_write,io_erase};
static void test_journal(void)
{
    NeckSettingsStore store,reopen; NeckSettings setting; NeckProcessingParameters defaults; unsigned i; uint8_t saved[2048];
    memset(flash,255,sizeof(flash)); budget=-1;
    assert(neck_settings_open(&store,&io)); assert(store.boot==0);
    neck_processing_default_parameters(&defaults);
    assert(store.settings.cutoff_millihz==defaults.pipeline.cutoff_hz*1000);
    assert(store.settings.thresholds_mdeg[0]==defaults.posture.flexion_warning_deg*1000);
    assert(store.settings.thresholds_mdeg[1]==defaults.posture.flexion_danger_deg*1000);
    assert(store.settings.thresholds_mdeg[2]==defaults.posture.lateral_warning_deg*1000);
    assert(store.settings.thresholds_mdeg[3]==defaults.posture.lateral_danger_deg*1000);
    assert(!neck_settings_set(&store.settings,3,30000)); /* Equality violates core parameters. */
    assert(neck_settings_reserve_boot(&store)); assert(store.boot==1);
    setting=store.settings; assert(!neck_settings_set(&setting,1,1));
    assert(neck_settings_set(&setting,1,5000)); assert(neck_settings_save(&store,&setting,2));
    assert(neck_settings_open(&reopen,&io)); assert(reopen.settings.cutoff_millihz==5000 && reopen.config==2);
    for(i=0;i<80;++i) { assert(neck_settings_open(&store,&io)); assert(neck_settings_reserve_boot(&store)); assert(store.boot==i+2); }
    memcpy(saved,flash,sizeof(saved));
    for(i=0;i<=32;++i) {
        memcpy(flash,saved,sizeof(flash)); budget=(int)i;
        assert(neck_settings_open(&store,&io));
        if(i<32) assert(!neck_settings_reserve_boot(&store));
        else assert(neck_settings_reserve_boot(&store));
        budget=-1; assert(neck_settings_open(&reopen,&io));
        assert(reopen.boot==(i<32?81u:82u));
        assert(neck_settings_reserve_boot(&reopen)); assert(reopen.boot==(i<32?82u:83u));
    }
    memset(flash,255,sizeof(flash)); assert(neck_settings_open(&store,&io));
    for(i=0;i<16;++i) assert(neck_settings_reserve_boot(&store));
    erase_fail=513; assert(!neck_settings_reserve_boot(&store)); erase_fail=0;
    assert(neck_settings_open(&reopen,&io) && reopen.boot==16);
    assert(neck_settings_reserve_boot(&reopen) && reopen.boot==17);
    flash[reopen.latest_offset+20]^=1; assert(!neck_settings_open(&store,&io));
    memset(flash,255,sizeof(flash)); assert(neck_settings_open(&store,&io));
    for(i=0;i<32;++i) assert(neck_settings_reserve_boot(&store));
    memcpy(saved,flash,sizeof(saved));
    for(i=1;i<1024;++i) {
        memcpy(flash,saved,sizeof(flash)); assert(neck_settings_open(&store,&io));
        erase_fail=(int)i; assert(!neck_settings_reserve_boot(&store)); erase_fail=0;
        assert(neck_settings_open(&reopen,&io) && reopen.boot==32);
        assert(neck_settings_reserve_boot(&reopen) && reopen.boot==33);
        assert(neck_settings_open(&store,&io) && store.boot==33);
    }
    /* Every program interruption around intent, erase completion and record commit. */
    for(i=0;i<=34;++i) {
        memcpy(flash,saved,sizeof(flash)); assert(neck_settings_open(&store,&io)); budget=(int)i;
        assert(neck_settings_reserve_boot(&store)==(i==34)); budget=-1;
        assert(neck_settings_open(&reopen,&io) && reopen.boot==(i==34?33u:32u));
        assert(neck_settings_reserve_boot(&reopen));
    }
    /* Latest valid record need not occupy last slot: that slot may be torn. */
    memcpy(flash,saved,sizeof(flash)); flash[2046]=flash[2047]=255;
    assert(neck_settings_open(&store,&io) && store.boot==31);
    erase_fail=513; assert(!neck_settings_reserve_boot(&store)); erase_fail=0;
    assert(neck_settings_open(&reopen,&io) && reopen.boot==31);
    assert(neck_settings_reserve_boot(&reopen) && reopen.boot==32);
    memset(flash,255,sizeof(flash)); assert(neck_settings_open(&store,&io));
    store.boot=UINT32_MAX; assert(!neck_settings_reserve_boot(&store));
    puts("PASS two-page journal / 80 boots / all 32 torn-write boundaries / erase failure / CRC / exhaustion");
}
static unsigned executions,decoded_count;
static uint32_t handler(void *u,const NeckWireControl *q,uint32_t args[4])
{ (void)u; ++executions; args[1]=q->args[0]; return q->opcode<=8?NECK_CMD_OK:NECK_CMD_UNSUPPORTED; }
static void decoded(const NeckWireMessage *m,void *u)
{ (void)u; assert(m->header.type==NECK_WIRE_COMMAND); ++decoded_count; }
static void test_commands(void)
{
    NeckControl c; NeckWireMessage m,out; NeckWireControl reply; NeckWireParser parser;
    uint8_t bytes[NECK_WIRE_MAX_FRAME]; size_t n,i;
    memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_COMMAND;
    m.header.boot_id=1; m.header.session_id=2; m.header.config_id=3; m.header.emit_time_us=100;
    m.body.control.request_id=1; m.body.control.opcode=NECK_CMD_ZERO;
    neck_control_init(&c,1,2,3); executions=0;
    assert(neck_wire_encode(&m,bytes,sizeof(bytes),&n)==NECK_WIRE_OK && n==62);
    assert(neck_wire_decode(bytes,n,&out)==NECK_WIRE_OK);
    neck_wire_parser_init(&parser); decoded_count=0;
    for(i=0;i<n;++i) neck_wire_parser_feed(&parser,bytes+i,1,decoded,NULL);
    assert(decoded_count==1); bytes[40]^=1; neck_wire_parser_feed(&parser,bytes,n,decoded,NULL);
    assert(decoded_count==1 && parser.stats.crc_errors==1);
    assert(neck_control_dispatch(&c,&m,100,handler,NULL,&reply) && reply.result==0 && executions==1);
    assert(neck_control_dispatch(&c,&m,9999999,handler,NULL,&reply) && reply.result==0 && executions==1);
    ++m.body.control.args[0]; neck_control_dispatch(&c,&m,100,handler,NULL,&reply);
    assert(reply.result==NECK_CMD_CONFLICT && executions==1);
    m.body.control.request_id=2; neck_control_dispatch(&c,&m,100,handler,NULL,&reply);
    assert(reply.result==0 && executions==2);
    m.body.control.request_id=1; neck_control_dispatch(&c,&m,100,handler,NULL,&reply);
    assert(reply.result==NECK_CMD_REPLAY);
    m.body.control.request_id=3; neck_control_dispatch(&c,&m,2000101,handler,NULL,&reply);
    assert(reply.result==NECK_CMD_EXPIRED && executions==2);
    ++m.header.session_id; neck_control_dispatch(&c,&m,100,handler,NULL,&reply);
    assert(reply.result==NECK_CMD_IDENTITY); --m.header.session_id;
    m.body.control.opcode=NECK_CMD_QUERY; neck_control_dispatch(&c,&m,100,handler,NULL,&reply);
    assert(reply.result==0 && reply.args[0]==2);
    m.body.control.opcode=99; neck_control_dispatch(&c,&m,100,handler,NULL,&reply);
    assert(reply.result==NECK_CMD_UNSUPPORTED);
    puts("PASS command codec / fragmentation / CRC / duplicate / conflict / replay / identity / expiry / query");
}
static void test_three_registration(void)
{
    NeckMeasurementProfile p; NeckMeasurementTimes times; NeckSensorReadTrace trace;
    NeckAcquisitionFrame f; NeckProcessingParameters params; NeckProcessingContext c; NeckProcessingOutput o;
    uint64_t now; unsigned i;
    memset(&p,0,sizeof(p)); p.config_id=4; p.world_id=9; p.node_mask=7;
    for(i=0;i<3;++i) p.sensor_to_body[i][0]=p.local_to_common_world[i][0]=1;
    assert(!neck_board_three_profile(4,&p));
    assert(neck_measurement_profile_valid(&p,4)); assert(!neck_measurement_profile_valid(&p,5));
    neck_processing_default_parameters(&params);
    assert(neck_processing_init(&c,4,9,(const double (*)[4])p.sensor_to_body,0,&params)==NECK_OK);
    assert(neck_processing_request_zero(&c,0,NECK_REFERENCE_EXTERNAL)==NECK_OK);
    for(now=10000;now<=4000000;now+=10000) {
        memset(&f,0,sizeof(f)); memset(&times,0,sizeof(times)); memset(&trace,0,sizeof(trace));
        f.acceleration.config_id=f.orientation.config_id=times.config_id=4;
        f.acceleration.session_id=f.orientation.session_id=times.session_id=2;
        f.acceleration.sequence=f.orientation.sequence=times.sequence=now/10000;
        f.acceleration.frame_time_us=f.orientation.frame_time_us=now;
        f.acceleration.source=f.orientation.source=NECK_SOURCE_HARDWARE;
        trace.selected_mask=trace.observed_mask=times.accel_mask=times.quat_mask=7;
        for(i=0;i<3;++i) {
            f.acceleration.nodes[i].status=f.orientation.nodes[i].status=NECK_NODE_PARTIAL;
            f.acceleration.nodes[i].signal_valid_mask=3; f.acceleration.nodes[i].accel_present=1;
            f.acceleration.nodes[i].calibration_accel=f.orientation.nodes[i].calibration_system=3;
            f.acceleration.nodes[i].accel_m_s2[2]=9.81;
            f.orientation.nodes[i].quat_present=1; f.orientation.nodes[i].sensor_to_world_wxyz[0]=1;
            times.read_start_us[i]=trace.read_start_us[i]=now-500;
            times.read_end_us[i]=trace.read_end_us[i]=now-100;
            times.accel_time_us[i]=times.quat_time_us[i]=now-3000+i*1000;
        }
        assert(!neck_board_three_times(&f,&trace,&times));
        assert(neck_measurement_apply(&p,&f,&trace,&times)==7);
        assert(neck_processing_process(&c,&f,now,&o)==NECK_OK);
    }
    assert(c.stats.relative_commits==1 && o.relative.reference_id!=0);
    assert(o.pose.quality.accel_usable_mask==7 && o.pose.quality.orientation_usable_mask==7);
    for(i=0;i<4;++i) assert(o.relative.pairs[i].valid_mask&NECK_REL_CORRECTED_EULER);
    ++times.sequence; assert(neck_measurement_apply(&p,&f,&trace,&times)==0);
    for(i=0;i<3;++i) assert(!f.acceleration.nodes[i].sample_time_known && !f.orientation.nodes[i].world_frame_id);
    --times.sequence; ++times.read_start_us[1]; assert(neck_measurement_apply(&p,&f,&trace,&times)==5);
    assert(!f.orientation.nodes[1].sample_time_known);
    --times.read_start_us[1];
    p.local_to_common_world[0][0]=p.local_to_common_world[0][3]=sqrt(0.5);
    f.orientation.nodes[0].sensor_to_world_wxyz[0]=sqrt(0.5);
    f.orientation.nodes[0].sensor_to_world_wxyz[3]=-sqrt(0.5);
    assert(neck_measurement_apply(&p,&f,&trace,&times)==7);
    assert(fabs(f.orientation.nodes[0].sensor_to_world_wxyz[0]-1)<1e-12);
    assert(fabs(f.orientation.nodes[0].sensor_to_world_wxyz[3])<1e-12);
    p.local_to_common_world[1][0]=NAN; assert(!neck_measurement_profile_valid(&p,4));
    puts("PASS three-node external zero / common-world transform / per-node timing / default unavailable / NaN");
}
int main(void)
{ test_journal(); test_commands(); test_three_registration(); puts("D5 portable software tests PASS; no physical flash/sensors/serial tested."); return 0; }
