/* Exercise production drivers/services with a register model and synthetic I2C. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensor_service.h"
#include "protocol_service.h"
#include <windows.h>
static uint32_t tick;
static int start_error, starts;
static unsigned selected, present_mask;
static uint8_t op_mode[3];
uint32_t xTaskGetTickCount(void) { return tick; }
int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { assert(h->Instance->PSC==71); ++starts; return start_error; }
AppStatus BspI2c_IsReady(BspI2c *b,uint8_t a) { (void)b; return a==0x70?APP_STATUS_OK:APP_STATUS_IO_ERROR; }
AppStatus BspI2c_MasterWrite(BspI2c *b,uint8_t a,const uint8_t *d,size_t n)
{ (void)b; assert(a==0x70 && n==1); selected=*d; return APP_STATUS_OK; }
static int index_of_node(void) { return selected==1?0:selected==2?1:2; }
AppStatus BspI2c_WriteRegister(BspI2c *b,uint8_t a,uint8_t reg,const uint8_t *d,size_t n)
{
    (void)b; assert(a==0x28 && n==1);
    if(!(present_mask & selected)) return APP_STATUS_IO_ERROR;
    if(reg==0x3d) op_mode[index_of_node()]=*d;
    return APP_STATUS_OK;
}
AppStatus BspI2c_ReadRegister(BspI2c *b,uint8_t a,uint8_t reg,uint8_t *out,size_t n)
{
    (void)b; assert(a==0x28 && selected && (selected & (selected-1))==0);
    if(!(present_mask & selected)) return APP_STATUS_IO_ERROR;
    memset(out,0,n);
    if(reg==0) *out=0xa0;
    if(reg==0x39) *out=5;
    if(reg==0x3d) *out=op_mode[index_of_node()];
    return APP_STATUS_OK;
}
static void check_commands(const wchar_t *bridge)
{
    HMODULE lib=LoadLibraryW(bridge);
    assert(lib);
    typedef int (__cdecl *Encoder)(uint32_t,uint32_t,uint32_t,uint64_t,uint32_t,uint32_t,const uint32_t*,uint8_t*,uint32_t);
    Encoder encode=(Encoder)(void*)GetProcAddress(lib,"neck_host_command");
    assert(encode);
    const AppCommandId expected[]={APP_COMMAND_NONE,APP_COMMAND_NONE,APP_COMMAND_CAPTURE_NEUTRAL,
        APP_COMMAND_NONE,APP_COMMAND_STOP_STREAM,APP_COMMAND_START_STREAM,APP_COMMAND_NONE,APP_COMMAND_NONE};
    for(unsigned op=1;op<=7;++op) {
        uint8_t wire[128]; uint32_t args[4]={0}; AppControlCommand cmd; size_t used=0;
        if(op==6) {args[0]=1; args[1]=750;}
        int length=encode(1,9,1,20000,7,op,args,wire,sizeof(wire));
        assert(length>0 && ProtocolService_ParseCommand(wire,length,&cmd,&used)==APP_STATUS_OK);
        assert(used==(size_t)length && cmd.opcode==op && cmd.id==expected[op]);
        assert(cmd.session==9 && cmd.result==(op>=6?6U:0U));
    }
    FreeLibrary(lib);
}
int wmain(int argc,wchar_t **argv)
{
    assert(argc==2); check_commands(argv[1]);
    FakeTimer regs={0}; TIM_HandleTypeDef timer={&regs}; BspTime time={0};
    assert(BspTime_Init(&time,&timer)==APP_STATUS_OK && starts==1 && BspTime_IsHighResolution(&time));
    regs.CNT=65535; assert(BspTime_NowUs64(&time)==65535);
    regs.CNT=0; regs.SR=1; assert(BspTime_NowUs64(&time)==65536);
    regs.SR=0; BspTime_OnOverflow(); regs.CNT=34464; assert(BspTime_NowUs64(&time)==100000);
    BspTime_OnOverflow(); BspTime_OnOverflow(); assert(BspTime_NowUs64(&time)==231072);
    start_error=1; tick=10; assert(BspTime_Init(&time,&timer)==APP_STATUS_OK && !BspTime_IsHighResolution(&time));
    assert(BspTime_NowUs64(&time)==10000);
    BspI2c bus={0}; SensorService svc; AppSampleFrame frame;
    present_mask=5; assert(SensorService_Init(&svc,&bus,&time)==APP_STATUS_OK);
    for(tick=20;tick<=4600;tick+=20) {
        if(tick==400) present_mask=7;
        if(tick==2000) present_mask=5;
        if(tick==2300) present_mask=7;
        SensorService_ReadCycle(&svc,&frame);
        if(tick>=200) {
            assert(frame.samples[0].flags & APP_SAMPLE_FLAG_VALID);
            assert(frame.samples[2].flags & APP_SAMPLE_FLAG_VALID);
        }
        if((tick>=200 && tick<=400) || (tick>=2000 && tick<=2300)) assert(!(frame.samples[1].flags&1));
        if(tick==1600 || tick>=3600) assert(frame.samples[1].flags&1);
    }
    puts("PASS PC to firmware opcodes 1..7; TIM2 normal/pending/wrap/multiwrap/fallback; C4 late arrival/disconnect/rejoin with healthy C0+C7");
    return 0;
}
