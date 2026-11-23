#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bno055.h"
#include "bsp_uart.h"
#include "protocol_service.h"

/* Assertions remain active for Release host tests too. All hardware-facing
 * functions below are mocks; production drivers above are linked unchanged. */
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
static uint32_t tick;
uint32_t mock_primask;
static uint8_t registers[256], selected;
static unsigned transfers, tx_calls, rx_calls;
static AppStatus i2c_result=APP_STATUS_OK;
static HAL_StatusTypeDef uart_result=HAL_OK;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick+=ms; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t) { (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h; (void)p; CHECK(n==128); rx_calls++; return uart_result; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n)
{ (void)h; (void)p; CHECK(n>0); tx_calls++; return uart_result; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n,uint32_t ms)
{ (void)h; (void)p; (void)n; (void)ms; return uart_result; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
AppStatus BspI2c_IsReady(BspI2c *b,uint8_t a) { (void)b; CHECK(a==0x70); return i2c_result; }
AppStatus BspI2c_MasterWrite(BspI2c *b,uint8_t a,const uint8_t *p,size_t n)
{ (void)b; CHECK(a==0x70 && n==1); transfers++; if(i2c_result==APP_STATUS_OK) selected=*p; return i2c_result; }
AppStatus BspI2c_ReadRegister(BspI2c *b,uint8_t a,uint8_t r,uint8_t *p,size_t n)
{ (void)b; CHECK(a==0x28 && (size_t)r+n<=256); transfers++; if(i2c_result==APP_STATUS_OK) memcpy(p,registers+r,n); return i2c_result; }
AppStatus BspI2c_WriteRegister(BspI2c *b,uint8_t a,uint8_t r,const uint8_t *p,size_t n)
{ (void)b; CHECK(a==0x28 && (size_t)r+n<=256); transfers++; if(i2c_result==APP_STATUS_OK) memcpy(registers+r,p,n); return i2c_result; }
static void Put16(uint8_t *p,int value) { p[0]=(uint8_t)value; p[1]=(uint8_t)((unsigned)value>>8); }
static void Command(uint8_t *p,uint8_t id,uint32_t value)
{
    const uint8_t header[]={0xA5,0x5A,1,16,9,0,0x78,0x56,0x34,0x12};
    memcpy(p,header,sizeof(header)); p[10]=id;
    for(unsigned i=0;i<4;++i) p[11+i]=(uint8_t)(value>>(8*i));
    Put16(p+15,ProtocolService_Crc16(p+2,13));
}
static void TestProtocol(void)
{
    CHECK(ProtocolService_Crc16((const uint8_t *)"123456789",9)==0x29B1);
    uint8_t bytes[17]; Command(bytes,5,20);
    AppControlCommand cmd; size_t used;
    for(size_t i=0;i<17;++i) CHECK(ProtocolService_ParseCommand(bytes,i,&cmd,&used)==APP_STATUS_BUSY && used==0);
    CHECK(ProtocolService_ParseCommand(bytes,17,&cmd,&used)==APP_STATUS_OK && used==17);
    CHECK(cmd.request_id==0x12345678 && cmd.id==APP_COMMAND_SET_SAMPLE_PERIOD_MS && cmd.value==20);
    ProtocolParser parser={0};
    CHECK(!ProtocolService_FeedCommand(&parser,0,&cmd));
    CHECK(!ProtocolService_FeedCommand(&parser,0xA5,&cmd));
    bytes[16]^=1;
    for(unsigned i=0;i<17;++i) CHECK(!ProtocolService_FeedCommand(&parser,bytes[i],&cmd));
    CHECK(parser.errors>0); bytes[16]^=1;
    for(unsigned repeat=0;repeat<2;++repeat)
        for(unsigned i=0;i<17;++i) CHECK(ProtocolService_FeedCommand(&parser,bytes[i],&cmd)==(i==16));
    /* A truncated command followed by a new command must resynchronize. */
    for(unsigned i=0;i<9;++i) CHECK(!ProtocolService_FeedCommand(&parser,bytes[i],&cmd));
    unsigned accepted=0;
    for(unsigned i=0;i<17;++i) accepted+=ProtocolService_FeedCommand(&parser,bytes[i],&cmd);
    CHECK(accepted==1);
    AppSampleFrame frame={0}; frame.sample_count=4; uint8_t out[192]; size_t length=7;
    CHECK(ProtocolService_EncodeSampleFrame(&frame,out,sizeof(out),&length)==APP_STATUS_INVALID_ARGUMENT);
    frame.sample_count=3;
    CHECK(ProtocolService_EncodeSampleFrame(&frame,out,132,&length)==APP_STATUS_BUFFER_TOO_SMALL && length==0);
    CHECK(ProtocolService_EncodeSampleFrame(&frame,out,sizeof(out),&length)==APP_STATUS_OK && length==133);
    CHECK(ProtocolService_EncodeSampleFrame(NULL,out,sizeof(out),&length)==APP_STATUS_INVALID_ARGUMENT);
}
static void TestBnoAndMux(void)
{
    BspI2c bus={0}; Tca9548a mux; Bno055 sensor; AppImuSample sample;
    TIM_HandleTypeDef timer={0}; timer.Instance=TIM2; timer.Init.Prescaler=71; timer.Init.Period=65535;
    BspTime clock; CHECK(BspTime_Init(&clock,&timer)==APP_STATUS_OK);
    CHECK(Tca9548a_Init(&mux,&bus,0x70)==APP_STATUS_OK && selected==0);
    CHECK(Tca9548a_SelectChannel(&mux,8)==APP_STATUS_INVALID_ARGUMENT);
    CHECK(Tca9548a_SelectChannel(&mux,2)==APP_STATUS_OK && selected==4);
    unsigned before=transfers; CHECK(Tca9548a_SelectChannel(&mux,2)==APP_STATUS_OK && transfers==before);
    CHECK(Bno055_InitObject(&sensor,&bus,&clock,&mux,0x28,2,APP_IMU_NODE_C7)==APP_STATUS_OK);
    memset(registers,0,sizeof(registers)); registers[0]=0xA0;
    CHECK(Bno055_Configure(&sensor)==APP_STATUS_OK);
    before=transfers; CHECK(Bno055_Service(&sensor)==APP_STATUS_BUSY && transfers==before);
    tick=700; CHECK(Bno055_Service(&sensor)==APP_STATUS_BUSY && sensor.present);
    tick+=20; CHECK(Bno055_Service(&sensor)==APP_STATUS_BUSY && registers[0x3D]==0);
    before=transfers; tick+=24; CHECK(Bno055_Service(&sensor)==APP_STATUS_BUSY && transfers==before);
    for(unsigned i=0;i<100 && !sensor.configured;++i) { tick+=20; Bno055_Service(&sensor); }
    CHECK(sensor.configured && registers[0x3D]==12 && registers[0x3B]==0 && registers[0x3F]==0x80);
    CHECK(registers[0x41]==0x24 && registers[0x42]==0);
    Put16(registers+0x08,-123); Put16(registers+0x0A,981); Put16(registers+0x14,-32);
    Put16(registers+0x20,16384); registers[0x34]=0xFE; registers[0x35]=0xFF; registers[0x39]=5;
    timer.counter=123;
    CHECK(Bno055_ReadSample(&sensor,&sample)==APP_STATUS_OK);
    CHECK(sample.node_id==2 && sample.timestamp_mid_us==123 && sample.flags==APP_SAMPLE_FLAG_VALID);
    CHECK(sample.quaternion.w==16384 && sample.acceleration.x==-123 && sample.acceleration.y==981);
    CHECK(sample.gyroscope.x==-32 && sample.temperature_c==-2);
    registers[0x35]=0;
    CHECK(Bno055_ReadSample(&sensor,&sample)==APP_STATUS_OK && (sample.flags & APP_SAMPLE_FLAG_NOT_CALIBRATED));
    CHECK(Bno055_CaptureCalibration(&sensor)==APP_STATUS_BUSY);
    registers[0x35]=0xFF;
    for(unsigned i=0;i<22;++i) registers[0x55+i]=(uint8_t)(i+1);
    CHECK(Bno055_CaptureCalibration(&sensor)==APP_STATUS_OK && sensor.calibration_available);
    CHECK(sensor.calibration[21]==22 && registers[0x3D]==12);
    registers[0x3A]=1;
    for(unsigned i=0;i<3;++i) CHECK(Bno055_ReadSample(&sensor,&sample)==APP_STATUS_IO_ERROR && !(sample.flags & APP_SAMPLE_FLAG_VALID));
    CHECK(!sensor.configured && sensor.init_step==0);
    i2c_result=APP_STATUS_IO_ERROR; tick=sensor.due_ms;
    CHECK(Bno055_Service(&sensor)==APP_STATUS_IO_ERROR && !sensor.present);
    i2c_result=APP_STATUS_OK; tick=sensor.due_ms; registers[0x3A]=0;
    for(unsigned i=0;i<100 && !sensor.configured;++i) { Bno055_Service(&sensor); tick+=20; }
    CHECK(sensor.configured && sensor.calibration_available);
    CHECK(registers[0x55]==1 && registers[0x6A]==22);
}
static void MockRxEvent(UART_HandleTypeDef *uart, uint16_t reported, uint16_t actual)
{
    uart->hdmarx->remaining=APP_UART_RX_DMA_SIZE-actual;
    HAL_UARTEx_RxEventCallback(uart,reported);
}
static void TestUart(void)
{
    static BspUart a,b; static UART_HandleTypeDef ha,hb;
    static DMA_HandleTypeDef dma_a,dma_b;
    ha.hdmarx=ha.hdmatx=&dma_a; hb.hdmarx=hb.hdmatx=&dma_b;
    CHECK(BspUart_Init(&a,&ha)==APP_STATUS_OK && BspUart_Init(&b,&hb)==APP_STATUS_OK);
    for(unsigned i=0;i<128;++i) a.rx_dma[i]=(uint8_t)i;
    MockRxEvent(&ha,64,64); MockRxEvent(&ha,64,64);
    MockRxEvent(&ha,128,128); MockRxEvent(&ha,128,0);
    uint8_t out[512]; CHECK(BspUart_ReadAvailable(&a,out,sizeof(out))==128);
    for(unsigned i=0;i<128;++i) CHECK(out[i]==i);
    CHECK(BspUart_ReadAvailable(&b,out,sizeof(out))==0);
    /* A pending DMA half-transfer IRQ can follow an IDLE ISR that read past it. */
    MockRxEvent(&ha,66,66);
    MockRxEvent(&ha,64,66);
    CHECK(BspUart_ReadAvailable(&a,out,sizeof(out))==66);
    for(unsigned i=0;i<66;++i) CHECK(out[i]==i);
    MockRxEvent(&ha,128,128);
    CHECK(BspUart_ReadAvailable(&a,out,sizeof(out))==62 && out[0]==66 && out[61]==127);
    /* Likewise a pending DMA transfer-complete IRQ can follow a wrap. */
    MockRxEvent(&ha,64,64);
    MockRxEvent(&ha,5,5);
    MockRxEvent(&ha,128,5);
    CHECK(BspUart_ReadAvailable(&a,out,sizeof(out))==133 && out[127]==127 && out[132]==4);
    MockRxEvent(&ha,128,128);
    CHECK(BspUart_ReadAvailable(&a,out,sizeof(out))==123);
    MockRxEvent(&ha,5,5); CHECK(BspUart_ReadAvailable(&a,out,3)==3 && out[2]==2);
    CHECK(BspUart_ReadAvailable(&a,out,3)==2 && out[1]==4);
    for(unsigned i=0;i<6;++i) { MockRxEvent(&ha,64,64); MockRxEvent(&ha,128,128); }
    CHECK(a.rx_overflows>0 && BspUart_ReadAvailable(&a,out,sizeof(out))==511);
    unsigned before=rx_calls; HAL_UART_ErrorCallback(&ha); CHECK(a.rx_restart);
    BspUart_Service(&a); CHECK(!a.rx_restart && rx_calls==before+1 && a.rx_head==a.rx_tail);
    uint8_t input[]={1,2,3}; CHECK(BspUart_SendDma(&a,input,3)==APP_STATUS_OK);
    input[0]=99; CHECK(a.tx_data[0]==1 && a.tx_busy);
    CHECK(BspUart_SendDma(&a,input,3)==APP_STATUS_BUSY);
    CHECK(BspUart_SendDma(&b,input,3)==APP_STATUS_OK);
    HAL_UART_TxCpltCallback(&ha); CHECK(!a.tx_busy && b.tx_busy);
    CHECK(BspUart_SendDma(&a,input,3)==APP_STATUS_OK); tick+=101;
    before=tx_calls; CHECK(BspUart_SendDma(&a,input,3)==APP_STATUS_OK && tx_calls==before+1 && a.errors>=2);
    HAL_UART_TxCpltCallback(&ha); uart_result=HAL_ERROR;
    CHECK(BspUart_SendDma(&a,input,3)==APP_STATUS_IO_ERROR && !a.tx_busy); uart_result=HAL_OK;
}
static void TestTime(void)
{
    TIM_HandleTypeDef timer={0}; timer.Instance=TIM2; BspTime time;
    CHECK(BspTime_Init(&time,&timer)==APP_STATUS_INVALID_ARGUMENT);
    timer.Init.Prescaler=71; timer.Init.Period=65535;
    CHECK(BspTime_Init(&time,&timer)==APP_STATUS_OK && BspTime_IsHighResolution(&time));
    timer.counter=65535; CHECK(BspTime_NowUs(&time)==65535 && mock_primask==0);
    timer.counter=3; timer.flags=TIM_FLAG_UPDATE;
    CHECK(BspTime_NowUs(&time)==65539); // Pending interrupt, not yet dispatched.
    timer.flags=0; BspTime_OnOverflow(&timer); CHECK(BspTime_NowUs(&time)==65539);
    mock_primask=1; CHECK(BspTime_NowUs(&time)==65539 && mock_primask==1); mock_primask=0;
    time.overflow_us=0xFFFF0000; timer.counter=0; BspTime_OnOverflow(&timer);
    CHECK(BspTime_NowUs(&time)==0); // uint32_t wrap is defined unsigned arithmetic.
}
static void PrintHex(const uint8_t *p,size_t n) { for(size_t i=0;i<n;++i) printf("%02x",p[i]); }
static void Fixtures(void)
{
    AppSampleFrame frame={0}; frame.sequence=42; frame.sample_count=3;
    for(unsigned i=0;i<3;++i) {
        AppImuSample *s=&frame.samples[i]; s->node_id=(uint8_t)i; s->tca_channel=(uint8_t)i;
        s->flags=APP_SAMPLE_FLAG_VALID; s->timestamp_start_us=1000000+i*5000;
        s->timestamp_end_us=s->timestamp_start_us+4500; s->timestamp_mid_us=s->timestamp_start_us+2250;
        s->quaternion.w=16384; s->acceleration.x=-123; s->acceleration.y=981;
        s->gyroscope.z=-32; s->temperature_c=-2; s->calibration_status=0xFF; s->system_status=5;
    }
    uint8_t bytes[192]; size_t n;
    CHECK(ProtocolService_EncodeSampleFrame(&frame,bytes,sizeof(bytes),&n)==APP_STATUS_OK);
    printf("{\"sample\":\""); PrintHex(bytes,n);
    Command(bytes,5,20); printf("\",\"command\":\""); PrintHex(bytes,17);
    AppCommandReply reply={0}; reply.request_id=0x12345678; reply.command=5; reply.value=20;
    CHECK(ProtocolService_EncodeReply(&reply,bytes,sizeof(bytes),&n)==APP_STATUS_OK);
    printf("\",\"reply\":\""); PrintHex(bytes,n); printf("\"}\n");
}
int main(int argc,char **argv)
{
    if(argc==2 && strcmp(argv[1],"--fixtures")==0) { Fixtures(); return 0; }
    TestProtocol(); TestBnoAndMux(); TestUart(); TestTime();
    puts("PASS: production protocol, BNO055/TCA state/recovery, dual UART ring/TX, timer wrap (mock HAL)");
    return 0;
}
