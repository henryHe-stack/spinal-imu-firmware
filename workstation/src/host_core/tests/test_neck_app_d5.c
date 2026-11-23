/* Executes production application orchestration with deterministic RTOS/HAL
 * substitutes. It does NOT simulate interrupt scheduling or physical timing. */
#include <stdlib.h>
#include "../Application/Src/neck_app.c"
struct TestQueue { unsigned depth,count; size_t size; unsigned char bytes[1024]; };
static struct TestQueue queues[5]; static unsigned queue_count;
static TickType_t tick; static unsigned notices,event_sends;
static uint8_t flash[2048],rx[NECK_WIRE_MAX_FRAME]; static size_t rx_used,rx_pos;
UART_HandleTypeDef huart2;
QueueHandle_t xQueueCreate(unsigned depth,size_t size) {
    struct TestQueue *q=&queues[queue_count++]; assert(queue_count<=5 && depth*size<=1024);
    q->depth=depth; q->size=size; return q;
}
BaseType_t xQueueSend(QueueHandle_t q,const void *p,TickType_t wait) {
    (void)wait; if(q->count==q->depth) return 0; memcpy(q->bytes+q->size*q->count++,p,q->size); return 1;
}
BaseType_t xQueueReceive(QueueHandle_t q,void *p,TickType_t wait) {
    (void)wait; if(!q->count) return 0; memcpy(p,q->bytes,q->size); --q->count;
    memmove(q->bytes,q->bytes+q->size,q->count*q->size); return 1;
}
BaseType_t xQueueOverwrite(QueueHandle_t q,const void *p) { assert(q->depth==1); q->count=0; return xQueueSend(q,p,0); }
BaseType_t xQueueReset(QueueHandle_t q) { q->count=0; return 1; }
unsigned uxQueueMessagesWaiting(QueueHandle_t q) { return q->count; }
unsigned uxQueueSpacesAvailable(QueueHandle_t q) { return q->depth-q->count; }
TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(TickType_t *p,TickType_t t) { *p+=t; tick=*p; }
void vTaskDelay(TickType_t t) { tick+=t; }
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t t) { (void)t; return 100; }
BaseType_t xTaskCreate(void (*f)(void *),const char *n,unsigned s,void *p,unsigned pri,TaskHandle_t *t) {
    (void)f; (void)n; (void)s; (void)p; (void)pri; if(t) *t=(void *)1; return 1;
}
void xTaskNotifyGive(TaskHandle_t t) { (void)t; }
uint32_t ulTaskNotifyTake(BaseType_t b,TickType_t t) {
    (void)b; (void)t;
    if(++notices==5) {
        assert(event_sends==2 && !event_pending && uxQueueMessagesWaiting(event_queue)==0);
        puts("PASS production app: UART parser / dedup / stop-set-save-restart / old identity / event retry+ACK with full reply queue / boot journal");
        puts("Host RTOS/HAL substitutes only; scheduling, interrupts and hardware timing NOT validated.");
        exit(0); /* The production task is intentionally infinite. End the test process. */
    }
    tick+=250; return 1;
}
size_t xPortGetFreeHeapSize(void) { return 1024; }
void Error_Handler(void) { abort(); }
static int read_flash(void *u,uint32_t off,uint8_t *b,size_t n) { (void)u; assert(off+n<=2048); memcpy(b,flash+off,n); return 1; }
static int write_flash(void *u,uint32_t off,const uint8_t *b,size_t n) {
    size_t i; (void)u; assert(off+n<=2048); for(i=0;i<n;++i) { assert((flash[off+i]&b[i])==b[i]); flash[off+i]&=b[i]; } return 1;
}
static int erase_flash(void *u,uint32_t page) { (void)u; assert(page<2); memset(flash+page*1024,255,1024); return 1; }
NeckSettingsIo neck_settings_stm32_io(void) { NeckSettingsIo io={NULL,read_flash,write_flash,erase_flash}; return io; }
int neck_uart_rx_start(void) { return 1; }
int neck_uart_rx_pop(uint8_t *b) { if(rx_pos==rx_used) return 0; *b=rx[rx_pos++]; return 1; }
static NeckWireMessage command(uint32_t id,uint32_t op,uint32_t a,uint32_t b) {
    NeckWireMessage m; memset(&m,0,sizeof(m)); m.header.type=NECK_WIRE_COMMAND;
    m.header.boot_id=wire_boot_id; m.header.session_id=current_session; m.header.config_id=current_config;
    m.header.emit_time_us=now_us(); m.body.control.request_id=id; m.body.control.opcode=op;
    m.body.control.args[0]=a; m.body.control.args[1]=b; return m;
}
static CommandReply submit(NeckWireMessage *m) {
    CommandReply reply; assert(neck_wire_encode(m,rx,sizeof(rx),&rx_used)==NECK_WIRE_OK); rx_pos=0;
    service_commands(now_us()); assert(xQueueReceive(reply_queue,&reply,0)); return reply;
}
int HAL_UART_Transmit(UART_HandleTypeDef *uart,uint8_t *b,uint16_t n,uint32_t timeout) {
    NeckWireMessage m; (void)uart; (void)timeout; assert(neck_wire_decode(b,n,&m)==NECK_WIRE_OK);
    if(m.header.type==NECK_WIRE_EVENT) {
        assert(m.body.event.event_id==1); ++event_sends;
        if(event_sends==2) { NeckWireMessage ack=command(1,NECK_CMD_EVENT_ACK,1,0);
            CommandReply occupied; memset(&occupied,0,sizeof(occupied)); assert(xQueueSend(reply_queue,&occupied,0));
            command_now=now_us(); received_command(&ack,NULL); assert(!event_pending);
            xQueueReset(reply_queue); }
    }
    return HAL_OK;
}
int main(void) {
    NeckWireMessage m; CommandReply reply; NeckSettingsStore reopen; NeckSettingsIo io=neck_settings_stm32_io();
    NeckAcquisitionFrame frame; NeckProcessingOutput output; NeckWireSnapshot old_snapshot;
    uint64_t time;
    memset(flash,255,sizeof(flash)); assert(neck_app_start()); assert(wire_boot_id==1 && current_session==1);
    m=command(1,NECK_CMD_ZERO,0,0); reply=submit(&m); assert(reply.body.result==NECK_CMD_OK);
    for(time=10000;time<=4000000;time+=10000) {
        neck_simulator_frame(&frame,time/10000,time);
        assert(neck_processing_process(&processing_context,&frame,time,&output)==NECK_OK);
    }
    tick=4000; assert(output.relative.reference_id!=0); publish_output(&output,1,&frame);
    assert(xQueueReceive(pose_queue,&old_snapshot,0) && old_snapshot.session_id==1);
    m=command(2,NECK_CMD_STOP,0,0); reply=submit(&m); assert(reply.body.result==0 && maintenance);
    reply=submit(&m); assert(reply.body.result==0 && acquisition_generation==2); /* duplicate */
    acquisition_ack=acquisition_generation; service_commands(now_us());
    assert(!maintenance && measurement_stopped && current_session==2);
    assert(processing_context.stats.relative_commits==0 && old_snapshot.session_id!=current_session);
    m=command(1,NECK_CMD_SET,1,5000); reply=submit(&m); assert(reply.body.result==0);
    acquisition_ack=acquisition_generation; service_commands(now_us());
    assert(current_session==3 && current_config==2 && active_settings.cutoff_millihz==5000);
    m=command(1,NECK_CMD_SAVE,0,0); assert(submit(&m).body.result==0);
    assert(neck_settings_open(&reopen,&io) && reopen.config==2 && reopen.settings.cutoff_millihz==5000);
    m=command(2,NECK_CMD_RESTART,0,0); assert(submit(&m).body.result==0);
    acquisition_ack=acquisition_generation; service_commands(now_us());
    assert(!maintenance && !measurement_stopped && current_session==4);
    m=command(1,NECK_CMD_SET,1,6000); assert(submit(&m).body.result==NECK_CMD_NOT_READY);
    /* Same old frame cannot be silently rebound to the new session. */
    assert(old_snapshot.session_id!=current_session && old_snapshot.config_id!=current_config);
    memset(&retained_event,0,sizeof(retained_event)); retained_event.events=NECK_EVENT_TRIGGER;
    retained_event.level=2; retained_event.previous_level=1; retained_event.reference_id=1;
    retained_event.source=NECK_SOURCE_SIMULATION; retained_event.invalidation_epoch=invalidation_epoch;
    assert(xQueueSend(event_queue,&retained_event,0));
    assert(neck_settings_reserve_boot(&reopen) && reopen.boot==2);
    telemetry_task(NULL); return 1;
}
