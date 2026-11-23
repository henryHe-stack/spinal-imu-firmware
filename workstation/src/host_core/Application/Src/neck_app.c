#include "neck_app.h"
#include "neck_simulator.h"
#include "neck_clock.h"
#include "neck_wire_producer.h"
#include "neck_control.h"
#include "neck_settings_stm32.h"
#include "neck_uart_rx.h"
#include "neck_measurement_port.h"
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_C4
#include "neck_c4_port.h"
volatile uint32_t g_neck_c4_mount_ready, g_neck_c4_time_rejected;
static uint8_t c4_mount_ready; /* Startup-owned, not a debugger bypass. */
#endif
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
#include "neck_sensor_bus_stm32.h"
#endif
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile NeckFrame g_neck_latest_input;
volatile NeckPoseBatch g_neck_latest_pose;
#if NECK_FULL_DEBUG_SNAPSHOTS
volatile NeckPipelineOutput g_neck_latest_pipeline;
volatile NeckPostureOutput g_neck_latest_posture;
volatile NeckOrientationFrame g_neck_latest_orientation;
volatile NeckRelativeOutput g_neck_latest_relative;
#endif
volatile NeckAppStats g_neck_stats;
volatile uint32_t g_neck_zero_request;
volatile uint32_t g_neck_simulation_pause;
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
volatile NeckSensorReadTrace g_neck_latest_sensor_trace;
volatile uint32_t g_neck_hw_reprobe_request;
#endif
static QueueHandle_t frame_queue, pose_queue, command_queue, event_queue;
static TaskHandle_t telemetry_handle;
static NeckProcessingContext processing_context;
static NeckTickClock wall_clock;
static NeckWireSession wire_session;
static uint32_t wire_boot_id;
static volatile uint32_t invalidation_epoch;
static NeckSettingsStore settings_store;
static NeckSettings active_settings;
static NeckControl control;
static NeckWireParser command_parser;
static volatile uint32_t current_session=1,current_config=1;
static volatile uint32_t acquisition_generation=1,acquisition_ack;
static volatile uint8_t measurement_stopped,maintenance;
static uint8_t restart_requested,settings_changed;
static uint64_t command_now;
typedef struct { uint32_t boot,session,config; NeckWireControl body; } CommandReply;
static QueueHandle_t reply_queue;
static NeckAlarmEvent retained_event;
static uint64_t retained_event_id,event_last_send;
static uint8_t event_pending;
static uint32_t event_session,event_config;
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_THREE
static NeckMeasurementProfile three_profile;
static uint8_t three_profile_ready;
#endif
enum { COMMAND_ZERO = 1U, COMMAND_CANCEL = 2U };

static int queue_command(uint8_t command)
{
    if (command_queue == NULL) return 0;
    return xQueueSend(command_queue, &command, 0U) == pdPASS;
}
int neck_app_request_zero(void) { return queue_command(COMMAND_ZERO); }
int neck_app_cancel_zero(void) { return queue_command(COMMAND_CANCEL); }

/* Shared monotonic clock; task callers serialize through the RTOS critical
 * section. Processing context itself has ONLY the algorithm-task owner. */
static uint64_t now_us(void)
{
    uint64_t time;
    NeckResult result;
    taskENTER_CRITICAL();
    result = neck_clock_update(&wall_clock, (uint32_t)xTaskGetTickCount(), &time);
    taskEXIT_CRITICAL();
    if (result != NECK_OK) { g_neck_stats.fatal_error = 0xE003U; Error_Handler(); }
    return time;
}
static void apply_command(uint32_t command, uint64_t now)
{
    if (measurement_stopped || maintenance) { ++processing_context.stats.command_errors; return; }
    if (command == COMMAND_CANCEL) neck_processing_cancel_zero(&processing_context);
    else if (command == COMMAND_ZERO)
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
        (void)neck_processing_request_zero(&processing_context, now, NECK_REFERENCE_EXTERNAL);
#else
        (void)neck_processing_request_zero(&processing_context, now, NECK_REFERENCE_SIMULATION);
#endif
    else ++processing_context.stats.command_errors;
}

static int initialize_processing(uint64_t now)
{
    static const double identity[3][4]={{1,0,0,0},{1,0,0,0},{1,0,0,0}};
    NeckProcessingParameters p;
    neck_processing_default_parameters(&p);
    p.pipeline.cutoff_hz=active_settings.cutoff_millihz/1000.0;
    p.posture.continuous_duration_us=(uint64_t)active_settings.hold_ms*1000;
    p.posture.flexion_warning_deg=active_settings.thresholds_mdeg[0]/1000.0;
    p.posture.flexion_danger_deg=active_settings.thresholds_mdeg[1]/1000.0;
    p.posture.lateral_warning_deg=active_settings.thresholds_mdeg[2]/1000.0;
    p.posture.lateral_danger_deg=active_settings.thresholds_mdeg[3]/1000.0;
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_C4
    {
        double mount[4]={1,0,0,0};
        c4_mount_ready=(uint8_t)(neck_c4_board_mount(mount)==1);
        if(!c4_mount_ready) memcpy(mount,identity[1],sizeof(mount));
        if(neck_processing_init_single_c4(&processing_context,current_config,mount,now,&p)!=NECK_OK) return 0;
        g_neck_c4_mount_ready=c4_mount_ready;
        if(!neck_wire_producer_c4_session(&wire_session,&p)) return 0;
    }
#elif NECK_INPUT_MODE == NECK_INPUT_HARDWARE_THREE
    memset(&three_profile,0,sizeof(three_profile));
    three_profile_ready=(uint8_t)(neck_board_three_profile(current_config,&three_profile)==1 &&
        neck_measurement_profile_valid(&three_profile,current_config));
    if(neck_processing_init(&processing_context,current_config,three_profile_ready?three_profile.world_id:1,
        three_profile_ready?(const double (*)[4])three_profile.sensor_to_body:identity,now,&p)!=NECK_OK) return 0;
    neck_wire_producer_session(&wire_session,&p);
    wire_session.source=NECK_WIRE_HARDWARE; wire_session.reference_origin=NECK_REFERENCE_EXTERNAL;
    wire_session.world_frame_id=three_profile_ready?three_profile.world_id:0;
    if(!three_profile_ready) { wire_session.capabilities&=~NECK_WIRE_CAP_RELATIVE; wire_session.relative_period_us=0; }
#elif NECK_INPUT_MODE == NECK_INPUT_HARDWARE_DIAGNOSTIC
    if(neck_processing_init(&processing_context,current_config,1,identity,now,&p)!=NECK_OK) return 0;
    if(!neck_wire_producer_d3_diagnostic_session(&wire_session,&p,NECK_HW_NODE_MASK)) return 0;
#else
    (void)identity;
    if(neck_processing_init(&processing_context,current_config,1,neck_simulator_mounts(),now,&p)!=NECK_OK) return 0;
    neck_wire_producer_session(&wire_session,&p);
#endif
    processing_context.invalidation_epoch=invalidation_epoch+1;
    invalidation_epoch=processing_context.invalidation_epoch;
    wire_session.firmware_id=0xd5000000u|NECK_INPUT_MODE;
    wire_session.capabilities|=NECK_WIRE_CAP_CONTROL;
    if(wire_session.capabilities&NECK_WIRE_CAP_EVENTS) wire_session.capabilities|=NECK_WIRE_CAP_EVENT_ACK;
    return 1;
}
static uint32_t request_transition(uint8_t stopped)
{
    if(maintenance || current_session==UINT32_MAX || acquisition_generation==UINT32_MAX)
        return NECK_CMD_NOT_READY;
    measurement_stopped=1; restart_requested=(uint8_t)!stopped;
    maintenance=1; ++acquisition_generation; return NECK_CMD_OK;
}
static uint32_t execute_remote(void *unused,const NeckWireControl *q,uint32_t args[4])
{
    uint32_t result=NECK_CMD_OK; (void)unused;
    args[2]=current_config; args[3]=current_session;
    if(q->opcode==NECK_CMD_QUERY) {
        uint32_t key=q->args[0];
        if(key==0) args[1]=maintenance?2u:(measurement_stopped?1u:0u);
        else if(key==1) args[1]=active_settings.cutoff_millihz;
        else if(key==2) args[1]=active_settings.hold_ms;
        else if(key<=6) args[1]=active_settings.thresholds_mdeg[key-3];
        else return NECK_CMD_ARGUMENT;
        return NECK_CMD_OK;
    }
    if(q->opcode==NECK_CMD_EVENT_ACK) {
        uint64_t id=(uint64_t)q->args[0]|((uint64_t)q->args[1]<<32);
        taskENTER_CRITICAL();
        if(event_pending && id==retained_event_id && event_session==current_session && event_config==current_config)
            event_pending=0;
        taskEXIT_CRITICAL(); return NECK_CMD_OK;
    }
    if(maintenance) return NECK_CMD_NOT_READY;
    switch(q->opcode) {
    case NECK_CMD_ZERO:
        if(measurement_stopped) return NECK_CMD_NOT_READY;
        return neck_processing_request_zero(&processing_context,command_now,
            NECK_INPUT_MODE==NECK_INPUT_SIMULATION?NECK_REFERENCE_SIMULATION:NECK_REFERENCE_EXTERNAL)==NECK_OK?
            NECK_CMD_OK:NECK_CMD_NOT_READY;
    case NECK_CMD_CANCEL: neck_processing_cancel_zero(&processing_context); return NECK_CMD_OK;
    case NECK_CMD_STOP: case NECK_CMD_RESTART:
        result=request_transition((uint8_t)(q->opcode==NECK_CMD_STOP)); break;
    case NECK_CMD_SET: {
        NeckSettings next=active_settings;
        if(!measurement_stopped || current_config==UINT32_MAX) return NECK_CMD_NOT_READY;
        if(!neck_settings_set(&next,q->args[0],q->args[1])) return NECK_CMD_ARGUMENT;
        result=request_transition(1);
        if(result==NECK_CMD_OK) { active_settings=next; settings_changed=1; args[2]=current_config+1; }
        break;
    }
    case NECK_CMD_SAVE:
        if(!measurement_stopped) return NECK_CMD_NOT_READY;
        return neck_settings_save(&settings_store,&active_settings,current_config)?NECK_CMD_OK:NECK_CMD_STORAGE;
    default: return NECK_CMD_UNSUPPORTED;
    }
    if(result==NECK_CMD_OK) args[3]=current_session+1;
    return result;
}
static void received_command(const NeckWireMessage *m,void *unused)
{
    CommandReply reply; (void)unused;
    if(m->header.type!=NECK_WIRE_COMMAND ||
        (m->body.control.opcode!=NECK_CMD_EVENT_ACK && uxQueueSpacesAvailable(reply_queue)==0)) return;
    reply.boot=control.boot; reply.session=control.session; reply.config=control.config;
    if(neck_control_dispatch(&control,m,command_now,execute_remote,NULL,&reply.body)) {
        /* EVENT_ACK is idempotent and intentionally unacknowledged (no ack loop). */
        if(m->body.control.opcode!=NECK_CMD_EVENT_ACK) (void)xQueueSend(reply_queue,&reply,0);
        xTaskNotifyGive(telemetry_handle);
    }
}
static void service_commands(uint64_t now)
{
    static uint64_t last_rx;
    unsigned i; int result; uint8_t byte;
    command_now=now;
    if(command_parser.used && now-last_rx>100000) neck_wire_parser_init(&command_parser);
    for(i=0;i<96;++i) {
        result=neck_uart_rx_pop(&byte);
        if(result==0) break;
        if(result<0) { neck_wire_parser_init(&command_parser); ++g_neck_stats.uart_errors; continue; }
        last_rx=now; (void)neck_wire_parser_feed(&command_parser,&byte,1,received_command,NULL);
    }
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
    if(g_neck_hw_reprobe_request && !maintenance) {
        g_neck_hw_reprobe_request=0; (void)request_transition(0);
    }
#endif
    if(maintenance && acquisition_ack==acquisition_generation) {
        taskENTER_CRITICAL();
        (void)xQueueReset(frame_queue); (void)xQueueReset(pose_queue);
        g_neck_stats.alarm_event_queue_drops+=(uint32_t)uxQueueMessagesWaiting(event_queue)+(event_pending?1u:0u);
        (void)xQueueReset(event_queue); event_pending=0;
        ++current_session; if(settings_changed) ++current_config;
        settings_changed=0;
        taskEXIT_CRITICAL();
        if(!initialize_processing(now)) { g_neck_stats.fatal_error=0xE006; Error_Handler(); }
        neck_control_init(&control,wire_boot_id,current_session,current_config);
        taskENTER_CRITICAL();
        measurement_stopped=(uint8_t)!restart_requested; maintenance=0;
        taskEXIT_CRITICAL();
        xTaskNotifyGive(telemetry_handle);
    }
}

#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
static uint64_t sensor_observation_clock(void *unused)
{
    (void)unused;
    return now_us();
}
#endif

static void acquisition_task(void *argument)
{
    TickType_t previous = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(NECK_SAMPLE_PERIOD_MS);
    uint64_t sequence = 0U;
    uint32_t generation=acquisition_generation;
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
    static NeckAcquisitionFrame frame; /* Keep 328 B off the small task stack. */
    static NeckSensorAcquisition acquisition;
    static NeckSensorReadTrace trace;
    NeckStm32SensorBusContext hal_context = { &hi2c1, 5U };
    NeckI2cBus sensor_bus = neck_stm32_sensor_bus(&hal_context);
#else
    NeckAcquisitionFrame frame;
#endif
    (void)argument;
    configASSERT(period > 0U);
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
    /* Both hardware modes start without a sensor sample clock. */
    sensor_bus.delay_ms(sensor_bus.context, 650U); /* BNO055 POR -> config margin. */
    (void)neck_sensor_acquisition_init(&acquisition, &sensor_bus,
        NECK_HW_NODE_MASK, NECK_HW_FUSION_MODE, NECK_HW_EXTERNAL_CLOCK, current_config, current_session);
    previous = xTaskGetTickCount();
#endif
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if(maintenance || measurement_stopped) {
            acquisition_ack=acquisition_generation;
            vTaskDelayUntil(&previous,period); continue;
        }
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
        if (generation != acquisition_generation) {
            generation=acquisition_generation;
            sensor_bus.delay_ms(sensor_bus.context, 650U);
            (void)neck_sensor_acquisition_init(&acquisition, &sensor_bus,
                NECK_HW_NODE_MASK, NECK_HW_FUSION_MODE, NECK_HW_EXTERNAL_CLOCK, current_config, current_session);
            previous = xTaskGetTickCount();
        }
        (void)neck_sensor_acquisition_frame(&acquisition, sequence++,
            sensor_observation_clock, NULL, &frame, &trace);
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_C4
        {
            NeckC4TimeEvidence evidence = {0};
            if (!c4_mount_ready || neck_c4_board_time(&frame, &trace, &evidence) != 1 ||
                !neck_c4_apply_time(&frame, &trace, &evidence))
                ++g_neck_c4_time_rejected;
        }
#endif
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_THREE
        {
            NeckMeasurementTimes times={0};
            if(three_profile_ready && neck_board_three_times(&frame,&trace,&times)==1)
                (void)neck_measurement_apply(&three_profile,&frame,&trace,&times);
        }
#endif
        g_neck_latest_sensor_trace = trace;
        g_neck_latest_input = frame.acceleration;
#if NECK_FULL_DEBUG_SNAPSHOTS
        g_neck_latest_orientation = frame.orientation;
#endif
        ++g_neck_stats.acquired_frames;
        if (xQueueSend(frame_queue, &frame, 0U) != pdPASS)
            ++g_neck_stats.acquisition_queue_drops;
#else
        generation=acquisition_generation; (void)generation;
        if (g_neck_simulation_pause == 0U) {
            neck_simulator_frame(&frame, sequence++, now_us());
            frame.acceleration.config_id=frame.orientation.config_id=current_config;
            frame.acceleration.session_id=frame.orientation.session_id=current_session;
            g_neck_latest_input = frame.acceleration;
#if NECK_FULL_DEBUG_SNAPSHOTS
            g_neck_latest_orientation = frame.orientation;
#endif
            ++g_neck_stats.acquired_frames;
            if (xQueueSend(frame_queue, &frame, 0U) != pdPASS)
                ++g_neck_stats.acquisition_queue_drops;
        }
#endif
        g_neck_stats.acquisition_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        now = xTaskGetTickCount();
        if ((TickType_t)(now - previous) >= period) {
            ++g_neck_stats.acquisition_overruns;
            /* Skip missed deadlines instead of emitting catch-up frames with
             * identical timestamps. Re-establish a future release time. */
            previous = now;
        }
        vTaskDelayUntil(&previous, period);
    }
}

static void publish_events(const NeckPostureOutput *output, uint32_t epoch)
{
    uint32_t axis;
    for (axis = 0U; axis < 2U; ++axis) {
        const NeckPostureAxisOutput *item = axis == 0U ? &output->flexion : &output->lateral;
        NeckAlarmEvent event;
        if (item->events == 0U) continue;
        memset(&event, 0, sizeof(event));
        event.sequence = output->sequence;
        event.frame_time_us = output->frame_time_us;
        event.time_us = output->time_us;
        event.reference_id = output->reference_id;
        event.invalidation_epoch = epoch;
        event.posture_flags = output->flags;
        event.events = item->events;
        event.axis = (uint8_t)axis;
        event.previous_level = item->previous_alarm_level;
        event.level = item->alarm_level;
        event.direction = item->direction;
        event.source = output->source;
        if (item->events & NECK_EVENT_TRIGGER) ++g_neck_stats.alarm_triggers;
        if (item->events & NECK_EVENT_CLEAR) ++g_neck_stats.alarm_clears;
        if (item->events & NECK_EVENT_LEVEL_CHANGE) ++g_neck_stats.alarm_level_changes;
        if (xQueueSend(event_queue, &event, 0U) != pdPASS)
            ++g_neck_stats.alarm_event_queue_drops;
        else xTaskNotifyGive(telemetry_handle);
    }
}

static void publish_output(const NeckProcessingOutput *output, uint8_t snapshot,
    const NeckAcquisitionFrame *input)
{
    NeckWireSnapshot wire_snapshot;
    if (output->pose.invalidation_epoch != invalidation_epoch) {
        invalidation_epoch = output->pose.invalidation_epoch;
        (void)xQueueReset(pose_queue);
        snapshot = 1U;
    }
    g_neck_latest_pose = output->pose;
#if NECK_FULL_DEBUG_SNAPSHOTS
    g_neck_latest_pipeline = output->c4;
    g_neck_latest_relative = output->relative;
    g_neck_latest_posture = output->pose.posture;
#endif
    publish_events(&output->pose.posture, output->pose.invalidation_epoch);
    if (snapshot) {
        /* One freshest snapshot, no extra queue/task. Event queue is separate. */
        neck_wire_producer_snapshot(&wire_snapshot, output, input, (uint8_t)(input != NULL));
        wire_snapshot.session_id=current_session; wire_snapshot.config_id=current_config;
        if (uxQueueMessagesWaiting(pose_queue)) ++g_neck_stats.telemetry_queue_drops;
        (void)xQueueOverwrite(pose_queue, &wire_snapshot);
        xTaskNotifyGive(telemetry_handle);
    }
}
static void processing_stats(void)
{
    const NeckProcessingStats *s = &processing_context.stats;
    g_neck_stats.algorithm_errors = s->pipeline_errors; g_neck_stats.posture_errors = s->posture_errors;
    g_neck_stats.relative_errors = s->relative_errors;
    g_neck_stats.zero_requests_started = s->zero_requests; g_neck_stats.zero_cancel_commands = s->zero_cancels;
    g_neck_stats.zero_command_errors = s->command_errors; g_neck_stats.zero_node_commits = s->node_commits;
    g_neck_stats.filter_resets = s->filter_resets; g_neck_stats.relative_zero_commits = s->relative_commits;
    g_neck_stats.relative_zero_aborts = s->relative_aborts;
    g_neck_stats.quality_rejected_frames = s->rejected_frames;
    g_neck_stats.input_timeout_transitions = s->timeout_transitions;
}
static void algorithm_task(void *argument)
{
    static NeckAcquisitionFrame packet; /* Algorithm owner; save task stack. */
    NeckProcessingOutput output;
    uint8_t command;
    uint64_t last_heartbeat = 0U;
    uint8_t published = 0U;
    const TickType_t wait = pdMS_TO_TICKS(NECK_SUPERVISION_PERIOD_MS);
    (void)argument;
    configASSERT(wait > 0U);
    for (;;) {
        uint64_t now = now_us();
        BaseType_t received;
        service_commands(now);
        if (xQueueReceive(command_queue, &command, 0U) == pdPASS) apply_command(command, now);
        if (g_neck_zero_request != 0U) {
            uint32_t debug_command = g_neck_zero_request;
            g_neck_zero_request = 0U;
            apply_command(debug_command, now);
        }
        received = xQueueReceive(frame_queue, &packet, wait);
        now = now_us();
        /* Poll BEFORE a resumed frame: do not hide an expired old alarm. */
        if (neck_processing_poll(&processing_context, now, &output)) {
            uint8_t heartbeat = (uint8_t)(!published || now - last_heartbeat >= NECK_TELEMETRY_PERIOD_MS*1000U ||
                output.pose.invalidation_epoch != invalidation_epoch);
            publish_output(&output, heartbeat, NULL);
            if (heartbeat) { last_heartbeat = now; published = 1U; }
        }
        if (received == pdPASS && !measurement_stopped && !maintenance) {
            uint32_t i;
            (void)neck_processing_process(&processing_context, &packet, now, &output);
            for (i = 0U; i < NECK_NODE_COUNT; ++i) {
                if (!(output.pose.quality.accel_usable_mask & (1U << i))) ++g_neck_stats.gated_accel_nodes;
                if (!(output.pose.quality.orientation_usable_mask & (1U << i))) ++g_neck_stats.gated_orientation_nodes;
            }
            {
                uint8_t snapshot = (uint8_t)(!published || now-last_heartbeat >= NECK_TELEMETRY_PERIOD_MS*1000U ||
                    output.pose.invalidation_epoch != invalidation_epoch);
                publish_output(&output, snapshot, &packet);
                if (snapshot) { last_heartbeat = now; published = 1U; }
            }
            ++g_neck_stats.processed_frames;
        }
        processing_stats();
        g_neck_stats.algorithm_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        g_neck_stats.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
    }
}

/* One blocking UART owner, lower priority than acquisition and algorithm.
 * Serialized payloads are bounded; producer queues never wait for UART. */
static uint32_t wire_tx_sequence;
static uint64_t wire_event_id;
static uint8_t wire_bytes[NECK_WIRE_MAX_FRAME];

static void wire_header(NeckWireMessage *message, uint8_t type, uint64_t now)
{
    memset(message, 0, sizeof(*message));
    message->header.type=type;
    message->header.tx_sequence=wire_tx_sequence++;
    message->header.boot_id=wire_boot_id;
    taskENTER_CRITICAL();
    message->header.session_id=current_session;
    message->header.config_id=current_config;
    if(type==NECK_WIRE_SESSION) message->body.session=wire_session;
    taskEXIT_CRITICAL();
    message->header.emit_time_us=now;
}
static int wire_send(NeckWireMessage *message)
{
    size_t length=0U;
    uint64_t now=now_us(), duration, finish;
    NeckWireResult result;
    size_t expected=NECK_WIRE_HEADER_SIZE+neck_wire_payload_size(message->header.type)+NECK_WIRE_CRC_SIZE;
    if(maintenance) return 0;
    /* 115200 8N1: ten bits per byte, rounded up. Validate projected end of
     * serialization, not merely the instant the UART call begins. */
    duration=((uint64_t)expected*10000000U+NECK_UART_BAUD-1U)/NECK_UART_BAUD;
    finish=now>UINT64_MAX-duration?UINT64_MAX:now+duration;
    message->header.emit_time_us=now;
    if(neck_wire_producer_expire(message,finish,invalidation_epoch))
        ++g_neck_stats.stale_snapshot_copies;
    result=neck_wire_encode(message,wire_bytes,sizeof(wire_bytes),&length);
    if(result!=NECK_WIRE_OK || length!=expected) {
        ++g_neck_stats.telemetry_queue_drops;
        return 0;
    }
    if(HAL_UART_Transmit(&huart2,wire_bytes,(uint16_t)length,
            (uint32_t)((duration+999U)/1000U+30U))!=HAL_OK) {
        ++g_neck_stats.uart_errors;
        return 0;
    }
    return 1;
}
static void telemetry_task(void *argument)
{
    NeckWireSnapshot snapshot;
    CommandReply reply;
    static NeckWireMessage message;
    uint64_t last_session=0U,last_status=0U;
    uint8_t registered=0U,status_sent=0U,next_relative=0U;
    uint32_t registered_session=0;
    (void)argument;
    for(;;) {
        uint64_t now=now_us();
        if(maintenance) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        if(xQueueReceive(reply_queue,&reply,0)==pdPASS) {
            wire_header(&message,NECK_WIRE_REPLY,now);
            message.header.boot_id=reply.boot; message.header.session_id=reply.session;
            message.header.config_id=reply.config; message.body.control=reply.body;
            (void)wire_send(&message); /* Lost response is retried by request id. */
        }
        if(registered_session!=current_session) { registered=0; status_sent=0; next_relative=0; }
        if(!registered || now-last_session>=1000000U) {
            wire_header(&message,NECK_WIRE_SESSION,now);
            if(wire_send(&message)) { registered=1U; last_session=now; registered_session=message.header.session_id; }
        }
        (void)ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20U));
        if(!registered) continue;
        /* Retain FIFO head until exact event-id ACK, retry every 250 ms.
         * Finite queue overflow and session retirement remain visible losses. */
        taskENTER_CRITICAL();
        if(!event_pending && xQueueReceive(event_queue,&retained_event,0)==pdPASS) {
            if(wire_event_id==UINT64_MAX) { g_neck_stats.fatal_error=0xE007; Error_Handler(); }
            retained_event_id=++wire_event_id; event_session=current_session; event_config=current_config;
            event_last_send=0; event_pending=1;
        }
        if(event_pending && (!event_last_send || now-event_last_send>=250000)) {
            wire_header(&message,NECK_WIRE_EVENT,now);
            message.header.session_id=event_session; message.header.config_id=event_config;
            neck_wire_producer_event(&message.body.event,&retained_event,retained_event_id);
            event_last_send=now;
            taskEXIT_CRITICAL(); (void)wire_send(&message);
        } else taskEXIT_CRITICAL();
        if(xQueueReceive(pose_queue,&snapshot,0U)==pdPASS) {
            if(snapshot.session_id!=current_session || snapshot.config_id!=current_config) continue;
            now=now_us();
            /* At 115200, C4+RELATIVE from one 30 ms-TTL snapshot need ~36 ms
             * of wire time. Alternate at 50 ms instead; STATUS gets its own
             * slot. Each frame gets a fresh end-of-UART validity check. */
            if(!status_sent || now-last_status>=500000U) {
                wire_header(&message,NECK_WIRE_STATUS,now);
                message.header.session_id=snapshot.session_id; message.header.config_id=snapshot.config_id;
                message.body.status=snapshot.status;
                message.body.status.input_drops=g_neck_stats.acquisition_queue_drops+
                    g_neck_stats.acquisition_overruns;
                message.body.status.telemetry_drops=g_neck_stats.telemetry_queue_drops;
                message.body.status.event_drops=g_neck_stats.alarm_event_queue_drops;
                message.body.status.uart_errors=g_neck_stats.uart_errors;
                if(now>snapshot.c4.valid_until_us) {
                    message.body.status.quality_flags|=NECK_Q_TRANSPORT_STALE;
                    message.body.status.accel_usable_mask=0U;
                    message.body.status.orientation_usable_mask=0U;
                    message.body.status.quality_state=0U;
                    message.body.status.frame_accepted=0U;
                }
                (void)wire_send(&message);
                last_status=now; status_sent=1U;
            } else if((wire_session.capabilities & NECK_WIRE_CAP_RELATIVE) && next_relative) {
                wire_header(&message,NECK_WIRE_RELATIVE,now);
                message.header.session_id=snapshot.session_id; message.header.config_id=snapshot.config_id;
                message.body.relative=snapshot.relative;
                (void)wire_send(&message);
                next_relative=0U;
            } else {
                wire_header(&message,NECK_WIRE_C4,now);
                message.header.session_id=snapshot.session_id; message.header.config_id=snapshot.config_id;
                message.body.c4=snapshot.c4;
                (void)wire_send(&message);
                next_relative=1U;
            }
        }
        g_neck_stats.telemetry_stack_free_words=uxTaskGetStackHighWaterMark(NULL);
    }
}

int neck_app_start(void)
{
    NeckSettingsIo io=neck_settings_stm32_io();
    configASSERT(sizeof(TickType_t) == 4U && 1000000U % configTICK_RATE_HZ == 0U);
    if (neck_clock_init(&wall_clock, (uint32_t)xTaskGetTickCount(), 1000000U/configTICK_RATE_HZ) != NECK_OK) return 0;
    if(!neck_settings_open(&settings_store,&io) || !neck_settings_reserve_boot(&settings_store)) {
        g_neck_stats.fatal_error=0xE008; return 0;
    }
    wire_boot_id=settings_store.boot; current_config=settings_store.config;
    active_settings=settings_store.settings;
    if(!initialize_processing(now_us())) return 0;
    neck_control_init(&control,wire_boot_id,current_session,current_config);
    neck_wire_parser_init(&command_parser);
    frame_queue = xQueueCreate(2U, sizeof(NeckAcquisitionFrame));
    pose_queue = xQueueCreate(1U, sizeof(NeckWireSnapshot));
    command_queue = xQueueCreate(1U, sizeof(uint8_t));
    reply_queue = xQueueCreate(1U, sizeof(CommandReply));
#if NECK_INPUT_MODE == NECK_INPUT_HARDWARE_C4 || NECK_INPUT_MODE == NECK_INPUT_HARDWARE_THREE
    event_queue = xQueueCreate(4U, sizeof(NeckAlarmEvent));
#elif NECK_INPUT_MODE == NECK_INPUT_HARDWARE_DIAGNOSTIC
    /* Quality never approves angle/alarm events in this diagnostic image. */
    event_queue = xQueueCreate(1U, sizeof(NeckAlarmEvent));
#else
    event_queue = xQueueCreate(8U, sizeof(NeckAlarmEvent));
#endif
    if (frame_queue == NULL || pose_queue == NULL || command_queue == NULL || event_queue == NULL || reply_queue == NULL) return 0;
    if(!neck_uart_rx_start()) return 0;
#if NECK_INPUT_MODE != NECK_INPUT_SIMULATION
    if (xTaskCreate(acquisition_task, "D3Probe", 384U, NULL, 3U, NULL) != pdPASS ||
#else
    if (xTaskCreate(acquisition_task, "SimAcquire", 256U, NULL, 3U, NULL) != pdPASS ||
#endif
        xTaskCreate(algorithm_task, "TiltAlgorithm", 896U, NULL, 2U, NULL) != pdPASS ||
        xTaskCreate(telemetry_task, "D0Telemetry", 576U, NULL, 1U, &telemetry_handle) != pdPASS) return 0;
    return 1;
}

void neck_assert_failed(const char *file, unsigned long line)
{
    (void)file;
    g_neck_stats.fatal_error = (uint32_t)line;
    Error_Handler();
}
void vApplicationMallocFailedHook(void) { g_neck_stats.fatal_error = 0xE001U; Error_Handler(); }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    g_neck_stats.fatal_error = 0xE002U;
    Error_Handler();
}
