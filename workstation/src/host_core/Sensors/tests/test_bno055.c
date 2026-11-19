#include "neck_bno055.h"
#include "neck_sensor_acquisition.h"
#include "neck_wire.h"
#include "neck_wire_producer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t reg[3][128];
    uint8_t selected;
    uint8_t fail_mux;
    uint8_t fail_read_reg;
    uint8_t external_clock_fails;
    uint8_t ignore_opmode_write;
    unsigned mux_writes;
    unsigned sensor_reads;
    unsigned delay_total_ms;
} FakeBus;

static unsigned selected_channel(const FakeBus *fake)
{
    assert(fake->selected == 1u || fake->selected == 2u || fake->selected == 4u);
    return fake->selected == 1u ? 0u : (fake->selected == 2u ? 1u : 2u);
}

static int fake_write(void *opaque, uint8_t address7,
                      const uint8_t *data, size_t length)
{
    FakeBus *fake = (FakeBus *)opaque;
    unsigned channel;
    if (address7 == NECK_TCA9548A_ADDRESS7) {
        assert(length == 1u);
        assert(data[0] == 1u || data[0] == 2u || data[0] == 4u);
        fake->mux_writes++;
        if (fake->fail_mux) return -1;
        fake->selected = data[0];
        return 0;
    }
    assert(address7 == NECK_BNO055_ADDRESS7);
    assert(length == 2u);
    channel = selected_channel(fake);
    assert(data[0] < 128u);
    if (data[0] != 0x3Du || !fake->ignore_opmode_write)
        fake->reg[channel][data[0]] = data[1];
    return 0;
}

static int fake_read(void *opaque, uint8_t address7, uint8_t reg,
                     uint8_t *data, size_t length)
{
    FakeBus *fake = (FakeBus *)opaque;
    unsigned channel;
    assert(address7 == NECK_BNO055_ADDRESS7);
    assert(reg + length <= 128u && length > 0u);
    channel = selected_channel(fake);
    fake->sensor_reads++;
    if (reg == fake->fail_read_reg) return -1;
    memcpy(data, &fake->reg[channel][reg], length);
    if (reg == 0x3Fu && fake->external_clock_fails) data[0] &= (uint8_t)~0x80u;
    return 0;
}

static void fake_delay(void *opaque, uint32_t milliseconds)
{
    FakeBus *fake = (FakeBus *)opaque;
    fake->delay_total_ms += milliseconds;
}

static NeckI2cBus make_bus(FakeBus *fake)
{
    NeckI2cBus bus;
    bus.context = fake;
    bus.write = fake_write;
    bus.read_register = fake_read;
    bus.delay_ms = fake_delay;
    return bus;
}

static void put_s16(FakeBus *fake, unsigned channel, unsigned reg, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    fake->reg[channel][reg] = (uint8_t)raw;
    fake->reg[channel][reg + 1u] = (uint8_t)(raw >> 8);
}

static void setup(FakeBus *fake)
{
    unsigned channel;
    memset(fake, 0, sizeof(*fake));
    fake->fail_read_reg = 0xFFu;
    for (channel = 0u; channel < 3u; ++channel) {
        fake->reg[channel][0x00] = 0xA0u;
        fake->reg[channel][0x3B] = 0x80u;
        fake->reg[channel][0x39] = 5u;
        put_s16(fake, channel, 0x08, (int16_t)(100 + channel * 100));
        put_s16(fake, channel, 0x0A, (int16_t)(-250 - (int)channel * 100));
        put_s16(fake, channel, 0x0C, 981);
        put_s16(fake, channel, 0x20, 16384);
        put_s16(fake, channel, 0x22, (int16_t)(-8192 - (int)channel));
        put_s16(fake, channel, 0x24, 0);
        put_s16(fake, channel, 0x26, 4096);
        fake->reg[channel][0x35] = (uint8_t)(0xE4u + channel);
    }
}

static void test_three_channels(void)
{
    FakeBus fake;
    NeckI2cBus bus;
    NeckBno055 node[3];
    NeckBno055Observation observation;
    unsigned channel;
    setup(&fake);
    bus = make_bus(&fake);
    for (channel = 0u; channel < 3u; ++channel) {
        assert(neck_bno055_configure(&bus, &node[channel], (uint8_t)channel,
                   NECK_BNO055_MODE_NDOF, 0u) == NECK_SENSOR_OK);
        assert(node[channel].configured == 1u);
        assert(fake.selected == (uint8_t)(1u << channel));
        assert(fake.reg[channel][0x3D] == NECK_BNO055_MODE_NDOF);
        assert(fake.reg[channel][0x3B] == 0u);
    }
    assert(fake.delay_total_ms == 90u);
    for (channel = 0u; channel < 3u; ++channel) {
        assert(neck_bno055_read(&bus, &node[channel], &observation) == NECK_SENSOR_OK);
        assert(observation.observation_valid == 1u);
        assert(observation.system_status == 5u);
        assert(observation.sample_time_known == 0u);
        assert(fabs(observation.accel_m_s2[0] - (1.0 + channel)) < 1e-12);
        assert(fabs(observation.accel_m_s2[1] - (-2.5 - channel)) < 1e-12);
        assert(fabs(observation.accel_m_s2[2] - 9.81) < 1e-12);
        assert(fabs(observation.quaternion_wxyz[0] - 1.0) < 1e-12);
        assert(fabs(observation.quaternion_wxyz[1] -
                    ((double)(-8192 - (int)channel) / 16384.0)) < 1e-12);
        assert(observation.calibration_system == 3u);
        assert(observation.calibration_accel == 1u);
    }
    assert(fake.mux_writes == 6u);
}

static void test_failure_paths(void)
{
    FakeBus fake;
    NeckI2cBus bus;
    NeckBno055 node;
    NeckBno055Observation observation;
    setup(&fake);
    bus = make_bus(&fake);
    assert(neck_bno055_probe(&bus, 3u) == NECK_SENSOR_BAD_ARGUMENT);
    fake.fail_mux = 1u;
    assert(neck_bno055_probe(&bus, 0u) == NECK_SENSOR_BUS_ERROR);
    fake.fail_mux = 0u;
    fake.reg[1][0x00] = 0x00u;
    assert(neck_bno055_probe(&bus, 1u) == NECK_SENSOR_CHIP_ID_MISMATCH);
    assert(neck_bno055_configure(&bus, &node, 1u, NECK_BNO055_MODE_NDOF, 0u)
           == NECK_SENSOR_CHIP_ID_MISMATCH);
    assert(node.configured == 0u);
    fake.reg[1][0x00] = 0xA0u;
    assert(neck_bno055_configure(&bus, &node, 1u, NECK_BNO055_MODE_NDOF, 0u)
           == NECK_SENSOR_OK);
    fake.reg[1][0x39] = 1u;
    assert(neck_bno055_read(&bus, &node, &observation)
           == NECK_SENSOR_FUSION_NOT_RUNNING);
    assert(observation.observation_valid == 0u);
    fake.reg[1][0x39] = 5u;
    fake.fail_read_reg = 0x20u;
    memset(&observation, 0xA5, sizeof(observation));
    assert(neck_bno055_read(&bus, &node, &observation) == NECK_SENSOR_BUS_ERROR);
    assert(observation.observation_valid == 0u);
    assert(observation.sample_time_known == 0u);
    assert(observation.accel_m_s2[0] == 0.0);
}

static void test_external_clock_and_config_mismatch(void)
{
    FakeBus fake;
    NeckI2cBus bus;
    NeckBno055 node;
    setup(&fake);
    bus = make_bus(&fake);
    assert(neck_bno055_configure(&bus, &node, 0u, NECK_BNO055_MODE_IMU, 1u)
           == NECK_SENSOR_OK);
    assert(fake.delay_total_ms == 680u);
    assert(node.configured == 1u);
    setup(&fake);
    bus = make_bus(&fake);
    fake.external_clock_fails = 1u;
    assert(neck_bno055_configure(&bus, &node, 0u, NECK_BNO055_MODE_IMU, 1u)
           == NECK_SENSOR_EXTERNAL_CLOCK_FAILED);
    assert(node.configured == 0u);
    setup(&fake);
    bus = make_bus(&fake);
    fake.ignore_opmode_write = 1u;
    assert(neck_bno055_configure(&bus, &node, 0u, NECK_BNO055_MODE_NDOF, 0u)
           == NECK_SENSOR_CONFIG_MISMATCH);
    assert(node.configured == 0u);
}

typedef struct { uint64_t time_us; uint8_t backwards; } FakeClock;
static uint64_t fake_now_us(void *opaque)
{
    FakeClock *clock = (FakeClock *)opaque;
    if (clock->backwards) {
        clock->time_us = clock->time_us == 0u ? 1000u : clock->time_us - 1u;
    } else clock->time_us += 1000u;
    return clock->time_us;
}

static void test_single_node_frame_and_quality_gate(void)
{
    FakeBus fake;
    FakeClock clock = {0u, 0u};
    NeckI2cBus bus;
    NeckSensorAcquisition acquisition;
    NeckAcquisitionFrame frame;
    NeckSensorReadTrace trace;
    NeckQualityParameters parameters;
    NeckQualityContext quality;
    NeckQualityOutput output;
    setup(&fake);
    bus = make_bus(&fake);
    put_s16(&fake, 1u, 0x08, 0);
    put_s16(&fake, 1u, 0x0A, 0);
    put_s16(&fake, 1u, 0x0C, 981);
    put_s16(&fake, 1u, 0x22, 0);
    put_s16(&fake, 1u, 0x26, 0);
    fake.reg[1][0x35] = 0xFFu;
    assert(neck_sensor_acquisition_init(&acquisition, &bus, NECK_SENSOR_C4_ONLY,
        NECK_BNO055_MODE_NDOF, 0u, 11u, 23u) == NECK_SENSOR_OK);
    assert(acquisition.ready_mask == NECK_SENSOR_C4_ONLY);
    assert(fake.mux_writes == 1u);
    assert(neck_sensor_acquisition_frame(&acquisition, 1u, fake_now_us,
        &clock, &frame, &trace) == NECK_SENSOR_OK);
    assert(trace.selected_mask == NECK_SENSOR_C4_ONLY);
    assert(trace.observed_mask == NECK_SENSOR_C4_ONLY);
    assert(trace.read_start_us[1] == 1000u && trace.read_end_us[1] == 2000u);
    assert(frame.acceleration.frame_time_us == 3000u);
    assert(frame.orientation.frame_time_us == 3000u);
    assert(frame.acceleration.source == NECK_SOURCE_HARDWARE);
    assert(frame.acceleration.nodes[0].status == NECK_NODE_MISSING);
    assert(frame.acceleration.nodes[2].status == NECK_NODE_MISSING);
    assert(frame.acceleration.nodes[1].status == NECK_NODE_PARTIAL);
    assert(frame.acceleration.nodes[1].signal_valid_mask ==
           (NECK_SIGNAL_ACCEL | NECK_SIGNAL_QUATERNION));
    assert(frame.acceleration.nodes[1].sample_time_known == 0u);
    assert(frame.orientation.nodes[1].sample_time_known == 0u);
    assert(frame.orientation.nodes[1].world_frame_id == 0u);
    assert(frame.acceleration.nodes[1].accel_m_s2[2] == 9.81);
    neck_quality_default_parameters(&parameters);
    assert(neck_quality_init(&quality, 11u, 1u, 0u, &parameters) == NECK_OK);
    assert(neck_quality_process(&quality, &frame.acceleration,
        &frame.orientation, 4000u, &output) == NECK_OK);
    assert(output.frame_accepted == 1u);
    assert((output.node_flags[1] & (NECK_QN_ACCEL_METADATA | NECK_QN_QUAT_METADATA)) == 0u);
    assert(output.accel_usable_mask == 0u && output.orientation_usable_mask == 0u);
    assert((output.node_flags[1] & NECK_QN_ACCEL_TIME_UNKNOWN) != 0u);
    assert((output.node_flags[1] & NECK_QN_QUAT_TIME_UNKNOWN) != 0u);
    assert((output.node_flags[1] & NECK_QN_WORLD_MISMATCH) != 0u);
}

static void test_three_node_partial_and_clock_error(void)
{
    FakeBus fake;
    FakeClock clock = {0u, 0u};
    NeckI2cBus bus;
    NeckSensorAcquisition acquisition;
    NeckAcquisitionFrame frame;
    NeckSensorReadTrace trace;
    setup(&fake);
    fake.reg[1][0x00] = 0x00u;
    bus = make_bus(&fake);
    assert(neck_sensor_acquisition_init(&acquisition, &bus, NECK_SENSOR_ALL_NODES,
        NECK_BNO055_MODE_NDOF, 0u, 1u, 1u) == NECK_SENSOR_OK);
    assert(acquisition.ready_mask == 5u);
    assert(acquisition.init_result[1] == NECK_SENSOR_CHIP_ID_MISMATCH);
    assert(neck_sensor_acquisition_frame(&acquisition, 2u, fake_now_us,
        &clock, &frame, &trace) == NECK_SENSOR_OK);
    assert(trace.observed_mask == 5u);
    assert(trace.result[1] == NECK_SENSOR_CHIP_ID_MISMATCH);
    assert(frame.acceleration.nodes[1].status == NECK_NODE_FAULT);
    assert(frame.acceleration.nodes[1].accel_present == 0u);
    assert(frame.orientation.nodes[1].quat_present == 0u);
    assert(frame.acceleration.nodes[0].status == NECK_NODE_PARTIAL);
    assert(frame.acceleration.nodes[2].status == NECK_NODE_PARTIAL);
    fake.fail_read_reg = 0x20u;
    assert(neck_sensor_acquisition_frame(&acquisition, 3u, fake_now_us,
        &clock, &frame, &trace) == NECK_SENSOR_NO_OBSERVATION);
    assert(trace.observed_mask == 0u);
    assert(frame.acceleration.nodes[0].status == NECK_NODE_FAULT);
    assert(frame.acceleration.nodes[0].accel_present == 0u);
    assert(frame.orientation.nodes[2].quat_present == 0u);
    clock.backwards = 1u;
    assert(neck_sensor_acquisition_frame(&acquisition, 4u, fake_now_us,
        &clock, &frame, &trace) == NECK_SENSOR_CLOCK_ERROR);
    assert(frame.acceleration.config_id == 0u);
    assert(trace.observed_mask == 0u);
}

static void test_hardware_diagnostic_session(void)
{
    NeckWireMessage message, decoded;
    NeckWireReceiver receiver;
    NeckProcessingParameters parameters;
    uint8_t bytes[NECK_WIRE_MAX_FRAME];
    size_t length = 0u;
    unsigned mode;
    memset(&parameters, 0, sizeof(parameters));
    parameters.pipeline.nominal_period_us = 10000u;
    parameters.pipeline.cutoff_hz = 2.0;
    parameters.quality.no_frame_timeout_us = 30000u;
    parameters.posture.continuous_duration_us = 10000000u;
    parameters.posture.flexion_warning_deg = 15.0;
    parameters.posture.flexion_danger_deg = 30.0;
    parameters.posture.lateral_warning_deg = 15.0;
    parameters.posture.lateral_danger_deg = 30.0;
    for (mode = 0u; mode < 2u; ++mode) {
        NeckWireSession *session;
        memset(&message, 0, sizeof(message));
        message.header.type = NECK_WIRE_SESSION;
        message.header.boot_id = 1u;
        message.header.session_id = 1u;
        message.header.config_id = 1u;
        session = &message.body.session;
        assert(neck_wire_producer_d3_diagnostic_session(session, &parameters,
            mode == 0u ? 2u : 7u));
        assert(session->world_frame_id == 0u);
        assert(session->relative_period_us == 0u);
        assert(neck_wire_encode(&message, bytes, sizeof(bytes), &length) == NECK_WIRE_OK);
        assert(neck_wire_decode(bytes, length, &decoded) == NECK_WIRE_OK);
        assert(decoded.body.session.source == NECK_WIRE_HARDWARE);
        assert(decoded.body.session.node_mask == session->node_mask);
        assert(neck_wire_receiver_init(&receiver, 500000u, 300000u) == NECK_WIRE_OK);
        assert(neck_wire_receiver_accept(&receiver, &decoded, 1000u) == NECK_WIRE_RX_SESSION);
        assert(receiver.session.world_frame_id == 0u);
        assert((neck_wire_receiver_poll(&receiver, 1000u) &
                NECK_WIRE_HOST_RELATIVE_STALE) != 0u);
    }
    assert(!neck_wire_producer_d3_diagnostic_session(&message.body.session,
        &parameters, 3u));
    assert(message.body.session.source == 0u);
}

int main(void)
{
    test_three_channels();
    test_failure_paths();
    test_external_clock_and_config_mismatch();
    test_single_node_frame_and_quality_gate();
    test_three_node_partial_and_clock_error();
    test_hardware_diagnostic_session();
    puts("D3 sensor/acquisition/quality/wire fake-bus tests: PASS");
    return 0;
}
