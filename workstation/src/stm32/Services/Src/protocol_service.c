#include "protocol_service.h"
#include "app_config.h"

#include <string.h>

static const uint8_t kSyncBytes[4] = {0x4EU, 0x43U, 0x4BU, 0x31U}; /* "NCK1" */

static void Put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8U);
}

static void Put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8U);
    p[2] = (uint8_t)(v >> 16U);
    p[3] = (uint8_t)(v >> 24U);
}

static void Put64(uint8_t *p, uint64_t v)
{
    uint32_t i;
    for (i = 0U; i < 8U; ++i)
    {
        p[i] = (uint8_t)(v >> (8U * i));
    }
}

static uint16_t Get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

static uint32_t Get32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8U)
         | ((uint32_t)p[2] << 16U)
         | ((uint32_t)p[3] << 24U);
}

static uint16_t Crc16CcittFalse(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint32_t bit;

    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (uint16_t)((crc & 0x8000U) ? (((uint32_t)crc << 1U) ^ 0x1021U) : ((uint32_t)crc << 1U));
        }
    }
    return crc;
}

static void WriteHeader(uint8_t *output,
                        uint8_t type,
                        uint16_t payload_size,
                        uint32_t tx_sequence,
                        uint32_t boot_id,
                        uint32_t session_id,
                        uint32_t config_id,
                        uint64_t emit_time_us)
{
    memcpy(output, kSyncBytes, 4U);
    output[4] = PROTOCOL_WIRE_VERSION;
    output[5] = type;
    Put16(&output[6], payload_size);
    Put32(&output[8], tx_sequence);
    Put32(&output[12], boot_id);
    Put32(&output[16], session_id);
    Put32(&output[20], config_id);
    Put64(&output[24], emit_time_us);
}

AppStatus ProtocolService_EncodeSessionFrame(uint32_t tx_sequence,
                                             uint32_t boot_id,
                                             uint32_t session_id,
                                             uint32_t config_id,
                                             uint64_t emit_time_us,
                                             uint32_t sample_period_us,
                                             uint8_t *output,
                                             size_t capacity,
                                             size_t *output_length)
{
    size_t total = PROTOCOL_WIRE_HEADER_SIZE + PROTOCOL_WIRE_SESSION_SIZE + PROTOCOL_WIRE_CRC_SIZE;
    uint8_t *p;

    if ((output == NULL) || (output_length == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (capacity < total)
    {
        *output_length = 0U;
        return APP_STATUS_BUFFER_TOO_SMALL;
    }

    memset(output, 0, total);
    WriteHeader(output,
                PROTOCOL_TYPE_SESSION,
                PROTOCOL_WIRE_SESSION_SIZE,
                tx_sequence,
                boot_id,
                session_id,
                config_id,
                emit_time_us);

    p = &output[PROTOCOL_WIRE_HEADER_SIZE];
    Put32(&p[0], 1U);                /* profile_id = NECK_WIRE_PROFILE_S09 */
    Put32(&p[4], APP_FIRMWARE_ID);
    Put32(&p[8], 1U);                /* world_frame_id */
    p[12] = 1U;                      /* source = NECK_WIRE_HARDWARE */
    p[13] = 3U;                      /* mode = NECK_WIRE_THREE_NODES */
    p[14] = 7U;                      /* node_mask = C0|C4|C7 */
    p[15] = 1U;                      /* reference_origin = external/host */
    Put32(&p[16], sample_period_us); /* 20000 us (50 Hz) */
    Put32(&p[20], 3U * sample_period_us);
    Put32(&p[24], 500U);             /* cutoff_millihz (0.5 Hz) */
    Put64(&p[28], 10000000ULL);      /* continuous_duration_us (10 s) */
    Put32(&p[36], 15000U);           /* flexion warning 15 deg */
    Put32(&p[40], 30000U);           /* flexion danger 30 deg */
    Put32(&p[44], 5000U);            /* lateral warning 5 deg */
    Put32(&p[48], 10000U);           /* lateral danger 10 deg */
    Put32(&p[52], 15000U);           /* max_node_skew_us */
    Put32(&p[56], 50000U);           /* c4_period_us */
    Put32(&p[60], 100000U);          /* relative_period_us */
    Put32(&p[64], 500000U);          /* status_period_us */
    Put32(&p[68], 127U);             /* capabilities: C4|REL|STATUS|EVENTS|CONTROL|ACK|RAW_FRAME */

    Put16(&output[total - 2U], Crc16CcittFalse(&output[4], total - 6U));
    *output_length = total;
    return APP_STATUS_OK;
}

AppStatus ProtocolService_EncodeSampleFrame(const AppSampleFrame *frame,
                                            uint32_t tx_sequence,
                                            uint32_t boot_id,
                                            uint32_t session_id,
                                            uint32_t config_id,
                                            uint64_t emit_time_us,
                                            uint32_t sample_period_us,
                                            uint8_t *output,
                                            size_t capacity,
                                            size_t *output_length)
{
    size_t total = PROTOCOL_WIRE_HEADER_SIZE + PROTOCOL_WIRE_RAW_FRAME_SIZE + PROTOCOL_WIRE_CRC_SIZE;
    uint8_t *p;
    uint32_t i;

    if ((frame == NULL) || (output == NULL) || (output_length == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (capacity < total)
    {
        *output_length = 0U;
        return APP_STATUS_BUFFER_TOO_SMALL;
    }

    memset(output, 0, total);
    WriteHeader(output,
                PROTOCOL_TYPE_RAW_FRAME,
                PROTOCOL_WIRE_RAW_FRAME_SIZE,
                tx_sequence,
                boot_id,
                session_id,
                config_id,
                emit_time_us);

    p = &output[PROTOCOL_WIRE_HEADER_SIZE];
    Put32(&p[0], frame->sequence);
    Put32(&p[4], sample_period_us);
    p[8] = 7U; /* C0|C4|C7 mask */
    p[9] = 0U;
    Put16(&p[10], 0U);

    for (i = 0U; i < APP_IMU_NODE_COUNT; ++i)
    {
        uint8_t *n = &p[12U + (i * 36U)];
        const AppImuSample *s = &frame->samples[i];

        Put32(&n[0], s->timestamp_start_us);
        Put32(&n[4], s->timestamp_end_us);
        Put32(&n[8], s->timestamp_mid_us);
        Put16(&n[12], (uint16_t)s->quaternion.w);
        Put16(&n[14], (uint16_t)s->quaternion.x);
        Put16(&n[16], (uint16_t)s->quaternion.y);
        Put16(&n[18], (uint16_t)s->quaternion.z);
        Put16(&n[20], (uint16_t)s->acceleration.x);
        Put16(&n[22], (uint16_t)s->acceleration.y);
        Put16(&n[24], (uint16_t)s->acceleration.z);
        Put16(&n[26], (uint16_t)s->gyroscope.x);
        Put16(&n[28], (uint16_t)s->gyroscope.y);
        Put16(&n[30], (uint16_t)s->gyroscope.z);
        n[32] = (uint8_t)s->temperature_c;
        n[33] = s->calibration_status;
        Put16(&n[34], s->flags);
    }

    Put16(&output[total - 2U], Crc16CcittFalse(&output[4], total - 6U));
    *output_length = total;
    return APP_STATUS_OK;
}

AppStatus ProtocolService_EncodeReplyFrame(const AppControlCommand *command,
                                           uint32_t result_code,
                                           uint32_t tx_sequence,
                                           uint32_t boot_id,
                                           uint32_t session_id,
                                           uint32_t config_id,
                                           uint64_t emit_time_us,
                                           uint8_t *output,
                                           size_t capacity,
                                           size_t *output_length)
{
    size_t total = PROTOCOL_WIRE_HEADER_SIZE + PROTOCOL_WIRE_CONTROL_SIZE + PROTOCOL_WIRE_CRC_SIZE;
    uint8_t *p;

    if ((command == NULL) || (output == NULL) || (output_length == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }
    if (capacity < total)
    {
        *output_length = 0U;
        return APP_STATUS_BUFFER_TOO_SMALL;
    }

    memset(output, 0, total);
    WriteHeader(output,
                PROTOCOL_TYPE_REPLY,
                PROTOCOL_WIRE_CONTROL_SIZE,
                tx_sequence,
                boot_id,
                session_id,
                config_id,
                emit_time_us);

    p = &output[PROTOCOL_WIRE_HEADER_SIZE];
    Put32(&p[0], command->request_id);
    Put32(&p[4], (command->opcode != 0U) ? command->opcode : (uint32_t)command->id);
    Put32(&p[8], result_code);
    Put32(&p[12], command->request_id);
    Put32(&p[16], command->value);
    Put32(&p[20], config_id);
    Put32(&p[24], session_id);

    Put16(&output[total - 2U], Crc16CcittFalse(&output[4], total - 6U));
    *output_length = total;
    return APP_STATUS_OK;
}

AppStatus ProtocolService_ParseCommand(const uint8_t *data,
                                       size_t length,
                                       AppControlCommand *command,
                                       size_t *consumed)
{
    size_t offset = 0U;
    const size_t frame_size = PROTOCOL_WIRE_HEADER_SIZE + PROTOCOL_WIRE_CONTROL_SIZE + PROTOCOL_WIRE_CRC_SIZE;

    if ((data == NULL) || (command == NULL) || (consumed == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    *consumed = 0U;
    command->id = APP_COMMAND_NONE;
    command->opcode = 0U;
    command->request_id = 0U;
    command->value = 0U;

    while ((offset + 4U) <= length)
    {
        if (memcmp(&data[offset], kSyncBytes, 4U) != 0)
        {
            offset++;
            continue;
        }
        if ((offset + frame_size) > length)
        {
            break;
        }

        const uint8_t *f = &data[offset];
        uint16_t payload_len = Get16(&f[6]);
        uint16_t expected_crc = Get16(&f[frame_size - 2U]);
        uint16_t actual_crc = Crc16CcittFalse(&f[4], frame_size - 6U);

        if ((f[4] != PROTOCOL_WIRE_VERSION) ||
            (f[5] != PROTOCOL_TYPE_COMMAND) ||
            (payload_len != PROTOCOL_WIRE_CONTROL_SIZE) ||
            (expected_crc != actual_crc))
        {
            offset++;
            continue;
        }

        const uint8_t *body = &f[PROTOCOL_WIRE_HEADER_SIZE];
        uint32_t req_id = Get32(&body[0]);
        uint32_t opcode = Get32(&body[4]);
        uint32_t arg0 = Get32(&body[12]);

        command->boot = Get32(&f[12]);
        command->session = Get32(&f[16]);
        command->config = Get32(&f[20]);
        command->result = 0U;
        command->opcode = opcode;
        command->request_id = req_id;
        command->value = arg0;
        switch (opcode)
        {
            case 2U: command->id = APP_COMMAND_CAPTURE_NEUTRAL; break;
            case 4U: command->id = APP_COMMAND_STOP_STREAM; break;
            case 5U: command->id = APP_COMMAND_START_STREAM; break;
            case 1U:
                if (arg0 != 0U) command->result = 6U;
                break;
            case 3U: /* PC cancels on successful reply. */
            case 8U:
                break;
            default: command->result = 6U; break;
        }

        *consumed = offset + frame_size;
        return APP_STATUS_OK;
    }

    *consumed = offset;
    return APP_STATUS_NOT_INITIALIZED;
}
