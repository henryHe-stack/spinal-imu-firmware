#include "protocol_service.h"
#include <string.h>

/* No struct casts or native-endian serialization. CRC16/CCITT-FALSE covers
 * version/type/length/payload, excluding the two synchronization bytes. */
uint16_t ProtocolService_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
static void U16(uint8_t **p, uint16_t v) { *(*p)++ = (uint8_t)v; *(*p)++ = (uint8_t)(v >> 8); }
static void U32(uint8_t **p, uint32_t v) { U16(p,(uint16_t)v); U16(p,(uint16_t)(v >> 16)); }
static uint32_t R32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24; }
static uint8_t *Header(uint8_t *out, uint8_t type, uint16_t size)
{
    out[0]=0xA5; out[1]=0x5A; out[2]=APP_WIRE_VERSION; out[3]=type;
    uint8_t *p=out+4; U16(&p,size); return p;
}
static void Finish(uint8_t *out, uint8_t *end, size_t *length)
{ uint16_t crc=ProtocolService_Crc16(out+2,(size_t)(end-out)-2); U16(&end,crc); *length=(size_t)(end-out); }
AppStatus ProtocolService_EncodeSampleFrame(const AppSampleFrame *frame, uint8_t *out, size_t capacity, size_t *length)
{
    if (!frame || !out || !length || frame->sample_count > APP_IMU_NODE_COUNT) return APP_STATUS_INVALID_ARGUMENT;
    *length=0;
    uint16_t payload=5U + frame->sample_count*APP_WIRE_SAMPLE_BYTES;
    if (capacity < (size_t)payload+8U) return APP_STATUS_BUFFER_TOO_SMALL;
    uint8_t *p=Header(out,APP_WIRE_SAMPLE,payload); U32(&p,frame->sequence); *p++=frame->sample_count;
    for (unsigned i=0; i<frame->sample_count; ++i)
    {
        const AppImuSample *s=&frame->samples[i];
        *p++=s->node_id; *p++=s->tca_channel; U16(&p,s->flags);
        U32(&p,s->timestamp_start_us); U32(&p,s->timestamp_end_us); U32(&p,s->timestamp_mid_us);
        U16(&p,(uint16_t)s->quaternion.w); U16(&p,(uint16_t)s->quaternion.x);
        U16(&p,(uint16_t)s->quaternion.y); U16(&p,(uint16_t)s->quaternion.z);
        U16(&p,(uint16_t)s->acceleration.x); U16(&p,(uint16_t)s->acceleration.y); U16(&p,(uint16_t)s->acceleration.z);
        U16(&p,(uint16_t)s->gyroscope.x); U16(&p,(uint16_t)s->gyroscope.y); U16(&p,(uint16_t)s->gyroscope.z);
        *p++=(uint8_t)s->temperature_c; *p++=s->calibration_status; *p++=s->system_status; *p++=s->system_error;
    }
    Finish(out,p,length); return APP_STATUS_OK;
}
AppStatus ProtocolService_EncodeReply(const AppCommandReply *reply, uint8_t *out, size_t capacity, size_t *length)
{
    if (!reply || !out || !length) return APP_STATUS_INVALID_ARGUMENT;
    *length=0; if (capacity<18) return APP_STATUS_BUFFER_TOO_SMALL;
    uint8_t *p=Header(out,APP_WIRE_REPLY,10); U32(&p,reply->request_id);
    *p++=reply->command; *p++=reply->status; U32(&p,reply->value); Finish(out,p,length);
    return APP_STATUS_OK;
}
AppStatus ProtocolService_ParseCommand(const uint8_t *data, size_t length, AppControlCommand *command, size_t *consumed)
{
    if (!data || !command || !consumed) return APP_STATUS_INVALID_ARGUMENT;
    *consumed=0;
    if (length<6) return APP_STATUS_BUSY;
    if (data[0]!=0xA5 || data[1]!=0x5A || data[2]!=APP_WIRE_VERSION || data[3]!=APP_WIRE_COMMAND || data[4]!=9 || data[5]!=0)
    { *consumed=1; return APP_STATUS_INVALID_ARGUMENT; }
    if (length<17) return APP_STATUS_BUSY;
    *consumed=17;
    if (ProtocolService_Crc16(data+2,13) != ((uint16_t)data[15] | (uint16_t)data[16]<<8)) return APP_STATUS_CRC_ERROR;
    command->request_id=R32(data+6); command->id=(AppCommandId)data[10]; command->value=R32(data+11);
    command->source=APP_PORT_PC; /* CommandTask supplies physical source. */
    return APP_STATUS_OK;
}
bool ProtocolService_FeedCommand(ProtocolParser *parser, uint8_t byte, AppControlCommand *command)
{
    if (!parser || !command) return false;
    if (parser->length == sizeof(parser->bytes)) { parser->length=0; parser->errors++; }
    parser->bytes[parser->length++]=byte;
    while (parser->length)
    {
        if (parser->bytes[0]!=0xA5 || (parser->length>1 && parser->bytes[1]!=0x5A)) goto discard;
        if (parser->length<6) return false;
        size_t consumed;
        AppStatus result=ProtocolService_ParseCommand(parser->bytes,parser->length,command,&consumed);
        if (result==APP_STATUS_BUSY) return false;
        if (result==APP_STATUS_OK)
        { parser->length=0; return true; }
        parser->errors++;
        /* Shift by one, not the declared length: recover a valid embedded frame. */
    discard:
        memmove(parser->bytes,parser->bytes+1,--parser->length);
    }
    return false;
}
