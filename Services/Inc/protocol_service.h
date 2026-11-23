#ifndef PROTOCOL_SERVICE_H
#define PROTOCOL_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#include "app_status.h"
#include "app_types.h"

#define APP_WIRE_VERSION 1U
#define APP_WIRE_SAMPLE  1U
#define APP_WIRE_COMMAND 16U
#define APP_WIRE_REPLY   17U
#define APP_WIRE_SAMPLE_BYTES 40U
#define APP_WIRE_MAX_BYTES 192U
typedef struct { uint8_t bytes[APP_WIRE_MAX_BYTES]; uint16_t length; uint32_t errors; } ProtocolParser;
uint16_t ProtocolService_Crc16(const uint8_t *data, size_t length);
/* Feed one byte at a time; preserve an independent parser for each port. */
bool ProtocolService_FeedCommand(ProtocolParser *parser, uint8_t byte, AppControlCommand *command);
AppStatus ProtocolService_EncodeReply(const AppCommandReply *reply, uint8_t *output, size_t capacity, size_t *length);

AppStatus ProtocolService_EncodeSampleFrame(const AppSampleFrame *frame,
                                            uint8_t *output,
                                            size_t capacity,
                                            size_t *output_length);
AppStatus ProtocolService_ParseCommand(const uint8_t *data,
                                       size_t length,
                                       AppControlCommand *command,
                                       size_t *consumed);

#endif /* PROTOCOL_SERVICE_H */
