#ifndef PROTOCOL_SERVICE_H
#define PROTOCOL_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#include "app_status.h"
#include "app_types.h"

/* D0 wire protocol v1 types for MCU Thin Client <-> PC Host Brain. */
#define PROTOCOL_WIRE_VERSION        (1U)
#define PROTOCOL_WIRE_HEADER_SIZE    (32U)
#define PROTOCOL_WIRE_CRC_SIZE       (2U)
#define PROTOCOL_WIRE_SESSION_SIZE   (72U)
#define PROTOCOL_WIRE_RAW_FRAME_SIZE (120U)
#define PROTOCOL_WIRE_CONTROL_SIZE   (28U)

#define PROTOCOL_TYPE_SESSION        (1U)
#define PROTOCOL_TYPE_RAW_FRAME      (6U)
#define PROTOCOL_TYPE_COMMAND        (16U)
#define PROTOCOL_TYPE_REPLY          (17U)

AppStatus ProtocolService_EncodeSessionFrame(uint32_t tx_sequence,
                                             uint32_t boot_id,
                                             uint32_t session_id,
                                             uint32_t config_id,
                                             uint64_t emit_time_us,
                                             uint32_t sample_period_us,
                                             uint8_t *output,
                                             size_t capacity,
                                             size_t *output_length);

AppStatus ProtocolService_EncodeSampleFrame(const AppSampleFrame *frame,
                                            uint32_t tx_sequence,
                                            uint32_t boot_id,
                                            uint32_t session_id,
                                            uint32_t config_id,
                                            uint64_t emit_time_us,
                                            uint32_t sample_period_us,
                                            uint8_t *output,
                                            size_t capacity,
                                            size_t *output_length);

AppStatus ProtocolService_EncodeReplyFrame(const AppControlCommand *command,
                                           uint32_t result_code,
                                           uint32_t tx_sequence,
                                           uint32_t boot_id,
                                           uint32_t session_id,
                                           uint32_t config_id,
                                           uint64_t emit_time_us,
                                           uint8_t *output,
                                           size_t capacity,
                                           size_t *output_length);

AppStatus ProtocolService_ParseCommand(const uint8_t *data,
                                       size_t length,
                                       AppControlCommand *command,
                                       size_t *consumed);

#endif /* PROTOCOL_SERVICE_H */
