#ifndef PROTOCOL_SERVICE_H
#define PROTOCOL_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#include "app_status.h"
#include "app_types.h"

AppStatus ProtocolService_EncodeSampleFrame(const AppSampleFrame *frame,
                                            uint8_t *output,
                                            size_t capacity,
                                            size_t *output_length);
AppStatus ProtocolService_ParseCommand(const uint8_t *data,
                                       size_t length,
                                       AppControlCommand *command,
                                       size_t *consumed);

#endif /* PROTOCOL_SERVICE_H */
