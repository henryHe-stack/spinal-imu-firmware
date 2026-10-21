#include "protocol_service.h"

AppStatus ProtocolService_EncodeSampleFrame(const AppSampleFrame *frame,
                                            uint8_t *output,
                                            size_t capacity,
                                            size_t *output_length)
{
    if ((frame == NULL) || (output == NULL) || (output_length == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    (void)capacity;
    *output_length = 0U;

    /*
     * TODO(PROTOCOL owner): serialize fields explicitly using a frozen byte
     * order and add version, payload length, frame sequence and CRC. Never
     * memcpy AppSampleFrame because compiler padding is not a wire contract.
     */
    return APP_STATUS_NOT_IMPLEMENTED;
}

AppStatus ProtocolService_ParseCommand(const uint8_t *data,
                                       size_t length,
                                       AppControlCommand *command,
                                       size_t *consumed)
{
    if ((data == NULL) || (command == NULL) || (consumed == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    (void)length;
    *consumed = 0U;
    command->id = APP_COMMAND_NONE;
    command->request_id = 0U;
    command->value = 0U;

    /* TODO(PROTOCOL owner): add streaming parser, framing recovery and CRC checks. */
    return APP_STATUS_NOT_IMPLEMENTED;
}
