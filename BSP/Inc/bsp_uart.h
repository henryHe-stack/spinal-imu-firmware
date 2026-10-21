#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include <stdint.h>

#include "app_status.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    UART_HandleTypeDef *hal;
} BspUart;

AppStatus BspUart_Init(BspUart *self, UART_HandleTypeDef *hal);
AppStatus BspUart_SendBlocking(BspUart *self, const uint8_t *data, size_t length);

/*
 * Placeholder for the future circular-DMA receive path. The command task is
 * the only consumer. Return value is the number of bytes copied to data.
 */
size_t BspUart_ReadAvailable(BspUart *self, uint8_t *data, size_t capacity);

#endif /* BSP_UART_H */
