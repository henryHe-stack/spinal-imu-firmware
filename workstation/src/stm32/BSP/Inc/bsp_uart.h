#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_status.h"
#include "stm32f1xx_hal.h"

#define BSP_UART_RX_RING_SIZE (256U)

typedef struct
{
    UART_HandleTypeDef *hal;
    uint8_t rx_ring[BSP_UART_RX_RING_SIZE];
    size_t rx_read_index;
    bool dma_rx_active;
} BspUart;

AppStatus BspUart_Init(BspUart *self, UART_HandleTypeDef *hal);
AppStatus BspUart_SendBlocking(BspUart *self, const uint8_t *data, size_t length);
size_t BspUart_ReadAvailable(BspUart *self, uint8_t *data, size_t capacity);

#endif /* BSP_UART_H */
