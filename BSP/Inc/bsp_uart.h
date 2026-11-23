#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_status.h"
#include "stm32f1xx_hal.h"
#include "app_types.h"
#include "app_config.h"

typedef struct
{
    UART_HandleTypeDef *hal;
    uint8_t rx_dma[APP_UART_RX_DMA_SIZE];
    uint8_t rx_ring[APP_UART_RX_RING_SIZE];
    uint8_t tx_data[APP_PROTOCOL_MAX_FRAME_SIZE];
    volatile uint16_t rx_head, rx_tail;
    uint16_t dma_position;
    volatile bool tx_busy, rx_restart;
    volatile uint32_t rx_overflows, errors, tx_drops;
    uint32_t tx_started_ms;
} BspUart;

AppStatus BspUart_Init(BspUart *self, UART_HandleTypeDef *hal);
AppStatus BspUart_SendBlocking(BspUart *self, const uint8_t *data, size_t length);
/* Single TX owner: CommunicationTask. Copies into persistent DMA storage. */
AppStatus BspUart_SendDma(BspUart *self, const uint8_t *data, size_t length);
void BspUart_Service(BspUart *self);

/*
 * CommandTask is the sole consumer; serialized DMA and UART ISRs produce.
 */
size_t BspUart_ReadAvailable(BspUart *self, uint8_t *data, size_t capacity);

#endif /* BSP_UART_H */
