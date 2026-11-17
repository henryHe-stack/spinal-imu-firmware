#include "bsp_uart.h"
#include <string.h>

/* Equal IRQ priority (6) serializes DMA/IDLE producers. CommandTask is the
 * sole ring consumer; CommunicationTask is the sole TX owner for both ports. */
static BspUart *ports[APP_PORT_COUNT];
static BspUart *Find(UART_HandleTypeDef *hal)
{
    for (unsigned i = 0; i < APP_PORT_COUNT; ++i)
        if (ports[i] && ports[i]->hal == hal) return ports[i];
    return NULL;
}
AppStatus BspUart_Init(BspUart *self, UART_HandleTypeDef *hal)
{
    if (!self || !hal || !hal->hdmarx || !hal->hdmatx) return APP_STATUS_INVALID_ARGUMENT;
    unsigned slot;
    for (slot = 0; slot < APP_PORT_COUNT; ++slot)
        if (!ports[slot] || ports[slot] == self) break;
    if (slot == APP_PORT_COUNT) return APP_STATUS_INTERNAL_ERROR;
    memset(self, 0, sizeof(*self)); self->hal = hal; ports[slot] = self;
    if (HAL_UARTEx_ReceiveToIdle_DMA(hal, self->rx_dma, sizeof(self->rx_dma)) != HAL_OK)
    { self->rx_restart = true; return APP_STATUS_IO_ERROR; }
    return APP_STATUS_OK;
}
AppStatus BspUart_SendBlocking(BspUart *self, const uint8_t *data, size_t length)
{
    if (!self || !self->hal) return APP_STATUS_NOT_INITIALIZED;
    if (!data || !length || length > UINT16_MAX) return APP_STATUS_INVALID_ARGUMENT;
    if (self->tx_busy) return APP_STATUS_BUSY;
    return HAL_UART_Transmit(self->hal, data, (uint16_t)length, APP_UART_TX_TIMEOUT_MS) == HAL_OK
        ? APP_STATUS_OK : APP_STATUS_IO_ERROR;
}
AppStatus BspUart_SendDma(BspUart *self, const uint8_t *data, size_t length)
{
    if (!self || !self->hal) return APP_STATUS_NOT_INITIALIZED;
    if (!data || !length || length > sizeof(self->tx_data)) return APP_STATUS_INVALID_ARGUMENT;
    /* A missing TX completion interrupt must not wedge the port indefinitely. */
    if (self->tx_busy && (uint32_t)(HAL_GetTick() - self->tx_started_ms) > APP_UART_TX_TIMEOUT_MS)
    { HAL_UART_AbortTransmit(self->hal); self->tx_busy = false; self->errors++; }
    if (self->tx_busy) { self->tx_drops++; return APP_STATUS_BUSY; }
    memcpy(self->tx_data, data, length);
    self->tx_busy = true; self->tx_started_ms = HAL_GetTick();
    if (HAL_UART_Transmit_DMA(self->hal, self->tx_data, (uint16_t)length) != HAL_OK)
    { self->tx_busy = false; self->errors++; return APP_STATUS_IO_ERROR; }
    return APP_STATUS_OK;
}
void BspUart_Service(BspUart *self)
{
    if (!self || !self->hal) return;
    if (self->rx_restart)
    {
        /* Recover in task context; DMA event callbacks never call blocking HAL. */
        HAL_UART_AbortReceive(self->hal);
        self->dma_position = 0; self->rx_tail = self->rx_head;
        self->rx_restart = false;
        if (HAL_UARTEx_ReceiveToIdle_DMA(self->hal, self->rx_dma, sizeof(self->rx_dma)) != HAL_OK)
            self->rx_restart = true;
    }
}
size_t BspUart_ReadAvailable(BspUart *self, uint8_t *data, size_t capacity)
{
    if (!self || !data) return 0;
    size_t count = 0;
    while (count < capacity && self->rx_tail != self->rx_head)
    {
        data[count++] = self->rx_ring[self->rx_tail];
        __DMB(); self->rx_tail = (self->rx_tail + 1U) % APP_UART_RX_RING_SIZE;
    }
    return count;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *hal, uint16_t size)
{
    BspUart *self = Find(hal);
    if (!self || self->rx_restart || size > APP_UART_RX_DMA_SIZE) return;
    uint16_t position = size % APP_UART_RX_DMA_SIZE;
    /* HT/TC/IDLE may repeat an offset; keep HT enabled so a full revolution
     * cannot be confused with zero new bytes. ISR latency must be < half ring. */
    while (self->dma_position != position)
    {
        uint8_t byte = self->rx_dma[self->dma_position];
        uint16_t next = (self->rx_head + 1U) % APP_UART_RX_RING_SIZE;
        if (next == self->rx_tail) self->rx_overflows++;
        else { self->rx_ring[self->rx_head] = byte; __DMB(); self->rx_head = next; }
        self->dma_position = (self->dma_position + 1U) % APP_UART_RX_DMA_SIZE;
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hal)
{ BspUart *self = Find(hal); if (self) self->tx_busy = false; }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *hal)
{ BspUart *self = Find(hal); if (self) { self->errors++; self->rx_restart = true; } }
