#include "bsp_uart.h"

#include <string.h>

#include "app_config.h"

AppStatus BspUart_Init(BspUart *self, UART_HandleTypeDef *hal)
{
    if ((self == NULL) || (hal == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    memset(self, 0, sizeof(*self));
    self->hal = hal;

    if (hal->hdmarx != NULL)
    {
        if (HAL_UART_Receive_DMA(hal, self->rx_ring, BSP_UART_RX_RING_SIZE) == HAL_OK)
        {
            self->dma_rx_active = true;
        }
    }

    return APP_STATUS_OK;
}

AppStatus BspUart_SendBlocking(BspUart *self, const uint8_t *data, size_t length)
{
    HAL_StatusTypeDef hal_status;

    if ((self == NULL) || (self->hal == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    hal_status = HAL_UART_Transmit(self->hal,
                                   (uint8_t *)data,
                                   (uint16_t)length,
                                   APP_UART_TX_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        return APP_STATUS_OK;
    }
    if (hal_status == HAL_BUSY)
    {
        return APP_STATUS_BUSY;
    }
    if (hal_status == HAL_TIMEOUT)
    {
        return APP_STATUS_TIMEOUT;
    }
    return APP_STATUS_IO_ERROR;
}

size_t BspUart_ReadAvailable(BspUart *self, uint8_t *data, size_t capacity)
{
    size_t write_index;
    size_t remaining_dma;
    size_t copied = 0U;

    if ((self == NULL) || (self->hal == NULL) || (data == NULL) || (capacity == 0U))
    {
        return 0U;
    }

    if (!self->dma_rx_active || (self->hal->hdmarx == NULL))
    {
        return 0U;
    }

    remaining_dma = (size_t)__HAL_DMA_GET_COUNTER(self->hal->hdmarx);
    if ((remaining_dma == 0U) || (remaining_dma > BSP_UART_RX_RING_SIZE))
    {
        write_index = 0U;
    }
    else
    {
        write_index = BSP_UART_RX_RING_SIZE - remaining_dma;
    }

    while ((self->rx_read_index != write_index) && (copied < capacity))
    {
        data[copied++] = self->rx_ring[self->rx_read_index];
        self->rx_read_index = (self->rx_read_index + 1U) % BSP_UART_RX_RING_SIZE;
    }

    return copied;
}
