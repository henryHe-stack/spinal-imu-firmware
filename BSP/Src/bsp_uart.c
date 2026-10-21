#include "bsp_uart.h"

#include "app_config.h"

AppStatus BspUart_Init(BspUart *self, UART_HandleTypeDef *hal)
{
    if ((self == NULL) || (hal == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    self->hal = hal;
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

    /*
     * Bring-up implementation only. Replace with UART TX DMA plus task
     * notification after the on-wire protocol has been frozen.
     */
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
    (void)self;
    (void)data;
    (void)capacity;

    /* TODO(COMM owner): expose bytes from the USART2 circular DMA ring. */
    return 0U;
}
