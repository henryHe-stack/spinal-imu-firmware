#ifndef NECK_UART_RX_H
#define NECK_UART_RX_H
#include <stdint.h>
int neck_uart_rx_start(void);
/* Single algorithm-task consumer; -1 means overflow/error, discard parser prefix. */
int neck_uart_rx_pop(uint8_t *byte);
#endif
