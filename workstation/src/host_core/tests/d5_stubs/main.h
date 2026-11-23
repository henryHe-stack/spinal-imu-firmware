#ifndef D5_TEST_MAIN_H
#define D5_TEST_MAIN_H
#include <stdint.h>
typedef int UART_HandleTypeDef;
extern UART_HandleTypeDef huart2;
#define HAL_OK 0
int HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
void Error_Handler(void);
#endif
