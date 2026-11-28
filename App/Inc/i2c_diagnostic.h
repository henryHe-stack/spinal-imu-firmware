#ifndef I2C_DIAGNOSTIC_H
#define I2C_DIAGNOSTIC_H

#include "stm32f1xx_hal.h"

_Noreturn void I2cDiagnostic_Run(I2C_HandleTypeDef *bus, UART_HandleTypeDef *uart);

#endif
