#ifndef APP_BOOTSTRAP_H
#define APP_BOOTSTRAP_H

#include "app_status.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    I2C_HandleTypeDef *i2c;
    UART_HandleTypeDef *uart;
    TIM_HandleTypeDef *timestamp_timer;
} AppBootstrapConfig;

/*
 * Creates all application-owned static queues and tasks.
 * Call once from the CubeMX default task after the scheduler has started.
 * This function does not take ownership of HAL handles.
 */
AppStatus AppBootstrap_Start(const AppBootstrapConfig *config);

#endif /* APP_BOOTSTRAP_H */
