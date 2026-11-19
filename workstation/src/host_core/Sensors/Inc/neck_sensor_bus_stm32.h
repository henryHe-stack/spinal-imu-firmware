#ifndef NECK_SENSOR_BUS_STM32_H
#define NECK_SENSOR_BUS_STM32_H

#include "neck_bno055.h"
#include "stm32f1xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint32_t timeout_ms;
} NeckStm32SensorBusContext;

/* Pass &hi2c1 and a finite timeout. HAL address shifting is contained here.
 * Not ISR-safe. The acquisition task must be the sole I2C bus owner.
 */
NeckI2cBus neck_stm32_sensor_bus(NeckStm32SensorBusContext *context);

#endif
