#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stddef.h>
#include <stdint.h>

#include "app_status.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    I2C_HandleTypeDef *hal;
} BspI2c;

AppStatus BspI2c_Init(BspI2c *self, I2C_HandleTypeDef *hal);
AppStatus BspI2c_IsReady(BspI2c *self, uint8_t address_7bit);
AppStatus BspI2c_MasterWrite(BspI2c *self,
                            uint8_t address_7bit,
                            const uint8_t *data,
                            size_t length);
AppStatus BspI2c_ReadRegister(BspI2c *self,
                             uint8_t address_7bit,
                             uint8_t reg,
                             uint8_t *data,
                             size_t length);
AppStatus BspI2c_WriteRegister(BspI2c *self,
                              uint8_t address_7bit,
                              uint8_t reg,
                              const uint8_t *data,
                              size_t length);

#endif /* BSP_I2C_H */
