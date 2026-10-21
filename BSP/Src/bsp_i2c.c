#include "bsp_i2c.h"

#include "app_config.h"

static AppStatus BspI2c_FromHalStatus(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:
            return APP_STATUS_OK;
        case HAL_BUSY:
            return APP_STATUS_BUSY;
        case HAL_TIMEOUT:
            return APP_STATUS_TIMEOUT;
        case HAL_ERROR:
        default:
            return APP_STATUS_IO_ERROR;
    }
}

AppStatus BspI2c_Init(BspI2c *self, I2C_HandleTypeDef *hal)
{
    if ((self == NULL) || (hal == NULL))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    self->hal = hal;
    return APP_STATUS_OK;
}

AppStatus BspI2c_IsReady(BspI2c *self, uint8_t address_7bit)
{
    if ((self == NULL) || (self->hal == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }

    return BspI2c_FromHalStatus(HAL_I2C_IsDeviceReady(self->hal,
                                                      (uint16_t)(address_7bit << 1U),
                                                      2U,
                                                      APP_I2C_TIMEOUT_MS));
}

AppStatus BspI2c_MasterWrite(BspI2c *self,
                            uint8_t address_7bit,
                            const uint8_t *data,
                            size_t length)
{
    if ((self == NULL) || (self->hal == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    return BspI2c_FromHalStatus(HAL_I2C_Master_Transmit(self->hal,
                                                        (uint16_t)(address_7bit << 1U),
                                                        (uint8_t *)data,
                                                        (uint16_t)length,
                                                        APP_I2C_TIMEOUT_MS));
}

AppStatus BspI2c_ReadRegister(BspI2c *self,
                             uint8_t address_7bit,
                             uint8_t reg,
                             uint8_t *data,
                             size_t length)
{
    if ((self == NULL) || (self->hal == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    return BspI2c_FromHalStatus(HAL_I2C_Mem_Read(self->hal,
                                                 (uint16_t)(address_7bit << 1U),
                                                 reg,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 data,
                                                 (uint16_t)length,
                                                 APP_I2C_TIMEOUT_MS));
}

AppStatus BspI2c_WriteRegister(BspI2c *self,
                              uint8_t address_7bit,
                              uint8_t reg,
                              const uint8_t *data,
                              size_t length)
{
    if ((self == NULL) || (self->hal == NULL))
    {
        return APP_STATUS_NOT_INITIALIZED;
    }
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX))
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    return BspI2c_FromHalStatus(HAL_I2C_Mem_Write(self->hal,
                                                  (uint16_t)(address_7bit << 1U),
                                                  reg,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  (uint8_t *)data,
                                                  (uint16_t)length,
                                                  APP_I2C_TIMEOUT_MS));
}
