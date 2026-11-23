#include "bsp_i2c.h"
#include <stdbool.h>

#include "app_config.h"

AppStatus BspI2c_Recover(BspI2c *self)
{
    if (!self || !self->hal || self->hal->Instance != I2C1) return APP_STATUS_INVALID_ARGUMENT;
    /* Only AcquisitionTask calls this. Open-drain pulses release the lines;
     * a physically stuck slave may still require a hardware power cycle. */
    HAL_I2C_DeInit(self->hal);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7; gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_SET); HAL_GPIO_Init(GPIOB, &gpio);
    for (unsigned i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; ++i)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); HAL_Delay(1);
    }
    bool scl_free = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    bool sda_free = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET;
    __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET();
    if (HAL_I2C_Init(self->hal) != HAL_OK || !scl_free || !sda_free) return APP_STATUS_IO_ERROR;
    return APP_STATUS_OK;
}

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
