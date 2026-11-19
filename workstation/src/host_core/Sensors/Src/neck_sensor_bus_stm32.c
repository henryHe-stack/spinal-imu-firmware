#include "neck_sensor_bus_stm32.h"
#include "FreeRTOS.h"
#include "task.h"

static int stm32_write(void *opaque, uint8_t address7,
                       const uint8_t *data, size_t length)
{
    NeckStm32SensorBusContext *context = (NeckStm32SensorBusContext *)opaque;
    if (context == NULL || context->i2c == NULL || data == NULL ||
        length == 0u || length > 65535u || context->timeout_ms == 0u)
        return -1;
    /* Blocking Master_Transmit generates STOP; needed before mux takes effect. */
    return HAL_I2C_Master_Transmit(context->i2c, (uint16_t)address7 << 1,
                                   (uint8_t *)data, (uint16_t)length,
                                   context->timeout_ms) == HAL_OK ? 0 : -1;
}

static int stm32_read_register(void *opaque, uint8_t address7, uint8_t reg,
                               uint8_t *data, size_t length)
{
    NeckStm32SensorBusContext *context = (NeckStm32SensorBusContext *)opaque;
    if (context == NULL || context->i2c == NULL || data == NULL ||
        length == 0u || length > 65535u || context->timeout_ms == 0u)
        return -1;
    /* F1 HAL memory-read uses a repeated START for the BNO055 data phase. */
    return HAL_I2C_Mem_Read(context->i2c, (uint16_t)address7 << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, (uint16_t)length,
                            context->timeout_ms) == HAL_OK ? 0 : -1;
}

static void stm32_delay_ms(void *opaque, uint32_t milliseconds)
{
    (void)opaque;
    /* Start-up/config only. Yield if the scheduler is already running. */
    if (milliseconds == 0u) return;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        vTaskDelay(pdMS_TO_TICKS(milliseconds));
    else
        HAL_Delay(milliseconds);
}

NeckI2cBus neck_stm32_sensor_bus(NeckStm32SensorBusContext *context)
{
    NeckI2cBus bus;
    bus.context = context;
    bus.write = stm32_write;
    bus.read_register = stm32_read_register;
    bus.delay_ms = stm32_delay_ms;
    return bus;
}
