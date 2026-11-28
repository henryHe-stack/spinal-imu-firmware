#include "i2c_diagnostic.h"
#include "app_config.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

static void Log(UART_HandleTypeDef *uart, const char *format, ...)
{
    char text[128];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (length > 0)
    {
        if ((size_t)length >= sizeof(text)) length = sizeof(text) - 1U;
        (void)HAL_UART_Transmit(uart, (uint8_t *)text, (uint16_t)length, 50U);
    }
}

static const char *StatusName(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK: return "OK";
        case HAL_BUSY: return "BUSY";
        case HAL_TIMEOUT: return "TIMEOUT";
        default: return "ERROR";
    }
}

static HAL_StatusTypeDef Probe(I2C_HandleTypeDef *bus, uint8_t address)
{
    return HAL_I2C_IsDeviceReady(bus, (uint16_t)(address << 1U), 2U, APP_I2C_TIMEOUT_MS);
}

static bool SetMux(I2C_HandleTypeDef *bus, UART_HandleTypeDef *uart, uint8_t mask)
{
    const uint16_t address = APP_TCA9548A_ADDRESS_7BIT << 1U;
    uint8_t actual = 0U;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(bus, address, &mask, 1U, APP_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        Log(uart, "MUX mask=0x%02X WRITE %s\r\n", (unsigned)mask, StatusName(status));
        return false;
    }
    status = HAL_I2C_Master_Receive(bus, address, &actual, 1U, APP_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        Log(uart, "MUX mask=0x%02X READBACK %s\r\n", (unsigned)mask, StatusName(status));
        return false;
    }
    Log(uart, "MUX mask=0x%02X read=0x%02X %s\r\n",
        (unsigned)mask, (unsigned)actual, actual == mask ? "OK" : "MISMATCH");
    return actual == mask;
}

static void ProbeImu(I2C_HandleTypeDef *bus, UART_HandleTypeDef *uart, unsigned ch, uint8_t address)
{
    HAL_StatusTypeDef status = Probe(bus, address);
    if (status != HAL_OK)
    {
        Log(uart, "CH%u addr=0x%02X PROBE %s\r\n", ch, (unsigned)address, StatusName(status));
        return;
    }
    uint8_t id = 0U;
    status = HAL_I2C_Mem_Read(bus, (uint16_t)(address << 1U), 0x00U, I2C_MEMADD_SIZE_8BIT,
                             &id, 1U, APP_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        Log(uart, "CH%u addr=0x%02X ACK ID_READ %s\r\n", ch, (unsigned)address, StatusName(status));
        return;
    }
    Log(uart, "CH%u addr=0x%02X ACK CHIP_ID=0x%02X %s\r\n", ch, (unsigned)address,
        (unsigned)id, id == APP_BNO055_CHIP_ID ? "OK" : "UNEXPECTED_ID");
}

_Noreturn void I2cDiagnostic_Run(I2C_HandleTypeDef *bus, UART_HandleTypeDef *uart)
{
    /* Let the sensors finish cold boot; leave their configuration untouched. */
    HAL_Delay(1000U);
    for (;;)
    {
        Log(uart, "I2C_DIAG I2C1 PB6/PB7 SCL=%u SDA=%u BUSY=%u\r\n",
            (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
            (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7),
            (unsigned)(__HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) != RESET));
        HAL_StatusTypeDef status = Probe(bus, APP_TCA9548A_ADDRESS_7BIT);
        Log(uart, "TCA addr=0x%02X PROBE %s\r\n", (unsigned)APP_TCA9548A_ADDRESS_7BIT, StatusName(status));
        if (status != HAL_OK)
        {
            /* Probe alternate straps without writing to unverified addresses. */
            bool found = false;
            for (uint8_t addr = 0x70U; addr <= 0x77U; ++addr)
            {
                if (addr != APP_TCA9548A_ADDRESS_7BIT && Probe(bus, addr) == HAL_OK)
                {
                    Log(uart, "MUX_RANGE ACK addr=0x%02X\r\n", (unsigned)addr);
                    found = true;
                }
            }
            if (!found) Log(uart, "MUX_RANGE no alternate ACK\r\n");
        }
        else if (SetMux(bus, uart, 0U))
        {
            for (unsigned ch = 0; ch < 3U; ++ch)
            {
                Log(uart, "CH%u SELECT\r\n", ch);
                if (!SetMux(bus, uart, (uint8_t)(1U << ch))) break;
                ProbeImu(bus, uart, ch, 0x28U);
                ProbeImu(bus, uart, ch, 0x29U);
            }
            (void)SetMux(bus, uart, 0U);
        }
        Log(uart, "END I2C_DIAG\r\n\r\n");
        HAL_Delay(2000U);
    }
}
