#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* MCU polling schedule, not simultaneous BNO055 hardware sampling. */
#define APP_ACQUISITION_PERIOD_MS        (20U)
#define APP_HEALTH_PERIOD_MS             (1000U)

/* TCA9548A and BNO055 use 7-bit I2C addresses in application code. */
#define APP_TCA9548A_ADDRESS_7BIT        (0x70U)
#define APP_BNO055_ADDRESS_7BIT          (0x28U)
#define APP_BNO055_CHIP_ID               (0xA0U)

#define APP_IMU_C0_TCA_CHANNEL           (0U)
#define APP_IMU_C4_TCA_CHANNEL           (1U)
#define APP_IMU_C7_TCA_CHANNEL           (2U)

#define APP_SAMPLE_QUEUE_LENGTH          (4U)
#define APP_COMMAND_QUEUE_LENGTH         (4U)
#define APP_PROTOCOL_MAX_FRAME_SIZE      (192U)
#define APP_REPLY_QUEUE_LENGTH           (8U)
#define APP_UART_RX_DMA_SIZE             (128U)
#define APP_UART_RX_RING_SIZE            (512U)
#define APP_PC_BAUD_RATE                 (115200U)
#define APP_BT_BAUD_RATE                 (115200U)
#define APP_BT_STREAM_ENABLED            (1U)
/* NDOF uses magnetometer: evaluate magnetic interference in the actual seat. */
#define APP_BNO055_OPERATION_MODE        (0x0CU)
#define APP_BNO055_EXTERNAL_CRYSTAL      (1U)
#define APP_BNO055_AXIS_MAP              (0x24U)
#define APP_BNO055_AXIS_SIGN             (0x00U)
#define APP_SENSOR_RETRY_MS              (1500U)
/* Enable after bench startup/recovery validation; IWDG cannot be stopped. */
#define APP_ENABLE_WATCHDOG              (0U)

/* FreeRTOS stack sizes are expressed in StackType_t entries, not bytes. */
#define APP_ACQUISITION_STACK_WORDS      (384U)
#define APP_COMMUNICATION_STACK_WORDS    (320U)
#define APP_COMMAND_STACK_WORDS          (256U)
#define APP_HEALTH_STACK_WORDS           (160U)

#define APP_ACQUISITION_TASK_PRIORITY    (4U)
#define APP_COMMUNICATION_TASK_PRIORITY  (3U)
#define APP_COMMAND_TASK_PRIORITY        (2U)
#define APP_HEALTH_TASK_PRIORITY         (1U)

#define APP_I2C_TIMEOUT_MS               (20U)
#define APP_UART_TX_TIMEOUT_MS           (100U)

#endif /* APP_CONFIG_H */
