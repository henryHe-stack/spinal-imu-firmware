#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Application timing. This is a software schedule, not hardware-synchronous sampling. */
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

/* FreeRTOS stack sizes are expressed in StackType_t entries, not bytes. */
#define APP_ACQUISITION_STACK_WORDS      (256U)
#define APP_COMMUNICATION_STACK_WORDS    (256U)
#define APP_COMMAND_STACK_WORDS          (192U)
#define APP_HEALTH_STACK_WORDS           (160U)

#define APP_ACQUISITION_TASK_PRIORITY    (4U)
#define APP_COMMUNICATION_TASK_PRIORITY  (3U)
#define APP_COMMAND_TASK_PRIORITY        (2U)
#define APP_HEALTH_TASK_PRIORITY         (1U)

#define APP_I2C_TIMEOUT_MS               (20U)
#define APP_UART_TX_TIMEOUT_MS           (100U)

#endif /* APP_CONFIG_H */
