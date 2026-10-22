# Spinal Tester 固件架构

## 1. 当前边界

本工程是 STM32F103C8T6 + FreeRTOS 的三节点 BNO055 采集框架。当前版本的目标是固定模块边界、任务所有权和多人协作接口，不代表 BNO055 初始化、无线协议、微秒时间戳或颈部姿态算法已经完成。

当前可编译路径：

```text
CubeMX/HAL
  -> BSP (I2C/UART/time abstraction)
  -> Devices (TCA9548A/BNO055)
  -> Services (sensor/protocol/diagnostics/algorithm)
  -> Tasks (acquisition/communication/command/health)
  -> AppBootstrap
```

## 2. 任务与资源所有权

| 任务 | 优先级 | 唯一职责 | 禁止事项 |
|---|---:|---|---|
| AcquisitionTask | 4 | 独占 I2C1、TCA9548A、3 个 BNO055；固定 C0 -> C4 -> C7 轮询；生成内部采集帧 | 其他任务不得直接读写 I2C 或切换 TCA 通道 |
| CommunicationTask | 3 | 从 sample_queue 取帧、协议编码、UART 发送 | 不得访问传感器；不得直接发送 C 结构体内存 |
| CommandTask | 2 | 消费 UART RX 字节、解析命令、投递 command_queue | 不得直接改变传感器寄存器 |
| HealthTask | 1 | 心跳、计数器、队列/栈水位和看门狗策略 | 不得消费业务队列或访问 I2C |

所有业务任务和队列静态分配。CubeMX 的 `defaultTask` 只负责调用 `AppBootstrap_Start()`，成功后立即删除自身。

## 3. 数据约定

- `AppSampleFrame` 是 MCU 内部数据模型，不是线上协议。
- 每个节点保留 `timestamp_start_us`、`timestamp_end_us` 和 `timestamp_mid_us`，用于描述顺序轮询产生的节点间时间偏移。
- 当前 `BspTime_NowUs()` 是 1 ms RTOS tick 的退化实现，并设置 `APP_SAMPLE_FLAG_TIME_COARSE`；完成 TIM2 1 MHz 溢出扩展后才能称为微秒时间戳。
- 三个 BNO055 没有共享硬件采样触发，因此只能称为软件近似同步，不能称为硬同步。
- 只有三个安装点时，算法的可辩护输出是 C0-C4、C4-C7 和 C0-C7 区域相对姿态，不是每一节椎体的独立角度。

## 4. 占位接口

以下函数有明确的 `TODO(owner)`，目前会返回 `APP_STATUS_NOT_IMPLEMENTED` 或空结果：

- `Bno055_Configure()`：复位、工作模式、单位、轴映射、校准参数和回读验证。
- `Bno055_ReadSample()`：寄存器突发读取及定点数据解码。
- `ProtocolService_EncodeSampleFrame()`：版本化二进制帧、字节序和 CRC。
- `ProtocolService_ParseCommand()`：流式命令解析、粘包/拆包和错误恢复。
- `BspUart_ReadAvailable()`：USART2 RX circular DMA 环形缓冲。
- `NeckMotionAlgorithm_*()`：中立位、时间对齐和区域相对姿态算法。
- `HealthTask_Entry()`：看门狗投喂条件、栈水位与队列水位策略。

## 5. CubeMX 配置缺口

当前生成代码中：

- TIM2 的 `Prescaler = 0`，尚不是计划中的 1 MHz 计数器。
- TIM3 只有 `Prescaler = 7199`，`Period = 65535`，尚未形成 50 Hz 硬件节拍。
- USART2 DMA 已生成，但业务层暂时使用阻塞 TX，RX circular DMA 的消费逻辑仍是占位。

这些状态必须在硬件 bring-up 和逻辑分析仪测量后更新，不应在汇报中写成已经验证。
