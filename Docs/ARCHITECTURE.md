# Spinal Tester 固件架构

## 1. 当前边界

本工程是 STM32F103C8T6 + FreeRTOS 的三节点 BNO055 采集系统。v0.1.0.1 已实现传感、时间、双串口协议和维护命令，MCU颈部运动算法保持占位。当前验证到ARM构建与主机模拟，未验证实板行为。

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
| CommunicationTask | 3 | sample_queue 单一消费者，同一帧广播到两口；reply_queue 向来源口优先应答 | 不得访问传感器；不得直接发送 C 结构体内存 |
| CommandTask | 2 | 消费 UART RX 字节、解析命令、投递 command_queue | 不得直接改变传感器寄存器 |
| HealthTask | 1 | 心跳、计数器、栈水位、LED/STATE和默认关闭的看门狗策略 | 不得消费业务队列或访问 I2C |

所有业务任务和队列静态分配。CubeMX 的 `defaultTask` 只负责调用 `AppBootstrap_Start()`，成功后立即删除自身。

## 3. 数据约定

- `AppSampleFrame` 是 MCU 内部数据模型，不是线上协议。
- 每个节点保留 `timestamp_start_us`、`timestamp_end_us` 和 `timestamp_mid_us`，用于描述顺序轮询产生的节点间时间偏移。
- `BspTime_NowUs()` 使用TIM2 1 MHz计数和16位溢出中断扩展；采样读取不设置TIME_COARSE。32位约71.6分钟回绕，采样中点按无符号模运算计算。时间分辨率不等于测量准确度。
- 三个 BNO055 没有共享硬件采样触发，因此只能称为软件近似同步，不能称为硬同步。
- 只有三个安装点时，算法的可辩护输出是 C0-C4、C4-C7 和 C0-C7 区域相对姿态，不是每一节椎体的独立角度。

## 4. 数据流与维护命令

```text
I2C1 → TCA通道 → BNO分步初始化/读取 → AppSampleFrame
  → sample_queue → CommunicationTask → ProtocolService → PC UART1 / HC05 UART2

UART1/2 RX DMA → 各自环形缓冲 → CommandTask独立解析器
  → command_queue → AcquisitionTask执行 → reply_queue → CommunicationTask → 来源口
```

采集任务默认20 ms固定节拍，始终推进三个节点的初始化/读取；STOP只关闭上传。命令每轮限定处理数量，失败节点保留槽位与错误标记。
SAVE只在停止上传时执行，校准记录写Flash末两页；采集等待锚点在维护后重置。
UART TX复制入端口私有持久缓冲，无异步引用栈内帧；RX ISR只搬运，不调用FreeRTOS队列、不操作I2C。
诊断快照在任务临界区更新/读取。无自动校准擦写，不实现原理图不存在的电池ADC和充电状态脚。

## 5. 保留接口与配置边界

`NeckMotionAlgorithm_*()` 与 MCU中立位命令继续返回NOT_IMPLEMENTED。网页的中立位/相对角仅用于可视化。
TIM3配置为50 Hz但未启动；采集节拍由vTaskDelayUntil驱动。IWDG默认关闭，完整健康快照未定期上报网页。
业务接入在USER CODE区；外设配置同步`.ioc`与生成区，尚未通过CubeMX GUI重新生成比对。再生成后尤其检查链接脚本的62 KB Flash边界。

详细责任划分、引脚/状态机、协议和验收见 [DRIVER_BRINGUP.md](DRIVER_BRINGUP.md)、[WIRE_PROTOCOL.md](WIRE_PROTOCOL.md)、[TESTING.md](TESTING.md)。
