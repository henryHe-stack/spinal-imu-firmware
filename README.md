# Spinal IMU Firmware

[![Firmware Build](https://github.com/henryHe-stack/spinal-imu-firmware/actions/workflows/firmware-build.yml/badge.svg)](https://github.com/henryHe-stack/spinal-imu-firmware/actions/workflows/firmware-build.yml)

基于 STM32F103C8T6、FreeRTOS、TCA9548A 和 3 颗 BNO055 的后颈柔性 IMU 采集固件。项目用于颈部 C0、C4、C7 三个区域的姿态采集、无线传输与后续区域相对运动分析。

> 当前状态：`v0.1.0.0` 固件框架基线。任务调度、模块边界和构建链已经建立；BNO055 完整配置、UART DMA 协议、微秒时间戳和颈部姿态算法仍是明确标注的占位接口，尚未经过实物板验证。

## 硬件基线

| 模块 | 当前方案 | 固件接口 |
|---|---|---|
| MCU | STM32F103C8T6，HSE 8 MHz，SYSCLK 72 MHz | STM32 HAL + FreeRTOS |
| IMU | 3 × BNO055，对应 C0、C4、C7 安装区域 | I2C，默认 7 位地址 `0x28` |
| I2C 扩展 | TCA9548A | I2C1，默认 7 位地址 `0x70`，通道 0/1/2 |
| 无线/串口 | HC-05 或兼容 UART 模块 | USART2，115200 8N1 |
| 调试 | SWD | PA13 / PA14 |
| 时间基准 | TIM2，目标 1 MHz 自由运行计数器 | 当前暂时退化为 1 ms RTOS tick |
| 采集节拍 | 目标 50 Hz | 当前由 FreeRTOS 20 ms 软件延时驱动 |

## 架构

```text
CubeMX / HAL / CMSIS / FreeRTOS
              |
             BSP          I2C / UART / Time
              |
           Devices        TCA9548A / BNO055
              |
           Services       Sensor / Protocol / Diagnostics / Algorithm
              |
            Tasks         Acquisition / Communication / Command / Health
              |
         AppBootstrap     Static queues and task creation
```

### FreeRTOS 任务所有权

| 任务 | 优先级 | 责任 |
|---|---:|---|
| `AcquisitionTask` | 4 | 唯一拥有 I2C1、TCA9548A 和 3 颗 BNO055，固定按 C0 -> C4 -> C7 轮询 |
| `CommunicationTask` | 3 | 从采集队列取帧、协议编码、UART 发送 |
| `CommandTask` | 2 | 消费 UART RX 数据、解析命令、向采集任务投递控制请求 |
| `HealthTask` | 1 | 任务心跳、错误计数、队列/栈水位和后续看门狗策略 |

I2C 复用器的“选择通道 + 访问传感器”必须保持为同一任务内的完整操作。其他任务不得直接调用 BNO055、TCA9548A 或 I2C BSP。

## 目录

```text
App/          系统启动与对象装配
BSP/          HAL 外设适配层
Common/       公共状态码和内部数据类型
Config/       任务、队列和业务参数
Devices/      TCA9548A、BNO055 器件驱动
Services/     传感、协议、诊断和算法接口
Tasks/        FreeRTOS 任务入口
Core/         CubeMX 生成代码，仅在 USER CODE 区接入
Drivers/      STM32 HAL / CMSIS
Middlewares/  FreeRTOS
Docs/         架构、协作与工程说明
```

## 构建

### 依赖

- CMake 3.22+
- Ninja
- GNU Arm Embedded Toolchain，提供 `arm-none-eabi-gcc`
- STM32CubeMX 6.15.0，用于修改和重新生成 `.ioc`

本机安装 STM32 VS Code 扩展后，工具通常位于 `%LOCALAPPDATA%\stm32cube\bundles\`。

### 命令

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出文件：`build/Debug/Spinal_tester.elf`。

首次发布基线的编译结果：

| 存储区 | 使用量 | 占比 |
|---|---:|---:|
| RAM | 10,824 B / 20 KB | 52.85% |
| Flash | 29,956 B / 64 KB | 45.71% |

这些数字只代表当前 Debug 构建，不代表最终功能版本或运行期栈余量已经验证。

## CubeMX 再生成规则

1. 使用 `Spinal_tester.ioc` 修改时钟、引脚、DMA 和中间件配置。
2. 业务代码只放在独立分层目录，不写入 `Drivers/` 或 `Middlewares/`。
3. `Core/` 中只修改 `USER CODE` 区。
4. 重新生成后必须检查 `main.c`、`freertos.c`、根 `CMakeLists.txt` 和中断优先级。
5. 重新执行 Debug 构建，并在 PR 中粘贴 RAM/Flash 占用结果。

## 当前占位接口

- `Bno055_Configure()`：复位、工作模式、轴映射、单位、校准参数与回读验证。
- `Bno055_ReadSample()`：四元数、加速度、角速度、温度和校准状态的突发读取。
- `BspUart_ReadAvailable()`：USART2 circular DMA 环形缓冲。
- `ProtocolService_*()`：版本化二进制帧、字节序、CRC、粘包/拆包恢复。
- `BspTime_NowUs()`：TIM2 1 MHz 计数和 16 位溢出扩展。
- `NeckMotionAlgorithm_*()`：中立位、时间对齐和 C0-C4/C4-C7/C0-C7 区域相对姿态。
- `HealthTask_Entry()`：栈/队列水位和独立看门狗投喂条件。

搜索 `TODO(owner)` 可以定位尚未实现的责任点。

## 协作流程

```bash
git clone https://github.com/henryHe-stack/spinal-imu-firmware.git
git switch -c feature/<module>-<short-description>
```

开发完成后推送分支并创建 Pull Request。`main` 只保存可编译、接口一致的基线；不要把未经验证的硬件行为写成已完成结果。

建议模块分工与验收标准见 [CONTRIBUTING.md](CONTRIBUTING.md)，整体边界见 [Docs/ARCHITECTURE.md](Docs/ARCHITECTURE.md)，阶段路线见 [ROADMAP.md](ROADMAP.md)。

## 证据等级

提交和汇报必须注明验证等级：

1. `static`：仅通过代码审查或静态检查。
2. `build`：ARM GCC 编译通过。
3. `bench`：开发板/转接板台架验证通过。
4. `wearable`：柔性颈贴或佩戴实验验证通过。

低等级结果不能被描述成更高等级的完成状态。

## 关键限制

- 三颗 BNO055 采用顺序 I2C 轮询，没有共享硬件采样触发，只能称为软件近似同步。
- 当前时间戳是主控访问时间，不是传感器内部真实采样时刻。
- 三个安装节点只能支持区域相对姿态估计，不能直接推断 C1-C7 每节椎体角度。
- 磁干扰、贴片滑移、柔性桥应变、校准状态和丢包都必须进入实验质量标记。
- 当前工程尚未完成 BNO055、无线链路和整机实物验证。

## 许可

当前仓库用于项目组内部研发，未附带开源许可证。未经项目负责人明确授权，不得对外再分发。
