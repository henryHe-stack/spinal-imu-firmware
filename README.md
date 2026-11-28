# Spinal IMU System

[![Firmware Build](https://github.com/henryHe-stack/spinal-imu-firmware/actions/workflows/firmware-build.yml/badge.svg)](https://github.com/henryHe-stack/spinal-imu-firmware/actions/workflows/firmware-build.yml)

基于 STM32F103C8T6、FreeRTOS、TCA9548A 和 3 颗 BNO055 的后颈柔性 IMU 采集固件与浏览器监测上位机。项目用于颈部 C0、C4、C7 三个区域的姿态采集、数据传输、可视化与区域相对运动分析。

> 当前里程碑：`v0.1.0.3`。STM32 默认通过 TCA9548A 采集三颗 BNO055，允许在单节点有效时输出 A5 5A v1 数据；MCU 颈部运动算法仍为占位。

浏览器上位机支持串口半帧重同步、节点姿态、曲线与 S06 分段角预览。Windows 工作站的 D0 正式测量与 A5 5A 预览是不同模式；使用新版桌面预览须从源码重新构建。本里程碑不代表实机整套验收。

## 硬件基线

| 模块 | 当前方案 | 固件接口 |
|---|---|---|
| MCU | STM32F103C8T6，HSE 8 MHz，SYSCLK 72 MHz | STM32 HAL + FreeRTOS |
| IMU | 3 × BNO055，对应 C0、C4、C7 安装区域 | I2C，默认 7 位地址 `0x28` |
| I2C 扩展 | TCA9548A | I2C1，默认 7 位地址 `0x70`，通道 0/1/2 |
| PC 串口 | 外部 3.3 V TTL USB-UART 转接器 | USART1 PA9/PA10，115200 8N1，RX/TX DMA |
| 无线串口 | HC-05 经典蓝牙透传 | USART2 PA2/PA3，115200 8N1，RX/TX DMA；PA4 STATE |
| 调试 | SWD | PA13 / PA14 |
| 时间基准 | TIM2，1 MHz 自由运行计数器 | 16 位中断扩展至 uint32 μs |
| 采集节拍 | 默认 20 ms 轮询，目标 50 Hz | vTaskDelayUntil，20–1000 ms 可配置，真实时序待测 |
| 校准存储 | Flash 末尾双 1 KB 页 | 应用 Flash 限为 62 KB，显式保存/重启恢复 |

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
| `HealthTask` | 1 | 任务心跳、错误计数、栈水位、LED/蓝牙状态及可选看门狗 |

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
HostApp/      Web Serial 上位机、三节点姿态与颈椎三维可视化
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

当前驱动版本 Debug 编译结果（GNU Arm 14.3.1）：

| 存储区 | 使用量 | 占比 |
|---|---:|---:|
| RAM | 14,560 B / 20 KB | 71.09% |
| Flash | 42,672 B / 62 KB | 67.21% |

这些数字只代表当前 Debug 构建，不代表最终功能版本或运行期栈余量已经验证。

## 上位机

STM32 固件和 PC 上位机独立构建。根目录的 CMake 目标只编译固件，不依赖 Node.js 或 `HostApp/`；相对角和三维模型在 PC 浏览器中计算。两端通过 [A5 5A 串口协议 v1](Docs/WIRE_PROTOCOL.md) 对接，可使用 USART1 有线串口或 USART2 的 HC-05 透传，参见 [蓝牙联调](Docs/DRIVER_BRINGUP.md#9-hc-05-到-windows-上位机)。`workstation/` 是另一套 D0 工作站，不能直接解读本固件的 `A5 5A` 帧。

`HostApp/` 提供浅色学术仪器风格的浏览器上位机：

- **姿态重建**：驾驶坐姿人体、透明座椅、突出的颈椎结构与三个独立 IMU 位姿视图；支持旋转、平移、缩放和预设视角。
- **数据曲线**：三个节点的 Roll/Pitch/Yaw、加速度与角速度，以及 C0–C4、C4–C7、C0–C7 相对旋转角；支持图例筛选、缩放、平移、悬停读数与选帧。
- **采集诊断**：同步跨度、解析错误、接收字节和原始数据流。
- **共享时间轴**：点击时间戳或曲线、逐帧与倍速回放、中立位归零、返回实时；切换窗口期间持续采集。

使用桌面版 Chrome/Edge 和 Node.js 18+ 启动：

```powershell
cd HostApp
npm ci
npm start
```

打开 <http://127.0.0.1:4173>，点击“运行模拟数据”即可预览。串口连接使用 Web Serial，可导出 CSV。固件使用 v1 二进制采样/命令/应答帧和 CRC16，上位机自动识别并兼容旧 NDJSON/CSV。二进制加速度换算为 m/s²、角速度为 °/s；旧文本保留传输值。设备控制位于采集诊断窗口。格式、测试命令和重建边界见 [HostApp/README.md](HostApp/README.md)。

## CubeMX 再生成规则

1. 使用 `Spinal_tester.ioc` 修改时钟、引脚、DMA 和中间件配置。
2. 业务代码只放在独立分层目录，不写入 `Drivers/` 或 `Middlewares/`。
3. `Core/` 中只修改 `USER CODE` 区。
4. 重新生成后检查 `main.c`、`freertos.c`、根 `CMakeLists.txt`、FreeRTOS 宏、中断优先级和链接脚本 **62 KB Flash 保留边界**。
5. 重新执行 Debug 构建，并在 PR 中粘贴 RAM/Flash 占用结果。

## 驱动交接与验证

- [驱动与实物联调](Docs/DRIVER_BRINGUP.md)：接线、状态机、任务所有权、Flash 布局及台架验收顺序。
- [线上协议 v1](Docs/WIRE_PROTOCOL.md)：字段、单位、CRC、命令、状态码与固定向量。
- [测试命令和边界](Docs/TESTING.md)：ARM 构建、实际 C 驱动 mock HAL、19 项 JS 单元及 12 项浏览器测试。

MCU `NeckMotionAlgorithm_*()` 与 CAPTURE_NEUTRAL 命令保持 `NOT_IMPLEMENTED`。网页中立位只是显示参考，不代表 MCU 算法完成。IWDG 已提供策略，默认关闭；完整健康快照目前通过调试接口查看，并未定期发送到网页。

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

本仓库公开展示项目代码，原创代码目前未指定开源许可证。第三方组件遵循各自许可证：STM32 HAL/CMSIS 与 FreeRTOS 见对应目录，Three.js 与 Inter 字体见 [HostApp/vendor/README.md](HostApp/vendor/README.md)。
