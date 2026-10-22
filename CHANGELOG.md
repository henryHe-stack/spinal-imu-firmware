# Changelog

本项目采用四段版本号 `MAJOR.MINOR.PATCH.MICRO`。所有重要变更记录在此文件中。

## [0.1.0.0] - 2026-10-21

### Added

- 建立 STM32F103C8T6、HAL、FreeRTOS 和 CMake 固件基线。
- 建立 App、BSP、Devices、Services、Tasks、Common 和 Config 分层。
- 新增 Acquisition、Communication、Command 和 Health 四个静态任务。
- 新增静态采集队列、命令队列和三节点 IMU 内部数据模型。
- 新增 TCA9548A 基础驱动和 BNO055 探测接口。
- 新增诊断计数器、协议接口与颈部相对姿态算法占位模块。
- 新增架构、协作、路线图和 GitHub 工作流文档。

### Known limitations

- BNO055 完整配置和采样读取尚未实现。
- USART2 circular DMA 接收与版本化线上协议尚未实现。
- TIM2 微秒时间戳、TIM3 50 Hz 节拍和硬件同步尚未验证。
- 尚未完成开发板、柔性板或佩戴实验验证。
