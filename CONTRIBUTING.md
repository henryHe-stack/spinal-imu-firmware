# Contributing

本仓库使用短生命周期功能分支和 Pull Request 协作。每次修改应保持范围明确、可编译、可回退。

## 分支命名

- `feature/sensor-bno055-read`
- `feature/protocol-uart-dma`
- `fix/tca-channel-recovery`
- `docs/update-bringup-guide`

不要直接在 `main` 上开发。

## 模块负责人建议

| 方向 | 主要目录 | 第一阶段交付 |
|---|---|---|
| 传感与时间 | `Devices/`、`Services/sensor_service.*`、`BSP/bsp_i2c.*`、`BSP/bsp_time.*` | 三节点探测、配置、读取与时间误差测试 |
| 通信与协议 | `BSP/bsp_uart.*`、`Services/protocol_service.*`、Communication/Command task | UART DMA、版本化协议、CRC 和上位机互通 |
| 系统与算法接口 | `App/`、Health task、Diagnostics、NeckMotionAlgorithm | 启动/故障状态、看门狗、中立位与相对姿态 |

## 强制边界

1. `AcquisitionTask` 是 I2C1、TCA9548A 和 BNO055 的唯一所有者。
2. `AppSampleFrame` 是 MCU 内部结构，禁止直接 `memcpy` 到串口。
3. 业务代码不得写入 `Drivers/` 或 `Middlewares/`。
4. `Core/` 只能修改 CubeMX `USER CODE` 区。
5. 协议字段变化必须同步修改版本、编码器、解码器和测试向量。
6. 占位接口完成后必须删除对应 `TODO(owner)` 并补充失败路径。

## 本地检查

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

如修改 `.ioc`，还要检查生成 diff、中断优先级、DMA 配置和根 CMake 的业务源文件列表。

## Pull Request 要求

PR 描述必须回答：

- 改了什么，为什么现在需要改？
- 哪个任务或模块拥有被修改的资源？
- 如何验证，验证等级是 `static/build/bench/wearable` 中的哪一级？
- 是否影响采样率、协议、RAM/Flash、功耗或硬件连接？
- 失败时如何恢复，是否存在兼容性问题？

涉及硬件行为的 PR 不得仅以“编译通过”宣称功能完成。

## Commit 约定

使用 `<type>: <summary>`：

- `feat: implement BNO055 burst read`
- `fix: recover TCA channel after I2C timeout`
- `docs: document sensor axis convention`
- `test: add protocol CRC vectors`
- `chore: update CubeMX generated files`

## 完成定义

- ARM GCC 构建通过且没有新增警告。
- 接口和错误路径经过审查。
- 文档与实际行为一致。
- RAM/Flash 变化已记录。
- 台架或佩戴验证声明包含硬件版本、固件提交、采样率和测试时长。
