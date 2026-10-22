# 多人协作约定

## 模块负责人建议

| 负责人 | 目录 | 第一阶段交付 |
|---|---|---|
| A：传感与时间 | `Devices/`、`Services/sensor_service.*`、`BSP/bsp_i2c.*`、`BSP/bsp_time.*` | 三节点探测、配置、稳定读取、时间戳误差测试 |
| B：通信与协议 | `BSP/bsp_uart.*`、`Services/protocol_service.*`、`Tasks/communication_task.c`、`Tasks/command_task.c` | UART DMA 收发、版本化协议、CRC、上位机互通测试 |
| C：系统与算法接口 | `App/`、`Tasks/health_task.c`、`Services/diagnostics_service.*`、`Services/neck_motion_algorithm.*` | 启动/故障状态、看门狗策略、中立位与相对姿态接口 |

## 修改规则

1. 不在 `Drivers/` 或 `Middlewares/` 中写业务代码。
2. `Core/` 中只允许修改 CubeMX 的 `USER CODE` 区；重新生成后必须检查 diff。
3. 一个外设只能有一个任务所有者。I2C1/TCA/BNO055 永远属于 AcquisitionTask。
4. 公共头文件先定义契约，再实现；不要跨层直接引用另一个模块的私有变量。
5. 新增协议字段必须同时更新协议版本、编码器、解码器和测试向量。
6. 每个占位函数完成后，删除对应 `TODO(owner)`，补充失败路径和台架验证记录。
7. 提交信息建议使用 `sensor: ...`、`protocol: ...`、`system: ...` 等模块前缀。

## 每个合并请求的最低验收

- `cmake --preset Debug` 和 `cmake --build --preset Debug` 通过。
- 没有新增编译警告。
- 写清测试硬件、固件版本、采样率和测试时长。
- 明确区分静态检查、无传感器启动、台架实测和佩戴实验。
- 任何采样/协议变更都要检查丢帧、超时、错误恢复和边界输入。
