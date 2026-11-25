# 串口协议 v1

适用固件 v0.1.0.1；USART1（PC）和 USART2（HC-05）均为 115200、8N1、无流控。
两端接收相同采集帧，命令应答仅发给来源串口。两端命令改变的是同一个采集系统。
不在二进制通道混入 printf 文本。HC-05 的透传波特率需另行配置为 115200。

## 帧格式

| 偏移 | 字段 | 大小 | 说明 |
|---:|---|---:|---|
| 0 | 同步字 | 2 | `A5 5A` |
| 2 | version | 1 | `01` |
| 3 | type | 1 | `01` 采样，`10` 命令，`11` 应答（十六进制） |
| 4 | payload_length | 2 | 小端，不包含帧头和 CRC |
| 6 | payload | 变长 | 见下文 |
| 末尾 | CRC | 2 | 小端，CRC16/CCITT-FALSE |

所有多字节字段均为 little endian，禁止直接发送结构体内存。最大帧长 192 B。
CRC 参数：poly=0x1021、init=0xFFFF、refin/refout=false、xorout=0；覆盖 version 到 payload 末字节。
标准检查串 `123456789` 的 CRC=0x29B1。拆包、粘包、CRC 错误后按同步字重新定位。

## 采样 payload

`sequence:u32 + count:u8 + count × sample[40]`，当前固件始终上报 3 槽，顺序 C0/C4/C7。
三节点帧长 133 B；50 Hz 为 6650 B/s，占 115200 8N1 线路理论有效载荷的约 58%，不含命令应答。
无线吞吐、真实采样频率与丢包率必须实测。序号是主控轮询序号，停止上传期间也递增，跳号不一定表示链路丢包。

| sample 内偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | node | u8 | 0=C0，1=C4，2=C7 |
| 1 | channel | u8 | TCA 通道，默认 0/1/2 |
| 2 | flags | u16 | bit0 有效，bit1 粗时间戳保留，bit2 传感错误，bit3 未完全校准 |
| 4 / 8 / 12 | start / end / mid | u32 ×3 | 主控 I2C 读取起止/中点，单位 μs |
| 16 | qw/qx/qy/qz | i16 ×4 | BNO055 原始四元数，每单位 16384 LSB |
| 24 | ax/ay/az | i16 ×3 | 每 m/s² 为 100 LSB，包含重力分量 |
| 30 | gx/gy/gz | i16 ×3 | 每 °/s 为 16 LSB |
| 36 | temperature | i8 | °C |
| 37 | CALIB_STAT | u8 | 原始 SYS/GYR/ACC/MAG，各 2 bit，自高到低 |
| 38 / 39 | SYS_STATUS / SYS_ERR | u8 ×2 | BNO055 原始状态/错误 |

只有 bit0=1 且 bit2=0 时使用姿态/向量；错误槽中的零值不是测量。上位机将其标为无效。
校准不足与传输失败分别标记，不将低校准质量隐去。四元数为器件轴映射坐标，未转换为解剖坐标。
时间戳为 uint32，每约 71.6 分钟回绕；同一轮询跨越回绕时也须按模 2^32 比较。
上位机扩展回绕以显示连续时间，`timestampRawUs` 保留设备原始值；CSV 的 timestamp_us 使用原始值。
没有共享采样触发，时间戳不代表 BNO055 内部融合采样时刻，也不保证 51 字节读取跨寄存器更新的原子快照。

## 命令与应答

命令 payload：`request_id:u32 + command:u8 + value:u32`，总帧长 17 B。
应答 payload：`request_id:u32 + command:u8 + status:u8 + value:u32`，总帧长 18 B。
request_id 用于主机匹配；不是去重或自动重发机制。尤其不要自动重复 SAVE。

| 命令 | 名称 | value / 前提 | 应答含义 |
|---:|---|---|---|
| 1 | START_STREAM | 忽略 value | 恢复两路上传；不重新初始化 |
| 2 | STOP_STREAM | 忽略 value | 停止后续上传，仍维护传感器/接收命令；已排队/在途帧可能继续到达 |
| 3 | CAPTURE_NEUTRAL | 算法占位 | 返回 NOT_IMPLEMENTED；与网页中立位按钮不同 |
| 4 | REINITIALIZE_SENSORS | 忽略 value | 接受异步重新初始化；不代表立即就绪 |
| 5 | SET_SAMPLE_PERIOD_MS | 整数 20–1000 ms | 设置主控轮询周期；不改变 BNO055 内部输出速率 |
| 6 | GET_INFO | 忽略 value | 返回 `0x00010001`，各字节为 0.1.0.1 |
| 7 | GET_STATUS | 忽略 value | bit0/1/2：各节点 configured，bit8：正在上传；不是完整健康快照 |
| 8 | SAVE_CALIBRATION | 先 STOP，三节点各 CALIB_STAT=0xFF | 读取三组 offset/radius 并保存；失败返回状态码 |

status：0 OK、1 INVALID_ARGUMENT、2 NOT_INITIALIZED、3 BUSY、4 TIMEOUT、5 IO_ERROR、6 DEVICE_NOT_FOUND、
7 CRC_ERROR、8 QUEUE_FULL、9 BUFFER_TOO_SMALL、10 NOT_IMPLEMENTED、11 INTERNAL_ERROR。
损坏的输入命令不应答（request_id 不可信）；仅累计解析错误。合法但未知命令返回 10。
命令队列/应答队列满时可能丢弃，主机需超时处理；当前网页等待 3 s，不自动重试。
当前不定期推送完整诊断快照；`DiagnosticsService_GetSnapshot()` 用于调试器/后续诊断接口。

固定互通向量：固件测试比对 `Tests/fixtures/firmware-v1.json` 和生产 C 编码器输出。JS 测试使用 `HostApp/tests/fixtures/firmware-v1.json`；完整仓库的 CTest 同时检查两份向量一致。
旧 NDJSON/CSV 仅保留给旧设备/测试工具，固件默认只发送上述二进制协议。
