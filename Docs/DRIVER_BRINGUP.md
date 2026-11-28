# v0.1.0.1 驱动交接与台架联调

依据 2026-11-17 原理图实现。当前完成软件实现与主机验证，以下硬件验收均待执行。
不包含 MCU 端颈部运动算法，也未实施电池电量估算或充电状态采样。

v0.1.0.3 仍默认走 TCA9548A 三节点拓扑，BNO055 默认使用内部振荡器（不同于下文历史基线），每轮至少有一颗 IMU 有效才发送采样帧。单颗直连台架可用 C 编译定义 `APP_USE_TCA9548A=0`，该模式不读写三节点 Flash 校准参数。

需要排查链路时可分别以 `cmake --preset Debug -B build/uart-smoke -DSPINAL_UART_SMOKE=ON` 和 `cmake --preset Debug -B build/i2c-diag -DSPINAL_I2C_DIAG=ON` 配置独立诊断镜像，再用 `cmake --build` 构建对应目录。前者在双串口发送固定文本，后者通过 USART1 输出 TCA 通道、BNO055 地址与芯片 ID；两种诊断选项不可同时启用，常规烧录应选正常固件。

## 1. 外设和接线

| 模块 | 配置 | 任务/接口 |
|---|---|---|
| 主控 | STM32F103C8T6，HSE 8 MHz，PLL 72 MHz | CubeMX / HAL |
| 传感总线 | I2C1 PB6 SCL / PB7 SDA，100 kHz，外部上拉 | AcquisitionTask 独占 |
| 复用器 | TCA9548A 7 位地址 0x70 | 单通道选择，避免同地址 BNO 并接 |
| 三颗 IMU | 通道 0=C0、1=C4、2=C7，7 位地址均为 0x28 | COM3 接地时地址对应此配置 |
| PC 接口 | USART1 PA9 TX / PA10 RX，115200 8N1 | TX DMA1 ch4，RX circular ch5 |
| 蓝牙 | USART2 PA2 TX / PA3 RX，115200 8N1 | TX DMA1 ch7，RX circular ch6 |
| 蓝牙 STATE | PA4，下拉输入 | 仅报告连接状态，不作为流控或关闭采集的条件 |
| 状态灯 | PC13，低电平点亮 | 健康任务每秒翻转；任务失活/栈不足则常亮 |
| 高分辨率时基 | TIM2，PSC=71，ARR=65535，1 MHz，更新中断优先级5 | 软件扩展为 32 bit μs |
| HAL / RTOS tick | TIM4 / SysTick，1 ms | 不占用 TIM2 |
| 轮询节拍 | `vTaskDelayUntil()`，默认20 ms | TIM3 配置50 Hz但未启动，不构成采样触发 |

PC 需要外部 **3.3 V TTL USB-UART**：转接器 RX 接 PA9，TX 接 PA10，GND 共地。不要接 RS-232 电平。
原理图 USB-C 为供电，固件未实现 USB CDC。HC-05 是经典蓝牙串口透传模块，不是 BLE GATT 设备。
模块默认透传速率不一定是 115200；先按模块资料单独配置，AT 波特率与透传速率不能混淆。
若 COM3 实际接高，需将对应地址改为 0x29。RESET 未接 MCU，软件无法独立硬复位 IMU 或 TCA。

## 2. 模块边界和扩展入口

| 负责人 | 首选入口 | 修改时检查 |
|---|---|---|
| 传感/时间 | `Devices/bno055.*`、`tca9548a.*`、`BSP/bsp_i2c.*`、`bsp_time.*`、`Services/sensor_service.*` | 通道选择与访问不可跨任务；硬件时间/轴方向/错误恢复 |
| 通信 | `BSP/bsp_uart.*`、`Services/protocol_service.*`、Command/Communication task、`HostApp/js/wire.js` | 两端同一版本/字节序/CRC；保留固定向量 |
| 系统 | `App/app_bootstrap.*`、`Tasks/health_task.c`、`Services/diagnostics_service.*`、`BSP/bsp_calibration.*` | 栈、RAM、启动、Flash 保留区和 IWDG |

业务参数集中在 `Config/Inc/app_config.h`；硬件波特率/引脚以 `.ioc` 和 `Core/` 初始化为准。
当前 baud 宏只是说明值；改 baud 时必须同步 CubeMX、上位机和 HC-05，单改宏不会重配 HAL。
算法组后续只补 `neck_motion_algorithm.*`；通过 AcquisitionTask 接入，禁止从算法另开任务切换 TCA。
每颗 BNO 都有独立状态和校准缓存，没有使用 Bosch 驱动的全局设备指针。

## 3. BNO055 状态机和数据

开机等待700 ms → CHIP_ID=0xA0 → CONFIGMODE等待25 ms → SYS_TRIGGER软复位等待700 ms → 再探测
→ PAGE0 → normal power等待10 ms → UNIT_SEL=0 → AXIS_MAP=0x24 / SIGN=0
→ 外部32.768 kHz晶振选择等待10 ms → 可选恢复22 B校准 → NDOF=0x0C等待20 ms
→ 回读工作模式和晶振选择位。每轮只推进一个到期步骤，三节点独立推进；启动期间上报无效槽。

定时读取 page0 `0x08..0x3A` 共51 B，解码加速度、陀螺仪、四元数、温度、校准和系统状态。
SYS_STATUS=5、SYS_ERR=0且四元数模平方在150000000..400000000才标有效；此门限只是基本异常排除，未经佩戴实验验证。
CALIB_STAT≠0xFF 另置未完全校准标志。连续3次失败安排软重初始化，失败启动按1500 ms重试。
全节点均无有效采样时，定时尝试最多9个SCL释放脉冲、STOP、I2C重初始化和TCA缓存重置。
物理短路、严重拉低、供电异常未必能被软件恢复，须电源/硬件处理。

100 kHz 下每颗51 B读约占5 ms量级；三颗加复用器/调度开销接近20 ms预算，50 Hz为目标而非已测结果。
出现 `APP_DIAG_ACQUISITION_OVERRUN` 时先测总线波形，可先将轮询周期改为50 ms；不要直接宣称提高I2C频率就可满足柔性总线信号完整性。
三颗BNO内部时钟独立，主控读取中点只是近似对齐依据。NDOF依赖磁力计，座椅金属、扬声器、电机与磁性安装件都需实测影响。
目前只为NDOF提供状态门限；改为IMUPLUS等模式时，需同步调整有效状态/校准标准、重做测试，不能只改模式宏。

## 4. 双串口和实时性边界

每口 RX DMA128 B + 软件环形缓冲512 B，保留HT/TC/IDLE事件；串口和DMA中断均优先级6，ISR仅搬字节/记错误。
CommandTask每2 ms每口最多消费64 B；每口独立解析器，溢出或串口错误会重置解析状态并在任务中重启接收。
满环丢弃新字节并计数；接收不是无限缓存。115200下半DMA周期约5.56 ms，必须实测高负载和Flash操作时的中断延迟。
CommunicationTask独占TX，将帧复制到各端口持久缓冲，忙则丢采样并计数；命令应答优先，TX超过100 ms未完成则中止后重试。
不保证无损上传；两口应答共用有限队列，不实现鉴权或重放防护，适用于受控实验链路。
命令处理每轮有数量上限；SAVE是显式停止上传后的维护操作，可打断连续采集时间。

## 5. 校准保存与Flash布局

三颗BNO的22 B offset/radius分别读取，全部完成后写一个版本化记录。
先STOP上传 → 完成三颗各SYS/GYR/ACC/MAG校准（0xFF） → SAVE → 收到OK → 断电重启检查恢复。
SAVE会短暂进入CONFIGMODE，返回NDOF；它不会计算人体中立位，也不保证磁场改变后旧校准仍适用。

| 区域 | 地址 | 大小 |
|---|---|---:|
| 应用 | 0x08000000–0x0800F7FF | 62 KB |
| 校准槽0 | 0x0800F800–0x0800FBFF | 1 KB |
| 校准槽1 | 0x0800FC00–0x0800FFFF | 1 KB |

88 B记录包含magic/version、模式/轴映射指纹、递增代次、三组偏移、CRC和最后提交的halfword。
写另一页、最后提交标记，重启选择最新有效记录；这是软件抗中断设计，尚未进行实际掉电注入验证。
仅显式保存，不周期擦写；换板、交换传感器通道、改变安装映射后应重新校准/擦除旧记录。
烧录器全片擦除会删除校准。擦写会使Flash取指/中断响应停顿，SAVE期间不要持续灌入串口命令。
要测量擦写耗时及TIM2溢出连续性；若不能保证中断延迟<65.536 ms，需采用更长硬件计数器/独立时基再继续长时实验。

## 6. 健康信息与看门狗

`DiagnosticsService_GetSnapshot()` 可在调试器查看四任务心跳、计数、复位标志、每任务最小剩余栈字数、两口错误/溢出、蓝牙STATE。
前3个业务任务连续3个健康窗口无心跳或任一任务剩余栈<24 words，则判不健康。
传感器掉线不等于调度器失活；单纯掉线不应靠无限重启主控处理。当前GET_STATUS只返回就绪/上传位，完整快照未推送到网页。
`APP_ENABLE_WATCHDOG=0` 默认关闭。台架验证后可置1，IWDG /256、reload4095，LSI40 kHz时约26 s，实际超时受LSI误差影响。
一旦启动无法软件停止；不要在首次下载/单步调试阶段盲目开启。栈溢出/分配失败进入故障钩子，启用IWDG后应由硬件复位恢复。

## 7. 推荐台架验收顺序

1. 限流供电，确认3V3、地、复位、SWD和各晶振接法；先使用PC串口，避免无线问题混入传感验证。
2. 构建并烧录 `build/Debug/Spinal_tester.elf`；确认主控启动和PC13状态。用GET_INFO核对0.1.0.1。
3. 逻辑分析仪确认I2C上拉、100 kHz、0x70及各通道0x28；确认两颗同地址器件未同时导通。
4. 先接单节点，网页对应卡片应更新，其余无效；再接三颗检查轴方向、四元数模长、温度与CALIB_STAT。
5. 固定三节点做相同方向转动/静置，记录器件状态、起止时间与帧内跨度；安装坐标冻结前不评价解剖角度准确率。
6. 比较串口波形与时间戳，连续跨至少2次TIM2溢出；再运行超过72分钟，检查主机回绕、设备重启和序号变化。
7. 测量20 ms轮询真实耗时/抖动、串口帧率/CRC/丢帧；诊断计数和栈水位应记录，不只看模型是否在动。
8. 故障注入：拔一颗、拔复用器、短暂拉低总线、PC串口断连/重连；验证其他有效节点继续可见以及重新接入恢复。
9. HC-05先单独测试透传速率，再双路同时接收，验证PC/BT请求应答不会串口混发；记录无线距离、掉包和供电峰值。
10. STOP→SAVE→重启恢复；在受控电源上对擦写不同阶段掉电，确认保留旧记录或拒绝坏记录。最后才启用IWDG并做任务卡死测试。

## 8. CubeMX再生成与测试

`.ioc`与当前生成区已手工同步，尚未运行CubeMX GUI重新生成验证。业务接入保持在USER CODE区，外设初始化由`.ioc`管理。
再生成后核对USART1句柄/调用/DMA4/5、中断分发、TIM2 PSC71/ARR65535、PA4/PC13、FreeRTOS宏、AppBootstrap双串口参数。
**链接脚本应用Flash必须仍为62 KB**，否则程序可能覆盖校准页。检查CMake业务源列表并重新构建/运行测试。
见 [TESTING.md](TESTING.md)，线上格式见 [WIRE_PROTOCOL.md](WIRE_PROTOCOL.md)。

参考：Bosch官方 [SensorAPI头文件](https://github.com/boschsensortec/BNO055_SensorAPI/blob/master/bno055.h) / [实现](https://github.com/boschsensortec/BNO055_SensorAPI/blob/master/bno055.c)。
HAL接收/中断行为以工程所带 `stm32f1xx_hal_uart.c`、`stm32f1xx_hal_tim.c` 与 `stm32f1xx_hal_iwdg.c` 为依据。

## 9. HC-05 到 Windows 上位机

1. `PA2 / USART2_TX → HC-05 RXD`，`PA3 / USART2_RX ← HC-05 TXD`，两端共地。按模块规格核对供电，UART 信号使用 3.3 V 逻辑。`STATE → PA4` 仅供诊断，不是发送使能。
2. 按模块的 AT 手册配置从机模式和 `115200 8N1` 数据透传。退出 AT 模式后在 Windows 配对，PIN 以模块配置为准；在蓝牙串口设置中选择该模块的**传出** COM 口。
3. 用支持 HEX 显示的串口工具检查是否持续出现三节点采样帧头 `A5 5A 01 01 7D 00`。关闭工具以释放端口；仅看到帧头不能证明 CRC 或姿态数据有效。
4. 用 [HostApp](../HostApp/README.md) 或兼容 [串口协议 v1](WIRE_PROTOCOL.md) 的上位机核对 CRC、序号和 C0/C4/C7 有效标记。分别轻转三个节点，确认对应四元数变化，记录有效帧率和跳号。

配对后无字节，先核对传出 COM、透传模式、USART2 接线和模块端速率，必要时用 USART1 有线口对照。收到字节但姿态无效时，检查 BNO055 初始化、TCA 通道和节点标记。这是台架验证步骤，不代表已完成实板蓝牙验收。
