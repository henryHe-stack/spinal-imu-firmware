# 三节点颈部姿态监测系统

本项目使用 STM32F103C8T6、三颗 BNO055 和 TCA9548A 采集 C0、C4、C7 节点运动数据，通过串口传输到 Windows 工作站。工作站集成三维姿态观察、区域相对角计算、安装配置、静止归零、数据记录、回放和采集诊断。

## 快速启动

1. 将仓库文件下载并解压到同一个文件夹。
2. 双击 **01_启动工作站.cmd**，或进入 app 文件夹双击 **颈部姿态三维工作站.exe**。
3. 无硬件时，在「三维观察」点击「运行模拟数据」，查看模型、三个节点和曲线。点时间轴可观察历史姿态，点「返回实时」回到最新帧。
4. 要运行测量算法，切换「正式测量与记录」，来源选择 SIM 单节点或 SIM 三节点，点击「启动」。
5. 双击 **03_打开数据目录.cmd**，查看示例数据和自己的测量记录。

运行环境为 Windows x64、.NET Framework 4.6.2 或更新版本、Microsoft Edge WebView2 Runtime。三维资源、字体和算法运行文件均随应用提供。日常运行直接使用 EXE。

## 系统组成

| 组成 | 本项目配置 |
|---|---|
| MCU | STM32F103C8T6，主测量固件使用 HSE 8 MHz 和 PLL，SYSCLK 72 MHz |
| 节点 | C0、C4、C7，各使用一颗 BNO055 |
| 通道 | TCA9548A CH0 对应 C0，CH1 对应 C4，CH2 对应 C7 |
| I²C 地址 | TCA9548A 为 0x70，三个 BNO055 各为 0x28 |
| 总线 | I²C1，PB6 为 SCL，PB7 为 SDA |
| 数据链路 | USART2，PA2 为 TX，PA3 为 RX，8N1 |
| 主固件 50 Hz 配置 | I²C 100 kHz，串口 115200 |
| 主固件 100 Hz 有线配置 | I²C 400 kHz，串口 230400 |
| 操作系统 | MCU 使用 FreeRTOS，Windows 端运行 C 算法核心 |
| 数据格式 | D0 v1 二进制协议，包含会话、三节点原始帧和控制应答 |

程序自检固件使用 HSI 8 MHz、TIM3 微秒计数、10 ms 轮询周期和 115200 文本报告。自检固件默认启用 BNO055 外部 32.768 kHz 晶体配置。根据样机的实际晶体装配选择自检构建参数。

## 工作站功能

### 三维观察

- 显示驾驶员参照场景、三个独立 IMU 和颈部区域运动模型。
- 提供姿态重建、数据曲线、采集诊断和历史时间轴。
- 支持旋转视角、缩放、预设视角、人体透视和座椅透视。
- 接收三节点原始四元数，显示节点姿态与区域总旋转角。
- 「设为中立位」更新三维模型和曲线的显示参考。
- 「导出 CSV」保存当前页面缓存的姿态数据。

### 正式测量与记录

- 提供单节点与三节点 C 算法模拟、USB 串口、HC-05 串口和 BIN 文件回放。
- 管理每颗 IMU 的安装方向、共同参考转换、滤波参数和角度阈值。
- 提供归零请求、取消、设备状态查询、暂停及恢复操作。
- 显示 C4 倾角，以及整体、上段、下段和基部四组相对姿态。
- 显示节点校准状态、有效位、参考编号、采集质量和告警。
- 保存原始 BIN、角度 CSV、事件、命令及本次安装配置。

两页共用 D0 采集链路。切换页签时，数据接收与记录继续运行。含三节点原始四元数的记录可以同时驱动测量结果和三维模型。示例 BIN 及 C 算法模拟的结果流在测量页呈现角度、告警和回放内容。

## 姿态计算

每颗 BNO055 使用内置融合输出姿态四元数。STM32 按 C0、C4、C7 顺序读取数据，并附带节点读取起点、终点、中点、校准状态和序号。

Windows C 算法依次执行数值与时间质量检查、共同参考转换、安装方向补偿、相对四元数运算、中立位补偿及角度分解，输出屈伸、侧屈和轴向旋转。节点读取中点作为工程估计时间保留在数据中。

| 输出 | 姿态关系 |
|---|---|
| 整体 | C0 相对于 C7 |
| 上段 | C0 相对于 C4 |
| 下段 | C4 相对于 C7 |
| 基部 | C7 相对于本次共同世界参考 |

安装四元数按 w、x、y、z 排列，将芯片坐标转换到身体坐标。共同参考四元数将各节点的本地融合世界转换到统一参考系。单位四元数为 1 0 0 0，表示对应坐标轴已对齐。中立位由本次采集中的静止窗口建立。

## 连接样机

1. 按样机原理图接好电源、公共地、TCA9548A 和三个 IMU。
2. 使用 ST-LINK 接 SWDIO、SWCLK 和公共地，核对供电连接。
3. 在 STM32CubeProgrammer 或调试工具中选择对应 ELF 并下载。firmware 目录中每个配置提供 ELF、HEX 和 BIN，常用 ELF 文件进行烧录。
4. USB-TTL 使用 3.3 V 逻辑电平。板端 PA2 接适配器 RX，PA3 接适配器 TX，GND 共地。
5. 在 Windows 设备管理器展开「端口（COM 和 LPT）」。拔插 USB-TTL，记录随模块出现的 COM 编号。
6. 打开工作站「正式测量与记录」，先在停止状态进入「PC测量配置」，填写三个节点的安装方向、共同参考和配置名称。核对后保存。
7. 来源选择「USB 串口 D0」，填入 COM 口。50 Hz 固件选择 115200，100 Hz 有线固件选择 230400，再点「启动」。
8. 等待会话登记与节点状态更新，点「开始记录」。保持中立姿势静止，在「板端控制」中点「请求归零」。
9. 等待零位状态显示「已提交」且参考编号大于零，再按计划施加动作。测量页查看有方向的屈伸、侧屈和旋转角，三维页观察节点姿态。
10. 结束后先点「停止记录」，等待 COMPLETE，再停止数据源。

### 单节点与三节点自检

首次检查 C4，可烧录 firmware/self_check_c4/TimingDiag_C4.elf。检查三节点时，使用 firmware/self_check_three/TimingDiag_Three.elf。

关闭占用设备串口的程序，在仓库根目录打开 PowerShell，按实际端口运行接收工具。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/self-check.ps1 -Port COM5 -Seconds 60
```

接收端打开后复位样机。工具输出读取次数、各节点状态、轮询周期和读取窗口统计，并生成中文报告。报告保存在 src/self_check/records 下。

## 数据、配置与回放

- data/示例数据 提供单节点和三节点两份 65 秒演示 BIN。
- data/我的记录 保存当前工作站生成的测量记录。每次记录使用独立子目录。
- measurement-profile.xml 保存 PC 测量配置，启动串口时加载。
- 每份新串口记录附带当次 measurement-profile.xml，回放时使用相同配置。

完整记录目录包含 capture.bin、chunks.csv、bootstrap_session.bin、parser_prefix.bin、角度与事件表、命令日志和 COMPLETE.txt。回放时保留整个目录，在工作站点「选择回放BIN」并选择 capture.bin。开始采集记录后再请求归零，可将零位建立过程一起保存。

## 目录说明

```text
.
├─ README.md / README.html       项目介绍与操作说明
├─ 01_启动工作站.cmd
├─ 02_打开使用说明.cmd
├─ 03_打开数据目录.cmd
├─ app/                         可直接运行的 Windows 工作站
├─ firmware/
│  ├─ main_50hz/                50 Hz、115200 主固件
│  ├─ main_100hz_usb/           100 Hz、230400 有线主固件
│  ├─ self_check_c4/            C4 单节点自检固件
│  └─ self_check_three/         三节点自检固件
├─ src/
│  ├─ stm32/                   主固件、CubeMX 配置、驱动和 FreeRTOS
│  ├─ host_core/               C 算法、协议、模拟器和测试
│  ├─ HostApp/                 三维界面及 Windows 桌面壳源码
│  ├─ pc_f103_dashboard/       正式测量、配置、记录与回放源码
│  ├─ self_check/              独立自检与报告工具源码
│  └─ data/                    示例 BIN 和回归测试固定输入
├─ data/                        运行数据
├─ dependencies/                固定版本构建依赖
└─ tools/                       编译、测试和自检接收入口
```

## 从源码构建

桌面程序构建使用 Windows 主机版 GCC 和系统 .NET Framework C# 编译器。主固件及自检构建使用 ARM GCC，主固件同时使用 CMake 和 Ninja。STM32CubeIDE 可提供 ARM 工具链、CMake 和 Ninja，CubeMX 用于编辑 src/stm32/Spinal_tester.ioc。

WebView2 SDK 1.0.2903.40 已固定在 dependencies 目录，Three.js、控制器和字体均在 src/HostApp/vendor。桌面构建会从 C 源码编译解算 DLL 和模拟器，再生成一体版 EXE。以下命令均在仓库根目录执行。

```powershell
# 构建桌面工作站，更新 app
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Target App

# 构建主固件，更新 firmware 对应配置
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Target Firmware50
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Target Firmware100

# 构建自检固件
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Target SelfC4
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Target SelfThree
```

主机 GCC 路径可通过 -Gcc 参数指定。主固件构建脚本支持 -ToolchainBin、-CmakeBin、-NinjaBin。自检构建脚本支持 -ToolchainBin 及 -InternalSensorClock。

## 运行测试

测试包含 C 算法数值与状态机、协议编解码、上位机记录与回放、模拟串口、自检统计和报告解析，以及定时器和 I²C 模型测试。JavaScript 数值测试使用 Node.js 18+。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/test.ps1 -Suite All
```

单独执行时，可将 All 替换为 Core、Host、Self、Protocol 或 Bench。测试输出写入各模块的 build、work、HostTests 或 records 目录，由 Git 忽略规则管理。

桌面窗口交互检查运行 powershell -NoProfile -ExecutionPolicy Bypass -File tools/test-desktop.ps1。先运行 Host 测试以生成测试用原始帧记录。窗口检查覆盖模型、曲线、串口选择框、CSV 保存、测量模拟、BIN 回放和共用数据流。

三维页面交互测试位于 src/HostApp/tests。前端开发可在 src/HostApp 下运行 npm ci、npm test 和 npm run test:ui。日常使用运行 app 下的 Windows 工作站。

## 第三方组件

STM32 HAL、CMSIS、FreeRTOS、Three.js、OrbitControls、Inter 字体及 WebView2 的许可随各自组件保留。WebView2 SDK 包的 SHA-256 为 EF128016DD1E51C59178C827ED5B8AA3322C57AFA8675D930F8109505542AD74。
