# 软件验证与硬件边界

## ARM构建

在工程根目录，PATH含CMake、Ninja、GNU Arm工具链：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

v0.1.0.1本机Debug构建（GNU Arm14.3.1）：RAM 14,560 B /20 KB，Flash 42,672 B /62 KB。
Release构建：RAM 14,552 B /20 KB，Flash 23,848 B /62 KB。后续代码变化需重新测量。
静态分配和链接成功不等于运行时栈够用；需实板检查高水位。

## 生产C驱动主机测试

需本机C编译器（MSVC、GCC或Clang）、CMake及Ninja。Windows从VS开发者终端运行，**不使用ARM交叉工具链**：

```powershell
cmake -S Tests -B build/host-tests -G Ninja
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

两个CTest入口：实际驱动行为测试，以及C编码结果与JS固定向量比对。
链接实际 `protocol_service.c`、`bno055.c`、`tca9548a.c`、`bsp_uart.c`、`bsp_time.c`，仅替换HAL/I2C边界。
覆盖CRC标准向量、错误/截断/连续命令、编码容量、BNO模式/延时/单位/寄存器解码/校准/失败重试、
TCA单通道缓存、双UART隔离/HT-TC-IDLE去重/环回/溢出/持久TX/超时、TIM2待处理中断/32位回绕。
这不是STM32运行：没有验证真实DMA寄存器、Flash擦写掉电、总线电气、IRQ延迟或FreeRTOS调度。

## 上位机

```powershell
cd HostApp
npm ci
npm test
npm run test:ui
```

浏览器测试默认使用Microsoft Edge；需先安装对应浏览器。CI可用`npx playwright install msedge`安装。
单元测试19项，浏览器12项：保留原文本/模型/回放/曲线场景，增加C协议向量、每种拆分位置、坏帧恢复、
无效节点、物理单位、时间回绕、串口断开与命令应答。浏览器使用内存Web Serial，不是实物通信证据。
若协议有意变更，更新版本、C编码器/解析器、JS、文档与固定向量；禁止为了让测试通过只改期望值。

## 下一阶段

按照 [DRIVER_BRINGUP.md](DRIVER_BRINGUP.md) 逐项记录硬件版本、固件提交、采样周期、运行时长、
逻辑分析仪波形、栈水位、复位原因、无线/有线丢包率和Flash恢复结果，之后才标为`bench`或`wearable`。
