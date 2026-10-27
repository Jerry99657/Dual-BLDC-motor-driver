# R93：第 7 批初始化与依赖收尾

固件 **UI-R93-INIT**。保留现有控制、保护、通信、存储和界面功能；没有修改任何原有 ST HAL、LL、CMSIS、USB 中间件或 HSO 源文件。

## 已落实的范围

USART1 初始化改用 **CubeMX 原生 LL 生成代码**。时钟源、引脚、GPIO 属性、波特率、预分频、帧格式、过采样和 NVIC 继续来自 `.ioc`。启动仍调用 `MX_USART1_UART_Init()`，随后 `Uart1_Init()` 只开启业务接收／错误中断。运行时收发、错误行丢弃与后续恢复沿用 R90，USB 路径没有变化。

移除了生成文件中的 `huart1`、UART MSP／DeInit、HAL UART 模块开关和两个 HAL UART 构建依赖。CubeMX 新增的六个官方 LL 源文件从 **STM32Cube FW_G4 V1.6.3** 原样复制；其中 USART／GPIO／RCC 使用 LTO 与生成初始化共享代码。没有改写官方函数。

SPI3 的 NSSP 原先由 `main.c` USER CODE 强制清除。现在在 `.ioc` 直接配置 `SPI_NSS_PULSE_DISABLE`，由生成的 `MX_SPI3_Init()` 执行；删除 SPI2／SPI3 的两条启动覆盖，板上有效配置保持一致。SPI2 按编码器／W25Q64 切换相位和分频的运行时例外继续保留。

## SPI／DMA 为什么保留 HAL 初始化

已实际生成三路 SPI 的 LL 版本评估，结果保存在本机 `build/init-r93/spi-evaluate/FOC_two/`：

- 生成代码直接调用 `LL_DMA_Set*` 配置通道，移除 `hspi*`、`hdma_spi*` 及 `__HAL_LINKDMA`。
- 当前运行时、恢复与错误诊断依赖这些生成句柄；R92 正是利用其映射让 DMA 通道变动自动生效。
- 直接采用该输出需要另做句柄／配置桥接或重构运行时和恢复。手工写回旧映射会破坏 CubeMX 配置来源规则。
- ADC 和 WS2812 仍使用 HAL DMA，其初始化／回调依赖也不能随 SPI 一起删除。

因此本轮保留 SPI／DMA 的原生 HAL 初始化与故障重初始化。未引入手写替代初始化；整套原生 LL SPI 未作为完整适配固件交付，所以没有给出其净节省量。ADC、TIM、COMP、DAC、USB、系统时钟保持原初始化方式。

## CubeMX 以后怎么使用

本工程 `.ioc` 已设置好，无需额外手动调整：

| 项目 | 当前配置 |
| --- | --- |
| Project Manager → Advanced Settings → Driver Selector → USART | LL |
| SPI、DMA、其他外设 | 保持现有 HAL |
| USART1 | 115200、8 数据位、无校验、1 停止位、FIFO 关闭 |
| SPI3 NSSP | Disabled |
| Keep User Code | Enabled |

正常修改 `.ioc` 并重新生成即可。USART1 波特率、分频、停止位、过采样、时钟源、GPIO／NVIC 由新生成代码生效，用户运行时不会恢复旧值。字节协议仍不支持 9 位／校验／FIFO／流控／USART DMA 等组合；构建检查拒绝不支持的组合，需一起适配传输代码。SPI 的格式限制和控制算法的采样／时钟限制沿用 [R92 规则](CUBEMX_COMPAT_R92.md)。

`scripts/check_cubemx.py` 同时识别原生 HAL 和 LL UART 初始化。普通构建不运行 CubeMX，仅只读检查当前生成结果；不会在后台修改初始化代码。

原生重生成测试使用 [ST 官方 CLI](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.0/en/docs/markup/CubeMX_CLI.html)，所有生成仅在 `build/` 副本进行。测试重新加载已保存 IOC，**不再发送 setDriver**，确认 LL 选择能够持久保留。

## 实测体积

| 构建 | R92 Flash | R93 Flash | 净节省 | Flash 剩余 | RAM 占用 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 126152 B | **125296 B** | **856 B** | **5776 B** | **27696 B** |
| Release | 126152 B | **125296 B** | **856 B** | **5776 B** | **27696 B** |

RAM 相对 R92 的 27840 B 净减少 **144 B**，该数值含链接预留堆栈，并非运行时栈峰值。本轮初始化属于启动路径，未测量新的实板 Ctrl Max；此前 FOC 45 us、HSO 90 us 不能作为本轮计时结果。

## 验证

- 本机 CubeMX **6.17.0**、G4 固件包 **1.6.3** 重生成：所有 Core C／头文件及 `.ioc` 与交付内容一致。
- 同一 IOC 的副本修改 USART1 为 230400、预分频 2、2 停止位、8 倍过采样、SYSCLK、NVIC 优先级 5；生成值逐项核对，两份原生 USART 初始化均通过真实 ARM 编译。
- 27 项配置接受／拒绝用例、七路 DMA 重新分配及 KEY1 换 EXTI 组；SPI 实际用户驱动的 600 项动态通道重映射回归。
- USART 全双工／错误恢复，SPI／编码器／Flash 恢复、LCD、WS2812、USB、完整电机／HSO／六步／辨识／Follow、UI／存储及单文件上位机回归通过。
- ADC／TIM／Break／KEY1 分发测试及实际采样集成通过；最终 ELF 检查确认控制、保护及通信路径保留，HAL UART 通用依赖与 `huart1` 不再链接。
- Debug／Release 构建与 ELF／BIN 对照通过。源码边界审核确认原有官方文件全部未变，新增官方 LL 文件与原生生成副本逐字节一致；原文件编译选项仅增加 CubeMX 的 `USE_FULL_LL_DRIVER` 宏。

```powershell
python tests/cubemx_host/run.py
python tests/cubemx_host/regenerate.py --cubemx D:/Tools/STM32CubeMX_6.17
python tests/uart_host/run.py
python tests/spi_host/run.py
python tests/ws2812_host/run.py
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
node tests/pc_tool/test_console.js
python tests/pc_tool/test_wire.py
cmake --build --preset Debug --clean-first --parallel 4
cmake --build --preset Release --clean-first --parallel 4
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/init-r93 --allow-native-ll-init
```

本轮未烧录实板。刷入后优先复测 USB／USART 查询与连续收发、Run／Follow、ENC1／ENC2 断线恢复、W25Q64 保存、LCD／WS2812、KEY1 与 IBUS 锁存／手动恢复。

R92 基线和分步报告在本机忽略目录 `build/init-r93/`，重生成结果在 `build/cubemx-regeneration/`。最终 ELF／BIN／MAP 保存在 `build/Debug/`、`build/Release/` 并继续纳入备份。

```text
Debug BIN   d6eb9a4f3409a425686e1579534db5cb7db3933965c09d32bcb875f413024b6c
Release BIN 7d7f21c7692822a3dcdc8f65f2479120d8cd0b94d75fefe5ef9dcb5046efe912
```
