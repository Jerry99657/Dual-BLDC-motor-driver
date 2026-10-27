# R92：CubeMX 配置来源与 LL 运行时适配

固件 **UI-R92-CMX**。按项目规则：无论 HAL 还是 LL，CubeMX 生成的初始化都是引脚、时钟、数据格式、DMA/DMAMUX、NVIC 的配置来源。不能靠用户驱动静默恢复旧配置。运行时不支持的组合必须说明并拒绝构建；不能以节省 Flash 为理由绕过配置。

本批没有推进第 7 批初始化替换。全部 `MX_*_Init` 与 HAL 初始化/重新初始化继续执行。后续如果不能让 LL 初始化遵守同一配置来源，就保留 HAL 初始化。这项约束也记录在根目录 `AGENTS.md`。

## 已修复

R91 曾把 SPI1 固定到 DMA1 CH1/2、SPI2 固定到 DMA2 CH1/2、SPI3 固定到 DMA1 CH4，并在对应 IRQ 的 USER CODE 中提前返回。这不仅会忽略新映射，重新生成后保留的旧入口还可能错误处理已转给 ADC/WS2812 的通道。

R92 的用户 SPI 运行时从生成并初始化的句柄读取：

- `hspi.hdmarx/hdmatx`：真正关联的接收/发送 DMA。
- `hdma.Parent`：DMA 所属外设。
- `hdma.DmaBaseAddress`：实际 DMA 控制器。
- `hdma.ChannelIndex`：HAL 初始化计算的标志偏移；换算为 ST LL 通道索引。

RX/TX 可使用不同 DMA 控制器；配置、完成、错误、超时中止只操作关联通道，DMA 优先级、方向、宽度和 DMAMUX 请求沿用初始化。运行时仍为当前应用的 8 位、普通模式 DMA，不支持的新格式由构建检查拒绝。

`stm32g4xx_it.c` 的 USER CODE Includes 引入 `cubemx_irq.h`。它仅在这个应用翻译单元中将生成的 `HAL_DMA_IRQHandler(&当前句柄)` 转给用户分发器；SPI 进入 LL，ADC/WS2812 返回原 HAL。原 ST 函数源码没有改写，其他源文件调用也不受宏影响。删除了五处 DMA USER CODE 固定入口，所以新增/移动的生成向量也自动使用新句柄。

KEY1 同样通过生成的 `HAL_GPIO_EXTI_IRQHandler(KEY1_Pin)` 路由，Pin/Port 使用生成标签；从 EXTI15_10 换到 EXTI9_5 等分组不需要复制旧的 USER CODE 调用。其他 EXTI 引脚继续走 HAL。必须保留 KEY1 标签并在 CubeMX 配置下降沿和对应 NVIC。

## 其他外设审查

| 部分 | 配置如何生效 / 当前边界 |
| --- | --- |
| SPI1/3 | 初始化/故障初始化继续 HAL；运行时保留 CR1 分频、CPOL、CPHA、位序等配置，仿真验证不会写回固定模式 |
| SPI2 | 编码器与 W25Q64 的设备协议例外：/256、2EDGE 与 /8、1EDGE 运行时切换；当前两设备要求 8 位全双工、低极性、MSB、软件 CS、NSS pulse 关闭，构建检查明确限定这些条件 |
| USART1 | 波特率、时钟选择、分频、停止位、过采样由生成初始化生效；LL 只管理收发中断。当前字节协议要求 8 位、无校验、无流控、关闭 FIFO，不支持 USART DMA |
| GPIO / NVIC | 初始化保留；引脚复用、速度、DMA 优先级与 IRQ 优先级不由用户 SPI 驱动重设。关键 IRQ 需启用 |
| ADC | 初始化、校准、规则组 DMA 保留 HAL。应用读取固定的双 ADC 四注入 Rank 映射与规则组 [VBUS,VREFINT]。HSO 依赖 42.5 MHz ADC 与注入采样孔径；改变这些条件必须先适配算法，构建会报错 |
| TIM1/8 / TIM2 | 生成配置仍执行；当前算法要求 170 MHz、10 kHz PWM/快环、1 kHz 外环及既有同步/触发关系。PSC/ARR 等改变超出当前契约会拒绝构建，而不会被驱动悄悄复原。死区在现有模式下可由 CubeMX 修改 |
| TIM3 / WS2812 | 保留 HAL PWM/DMA，DMA 移动由新路由自动适配；缓冲按生成的半字/字内存宽度排列，保留 800 kHz 波形约束 |
| COMP4 / DAC3 | 保留 HAL 初始化，检查既有 IBUS→COMP4→TIM1 Break 和 DAC3 CH2 内部阈值链路 |
| USB / I2C / FDCAN / HAL 时基 | 继续使用原初始化及驱动，本批没有缩减/迁移这些功能 |

PWM 占空比、启停/故障门控、六步相位和触发窗口、IBUS DAC 阈值属于应用运行时控制，不是对初始化来源的替代。TIM6 由采样模块发软件 UPDATE，TIM7 是六步调度器管理的一次性计时资源。它们的既有运行策略继续保留。

TIM1 Break 与 TIM15 共用向量；当前分发器不支持 TIM15 事件，若生成代码新增 TIM15 IRQ 调用，检查会提示先扩充分发器。

同时修复 WS2812 的既有不一致：生成配置是半字内存传输，原缓冲却使用 uint32_t 元素，DMA 会在每个占空比后读到一个多余的零半字。`HAL_TIM_PWM_Start_DMA` 的 uint32_t 指针签名不会自动转换内存元素宽度。现在使用同样大小/对齐的联合缓冲，依据生成句柄的 MemDataAlignment 按 uint16_t 或 uint32_t 排列；两种宽度都有完整 GRB/复位时隙测试。原代码在半字测试中失败，修复后通过。未修改 CubeMX 或 HAL。

## 每次构建检查

根 CMake 新增 `cubemx_config_check` 依赖，Debug/Release 每次普通构建都运行 `scripts/check_cubemx.py`，包括源文件未改变时。需要 Python3；本机 CMake 找到 `D:/python/python.exe`。检查只读取生成 C，不修改 `.ioc` 或源码，也不增加 MCU 上的配置表/RAM。

检查范围包括支持的格式、实际 DMA 归属与对应生成向量、通道冲突、请求/方向/数据宽度、关键 IRQ、ADC 映射/采样孔径、控制频率、KEY1 分组以及保护链路。支持的 SPI 分频、DMA 通道/优先级、USART 波特率等没有锁定到 R91 的旧数值。

修改流程：在 CubeMX 配置并生成 → 常规构建自动检查 → 支持的变更直接使用生成配置；不支持的变更显示字段、实际值和适配位置，并终止构建。不要为通过检查而在用户代码中写回旧值。

CubeMX 保持 `Keep User Code when re-generating` 开启，保留 `cubemx_irq.h` 的 USER CODE 引入。暂时维持当前 HAL Driver Selector：直接改成 LL 会改变生成的句柄接口，需要配套迁移，检查不会将它当作已适配。

## 体积与验证

| 构建 | R91 Flash | R92 Flash | 净变化 | 剩余 |
| --- | ---: | ---: | ---: | ---: |
| Debug | 126236 B | 126152 B | 节省 84 B | 4920 B |
| Release | 126236 B | 126152 B | 节省 84 B | 4920 B |

RAM 分配 27840 B，包含既有堆栈预留，保持不变。205 个受保护文件中，仅根 CMake 与 IRQ USER CODE 的精确变更被允许，其余 203 个逐字节一致；官方/原 Core 编译参数一致。

验证包括：

- 真实用户 SPI 模块 + 本地 ST LL 函数体，600 组跨 DMA1/2、全部 12 个通道的重映射，RX/TX 分属不同控制器，三端口并发，TE/TC/BSY/中止以及 ADC/WS2812 隔离。
- 27 项生成配置变更的接受/拒绝测试；在临时副本重生成七个 DMA 归属/向量，移动 KEY1 到 EXTI9_5，编译真实路由头和生成向量，验证各自的新句柄。
- WS2812 真实用户模块按生成 DMA 半字/字宽度输出，验证精确 GRB 波形、低电平复位时隙、等待刷新、提前完成与错误恢复。
- USART 原配置寄存器保留、完整电机/编码器/Flash恢复与存储、UI/LCD、ADC/保护分发、USB、单 HTML 上位机回归。
- Debug/Release 全量构建与实际 ELF/BIN 检查。HAL DMA 分发因从 LTO 用户模块调用可被内联，但 ADC/WS2812 的 HAL 回调与状态机仍在，不表示移除了 HAL DMA。

```powershell
python tests/spi_host/run.py
python tests/cubemx_host/run.py
python tests/uart_host/run.py
python tests/ws2812_host/run.py
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
node tests/pc_tool/test_frontend.js
python tests/pc_tool/test_wire.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/cubemx-r92 --allow-cubemx-user-code
```

主机回归不访问串口、不烧录实板。重新生成场景在临时副本验证，没有改变当前板子的引脚、频率或通道。实板仍应核对 Run/Follow、断线恢复、保存/LCD、KEY1/IBUS 保护；用户前报 FOC 45 us、HSO 90 us 是 R91 的板上观测，本批未测量新的控制耗时。

R91 基线与报告在忽略目录 `build/cubemx-r92/`；最新备份产物位于 `build/Debug/` 与 `build/Release/`。BIN SHA256：

```text
Debug   5db0d612ee504691f4de48bf4a589650c17e1a0537c898adb9255ef1faa66ed7
Release ac7fa2407d8463083c29e3db49195c15dc52484a4ef06ee2a6c8e751bc7a4cf1
```
