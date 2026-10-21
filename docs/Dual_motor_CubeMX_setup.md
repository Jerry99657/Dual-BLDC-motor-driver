# 双电机有感控制：CubeMX 前置配置

当前 **UI-R36-SMOOTH** 已核对重新生成的外设配置并接入双有感控制；本页的准备阶段说明保留作历史参考。当前功能、逐路型号设置、HSO 入口及上板步骤以 [Dual_motor_control.md](Dual_motor_control.md) 为准。

核查对象：STM32G431VBT6，当前 FOC_two.ioc、Core/Src/spi.c、tim.c、adc.c、dma.c。本文是接入第二套控制环前的配置清单，不表示双电机控制已完成。

已开始的应用层修改：motor_params 增加 M1/M2 分别选择型号的接口；现有 GetActive/Select 固定对应 M1，保持现有有感/HSO 控制和保护行为。M2 PWM 输出目前仍禁用，原来的可选 ENC2 五次失败停止探测策略保留。后续双电机仅使用编码器反馈，M1 HSO 仅在单 M1 模式开放。

## 需要在 CubeMX 修改

### 1. SPI2 添加 RX 和 TX DMA

进入 Connectivity → SPI2 → DMA Settings，新增两条请求。当前 DMA1 Channel1～5 已分别用于 SPI1 RX、SPI1 TX、TIM3_CH1、SPI3 TX、ADC1；采用以下分配避免移动这些通道：

| 项目 | SPI2_RX | SPI2_TX |
| --- | --- | --- |
| DMA 通道 | DMA2 Channel1 | DMA2 Channel2 |
| 方向 | Peripheral to Memory | Memory to Peripheral |
| Mode | Normal | Normal |
| Peripheral increment | Disable | Disable |
| Memory increment | Enable | Enable |
| Peripheral / Memory data width | Byte / Byte | Byte / Byte |
| DMA Priority | High | High |
| DMAMUX Synchronization | Disable | Disable |

Normal 模式由应用每个采样周期提交一帧 3 字节，不使用 Circular 连续拉时钟。

进入 NVIC Settings，启用 DMA2 Channel1、DMA2 Channel2 和 SPI2 global interrupt，三者抢占优先级均为 1、子优先级 0。保持 ADC1_2=0；现有 TIM2=3、LCD SPI3/DMA=4、USB=5 继续保留。

SPI2 的默认参数继续供 W25Q64 使用：Full-Duplex Master、8 bit、MSB first、Software NSS、CPOL Low、CPHA 1 Edge、分频 /8。若有 NSS Pulse 选项，选择 Disable。不要为了编码器把整个 SPI2 默认改为 Mode 1；编码器事务将在应用代码里临时切换到 CPOL Low、CPHA 2 Edge、/256，完成后恢复 Flash 参数。170 MHz 时 /256 约为 664 kHz，24 bit 的线传输时间约 36 us，与 M1 保持一致。

### 2. TIM8 六个 PWM 引脚使用下拉

在 TIM8 → GPIO Settings，将下面六个引脚的 GPIO Pull-up/Pull-down 设置为 Pull-down，仍保持 Alternate Function Push Pull 和原有 AF：

| 引脚 | 功能 |
| --- | --- |
| PC6 | TIM8_CH1 |
| PC7 | TIM8_CH2 |
| PC8 | TIM8_CH3 |
| PB3 | TIM8_CH1N |
| PB4 | TIM8_CH2N |
| PB5 | TIM8_CH3N |

与 M1 的 PE8～PE13 一致，供主动高电平的驱动输入在相输出断开时保持低电平。PWM 极性 High、互补极性 High、两类 Idle State Reset、Dead Time=17 均沿用现值；不要因加入第二电机而修改死区。SYS Debug 使用 Serial Wire，保留 PB3/PB4 作为 TIM8 互补输出。

## 继续沿用的配置

- TIM1、TIM8：Prescaler=0、ARR=8499、Center Aligned Mode 3、ARR Preload Enable，时钟 170 MHz，PWM 约 10 kHz。
- TIM1：TRGO=Enable、Master/Slave Enable；TRGO2=OC4REF，CH4 为 PWM1、Pulse=1，供 ADC 注入触发。
- TIM8：Slave Mode=Trigger Mode、Input Trigger=ITR0。从机不另设一套 ADC 中断，不启用额外 PWM update 中断。
- 双 ADC 注入同时模式，时钟同步 /4=42.5 MHz，ADC1 使用 TIM1 TRGO2 上升沿，四个 Rank 保持下表。

| Rank | ADC1 | ADC2 | 采样周期 |
| --- | --- | --- | --- |
| 1 | IN1 / PA0 / M1 IU | IN2 / PA1 / M1 IV | 12.5 / 12.5 |
| 2 | IN6 / PC0 / M1 IW | IN8 / PC2 / M1 VV | 47.5 / 47.5 |
| 3 | IN7 / PC1 / M1 VU | IN9 / PC3 / M1 VW | 47.5 / 47.5 |
| 4 | IN14 / PB11 / M2 IU | IN5 / PC4 / M2 IV | 12.5 / 12.5 |

M2 两相电流同时采样，第三相在控制代码中由 Iw=-(Iu+Iv) 重建，无须增加第三个 ADC 通道。

- ADC1 规则组：Rank1=IN15/PB0/VBUS，47.5 周期；Rank2=VREFINT，640.5 周期。扫描长度 2、Continuous Disable、TIM6 TRGO 上升沿、DMA Continuous Requests Enable。
- ADC1 DMA：DMA1 Channel5、Circular、Half Word / Half Word、Memory Increment Enable。VREFINT 不另加 DMA，也不启用 VREFBUF 输出。
- TIM6 保留 PSC=169、ARR=499、TRGO Update。应用继续在注入组安全窗口触发其软件 UPDATE，不自由运行，不另开定时中断。
- 编码器片选：M1 PA4、M2 PB12（MT6701_CSN2），Flash 片选 PD15，均为 GPIO Output、初始 High。不把 PB12 改成 SPI2 硬件 NSS。
- SPI1 保留 CPOL Low、CPHA 2 Edge、/256；SPI3 保留 /16，避免重新引入 LCD 时序问题。

## 生成后核查

Project Manager → Code Generator 勾选 Keep User Code when re-generating。生成到原工程，保留原有应用 CMakeLists.txt 的源文件、ST HSO 库和 Debug 优化配置。

预计生成结果：

- spi.c/spi.h 有 hdma_spi2_rx、hdma_spi2_tx，分别链接 hspi2.hdmarx/hdmatx；两条 DMA 采用上述字节宽度与 Normal 模式。
- dma.c 开启 DMA2 时钟，并启用两个 DMA2 IRQ。
- stm32g4xx_it.c 有两个 DMA2 Channel IRQ handler 和 SPI2_IRQHandler。
- tim.c 的 TIM8 六个 GPIO 均使用 GPIO_PULLDOWN；TIM1/TIM8 的同步参数保持原值。
- adc.c 中仍有 VBUS + VREFINT 两个规则转换和现有四个注入 Rank。

此时尚不能只烧录新增 DMA 初始化就试运行 M2。应用还需要将 ENC2 从主循环诊断轮询改为 10 kHz DMA 采样，接入第二套 FOC/电流保护/对齐，并处理 SPI2 总线所有权。M2 连续采样期间，Flash 存取延后到 M2 停止并释放总线；待保存设置保留在 RAM，不能让 Flash 事务打断编码器帧。默认仍选择单 M1，进入双电机不自动运行，两路完成就绪检查后显式启动。

STM32G431 的通道资源和 DMAMUX 路由参考 [ST RM0440](https://www.st.com/resource/en/reference_manual/dm00355726.pdf) 及 [AN5094](https://www.st.com/resource/en/application_note/dm00442720-migrating-between-stm32f334303-lines-and-stm32g474xxg431xx-microcontrollers-stmicroelectronics.pdf)。G431 的 DMA1、DMA2 各有六个通道，不能分配 DMA1 Channel7。
