# UI-R39-SPI332：两路编码器降速验证

本页保留 R39 的降速实验记录。当前 R42 已按用户要求恢复 APB1/APB2=170 MHz、编码器约 664 kHz，并对启用的 ENC2 增加低频重连；当前设置、烧录标识和资源见 [Encoder_reconnect_clock_restore.md](Encoder_reconnect_clock_restore.md)。下文 85 MHz、332 kHz 和 Flash /4、LCD /8 均为历史配置。

本版只调整通信时钟，保留 R38 的控制参数和主从算法。目的在于比较相同接线下 ENC1/ENC2 的通信错误，不代表已经完成实板验证。

## 调整方法

STM32G431 的硬件 SPI 波特率最大分频仍为 /256。本版将 APB1、APB2 从 170 MHz 改为 85 MHz，使编码器时钟进一步减半，继续使用原来的硬件 SPI + DMA。

| 链路 | 原时钟 | 当前时钟 | 24 位纯线传输时间 |
| --- | --- | --- | --- |
| ENC1 / SPI1 | 664.06 kHz | 332.03 kHz | 36.14 → 72.28 μs |
| ENC2 / SPI2 编码器事务 | 664.06 kHz | 332.03 kHz | 36.14 → 72.28 μs |

两路仍为 CPOL Low、CPHA 2 Edge、MSB first，完整读取角度、状态和 CRC。未放宽 CRC、磁场、样本超时、掉线或电流保护；ENC2 的五次连续失败停止与显式重试也保留。采样仍由 10 kHz 调度，72.28 μs 只是线传输时间，不包含 DMA 提交和回调延迟。更慢的速度不能直接塞入当前 100 μs 周期；后续若需要 100～200 kHz，应另行设计异步采样及延迟补偿。

## 共享时钟补偿

CPU、AHB 保持 170 MHz。APB 分频为 /2 时，TIM 内核时钟为 2 × PCLK，仍为 170 MHz，因此 TIM1/TIM8 的 10 kHz PWM、TIM2 的 1 kHz 外环、TIM3 的 WS2812 波形和 TIM6 计数时基均保留。ADC 同步时钟取 HCLK/4，继续为 42.5 MHz。

| 项目 | 当前配置 | 结果 |
| --- | --- | --- |
| SPI2 默认 Flash 事务 | 85 MHz /4，Mode 0 | 保持 21.25 MHz |
| SPI2 编码器事务 | 临时切为 /256、Mode 1，完成后恢复 CR1/CR2 | 332.03 kHz，不影响 Flash 的配置恢复 |
| SPI3 LCD | 85 MHz /8 | 保持 10.625 MHz |
| I²C3 | 改从 SYSCLK 取 170 MHz | 保留现有 TIMINGR 和总线时序 |
| FDCAN | 改从 PLLQ 取时钟，PLLQ 从 /4 改 /2 | 保持 170 MHz 内核时钟和现有位时序 |
| USART1 | PCLK2=85 MHz，HAL 初始化重新计算 BRR | 仍设为 115200 baud |
| USB CDC | HSI48 + 原 CRS | 保持 48 MHz |

已同步修改 `FOC_two.ioc` 和 `Core/Src` 初始化代码，本轮不需要先操作 CubeMX。今后重新生成时保留这些时钟选择，SPI2 默认 /4、SPI3 /8；不能只把 APB 改回 /1 而保留新分频，否则 Flash/LCD 将变为原速的两倍。

依据：[ST STM32G431 数据手册及其时钟树](https://www.st.com/resource/en/datasheet/stm32g431cb.pdf)、[RM0440 参考手册](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)，以及工程自带 HAL 的 RCC/ADC/SPI 实现。未采用在 DMA 前后加延时来代替 SCK 降速。

## 上板比较

1. 本节为 R39 实验流程；当前 `build/Release/FOC_two.bin` 已更新为 R42，时钟和上板检查请使用上方当前文档。
2. 使用与此前相同的接线、供电和工作模式，先查看静止时，再查看运行时的错误增量。
3. Status 的 `Enc CRC`、`Enc SPI`、`Enc Frame`、`Enc Age`、`Enc Last` 跟随当前查看轴；另有 `Enc2Err` 累计值。比较相同运行时长内计数增加多少，不把上一次累计值当成本次新增错误。
4. 若出现掉线，仍区分 `ENCODER1 LOST` / `ENCODER2 LOST`。磁场类 `FIELD` / `TRACK` 与通信 CRC 错误分别判断；降速不保证消除接线干扰或磁铁安装问题。

本版尚未烧录或实测。CPU 频率与定时频率虽保留，APB 寄存器访问和 DMA 中断延迟仍需上板观察，实际闭环耗时不能用主机模型替代。

## 验证与构建

`python tests/motor_host/check_spi_clock.py` 从初始化代码计算频率，并检查与 `.ioc` 一致及各共享时钟补偿；`python tests/motor_host/run.py` 同时运行它。编码器、实际 W25Q64/存储共享总线、控制和保护回归通过。最终启动顺序及 ELF/BIN 一致性检查通过。

R39 历史构建：Debug Flash **130996 / 131072 B**，余 **76 B**；RAM **21616 B**。Release Flash **130220 B**，余 **852 B**；RAM **21520 B**。两种构建均可用，保留已有 RWX LOAD 链接警告。
