# CubeMX 与用户驱动规则

- `.ioc` 及 CubeMX 生成的初始化代码管理引脚、时钟、通信格式、DMA/DMAMUX 映射和 NVIC 配置。HAL/LL 混用也必须遵守；禁止用写死的用户初始化跳过或覆盖生成配置。
- 不修改 ST 官方 HAL、LL、CMSIS 或 USB 中间件源码。保留现有功能及保护。
- 用户运行时驱动从生成的句柄/寄存器读取配置。SPI DMA 必须依据 `hdmarx/hdmatx`、`Parent`、`DmaBaseAddress`、`ChannelIndex`，WS2812 使用生成的 TIM3 CC1 DMA 链接与数据宽度，不得固定 DMA 控制器、通道或物理 IRQ 归属。
- 生成 IRQ 的 DMA 调用通过 `mycode/cubemx_irq.h` 路由；不要在 DMA USER CODE 区域添加固定设备调用/提前返回。KEY1 的生成 EXTI 调用也由该头文件路由，以适应换引脚/中断组。
- SPI2 的编码器/W25Q64 分频与相位切换为明确的设备协议例外。PWM 占空比/输出门控、ADC 触发调度、DAC 电流阈值等业务运行时动作继续由应用管理。
- 驱动或算法不支持的新格式/采样顺序/控制频率必须明确拒绝构建并说明适配点，不得静默恢复旧值。同步维护 `scripts/check_cubemx.py`；每次正常构建都执行该检查。
- 在初始化精简无法保留 CubeMX 配置来源时，保留 HAL 初始化。初始化迁移与故障重初始化必须保持一致。
- 改动后运行有意义的配置变更与驱动回归；构建 Debug/Release 并报告实际 Flash/RAM 变化。ELF/BIN/MAP 继续备份。
