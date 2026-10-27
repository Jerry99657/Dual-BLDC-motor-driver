# R94：GPIO 配置清理与 WS2812 运行时

固件 **UI-R94-LED**。本轮完成按键 GPIO 重复初始化清理和 WS2812 PWM／DMA 运行时迁移。I2C3 保持现状，电机、保护、通信、存储和界面功能保留。

## 配置与运行时边界

`ui_input_init()` 原先重新配置六个导航键的输入模式与上下拉，会覆盖 `MX_GPIO_Init()`。现在只清除扫描状态，电气配置全部来自 CubeMX；按键消抖、长按、连发、KEY2 返回与 KEY1 全局停止规则保持。

按键逻辑仍默认低电平有效。若改成高电平有效，需要同步调整用户代码的 `UI_INPUT_ACTIVE_HIGH_MASK` 和 CubeMX 电气配置；应用不会再根据逻辑极性擅自改上下拉。

WS2812 保留 `MX_TIM3_Init()`、HAL DMA 初始化及 `__HAL_LINKDMA`，仅发送、完成、错误处理改用用户代码和 ST LL 头文件接口。通过 `htim3.hdma[TIM_DMA_ID_CC1]` 的 `Instance`、`DmaBaseAddress`、`ChannelIndex` 和数据宽度跟随新生成的 DMA 分配。DMA 优先级、DMAMUX 请求、半字／字宽配置不被运行时覆盖。

生成的 DMA 中断调用由 `cubemx_irq.h` 识别当前 TIM3 CC1 句柄，交给 `WS2812_DMAIRQ()`；SPI 继续进入原用户分发器，ADC 规则组继续走 HAL DMA。Core、`.ioc`、CMake 以及所有官方库文件和编译选项相对 R93 均未修改。

当前无需在 CubeMX 做额外配置。之后正常修改并生成即可；每次构建只读执行 `scripts/check_cubemx.py`。不支持的计时或组合会明确拒绝构建，提示适配驱动，不会恢复旧初始化值。

TIM3 专用于灯带，当前支持向上计数、内部时钟、CH1 高有效 PWM1、170 MHz／ARR 212、Normal DMA、内存递增，外设和内存宽度均可为半字或字。改计时、共享 TIM3、使用从模式／单脉冲／定时器中断等，需要一起适配 WS2812 驱动。DMA 控制器／通道、优先级和 GPIO 属性继续由生成代码决定。

## 发送与异常处理

- 保留 GRB 排列、原亮度和状态色、2 个前导低电平与 64 个尾部复位时隙。主机寄存器模型验证 0／1 高脉宽分别为 68／136 个计数，位周期 213 个计数，物理复位低电平超过 50 us。
- 发送期间只记录最新待发颜色，活动 DMA 缓冲保持不变；完成后由服务函数发送待更新颜色。
- 仅处理本通道已启用的 TC／TE，TE 优先；带有非零剩余计数的异常 TC 按截断帧处理。清标志只作用于本 DMA 通道，不清相邻 ADC／SPI 标志。
- 停止时先关闭 DMA 请求，再停止计数／DMA，写入零比较值并通过 UPDATE 锁存，随后关闭 CH1 输出。没有访问 TIM1／TIM8。
- 截断帧后等待两个 HAL 毫秒 tick 边界，确保至少 1 ms 的复位低电平，再重发最新状态色；支持 tick 回绕。此重试仅影响灯带。
- 服务函数可补处理未及时分发的 DMA 完成／错误；进入临界区前的 PRIMASK 状态被保留。没有增加 DMA 半传输中断。

最终 ELF 不再链接 `HAL_TIM_PWM_Start_DMA`、`HAL_TIM_PWM_Stop_DMA`、通用 TIM DMA 完成／错误回调。HAL TIM 初始化、ADC DMA 回调和其他运行路径继续保留。

## 实测体积

| 构建 | R93 Flash | R94 Flash | 净节省 | Flash 剩余 | RAM 分配 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 125296 B | **125060 B** | **236 B** | **6012 B** | **27696 B** |
| Release | 125296 B | **125060 B** | **236 B** | **6012 B** | **27696 B** |

GPIO 单独构建净省 48 B；计入新错误复位／重试、路由和版本资源后，总计净省 236 B。RAM 总分配不变，包含链接预留堆栈；新错误时间戳被其他段布局变化抵消，不代表没有新增状态变量。

基线及报告保存在本机忽略目录 `build/gpio-ws-r94/`；最终 ELF／BIN／MAP 继续位于 `build/Debug/` 和 `build/Release/` 并纳入备份。

## 验证与实板复测

- 192 组 DMA1／DMA2 通道、半字／字宽和优先级组合，使用未修改的 ST LL 函数体；模拟真实 PWM 比较事件、CCR 预装载和 DMA 计数，核对 GRB 脉冲及复位低电平。
- TC、TE、同时发生、无效提前完成、禁用／过期来源、IRQ 延迟、活动帧重初始化、最新颜色合并及 tick 回绕回归通过。
- 31 项生成配置接受／拒绝测试，以及七路 DMA 全部重分配、KEY1 换 EXTI 组的实际向量编译与路由测试通过。
- 复核 R93 已保存的 CubeMX 实际重生成结果与串口参数变更结果，继续通过配置审查和 ARM 编译；本轮没有修改生成文件。
- 电机／HSO／六步／辨识／Follow、SPI／编码器／Flash、ADC／TIM／Break／KEY1、USART、USB、UI／存储／LCD及单文件上位机回归通过。HSO 主机测试使用 ST 核心接口桩，不代表实板 HSO 测试。
- Debug／Release 均构建成功，ELF／BIN 一致；211 个受保护源码／配置及官方／Core 编译命令与本轮基线一致。

主要验证入口：

```powershell
python tests/ws2812_host/run.py
python tests/cubemx_host/run.py
python tests/cubemx_host/regenerate.py --cubemx D:/Tools/STM32CubeMX_6.17 --verify-only
python tests/ui_host/run.py
python tests/spi_host/run.py
python tests/irq_host/run.py
python tests/uart_host/run.py
python tests/usb_host/run.py
python tests/motor_host/run.py
node tests/pc_tool/test_console.js
python tests/pc_tool/test_wire.py
cmake --build --preset Debug --parallel 4
cmake --build --preset Release --parallel 4
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/gpio-ws-r94
```

本轮未烧录实板，波形结论来自寄存器模型，没有新增实板 Ctrl Max 计时。刷入后复测状态色切换、连续刷新、KEY1 停止、ENC1／ENC2、W25Q64 保存与电机运行即可；若测波形，重点核对高脉宽和尾部低电平。

```text
Debug BIN   bbcf8819da807a5fef86059c50aba60629ab527f26210805866919e25239aa8c
Release BIN edad2f8227c608892b2d1660612e48047df4d26eb10461dc7c07b05bfe9578ab
```
