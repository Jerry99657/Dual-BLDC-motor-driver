# R82：HAL＋LL 第一轮迁移（第 1～4 批）

固件 `UI-R82-LL`。完成基线保存、KEY1 GPIO／EXTI、TIM 和 ADC 中断迁移。初始化、校准、HAL 时基、DMA、SPI、USART、USB 等继续使用既有实现；本轮无需修改 CubeMX。

## 完成范围

| 批次 | 实现 |
| --- | --- |
| 1：基线与接口 | 保存 R81 的 ELF／BIN／MAP、用户源码、Core、CubeMX 配置、构建文件和 205 个受保护文件的哈希；新增独立的 `motor_irq` 用户模块 |
| 2：GPIO／EXTI | KEY1 电平／待决标志读取使用 LL；EXTI 只清除 KEY1 对应的 W1C 标志，再执行既有全局停止逻辑 |
| 3：TIM | TIM2 只分发启用的 UPDATE 事件；TIM1 分发普通 Break、系统 Break，并对意外 Break2 采取相同锁存处理 |
| 4：ADC | ADC12 中断直接处理当前板级配置的完整注入帧及错误；保持两 ADC 配对、规则组 DMA 和 HAL 启停状态兼容 |

`mycode/motor_irq.c/.h` 只负责硬件事件分发。`JerryFOC_FastIRQ()` 与 `JerryFOC_OuterIRQ()` 是原有 10 kHz／1 kHz 应用入口；原 HAL 回调保留为薄适配器，共用同一份业务代码。Break 和 ADC 错误同样委托给采样模块的应用接口。

### 中断语义

- **KEY1**：清除本行 EXTI 标志后执行原全局关断。共享向量中的其他 EXTI 标志保留。低电平兜底、待决边沿检查、30 ms 释放资格及手动重启规则保留。
- **TIM2**：同时检查 UIE 与 UIF，按定时器 rc_w0 语义仅确认 UPDATE，再执行原外环。外环期间出现的新 UPDATE 保留，其他状态位不被清掉。
- **TIM1 Break**：检查 BIE 与 BIF／SBIF／B2IF，确认事件后锁存 M1 故障、撤销功率请求并禁止持续故障造成中断风暴。COMP4→TIM1 的硬件 MOE 关断、阈值、死区、自动输出禁止和手动清除资格未改变。Break2 在当前配置中仍为关闭。
- **ADC12**：先检查两路的 `ISR & IER`。启用的 OVR／JQOVF 记录 HAL State／ErrorCode，并在本次控制执行前关断功率；本次完整帧被丢弃。未启用的普通完成标志不会触发业务。
- **完整帧**：仅 ADC1 JEOS 调度快速入口。ADC1 的 JEOS／JEOC 在入口前确认，避免清掉控制期间新到的完成事件；ADC2 的 JEOS 保留给 `MotorSampling_Capture()` 检查、读取和清除。缺少 ADC2 完成标志继续按原规则报配对故障。
- **配置约束**：当前只允许主 ADC JEOS、OVR／JQOVF 错误中断。意外启用的 JEOC、规则组完成、看门狗或从 ADC 完成中断按配置故障处理，记录 INTERNAL、屏蔽该源并关断输出，不执行半帧控制。以后启用这些能力时，需要同时扩充分发器。

规则组 DMA 的半满／全满／错误回调仍由 HAL 管理，VBUS 与 VREFINT 的调度、滤波、换算和故障后继续采样规则保留。外部触发注入组保持 HAL 的 BUSY 状态，在完成时补充 INJ_EOC；启停／校准／DMA 错误仍由原 HAL 接口管理。本轮没有自行改写 HAL 句柄锁或 DMA 状态机。

## CubeMX 与源码边界

`Core/Src/stm32g4xx_it.c` 仅在 USER CODE 的 Includes 和四个 IRQ 的入口区域增加用户调用与 `return`。生成的 HAL 分发语句保留原样，但不再可达；链接检查确认三个通用 HAL IRQ 分发器已经移除。

当前 `.ioc` 的 `ProjectManager.KeepUserCode=true`。重新生成时保留这些 USER CODE 区域即可；不需要切换 Driver Selector，不需要改 ADC、PWM、时钟、NVIC 优先级或引脚配置。TIM1_BRK_TIM15 共享向量目前没有启用 TIM15；以后增加 TIM15 时需补充对应分发。

根 CMake 仅增加 `motor_irq.c` 及该用户文件的 `-Os;-flto` 选项。ST HAL／LL、CMSIS、USB 中间件、HSO、KK_UI／KK_OLED 核心、所有外设初始化、链接脚本、`.ioc` 和官方／原 Core 文件编译命令均与 R81 基线一致。

`tests/flash_report.py --allow-irq-user-code` 只允许上述五个 USER CODE 区域和精确的 CMake 用户模块增量；其余受保护内容继续严格比较。默认执行仍拒绝任何受保护文件变化。

## 实际体积

Debug 与 Release 的占用一致：

| 阶段 | Flash | 剩余 | 本批净节省 |
| --- | ---: | ---: | ---: |
| R81 基线 | 131068 B | 4 B | — |
| 第 2 批完成 | 131060 B | 12 B | 8 B |
| 第 3 批完成 | 130844 B | 228 B | 216 B |
| 第 4 批完成 | **130476 B** | **596 B** | **368 B** |

合计净节省 **592 B**；RAM 仍为 **23144 B**（含预留堆栈），没有新增持久状态。

原 `HAL_TIM_IRQHandler` 的 4496 B、`HAL_ADC_IRQHandler` 的 6492 B 包含 LTO 内联的外环、FOC、HSO 和协调器业务。迁移后这些业务仍需保留，因此不能把原符号大小当作净节省量。本轮已移除通用分发依赖，但没有释放数 KB；596 B 余量仍不足以支持较大的新功能。初始化与 SPI／DMA 的迁移属于后续批次。

## 验证与交付

```text
python tests/irq_host/run.py
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/usb_host/run.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/ll-r82 --allow-irq-user-code
```

新增 IRQ 测试从当前 ST 头文件提取未经修改的 LL 函数体和寄存器常量，只模拟外设读写语义。验证 EXTI W1C、TIM rc_w0、共享标志保留、禁用中断源、新事件待决及 **8388608 种 ADC ISR／IER／轴组合**。随后接入真实 `motor_sampling.c`，覆盖完整帧、缺少配对、ADC 错误、KEY1、BIF、SBIF 和 B2IF，验证实际功率门控、诊断继续更新及 HAL 启停接口可用。

原电机、HSO、六步、辨识、双机、编码器恢复、UI／存储／LCD、USB 回归与最终链接检查一起验证。链接检查要求实际向量进入用户 LL 分发器，且 FOC／HSO／辨识／主从／KEY1／Break 路径仍在固件中；检查 ELF 与 BIN 一致。

本轮未烧录实板，未测量新的 Ctrl Max。实板复测重点是：

1. 停止状态下 VBUS／VREFINT 连续更新，后接母线仍可识别。
2. Single M1／M2、双独立与 Follow 的启动、改速和停止。
3. KEY1 任意页面停止两轴，松开不自动恢复；Run Back 与 Follow 再按 OK 正常停止。
4. 原先稳定工况下的 HSO、六步及参数辨识，以及 IBUS 触发后的锁存／手动清除。
5. 比较 Ctrl Max、ADC 错误计数与编码器错误计数。Ctrl Max 仍是应用控制段耗时，不代表整个中断的所有指令耗时。

R81 回退和各批产物保存在本机忽略目录 `build/ll-r82/`。当前可备份的最终 ELF／BIN／MAP 位于 `build/Debug/`、`build/Release/`。

最终 BIN SHA256：

```text
Debug   9e50c41a5c87c8d27908ad8ae7de573234e522cb64f9d6d9d11fd7a15314dafd
Release d1826c508686f33a65435bba79f0f0c7b3b31de8311e162da2586cabadb1d3f8
```
