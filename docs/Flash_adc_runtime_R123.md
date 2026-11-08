# R123：ADC 规则组 DMA 与用户代码体积优化

固件 `UI-R123-ADC`，Native 配置回执编号 123。本轮按 ADC 规则组 DMA、重复计算、UI 重复片段的顺序实测，保留具有净收益的改动。无需修改 CubeMX。

## 实际占用

| 阶段 | Debug / Release Flash | Flash 余量 | RAM 总分配 |
| --- | ---: | ---: | ---: |
| R122 基线 | 130624 B | 448 B | 30920 B |
| ADC 规则组 DMA 迁移 | 130400 B | 672 B | 30920 B |
| 共用 M1 电流滤波 | 130312 B | 760 B | 30920 B |
| 共用 UI 单位选择及 R123 资源 | **130288 B** | **784 B** | **30920 B** |

净回收 **336 B**，包含新增驱动本身及版本资源。RAM 链接余量为 1848 B，分配包含既有堆、栈和诊断保留区。距 1 KiB Flash 余量目标还差 240 B，后续功能仍需单独计算预算。

## ADC 规则组 DMA

`mycode/adc_port.c/.h` 接管规则组的循环 DMA 启停和中断发布。`HAL_ADC_Start`、`HAL_ADC_Stop` 继续负责 ADC 使能、转换停止、状态和超时；校准及注入组启停沿用原 HAL 路径。

- DMA 实例、控制器、通道编号和 Parent 来自生成的 `hadc1.DMA_Handle`、`DmaBaseAddress`、`ChannelIndex`，格式和请求映射由生成初始化提供。
- 运行时设置本次地址、计数和 HT/TC/TE 中断。四个半字仍表示两组完整的 `[VBUS,VREFINT]` 扫描。
- 沿用 `cubemx_irq.h` 对生成向量的路由。SPI 模块将非 SPI 句柄交给 ADC 规则组驱动，WS2812 使用独立分发。
- DMA TE 先于 HT/TC 处理。硬件清除 EN 后仍识别 TE，先退出传输和清标志，再发布 ADC 错误；损坏数据不更新测量有效期。停机后迟到的 DMA IRQ 只清理标志。
- 沿用 HAL 的错误状态保留规则：普通 Stop/Start 不清除 ADC 内部/DMA 错误状态，不会把故障当成已恢复。正常无故障启停可重新采样。
- 半传输和完成继续调用现有 ADC 应用回调。VBUS/VREFINT 平均、滤波、母线恢复检测和输出保护使用原应用实现。

最终两个镜像均已退出 `HAL_ADC_Start_DMA`、`HAL_ADC_Stop_DMA`、`HAL_DMA_Start_IT`、`HAL_DMA_Abort`、`HAL_DMA_IRQHandler` 和通用 ADC DMA 回调链。新实现的最终净收益为 224 B。

`scripts/check_cubemx.py` 新增检查 ADC DMA 父句柄链接、扫描模式及不支持的 DMAMUX 同步/请求生成器。新增其他 DMA 使用者时，构建会要求先扩展路由。通道重映射、优先级和当前支持的生成配置继续由 CubeMX 决定。

## 重复计算与 UI

- M1 的停止监测、开环诊断、正常闭环共用 `filter_m1_current`。保持 q 轴历史共享、普通与开环 d 轴历史分别管理，原浮点运算顺序沿用，净回收 88 B。
- Run 和 Run Steps 共用 `run_unit`，单位、文字、字号、坐标和输入规则沿用，净回收 24 B。

已撤回的试验包括将新滤波函数放入 RAM（少省 8 B 且多用 88 B RAM）、强制 Park 禁止内联（0 B）、用表遍历清空四个编码器诊断字段（0 B）。

## 验证

- ADC 寄存器模型执行实际新驱动，覆盖两个控制器/全部 12 个通道、相邻通道标志隔离、HT/TC 合并、TE 与完成并发、硬件 EN 清零、迟到 IRQ、正常重新启动、HAL 错误状态保留、HAL 启停失败和 PRIMASK 保留。HAL 边界替身按本地官方实现保留内部/DMA 错误状态，不以重新 Start 模拟清错。
- 55 项生成配置变化的接受/拒绝回归，以及七路 DMA 和 KEY1 中断组完整重映射通过。
- 真实采样/保护与 LL ADC 中断集成、低压后恢复、VREFINT、双轴、KEY1/IBUS、缺帧及硬件 Break 回归通过。高层采样测试对 ADC 端口使用接口替身，寄存器行为由独立端口测试覆盖。
- 原版与共用滤波版通过同一电流环/RL 模型回归。6000 组停止监测/开环/FOC 交替输入逐位匹配独立浮点公式，两版测试输出逐字节相同。
- FOC、电流饱和/前馈、Follow、独立 HSO、辨识、编码器恢复/SPI2 仲裁、协议、USB/USART 和 Native/JustFloat 回归通过。
- 68 份 UI 截图/动画流、64857 次文字绘制和交互输出与基线一致。比较统一 About 固件文字；870 条诊断查询也逐字一致。
- 两个实际 ARM 镜像的启动/MSP/Break/KEY1/IRQ、各 139 项 CPU 异常保护、两轴外环时序保护仿真以及 ELF/BIN 一致性检查通过。
- HTML 兼容上限更新为 123，前端及 Edge 模拟 Web Serial 格式握手和 Stop 回归通过。

207 个受保护文件中，仅根 CMake 增加用户 ADC 模块及其 `-Os -flto` 编译项。ST 官方源码、生成初始化及 IOC 的散列通过核对。CMake 重新配置使 Release 命令中重复的一组目标/警告/分段选项只保留一份，共计 73 条命令。报告只允许这组完全相同选项的重复消除，并核对其余选项逐项一致。

本轮未烧录开发板，实板 ADC/PWM 时序和 `Ctrl Max` 待验证。上板重点检查母线延迟接入、VBUS/VREFINT 连续更新、反复启停、双机/HSO 和全局停止。CubeMX 无需重新配置。

## 产物和复核

ELF/BIN/MAP 已更新至 `build/Debug/`、`build/Release/`。R122 基线、试验日志与 `final.json` 保存在忽略目录 `build/adc-dma-r123/`。

```powershell
python -X utf8 tests/adc_port_host/run.py
python -X utf8 tests/cubemx_host/run.py
python -X utf8 tests/spi_host/run.py
python -X utf8 tests/irq_host/run.py
python -X utf8 tests/motor_host/run.py control sampling foc current inverter sensorless motion detect encoder protocol telemetry justfloat
python -X utf8 tests/ui_host/run.py --view-baseline build/adc-dma-r123/mycode/ui_app.c --formatter-baseline build/adc-dma-r123/mycode/ui_app.c
python -X utf8 tests/motor_host/check_startup.py
python -X utf8 tests/diag_host/check_arm.py --both
python -X utf8 tests/diag_host/check_outer_arm.py
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/flash_report.py final --allow-adc-user-build --directory build/adc-dma-r123
```
