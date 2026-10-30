# R117：SPI DMA 错误中断与 Follow 停止修复

## 现象与证据范围

R116 上板反馈：Follow 启动后旋转主机，从机震颤；OK 无法停止；KEY1 全局停止后调试器停在 `SpiPort_DMAIRQ()`。截图中的句柄是 `hdma_spi1_rx`，其 Parent 是 `hspi1`，因此该次停点属于 ENC1 的 SPI1 RX DMA。

截图没有 CCR、DMA ISR、PRIMASK 或 IPSR，不能仅凭函数停点断言硬件已发生 TE，也不能确定全部震颤都由同一原因引起。本轮定位并通过故障注入复现了两个独立的软件缺陷；实际板上触发条件仍需复测。

## 1. DMA EN 自动清除后的中断风暴

STM32G4 DMA 传输错误会由硬件清除 EN；工程自带官方 `stm32g4xx_hal_dma.c` 的 TE 分支也明确注明这一行为。旧用户驱动先判断 EN，EN 为零就直接返回，之后才判断 TE。因此 `TEIF=1、TEIE=1、EN=0` 会反复进入中断，错误标志与中断使能始终未清除。

这个缺陷可能使主循环和较低优先级的控制／UI 任务无法执行，符合“停在 DMA IRQ、按键停止不响应”的部分表现。它解释的是错误中断如何无法退出，不等于已经找到 DMA 首次传输错误的硬件来源。

修正位于 `mycode/spi_port.c`：

- 依据生成句柄的 Parent、hdmarx/hdmatx、DmaBaseAddress 和 ChannelIndex 确定设备及通道，保留 ADC／WS2812 的原路由。
- 在短临界区内先检查两个关联通道的 TE，再判断当前通道 EN；TE 先于 TC 处理。
- 错误时关闭该 SPI 的 DMA 请求、关联 DMA 通道及中断，清除关联标志，再通知原有应用错误恢复流程。
- 空闲或已停止设备收到迟到的 DMA 中断时，同样清除对应通道的标志与中断使能，避免空转。
- 所有分支恢复调用者 PRIMASK；中断内没有 BSY 等待或新增轮询。

保留原有故障停机与主循环恢复机制；恢复通信不会自动重新运行电机。

## 2. Follow 的 OK 停止被对齐检查拦截

Run 页原来先根据 UI 的 `s_run_active` 判断是否需要对齐，后面才判断实际 Follow 是否活动。当 UI 状态落后于控制器、且从轴未对齐时，OK 可能进入对齐分支而没有执行停止。

现在运行中的 Follow 在进入对齐判断之前，根据 `MotorControl_IsGroupActive()` 执行 StopAll，并同步两轴 UI 运行标志。未运行时仍保留原有对齐、准入和显式启动流程。M1／M2 作为主机的情况均覆盖；没有修改位置环、速度环或 Follow 增益。

## 诊断与测试

编码器最后错误文本新增 `DMA TE`，表示真实 DMA 传输错误或完成时长度不完整；`DMA TMO` 表示 DMA 超时。两轴使用相同分类，错误仍计入原 SPI 错误总数。

- 将修复前 R116 的 `spi_port.c` 与新增测试一起编译，模拟硬件 TE 自动清 EN，原版断言失败；修复版通过。旧测试只设置 TE 标志而保持 EN，没有覆盖本次关键状态。
- 将修复前 R116 的 `ui_app.c` 与新增测试一起编译，控制器 Follow 已运行、UI 仍为停止且从轴未对齐时，原版无法停止；修复版两种主机方向均通过。
- SPI 真实用户驱动测试覆盖 RX／TX TE、TE 与 TC 同时发生、迟到中断、PRIMASK、回调次数、其他 DMA 通道隔离，以及 600 组生成句柄 DMA 重映射。
- 编码器测试覆盖两轴 DMA 错误分类及恢复；完整电机、HSO、六步、辨识、协议、设置／存储、UI／LCD 和 ADC／Break／KEY1 中断回归通过。
- CubeMX 的 50 项配置变更回归通过；单 HTML 前端、真实 C 协议帧及浏览器格式切换回归通过。
- Debug／Release 构建、ELF／BIN 一致性、UI 资源一致性及两版各 139 项实际 Thumb 异常诊断测试通过。

本轮保留 R116 零码连续性修复与用户已生成的 MISO Pull-up，不编辑 CubeMX 初始化或 ST 官方源码。相对本轮开始的 R116 快照，207 个受保护文件及官方／核心编译命令校验一致。

## 固件与占用

About → FW 为 `UI-R117-DMA`，Native 配置回执版本为 117。

| 配置 | R116 Flash | R117 Flash | 剩余 Flash | R117 RAM | RAM 变化 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 130688 B | 130760 B | 312 B | 30856 B | 0 B |
| Release | 130688 B | 130760 B | 312 B | 30856 B | 0 B |

Flash 净增 72 B。RAM 包含链接器预留的堆栈。ELF／BIN／MAP 均已更新至 `build/Debug/`、`build/Release/`。基线、失败复现日志、修复后日志和尺寸报告保存在忽略的 `build/encoder-irq-r117/`。Debug 首次链接曾因调试器占用 ELF 失败，释放后标准 Debug 目录已成功重新链接；`build/Debug-R117/` 只是过程中使用的独立构建目录。

## 上板复测

本轮没有烧录或操作电机。先核对 FW，再小范围转动主机，检查 Follow 响应、OK 停止和 KEY1 全局停止。若仍有震颤或 LOST，保存同轴 E1／E2 Last、First、F Enc Gate、Ctrl Max／ADC IRQ Max／Outer Max；若仍停在 DMA IRQ，再读 CCR、CNDTR、DMA ISR、PRIMASK 和 IPSR。这样才能进一步区分 DMA 首次错误、采样超时及实际控制波动。
