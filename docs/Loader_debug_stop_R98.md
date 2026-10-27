# R98：F5 下载后保留的 RAM 调试记录导致启动暂停

固件 **UI-R98-BOOT**。用户确认通过 VS Code F5／ST-Link 下载后直接运行。现场为 Single M1、ENCODER、GLOBAL STOP、Aligned=NO；重新断电后能对齐。

## 现场与判断

| 字段 | 用户确认值 |
| --- | --- |
| CPU Fault | 3（HardFault） |
| stage | 1430396978（0x55422032） |
| PC | 0x20000000 |
| LR | 0x200007C1 |
| SP | 0x20000BD8 |
| XPSR | 0x21000200 |
| EXC_RETURN | 0xFFFFFFF9 |
| CFSR | 0 |
| HFSR | 0x80000000 |

HFSR 仅 DEBUGEVT 位，CFSR 没有配置故障状态。EXC_RETURN 对应基本线程／MSP 帧，XPSR 为 Thumb 线程状态。PC、LR、SP 都位于低地址 RAM；当前链接镜像中 0x20000000 是 SystemCoreClock 数据，应用启动阶段只写 1～7。它高度符合应用尚未正常初始化时，下载器在 RAM 中执行烧录算法、返回调试停靠点的现场。

这是结合现场和映像的推断，尚未用实板跟踪捕获下载器指令。官方背景可参考 [Arm/Keil AN334，第 2～3 页](https://www.keil.com/appnotes/files/apnt_334.pdf)：烧录算法下载到目标 RAM 执行，调试器配置 PC／SP／LR，利用 BKPT 停止算法。该资料解释一般机制，不证明 ST-Link 某一版本内部算法的具体指令地址。HFSR 位定义也与工程自带官方 CMSIS `SCB_HFSR_DEBUGEVT_Msk` 一致。

R95 的记录校验只确认内容完整，没有区分这类非应用执行上下文。下载器期间保存的记录经过 Reset 后仍在 SRAM，随后 main 根据非零 crash.kind 请求全局暂停，取消自动对齐。R97 修复 Run 的手动恢复，但没有解决这类记录在下一次下载后被当成应用故障的问题。

## 修复边界

仅在校验通过后、以下条件**同时满足**时，初始化清除 CPU 记录部分并重新计算校验；同一记录中的首次电机故障仍保留：

- HardFault，stage 不在 1～7。
- PC 恰为 RAM 起始的 0x20000000；不是把任意 RAM PC 视为下载器。
- LR 为 Thumb 地址，位于 0x20000001～0x20000FFF。
- 原始 SP 按 8 B 对齐，位于 0x20000100～0x20000FE0。
- EXC_RETURN 恰为 0xFFFFFFF9，XPSR 的 Thumb 位有效且 IPSR 为线程模式。
- CFSR 为 0，HFSR 恰为 DEBUGEVT，没有 FORCED／VECTTBL 等其他位。

这不是按 stage 单独过滤，也不是屏蔽所有 DEBUGEVT 或 HardFault。正常应用阶段、Flash／其他 RAM PC、真实故障位、异常／PSP／浮点返回和其他未知现场，继续保留并禁止自动对齐。

异常发生当下的 TIM1／TIM8 关断、独立应急栈、记录和停留全部保持；仅改变下一次初始化如何识别这组严格的非应用调试记录。记录布局及 magic 不变，真实旧应用异常记录仍可保留。没有修改 Core、ST 官方库、CubeMX 配置、KEY1 中断、保护阈值或调试器配置。

## 资源与验证

| 构建 | R97 Flash | R98 Flash | 增加 | 剩余 Flash | RAM 总分配 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 129024 B | 129136 B | 112 B | 1936 B | 29424 B |
| Release | 129024 B | 129136 B | 112 B | 1936 B | 29424 B |

- `tests/diag_host/run.py`：先在旧实现复现保留记录造成启动误判，再检查精确现场清除、首次电机故障／校验保持；逐项改变启动阶段、类型、PC／LR／SP、CFSR／HFSR、XPSR 和 EXC_RETURN，确认未知与真实应用异常不被过滤。
- `tests/diag_host/check_arm.py --both`：执行实际链接 ELF 的 Thumb 异常路径；两个镜像各 139 场景。额外模拟用户现场、下一轮 BSS 清零和正常初始化；确认当下仍关断并停留，仅精确匹配的记录在下一次初始化清除，后续真实异常还能首次锁存。
- UI／存储／LCD、协议／遥测、单文件上位机、启动 ELF／BIN 校验通过。ST 官方／Core 等 211 个受保护文件及编译选项与 R97 基线一致。

构建产物 ELF／BIN／MAP 已更新，尚未刷板。请用同样 F5 流程复测：About 为 UI-R98-BOOT，KEY1 松开且没有真实故障时应恢复原有上电对齐；CPU Fault 不再被上述记录占用。若仍 GLOBAL STOP，保留当次 CPU 字段，并在 MotorSampling_GlobalStopIRQ 断点查看第一次调用栈，不能仅凭重新上电恢复认定原因相同。
