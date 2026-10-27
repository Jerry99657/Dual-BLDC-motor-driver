# R95：异常关断、控制时序与首次故障快照

固件 **UI-R95-DIAG**，完成本轮第 1～3 项。现有控制、保护、通信、存储、UI 和驱动功能保留。无需调整 CubeMX。ST HAL／LL／CMSIS、USB 中间件、HSO 官方库、链接脚本和 `.ioc` 均保持本轮基线内容；Core 仅新增 USER CODE 钩子。

## 1. CPU 异常关断与记录

NMI、HardFault、MemManage、BusFault、UsageFault 使用生成中断函数的 USER CODE 入口，先取得异常的 MSP／PSP，再切到独立的 256 B 应急栈。C 入口首先根据 RCC 时钟使能状态关断 TIM1／TIM8 的 MOE 和六路功率输出 CCER，然后记录异常类型、启动阶段、PC、LR、原始 SP、XPSR、EXC_RETURN、CFSR、HFSR、BFAR、MMFAR。

支持基本栈帧与浮点扩展栈帧。检测到堆栈异常、未对齐或超出实际 32 KiB SRAM 的地址时，跳过 PC／LR／XPSR 读取，保留异常状态和原始 SP。这几项显示 0 时需结合 CFSR 判断栈帧是否可读。BFAR／MMFAR 是否有效也由 CFSR 的有效位判断。

异常路径关闭普通中断，关断与记录不调用 HAL、SPI、USB、格式化或浮点代码，记录后停在循环中。它不自动复位。手动 Reset 后若保留了有效 CPU 异常记录，启动会进入已有全局暂停路径，取消自动对齐；后续仍需明确的手动运动／对齐命令通过原有准入条件。

异常类型编号：2=NMI、3=HardFault、4=MemManage、5=BusFault、6=UsageFault。`stage` 是已有 `g_boot_stage`，1～7 对应 main 中各启动阶段，7 为完成初始化。

## 2. 控制路径耗时与抖动

进入 **Status → Diagnostics**，或上位机选择 **Diagnostics / Flash** 读取专题。保留原 11 行，追加 26 行，总计 37 行；USB 和 USART 查询相同。

| 字段 | 定义 |
| --- | --- |
| ADC IRQ Max | ADC12 分发器时间戳至采样、两路快速控制、HSO 后处理全部完成的最大经过时间 |
| ADC Jitter | 相邻有效 ADC 主序列中断入口间隔相对 100 us 的最大绝对偏差 |
| PWM Age | 中断入口 TIM1 相对最近零点／顶点的最大计数年龄，换算为 us，包含 ADC 序列转换及等待中断的时间 |
| Outer Max | TIM2 有效更新路径中，完整两轴外环和运动快照缓存的最大经过时间 |
| Outer Jitter | 相邻外环入口间隔相对 1 ms 的最大绝对偏差 |
| IRQ Overrun | 上述 ADC 路径耗时达到／超过 100 us 的次数 |

时间显示精度为 0.1 us，来自 DWT 的 170 MHz 周期计数。10 kHz／1 kHz 周期和时钟继续受每次构建的生成配置检查约束。计数差采用无符号减法，支持 DWT 回绕；间隔不能超过一次 DWT 完整回绕。

统计包含高优先级中断抢占带来的经过时间；不含 CPU 自动入／出栈、进入分发器前的跳转及最后的统计／保护函数尾部。它是运行诊断，不能代替示波器或严格的最坏执行时间证明。只有有效 ADC1 JEOS／TIM2 更新事件进入相应统计。

六步模式切换采样中心时，可能合法地出现一次半个 PWM 周期的 ADC 间隔变化；累计最大抖动也会保留它。进入稳定工况、停机并清除后可重新观察。抖动值本身不触发新增停机。已有 Ctrl Max／HSO 控制耗时及 100 us 保护改为接收较完整的 ADC 控制路径计时，原门限保留。

## 3. 首次故障快照

采样／电流／母线／VREF／IBUS、编码器、HSO、六步、联动／控制超时、参数辨识的故障停机统一进入记录器。在断开危险输出后、原停止流程清空目标前，锁存首个故障的来源、轴号、原因、HAL tick、帧序号、VBUS、VREF、两轴三相电流、ADC 原始值、编码器年龄／状态／错误累计，以及工作模式、驱动、型号和运动缓存。

普通首次故障和首次 CPU 异常分别锁存。后续故障、通信恢复、切模式、再次 Run 或 Reset 均不覆盖有效记录。正常用户 Stop／KEY1 不建立故障记录；未启用 M2 的电流零点异常也不占用首次记录，其原有就绪保护保留。

`F` 开头的行是冻结值；原有 Status 行仍是实时值。选择 Axis M1／M2 查看同一记录中对应轴的数据，无需启动电机。

| 字段 | 说明 |
| --- | --- |
| First | `来源 M轴 原因`。来源：1=采样／IBUS，2=编码器，3=HSO，4=六步，5=控制／联动，6=辨识；M0 表示共享／双轴故障 |
| F Time | 故障发生的本次启动 HAL tick，ms；Reset 后仍保留故障当时值 |
| F W/D/P/C | `工作编号/驱动/型号编号/控制模式编号` |
| F Bus/Ref | 母线／ADC 参考电压，mV |
| F Cmd/Act | 速度目标／实际速度，mrad/s；六步油门为目标百分比／实际值，ESC 通用实际值为 mHz |
| F Position | 位置目标／实际值，mrad；六步无位置观测，显示 `--`；HSO 使用自身估算相对位置 |
| F Current | U／V／W，mA；M2 的 W 由 U、V 重构 |
| F ADC | U／V／W 原始码；M2 无第三相 ADC，W 占位为 0 |
| F Enc | E1／E2、最后错误、年龄周期数／十六进制状态。最后错误可能早于本次故障，需结合年龄及其他字段判断 |
| F Counts | 捕获时的 ADC 帧计数／该轴编码器累计错误数；故障帧可能尚未提交到帧计数 |
| F Age/IRQ | 运动缓存年龄 ms／捕获时已统计的 ADC 最大路径耗时 us |
| CPU … | 首次 CPU 异常字段；`CPU B/M` 为 BFAR／MMFAR；没有记录显示 `--` |

工作编号 0～6：Single M1、Single M2、Independent、Sync Speed、Sync Position、Master M1、Master M2。型号编号：0=2804、1=4015、2=通用 ESC。控制模式编号：0=Torque、1=Speed、2=Position、3=六步 Throttle。

运动目标／模式和实际速度／位置缓存由 1 kHz 外环更新，通常比故障早不超过一个外环周期；外环尚未运行或被延迟时以 `F Age/IRQ` 为准。六步故障直接读取其目标／速度快照。没有增加编码器采样或 SPI 事务。格式化遇到 NaN、无穷或超出可显示缩放范围的浮点值显示 `INVALID`。

## 保留与清除

记录和校验码放在 `.noinit` SRAM。实际 ELF 检查确认该段为 NOBITS，位于启动清零范围之外、堆栈保留区之前。它用于 SRAM 内容保留时的复位诊断，**不保证断电、掉电、重新下载或调试器清 RAM 后保留**。记录有版本标识和完整性校验，损坏／中断写入的记录会被丢弃。没有增加内部 Flash 或 W25Q64 写入。

- 板端：**Settings → System → Clear diag**。
- 上位机：**板端诊断详情 → 清除诊断记录**，要求 R95 或更新固件。
- 原始 USB／USART 命令：`App:0,17,0`；专题为 `GetRow:1,6,0` 开始，M2 改轴号为 2，行号 0～36。

只有两轴均无输出请求才允许清除。该操作同时清除首次故障、CPU 异常和本轮新增时序统计，不修改设置，不请求掉电保存，不解除任何驱动／IBUS／全局暂停锁存，也不启动电机。原有编码器累计计数、Ctrl Max／HSO 历史计数保持原逻辑。

## 构建与验证

| 构建 | R94 Flash | R95 Flash | 增加 | 剩余 | RAM 总分配（含预留堆栈） |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 125060 B | 128252 B | 3192 B | 2820 B | 29408 B |
| Release | 125060 B | 128252 B | 3192 B | 2820 B | 29408 B |

RAM 比 R94 增加 1712 B，包含应急栈、保留记录、运动／时序缓存及 UI 行缓冲；剩余 3360 B 在既有堆栈预留之外。此次完整功能接入后 Flash 余量小于先前建议的 4 KiB。

验证包括：

- `tests/diag_host/run.py`：真实记录器的首次锁存、递归保护、校验损坏／复位保留、两轴清除准入、PRIMASK、原因截断、时间回绕、六步／HSO 数据、无效浮点和计时门限。
- `tests/diag_host/check_arm.py --both`：执行 Debug／Release 实际链接的 Thumb 异常指令，各 127 个场景，检查五个异常入口、MSP／PSP／浮点帧、独立应急栈、初始化早期及损坏栈、TIM1／TIM8 先关断再记录、首次异常保留和启动 BSS 清零边界。可选依赖 unicorn／pyelftools 位于忽略的 `build/diag-testdeps`。
- 原电机／HSO／六步／辨识／采样／协议回归，追加未启用 M2 的首次故障过滤；真实 LL IRQ 的 8388608 种 ADC 状态／使能组合和采样集成回归；UI／存储／LCD、单文件上位机与通信测试。
- `tests/cubemx_host/run.py`：34 项配置变更和 DMA／KEY1 改映射；实际 CubeMX 重复生成保留全部 Core C／头文件和 IOC，变更 USART 配置可通过 ARM 编译。每次构建新增检查诊断 USER CODE 路由是否保留。
- `tests/motor_host/check_startup.py`：两种 ELF 的异常入口、应急栈路由、无 HAL／浮点异常处理、启动顺序、控制时序调用和 ELF／BIN 一致性。
- `tests/flash_report.py final --directory build/diagnostics-r95 --allow-diagnostics-user-code`：相对本轮基线的 211 个受保护文件及官方／Core 编译选项。只允许精确的 main／异常 USER CODE 钩子和用户诊断模块 CMake 增项。

目前为构建、主机模型和 ARM 指令仿真验证，尚未刷板测量新增统计的实板值。建议依次复测 Single M1 有感、HSO、六步和双机 Follow，记录稳定工况下的 ADC IRQ Max／Outer Max／IRQ Overrun；模拟一次编码器故障后恢复通信，确认冻结记录仍在，停机 Clear diag 后清空。异常验证可在电机电源关闭时由调试器触发，Reset 后检查 CPU 记录及自动对齐抑制。
