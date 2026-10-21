# 双电机 ADC / PWM 配置核查

当前 **UI-R36-SMOOTH** 已核对重新生成的外设配置并接入双有感控制；本页早期版本说明保留作历史参考。当前功能、逐路保护及上板步骤以 [Dual_motor_control.md](Dual_motor_control.md) 为准。

核查日期：2026-10-21。对象为本工程当前 `FOC_two.ioc`、`Core/Src/adc.c`、`Core/Src/tim.c`、`Core/Src/main.c` 与 `mycode/JerryFOC.c`，以及用户提供的分配表和电压分压原理图。

## UI-R22-VREF：规则组增加内部参考（当前）

ADC1 规则组改为 Rank1=VBUS/IN15、47.5 周期，Rank2=VREFINT/IN18、640.5 周期。DMA 为两组完整扫描，共 4 个半字。仍每五个注入帧在安全窗口中触发一次，约 2 kHz，总转换时间约 16.78 μs。每 64 个有效参考样本平均后反算 VREF+，再进行 alpha=0.125 的低通滤波，统一用于全部电流、相电压和母线换算；首个有效参考前禁用输出，参考超时锁存 ADC REF。注入组和 PWM 配置保持原样。详见 [VREFINT_reference_tracking.md](VREFINT_reference_tracking.md)。

R21 的 ENC2 连续失败 5 次后停止探测、R20 的 USB TX 状态与缓冲修复继续保留。

## UI-R19-45RPM：HSO 减至机械速度边界

用户要求撤销大幅改速提前切换：HSO CONTROL 高于实测 45rpm 时正常改速仍由 HSO 控制，低速/停车/反转先由 HSO 减至边界后才切换编码器。45rpm=4.712389rad/s，7极对下为5.25Hz。HSO 减速参考限为50rad/s²，真正异常仍停机。ADC/PWM 与保护门限保持现有配置，见 [HSO_recovery.md](HSO_recovery.md)。

## UI-R18-SPEED：首次大幅改速保留目标

大幅目标调整（如 100→20）先正常返回编码器并保留新指令，小幅改速继续 HSO；处理提交前已经待定的 HSO 故障，避免后续回调清零新目标。Run 页同步模式/目标提交顺序。ADC/PWM 及保护门限未变，见 [HSO_recovery.md](HSO_recovery.md)。

## UI-R17-AUTO：正常退出和异常停机恢复

低速、停车目标和反转目标触发正常返回编码器，保留模式和目标；真正失锁先零输出停机，反馈健康时可直接用新的 Speed 指令重新启动。底层采样/过流/过载锁存保留，HSO 磁链不因正常退出而重学。新增 Feedback，H Lost 改为保留的异常历史。详见 [HSO_recovery.md](HSO_recovery.md)。

## UI-R16-QUAL：首次资格的有限角度暂停

根据 H Gate=OK/ANGLE、H Qual=0～24ms 的反馈，累计 2000 个严格合格帧时，允许总计 50 帧、连续 10 帧的 15～20° 小角度越界暂停；暂停帧不增加进度。更大/持续偏差和其他异常仍重置。新增 H Jitter / H Reset 诊断，R15 保持策略和 ADC/PWM 配置不变。详见 [ST_HSO_integration.md](ST_HSO_integration.md)。

## UI-R15-HSO：接管保持与失锁诊断

保留原始 15° / 200 ms 首次资格，新增保持滞回与 2 ms 轻微异常确认，硬故障仍立即失锁；角度和速度同为 10 ms 过渡。Status 增加 H Gate / H Qual / H Lost / H Actual。详见 [ST_HSO_integration.md](ST_HSO_integration.md)。ADC/PWM 及电机过载参数本轮未变。

## UI-R14-LOAD：放宽手动加载预算

过载累计基准改为 0.8 A、预算 3 秒，与铭牌额定电流和对齐参数独立。每相 2 A 瞬时停机、ADC 范围检查及 R13 同步采样均保留。详见 [2804_current_limits.md](2804_current_limits.md)。

## UI-R13-SYNC：VBUS 与注入组同步

修正规则组 TIM6 自由运行与注入采样异步冲突的问题：TIM6 停止计数，每 5 帧完整注入序列后通过软件 UPDATE/TRGO 触发一次 VBUS；迟到回调跳过不安全触发窗口。ADC/DMA/CubeMX 选择保持不变。此修改针对 ES0431 §2.5.10 记录的首项错误为 0 风险；保留保护与持续采样。详见 [current_adc_diagnostics.md](current_adc_diagnostics.md)。下面各版描述及旧 TIM6 周期启动行为保留为历史记录。

## UI-R11-HSO：按铭牌修正电流与保护

额定电流由 2 A 改为 0.5 A、峰值由 5 A 改为 2 A；补齐电流目标限幅、低电压对齐、三相瞬时过流/持续过载和 ADC 输入异常锁存。保护后仅关闭功率通道，继续采样。详见 [2804_current_limits.md](2804_current_limits.md)。下文旧版 5 A 待确认项及资源占用保留为历史记录，已由本版修正。

## UI-R10-HSO：官方观测器接入

已直接集成 MCSDK 6.4.2 的 HSO / ImpedCorr 官方库，电机 1 支持旁路观测、编码器辅助磁链估算和旋转中实验接管。操作、限制、测试范围见 [ST_HSO_integration.md](ST_HSO_integration.md)。ADC/PWM 的 CubeMX 配置本轮未变；电机 2 仍未开放功率输出。以下 UI-R9 及更早章节保留对应修复和历史核查记录。

## UI-R9：连续 VBUS 测量与误超时修复

用户上板反馈：复位后 Bus 约 11.9 V，Run 起初能运行，修改目标后显示未就绪；同时希望先上 MCU 电源、稍后接通电机母线时仍可持续检测。已确认上一版有两类源码问题：

1. 上电等待有效 VBUS 最长 20 ms，超时后停止 TIM6 和 ADC DMA。测到 0 V 会使后续接入的母线无法被看见。
2. `HAL_GetTick() - s_frame_at` / `s_bus_at` 的两个操作数不是一个一致快照。主循环先取得旧时刻，中断随后发布新时刻时，减法可能下溢为很大的无符号数，引发误超时并锁存驱动故障。新增抢占测试已在旧代码实际触发失败，修复后通过。该源码缺陷可以造成用户描述的现象；尚未在板上排除其他 ADC 故障。

当前行为：

- `MotorSampling_Start()` 不再等待母线有电，0 V 也正常启动 TIM6/规则组 DMA、双 ADC 注入采样及同步计数。M1 的六个相输出先通过 CCER 禁用，只开启内部采样触发通道；三相功率输出不会随采样启动而开启。
- 母线连续有效至少 100 ms 后才累计 2000 个电流零点样本（约 200 ms），完成后主循环服务才使能 M1 相输出。0 V 或模拟前端未供电时的原始值不会污染零点；初始化期间母线再次无效会丢弃未使能功率输出前的零点采集结果。先上 MCU、后上 Bus 可继续初始化，不再因最初 0 V 要求复位。当前“母线有效”的最低门槛仍为 1 V，用于存在性判断，不是电机额定欠压保护阈值。
- 所有跨中断采样时间判断先复制时间戳、后读取系统时间，保持无符号差值的顺序；M1 编码器健康检查同样修正。计时回绕继续受测试覆盖。
- 测量数据新鲜与母线能否驱动分开判断。ADC 正常测量 0 V 时，Bus 显示真实 `0.0 V`；没有新 ADC/DMA 数据时显示 `NO DATA`，避免将停止更新伪装成 0 V。
- 控制故障只关闭功率输出，应用不会因此停止 TIM6 / VBUS DMA；Bus 可以继续从 0 V 更新到 11.9 V 或 24 V。底层 DMA 硬件错误若本身阻断传输，会由 NO DATA 和计数停止显现。
- 实际运行后发生母线丢失、ADC 错误、注入配对缺失或超时，仍锁存首个故障并关闭功率输出。母线恢复会更新显示，但不会自行解除故障、重启电机；处理真实故障后重新启动开发板。

Status 将 Bus、Drive 放到靠前位置，新增 BusRaw、BusCnt、FrameCnt、ADC1Err、ADC2Err。Run 改为显示具体原因，异常文字不再与 Step 重叠：

| Drive / Run 提示 | 含义 |
| --- | --- |
| WAIT ADC | 刚启动，尚无新母线样本 |
| WAIT BUS | 规则组数据在更新，但母线低于存在性门槛 |
| ZERO CAL / BUS SETTLE | 正在电流零点采集 / 等待母线稳定 |
| READY | 采样和功率输出已就绪；编码器在线与对齐仍另外检查 |
| ADC PAIR | ADC1 整帧入口未见配对 ADC2 JEOS |
| FRAME TIMEOUT / BUS TIMEOUT | 注入帧 / 规则组母线数据超过 5 ms 未更新 |
| ADC ERROR | HAL ADC 错误，查看 ADC1Err / ADC2Err 十六进制错误码 |
| BUS LOST / BUS RANGE | 功率使能后母线丢失 / ADC 达到量程上限 |
| INIT ERROR | HAL 外设启动失败，区别于单纯没有母线电源 |

正常情况下 BusCnt 约每秒增加 2000，FrameCnt 约每秒增加 10000；Status 每 500 ms 刷新。Bus=11.9 V、Drive=FRAME TIMEOUT 等组合是有意义的诊断，不再统一归因于“Bus 不正常”。

验证：`python tests/motor_host/run.py` 与 `python tests/ui_host/run.py` 全部通过，覆盖 0 V 等待后供电、模拟前端未供电时的零点隔离、稳定期、真实掉电后持续测量、故障首因、抢占时序、时钟回绕、采样帧和 UI 原有交互。Debug / Release 构建成功，ELF/BIN 已重新生成；Debug Flash 100740 B / RAM 19952 B，Release Flash 99596 B / RAM 19736 B。尚未烧录或进行本轮板上复测。About 首行固件标记 UI-R9。

## UI-R8 应用适配与构建（历史）

用户再次生成后，已确认 ADC1 使用 `ADC_OVR_DATA_OVERWRITTEN`，DMA1 Channel5 的 NVIC 抢占优先级为 2，ADC1_2 保持 0。本轮保留这些 CubeMX 配置，在应用层完成以下接入：

- 新增 `mycode/motor_sampling.c/.h`，统一管理采样启动、完整帧、零点与故障状态。
- 启动顺序为：关闭两路 MOE、复位两计数器方向/CNT、装载相同的三相 50% 占空比、校准两颗 ADC、启动 ADC1 规则组 DMA 和 TIM6、等待有效 VBUS、准备 ADC2 注入从机及 ADC1 注入中断，最后启动 TIM1。TIM8 由 ITR0 触发计数，没有提前软件开启 CEN。
- 通过 CCER 一次性准备 CH1～CH3/CHxN 与 TIM1 CH4；M1 在主机成功启动后启用 MOE，M2 的 MOE 保持关闭。这里绕过会逐通道提前使能 CEN/MOE 的 HAL PWM Start，仍由 HAL 管理 ADC 与定时器基础状态。
- ADC1 是唯一控制入口。完整序列回调先检查 ADC2 JEOS，再一次性读取两颗 ADC 的八个注入结果；ADC2 不开注入中断。
- 按当前 Rank 映射保存 M1 三相电流、三相端电压、M2 两相电流的原始值和物理量。换算按 ADC 参考 3.3 V、INA240 增益 50、分流电阻 0.01 Ω、分压 1/11；实际误差仍须标定。
- 首先采集五路电流各 2000 个零点样本，约 200 ms，然后才启动现有 M1 转子对齐。避免把对齐电流算进零点。
- TIM6 以 2 kHz 触发 VBUS，双半字循环 DMA 的半满/全满回调发布最新电压；发布过程用短临界区，防止高优先级 ADC 中断读到半更新状态。
- FOC 电流改用完整帧，PWM 占空比与实际施加电压的回算改用实测 VBUS。电机参数 `bus_voltage` 保留为配置电压上限，因此接入 24 V 电源不会把原 12 V 电机的电压指令同比放大。
- ADC 错误、配对结果缺失、VBUS 无效/超量程、采样超过 5 ms 未更新时，锁存故障并无条件关闭两路 MOE。HAL 的普通 `__HAL_TIM_MOE_DISABLE` 只会在 CCER 关闭时生效，因此这里使用 `__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY`；停止时也显式清 CEN。
- 上电等待有效 VBUS 最长 20 ms。若只有 USB、没有电机电源，启动失败会保持功率输出关闭，界面和 USB 继续运行，Run 显示采样未就绪；修复供电/故障后重新启动开发板。运行中锁存故障也不自动重新使能功率输出。

### 第二路编码器与功能边界

用户确认 M2 为 MT6701，SPI2，`MT6701_CSN2`（当前生成代码 PB12）。已增加主循环约 1 ms 间隔的诊断采样，实际频率受主循环耗时影响：

- SPI2 与 W25Q64 共用。主循环在存储服务返回后访问编码器，检查 SPI 空闲及 Flash CS 已释放。
- 临时使用与 M1 一致的 Mode 1 / 256 分频，CS2 连续拉低接收 24 位 SSI 帧，检查 CRC 和磁场/失跟踪状态；完成后恢复 CR1/CR2，错误时中止传输再恢复。
- 两个客户端均必须留在主循环中；不能将现有 M2 访问直接搬进 ADC ISR，否则会打断 Flash 的多段命令。后续 M2 10 kHz 闭环需要统一的 SPI2 总线调度/异步驱动。
- `MT6701_2_GetSnapshot()` 提供原始角度、机械角、健康状态、成功数和错误数。M1 还增加了真实时间超时，ADC 调度停止时不会仅因“周期年龄不再递增”而一直显示健康。
- M1 DMA 完成回调现在保持缓冲区占用标志，直到解析和发布全部结束，防止高优先级 ADC 抢占时启动下一次 DMA、覆盖正在读取的帧；已加入中断抢占模拟回归。

Status 的 Bus 已改成 ADC 实测值；新增 Drive、Enc2（健康状态与原始角度）、Enc2Err，About 首行固件标记为 UI-R8。Run 在采样未就绪时拒绝启动，运行中失去采样就绪状态则停止本次运行。

目前仍只有 M1 的一套有感 FOC。M2 电流已采集、编码器已提供诊断数据，但极对数、额定/最大电流及控制参数尚未提供，因此没有启用 M2 功率输出，也没有实现两套闭环或移植 HSO/SMO。保留的三相电压/电流帧是后续算法适配输入。

### Debug Flash 优化与验证

128 KiB Flash / 32 KiB RAM 的链接容量保持不变。顶层 CMake 使 Debug 应用代码使用 `-Og`，HAL / USB 库使用 `-Os`，已有 UI/存储等代码继续 `-Os`；所有 Debug 源仍保留 `-g3`。已检查实际编译命令和 ELF 的 `.debug_info` / `.debug_line`，没有通过删除调试符号解决 Flash 容量问题。

| 构建 | Flash | RAM | 结果 |
| --- | ---: | ---: | --- |
| 修改前 Debug | 134812 B | 19760 B | 超出 Flash 3740 B |
| 当前 Debug | 99604 B | 19856 B | 成功，Flash 剩余 31468 B |
| 当前 Release | 98548 B | 19640 B | 成功 |

`build/Debug` 和 `build/Release` 均已生成新的 ELF/BIN。链接器仍提示既有 `.RamFunc` / `.data` LOAD 段的 RWX 属性；本轮未调整这部分链接段布局。

已执行：

- `python tests/motor_host/run.py`：链接实际采样、FOC、MT6701、W25Q64 代码，模拟同步启动、八路映射、五路零点、12/24 V 换算、缺帧/超时/错误停机、启动失败清理、启动途中错误、时钟回绕、SPI2 模式/片选恢复、Flash 交替访问、CRC 和传输失败。
- `python tests/ui_host/run.py`：现有按键长按、确认/取消、范围/步长、动画、存储和显示驱动回归通过，增加了采样故障时 Run 停止/拒绝启动及新 Status 字段的验证。
- `cmake --build --preset Debug -j4` 与 `cmake --build --preset Release -j4`：均编译链接成功。

尚未烧录或进行板上验证。同步相位、采样点与开关边沿、实际 ADC 参考/增益、死区、负载下中断最坏耗时仍须实测；下文 INA240 量程与 `max_current=5 A` 的待确认项仍然适用。

## 上一轮重新生成后的复核（历史）

本节记录用户按方案重新生成后的状态；下方“首次核查记录”保留最初问题及计算依据，不代表这些旧配置问题仍然全部存在。

| 检查项 | 最新结果 |
| --- | --- |
| TIM1 | 中心对齐 3、PSC=0、ARR=8499、预装载打开；TRGO=Enable、TRGO2=OC4REF，符合方案 |
| TIM8 | 与 TIM1 相同计数参数；`TIM_SLAVEMODE_TRIGGER`、`TIM_TS_ITR0` 和 `HAL_TIM_SlaveConfigSynchro()` 均已生成，从触发已配置正确 |
| IOC 对 TIM8 的保存 | `VP_TIM8_VS_ControllerModeTrigger` 和 `VP_TIM8_VS_ClockSourceITR` 已保存为 Trigger Mode / ITR0，与生成代码一致 |
| ADC1 注入 Rank1～4 | IN1、IN6、IN7、IN14；采样 12.5、47.5、47.5、12.5，正确 |
| ADC2 注入 Rank1～4 | IN2、IN8、IN9、IN5；采样 12.5、47.5、47.5、12.5，正确 |
| ADC 公共配置 | 双注入同步、12 位、HCLK/4、EOC_SEQ_CONV；两边均约 4 μs，符合方案 |
| ADC1 规则组 | IN15、47.5 周期、TIM6_TRGO 上升沿、DMA 连续请求打开，正确 |
| TIM6 | PSC=169、ARR=499、TRGO=Update；2 kHz，正确 |
| ADC1 DMA | DMA1_Channel5、ADC1 请求、外设到内存、半字、循环、内存递增；IRQ handler 已生成，正确 |

建议再在 CubeMX 调整两项：

1. NVIC 的 DMA1 Channel5 优先级目前为 0，与 ADC1_2 同级，建议改为 2 或 3，ADC1_2 保持 0，让低速 VBUS DMA 中断可以被 FOC 抢占。DMA 传输本身的 `DMA_PRIORITY_LOW` 与 NVIC 优先级是两个设置，前者可以保留。
2. ADC1 Overrun 当前为 `ADC_OVR_DATA_PRESERVED`，并非非法配置；但若 VBUS 使用最新值，建议选 `ADC_OVR_DATA_OVERWRITTEN`，同时在代码处理溢出/采样故障。

CubeMX 生成的外设初始化已基本符合方案，应用启动仍未完成：

- main 仍在 ADC 校准和 TIM8 准备前启动 TIM1，此时 TIM1_TRGO 的启动沿已经发出；后续必须改成从机先准备、主机最后启动。
- main 尚无 TIM8 六路 PWM 启动，没有 `HAL_ADC_Start_DMA()` 和 `HAL_TIM_Base_Start(&htim6)`，因此新外设目前只是初始化，尚未运行。
- ADC2 仍使用 `HAL_ADCEx_InjectedStart_IT()`。当前回调只在 ADC1 分支计算 FOC，不会仅因为 ADC2 也开中断就计算两次；但为减少额外中断源，建议 ADC2 改为不带 IT 的启动，ADC1 保留 IT。
- JerryFOC 仍只读取两颗 ADC 的 JDR1；第二电机电流现位于 JDR4，其余 HSO 电流/电压必须按新表形成完整帧并接入处理。

执行构建检查：Release 编译链接成功，Flash 95444 B，RAM 19448 B。Debug 各源文件编译成功但链接失败，Flash 需求 134812 B，超过 128 KiB 容量 3740 B；RAM 19760 B。此处是代码体积问题，不是 ADC/TIM 配置的语法错误。Debug 不能据此当作已产生可用的新固件；构建目录可能仍保留此前的旧 BIN。

本轮只更新复核记录，没有修改用户刚生成的 IOC、外设代码或启动代码，也没有烧录或实测同步波形。两路 DeadTime=17 和 TIM1 CCR4=1 是已经保存的数值，其电气适用性仍需实测核定。

## 首次核查记录

已确认：实际器件为 STM32G431VBT6；两路采用相线串联的 0.01 Ω 分流电阻和 INA240A2PWR，INA240 供电 3.3 V，输出仅串联 10 Ω 接 ADC、无外部电容；母线按电机选择 12 V / 24 V；计划 TIM1/TIM8 同频同步 10 kHz，在一个采样完成中断内计算两套控制环。电机 1 兼顾有感与 HSO/SMO，电机 2 仅有感。

待确认：母线满电/回馈最高电压、INA240 REF1/REF2 接法，以及 ADC 实际 VREF+。以下涉及 3.3 V ADC 满量程和 1.65 V 电流零点的计算均明确为条件假设。本次是配置审查，没有改动 IOC、驱动代码或烧录开发板。

## 结论

引脚资源和八个注入通道的容量够用，图片的分配与工程一致。相线电流采样允许共用同步 PWM 节拍，不受低侧分流的导通窗口限制，但仍需避开开关瞬态。当前工程尚未完成“双电机同步采样、单次完整帧中断、HSO 输入处理”。

必须先处理的事项：

1. `EOC_SINGLE_CONV` 会使当前 HAL 在每个注入通道完成时触发回调，不能把回调频率继续当作固定 10 kHz。
2. 两颗 ADC 的注入序列转换总时长不同，ADC1 的序列完成不能直接作为 ADC2 全部结果已就绪的保证。
3. TIM8 仍是默认向上计数、ARR=65535，没有与 TIM1 同步，也没有在 main 中启动。
4. VBUS 规则组尚无定时触发、ADC DMA 或运行时启动。
5. FOC 目前只读取 ADC1/ADC2 的 JDR1，另外六路注入结果尚未进入第二电机或 HSO。

## 引脚和当前注入组

| Rank | ADC1 | ADC1 采样周期 | ADC2 | ADC2 采样周期 |
| --- | --- | --- | --- | --- |
| 1 | PA0 / IN1 / M1_IU | 12.5 | PA1 / IN2 / M1_IV | 12.5 |
| 2 | PB11 / IN14 / M2_IU | 12.5 | PC4 / IN5 / M2_IV | 12.5 |
| 3 | PC0 / IN6 / M1_IW | 12.5 | PC2 / IN8 / M1_VV | 47.5 |
| 4 | PC1 / IN7 / M1_VU | 47.5 | PC3 / IN9 / M1_VW | 47.5 |

VBUS 为 PB0 / ADC1_IN15，规则组 Rank1，采样 47.5 周期。PC0～PC3 的 ADC12 通道可按当前表在 ADC1/ADC2 间选择，原理图上写 ADC1_IN8 不会禁止 PC2 改由 ADC2_IN8 采样。最终以 PCB 实际引脚连接为准。

ADC1 主机、ADC2 从机，`ADC_DUALMODE_INJECSIMULT`，12 位、单端、不连续注入关闭、自动注入关闭、队列关闭，适合固定四通道序列。ADC2 不配置独立 TIM1 触发不算错误：注入同步模式使用主 ADC 的触发源。main 中先启动 ADC2 再 ADC1，顺序正确。[RM0440，ADC 双重模式](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## 中断与完整数据帧

`Core/Src/adc.c` 中两颗 ADC 都是 `ADC_EOC_SINGLE_CONV`。本地 HAL 的 `HAL_ADCEx_InjectedStart_IT()` 根据这个字段选择 JEOC 或 JEOS：`ADC_EOC_SEQ_CONV` 才启用 JEOS。

当前回调在 ADC1 分支直接调用 `MT6701_Service()` / `JerryFOC_run()`，没有判断整组是否完成。增加到四个 Rank 后，可能每 PWM 周期多次进入控制回调；ISR 执行较长时还可能合并待处理事件，实际次数不能简单固定为四次。这会使控制器固定 `Ts_curr=0.0001f` 与执行节拍不一致。

应设为 `ADC_EOC_SEQ_CONV`，并只让选定的完整帧完成点触发控制环。两颗 ADC 各开一次序列中断也不是一个公共 ADC IRQ 内只执行一次计算的充分保证：仍需要选定唯一处理分支或用帧完成条件汇合。

同步模式不会把不等长的序列自动变成“主机结束就等于从机结束”。RM0440 说明，两颗 ADC 序列等时长时，才可以只用其中一颗 JEOS 中断读取两颗全部结果。[RM0440 注入同步说明](https://www.st.com/resource/en/reference_manual/dm00355726.pdf)

按当前采样配置估算，忽略触发同步及规则组抢占附加延迟：

| 项目 | 时间 |
| --- | --- |
| 有效 ADC 时钟：HCLK 170 MHz / 4 | 42.5 MHz |
| 12.5 周期采样保持时间 | 0.294 μs |
| 12.5 周期采样 + 12 位转换的 12.5 周期 | 0.588 μs |
| 47.5 周期采样保持时间 | 1.118 μs |
| 47.5 周期采样 + 12 位转换的 12.5 周期 | 1.412 μs |
| ADC1 当前四次转换合计：(25+25+25+60)/42.5 MHz | 3.176 μs |
| ADC2 当前四次转换合计：(25+25+60+60)/42.5 MHz | 4.000 μs |
| 两者差值 | 0.824 μs |

因此仅把 ADC1 改成 JEOS 还不完整：将来在其回调里立即读取 ADC2_JDR4，存在拿到上一帧 W 相电压的风险。保留当前顺序时，可以将较晚完成的 ADC2 JEOS 作为唯一计算入口；也可以均衡两颗 ADC 的采样时长，再选用一个完成入口。读取数据后统一形成帧快照，避免控制计算过程中零散读取不同帧数据。[STM32G431 数据手册，ADC 时序](https://www.st.com/resource/en/datasheet/stm32g431cb.pdf)

## 面向 HSO 的可选顺序

ST 对两 ADC 的 HSO 建议是先尽量相邻采完三相电流，再尽量相邻采完三相电压。将 M2 的电流放在 M1 电流中间会增加 M1 的采样时间离散。[ST 工程师对 HSO 采样顺序的说明](https://community.st.com/stm32-mcus-motor-control-34/hso-adc-config-145373)

在现有 PCB 引脚不变、固定单帧扫描的前提下，可评估以下顺序：

| Rank | ADC1 | ADC2 | 两边采样周期 |
| --- | --- | --- | --- |
| 1 | M1_IU / IN1 | M1_IV / IN2 | 12.5 / 12.5 |
| 2 | M1_IW / IN6 | M1_VV / IN8 | 47.5 / 47.5 |
| 3 | M1_VU / IN7 | M1_VW / IN9 | 47.5 / 47.5 |
| 4 | M2_IU / IN14 | M2_IV / IN5 | 12.5 / 12.5 |

此方案两边均约 4 μs，M2 的两路电流同时采样。Rank2 的电流通道采用 47.5 周期是为了与对侧电压通道保持时序一致，不是说 INA240 必须用这么长的 ADC 采样时间。电流和电压仍不是所有相同时采样，HSO 的采样延迟参数应按最终顺序和实际时序计算，不能直接照抄不同排列下的系数。实际采用 ST HSO 时还应核对所用 MCSDK 的驱动、板描述和输入接口，不能把手写注入序列直接视为原生 HSO 驱动适配完成。

## TIM1 / TIM8

TIM1 当前 PSC=0、中心对齐 3、ARR=8499，约为 10 kHz；TRGO=OC4REF、CH4 为 PWM1、CCR4=1。其上升沿把采样放在计数谷底附近。相线采样可以在这里工作，但需要观察两电机最接近该点的开关边沿及 INA240 输出，验证整个采样窗口内没有明显瞬态。

TIM8 当前 PSC=0、向上计数、ARR=65535、预装载关闭、DeadTime=0；如果启动，频率约为 2.594 kHz。main 当前只启动 TIM1，没有启动 TIM8。应将 TIM8 的频率、中心对齐模式、ARR、预装载及同步启动配置与方案一致；死区应按两路实际栅极驱动和 MOSFET 确认，不能直接保留默认 0。

一种可评估的触发分工是：

- TIM1 TRGO=Enable，用于同步启动 TIM8。
- TIM8 从触发模式，输入 ITR0（G431 的 TIM1_TRGO），两路定时器预置相同计数起点和方向。
- TIM1 TRGO2=OC4REF，ADC1 注入触发改为 TIM1_TRGO2 上升沿，ADC2 跟随 ADC1。
- 从机和 ADC 先准备，主机后启动；重启时重新同步，不用两个顺序软件 Start 代替同步。

这样将定时器同步启动与周期 ADC 触发分开。工程现有 HAL 已提供对应的 `TIM_TRGO_ENABLE`、`TIM_TRGO2_OC4REF`、`TIM_TS_ITR0` 和 `ADC_EXTERNALTRIGINJEC_T1_TRGO2`。CH4 的最终比较值需按波形和算法的采样延迟选择，不在缺少实测时直接指定。[RM0440 定时器互联表](https://www.st.com/resource/en/reference_manual/dm00355726.pdf)

10 kHz 的完整周期约为 100 μs，约 4 μs 的 ADC 序列本身可以容纳。两套 FOC、HSO 和编码器服务的最坏执行时间还需 DWT/示波器测量，并且必须赶上实际 PWM 预装载更新期限；只有平均执行时间小于 100 μs 不足以证明时序正确。

## 电压采样时间与分压滤波

原理图每路上臂为 10 kΩ+10 kΩ，下臂 2 kΩ；22 nF 跨在下臂，后面串 1 kΩ 接 ADC。忽略 ADC 动态取样负载的分压传递为：

```text
Vadc / Vphase = 2k / (20k + 2k) = 1/11
Rth = 20k || 2k = 1.818kΩ
相电压：τ = Rth × 22nF = 40μs，fc ≈ 3.98kHz
母线电压：τ = Rth × 100nF = 181.8μs，fc ≈ 875Hz
```

这里不能按 `1kΩ × 22nF` 算相电压低通：电容在 1 kΩ 的前面。ADC 取样瞬态下，1 kΩ、22 nF 储荷、分压网络及 ADC 输入采样电容共同影响充电；也不能忽略 22 nF，直接把纯电阻 2.818 kΩ 当成所有频率下的源阻抗。

对该前置储荷电容加 1 kΩ 串阻的结构，电压采样 47.5 周期（1.118 μs）可作为合理的调试起点，但尚未构成实际精度验收。若对比 92.5 周期时数值明显变化，应检查 ADC 充电、串扰及信号源；改变采样时间时要同步重算双 ADC 配对和完成点。数据手册把 IN1～IN5 列为快速通道，IN6～IN9/IN14/IN15 为慢速通道，所需采样时间不能只看通道数。[STM32G431，ADC 输入阻抗与采样时间表](https://www.st.com/resource/en/datasheet/stm32g431cb.pdf)，[AN5346，源阻抗和并行 ADC 注意事项](https://www.st.com/resource/en/application_note/an5346-stm32g4-adc-use-tips-and-recommendations-stmicroelectronics.pdf)

HSO 的相电压低通不能只按“带宽越高越好”处理。ST 工程师给出的参考平台相电压截止频率约 518 Hz，也明确说明约 500 Hz 可用。现有 3.98 kHz 不能因此判错，但 PWM 纹波残留、三路 RC 一致性、滤波建模和延迟补偿都应按当前电路配置。40 μs 是外部滤波的时间常数，与 ADC 的 1.118 μs 采样保持时间不是同一个参数，不应通过把 ADC 采样拉到几十微秒来代替滤波配置。[ST 对 HSO/ZeST 相电压滤波的说明](https://qa-community.st.com/topic/show?fid=34&tid=126575)

若 ADC 参考为 3.3 V，该分压的直流满量程为 36.3 V；12 V / 24 V 母线对应约 1.091 V / 2.182 V，正常工作电压量程合适。实际还需给电阻误差、ADC 余量及母线回馈留空间，不能把未确认的回馈最高电压直接视为 24 V。

## INA240 电流采样

INA240A2 增益为 50 V/V，配 0.01 Ω 得到 0.5 V/A；当前 JerryFOC 的 `SHUNT_RESISTOR=0.01`、`AMP_GAIN=50` 与此一致。用户确认输出仅有 10 Ω、无外部滤波电容。对这样的低阻驱动，12.5 周期约 0.294 μs 可以作为合理的 ADC 采样保持起点，不需要因为相电压有 22 nF 就把相电流也按相同外部 RC 处理。最终精度仍应通过实测确认。[TI INA240 数据手册](https://www.ti.com/lit/ds/symlink/ina240.pdf)

ADC 采样保持时间和 INA240 对开关共模瞬态/信号阶跃的恢复时间必须分开看；不应把数据手册中的约 9.6～9.8 μs 大信号稳定时间直接要求成每次 ADC 的采样时间，也不能因此认定每个 PWM 边沿都固定需要相同空白时间。应在实际输入、输出 RC 与 PWM 波形下测量采样点的残余误差。

INA240 供电已确认是 3.3 V。若 REF 接法令零电流输出为 1.65 V、ADC 满量程 3.3 V，理论可测约 ±3.3 A，实际还要扣除输出摆幅等余量。5 A 对应 2.5 V 的输出偏移，无法在这个零点和供电下双向测量；工程电机参数里 `max_current=5 A` 应与实际测量量程一起修订。REF1/REF2 接法尚未给出：如果零点不是 1.65 V，正反向量程应按实际零点和输出摆幅分别计算。

## VBUS 规则组

PB0 / ADC1_IN15 与图一致，47.5 周期可以作为起点。当前配置仍是软件启动、无外部触发、DMAContinuousRequests 关闭；工程没有 TIM6，没有 ADC DMA 请求，也没有启动 VBUS 的 HAL_ADC_Start/Start_DMA，因此尚未实现图中计划的 2～5 kHz 采样。

建议仅对实际有低速测量通道的 ADC 设置规则组，不必给 ADC2 添加无用途的规则转换：

- TIM6 Update → TRGO，2～5 kHz。
- ADC1 规则组 TIM6_TRGO 上升沿；连续转换关闭，由定时器每次触发。
- ADC1 DMA 外设到存储器、半字、循环模式；重复外部触发时保持 DMA 请求。
- 按目标应用选定 Overrun 策略并检测溢出；注入组优先，VBUS 消费端允许相应延迟。

目前 FOC 用的是电机参数中固定 `bus_voltage=12.0f`，UI 的 Bus 也来自这个参数，并非 ADC 实测母线。完成规则采样后还需接入换算和控制侧更新，单独勾选 ADC 通道不会使这些值自动更新。

## 实板验证要点

- 确认每秒只有约 10000 次完整帧控制入口，JEOC 不重复执行 FOC。
- 确认读取 JDR1～JDR4 时，两颗 ADC 均是同一轮完整结果。
- 观察 TIM1/TIM8 同步启动及持续相位关系，检查恢复/重启后是否一致。
- 在两电机占空比和转速变化时检查采样窗口相对所有相关开关边沿的位置。
- 用已知电压/电流校准分压比例、零点和增益；比较不同 ADC 采样时间下的读数差。
- 测量两套环路及 HSO 的最坏执行时间，核对 PWM 预装载更新期限。

以上为源码和参数计算审查，尚未进行开发板波形、母线量程或双电机带载验证。
