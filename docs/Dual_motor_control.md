# UI-R43-ZEROLOG：双电机有感控制与手动主从跟随

R43 在 Status 增加物理 E1/E2 的 Zero、ZPre、ZPost、ZTime，记录全零事件前后角度、次数和持续时间；区分零点附近、远离零点和前样本过期，不放行可疑全零帧。解释与验证见 [Encoder_zero_event_diagnostics.md](Encoder_zero_event_diagnostics.md)。

2026-10-23 的 Follow 失联、M2 磁铁对准后改善及单机约 20 min 对照结果已记入 [实板观察记录](Encoder_follow_field_log_2026-10-23.md)。本次只更新文档，保留 R43 代码和保护，等待再次复现。

R42 对当前模式所需的 ENC2 增加故障停机后每 250 ms 单帧重试；未启用的 ENC2 保留五次连续失败停止。恢复采样不会自动启动电机。APB1/APB2 恢复 170 MHz、编码器约 664 kHz，并保持 Flash、LCD、I²C、串口、CAN、USB 速率不变，见 [Encoder_reconnect_clock_restore.md](Encoder_reconnect_clock_restore.md)。

R41 将编码器诊断明确标注为 E1/E2，并补上 M1 在途 DMA 超时检测和主循环恢复；恢复仅重建采样，不自动重启电机。ENC2 五次连续失败后停止探测的策略继续保留，见 [Encoder_axis_dma_recovery.md](Encoder_axis_dma_recovery.md)。

R40 在 Status → Enc Last 区分全零 `ALL 0` 与全一 `ALL 1` 帧，明确当前健康、历史样本年龄和错误恢复规则；诊断边界和上板定位见 [Encoder_constant_frame_diagnostics.md](Encoder_constant_frame_diagnostics.md)。

R39 将两路编码器时钟减半至约 332 kHz，继续使用硬件 SPI + DMA；共享外设时钟已补偿，配置和验证说明见 [Encoder_SPI_slow_clock.md](Encoder_SPI_slow_clock.md)。本轮未继续优化控制算法。

R38 根据速度和经过滤波的加速度共同调度主轴输入整形，改善手转过程的快慢起伏；保留已在板上改善停止高频振动的 R37 近静止保持。适用于 4015 电流闭环，模型结果、角度滞后取舍及实机边界见 [4015_moving_smoothness.md](4015_moving_smoothness.md)。R37 停止修正见 [4015_follow_stability.md](4015_follow_stability.md)，R35 高速基线见 [4015_fast_follow.md](4015_fast_follow.md)。

R33 修复启动注释吞掉 HAL_Init 导致上电进入 Error_Handler 的问题，增加启动阶段和错误入口返回地址，并对最终 ELF 的初始化调用顺序增加检查；详见 [Startup_error_handler.md](Startup_error_handler.md)。主从轨迹沿用 R32。

R32 将手动主从改为有限加减速度的追赶参考、轨迹速度前馈和位置/速度误差修正，独立使用 P=2、速度阻尼=2；普通位置模式仍用 P=6。参数、模型比较和上板步骤见 [Hand_follow_trajectory.md](Hand_follow_trajectory.md)。

R31 将两型号位置 P 从 4 提到 6，继续使用位置 P + 速度 PI，不加入位置 I/D。接近目标时的减速原因、I/D 的适用条件和响应验证见 [Position_response_tuning.md](Position_response_tuning.md)。

R30 将编码器失联明确为 ENCODER1 LOST / ENCODER2 LOST，并在联动停止时保留首先检测的来源；Run Stop 的 Stop Age/Err 跟随该来源读取，具体见 [Run_stop_diagnostics.md](Run_stop_diagnostics.md)。

R29 将 Pos Speed 编辑范围改为 1～当前轴 Speed Limit，扩展存储到 600，调整两型号位置响应和 4015 电压模式速度 PI，并单独处理手动跟随的追赶与 FOLLOW ERROR。参数、原因及验证边界见 [Position_response_tuning.md](Position_response_tuning.md)。原 Pos Speed=30 保留，需要在从轴设置中手动提高。

## CubeMX 核查结果

已对照用户重新生成的 `.ioc` 和初始化代码核对：SPI2_RX=DMA2 Channel1、SPI2_TX=DMA2 Channel2，均为 Normal、Byte/Byte、Memory Increment、High DMA priority；两个 DMA IRQ 和 SPI2 IRQ 的抢占优先级均为 1。DMA2 时钟、HAL 链接和三个 IRQ handler 齐全。

TIM8 的 PC6/7/8、PB3/4/5 均已下拉；TIM1/TIM8 保留 PSC=0、ARR=8499、中心对齐 10 kHz、TIM1 TRGO Enable → TIM8 ITR0 Trigger、死区 17。ADC 注入四个 Rank、双 ADC 同时模式、规则组 VBUS + VREFINT 及其采样时间均保留正确。当前 APB1/2=170 MHz；SPI1 /256、SPI2 默认 /8、编码器临时 /256、SPI3 /16，见 [Encoder_reconnect_clock_restore.md](Encoder_reconnect_clock_restore.md)。SPI2/SPI3 的 NSSP 仍由 main 的 USER CODE 清除，与软件 GPIO 片选配合。

**本轮不需要再修改 CubeMX。** 后续重新生成仍应保留 USER CODE 和应用 CMake 配置。完整外设清单见 [Dual_motor_CubeMX_setup.md](Dual_motor_CubeMX_setup.md)。

## 控制组织与入口

`motor_control.c` 统一路由两路命令、配置和协同关系。M1 保留原 JerryFOC 有感/HSO 控制；M2 独立维护编码器零点、机械位置与速度、电流/速度/位置 PI、目标、对齐、启停和故障状态。两路共用数学变换与调制实现，各自写入 TIM1/TIM8，型号、极对数和保护预算互不替换。

| Settings → System → Work Mode | 含义 | Run 控制模式 |
| --- | --- | --- |
| Single M1 | 仅 M1；编码器或 HSO 混合反馈 | Torque / Speed / Position |
| Single M2 | 仅 M2；固定编码器 | Torque / Speed / Position |
| Dual Independent | 两路分别接收目标，独立启停 | 两路各自 Torque / Speed / Position |
| Dual Sync Speed | 两路跟随共同速度轨迹 | Speed |
| Dual Sync Position | 两路跟随共同位置轨迹 | Position |
| Dual Master M1 | 手转 M1，M2 按测得机械位置跟随；M1 功率输出关闭 | 位置跟随 |
| Dual Master M2 | 手转 M2，M1 按测得机械位置跟随；M2 功率输出关闭 | 位置跟随 |

双电机模式一律编码器反馈，界面禁用 HSO Feedback，串口也拒绝 HSO 接管/学习。返回 Single M1 后，可在 System → HSO Feedback 开启旁路观察、学习和手动 Use；它仍需编码器启动和资格判断，不是脱离编码器的无感启动。默认新配置和旧记录迁移均为 Single M1 + Encoder。

## 第一次上板操作

1. 在原单 M1 模式确认 M1 正常。先通过 System → View M1 / View M2 选择配置对象，再在 Motor Model 中分别选择实际接入的 2804 / 4015。Curr Loop、Pos Speed、Speed Limit、Run Steps 也分别对应当前查看的电机；LED 属于整板设置。
2. 先选择 Dual Independent。两路完成电流零点校准后依次对齐，每路约 3 秒。M2 编码器未接好时不会开启其功率输出。Status 的 Axis、M1 Model、M2 Model、Encoder、Aligned、Drive、Output 可核对对象和就绪状态。
3. 在 Run 中先低速验证每路的实际方向、采样和停止行为，再验证同步/主从。两路型号可以不同，速度和位置关系采用机械量，不复制 PWM 或电角度。
4. 新控制路径尚未在真实双电机上运行验证。Status → Ctrl Max 显示自启动以来 ADC 控制回调的最大执行时间；双路 10 kHz 的单次预算为 100 us。代码在执行达到 17000 个 DWT 周期且输出已请求时锁存 CONTROL OVERRUN 并停止两路。这项检查不等于已经测得板上最坏执行时间，也不覆盖全部中断延迟。

型号、工作组合、反馈和比例/偏置修改要求两路输出已关闭、无对齐正在进行，测得速度均不超过 1 rad/s。换型号清空该路目标、关闭电流环、加载对应 PI/阈值并重新对齐；不清除 ADC 电流故障或过载保护锁存。

## Run 按键

- 上/下：按当前模式步长修改草稿，支持按住连续加减。
- 左/右：切换允许的控制模式；切换时先停止，需要再次确认启动。
- OK：草稿有变化时始终提交新目标；数值不变时在启动/停止之间切换。
- KEY1：仅 Dual Independent 的 Run 页切换 M1/M2 视图，保持另一电机运行，并保留每路自己的草稿和已确认值；20 ms 去抖，一直按住只切换一次。
- KEY2：停止并返回；退出 Run 时停止两路。单路启动对齐期间的空闲页面进出保留原对齐流程。

独立模式中，不改数值再次按 OK 只停止当前路；KEY2 或无前缀 Stop 停止两路。同步/主从的启动和停止作用于整个组。专用 Stop 为关闭相输出的滑行停止，Speed=0 继续正常闭环减速；没有屏蔽过流，也没有增大原有门限。

Torque 的单位继续由 Curr Loop 决定：关闭时为 Uq 电压 V，开启时为 Iq 电流 A。尚不能将电压模式的数值解释成 N·m。

## 同步、主从与保护

Gear Ratio 以百分数编辑，100=1:1、-100=反向 1:1，允许 -1000～1000，0 无效；Gear Offset 以 0.01 rad 为单位编辑。串口 Gear 命令使用实际比例和 rad。

同步模式两路跟随共同轨迹。R28 将主从模式改为手动示教跟随：主轴关闭六路相输出、保持滑行，仅持续读取编码器，不自动通电对齐；从轴完成正常对齐后，在 Run 按 OK 或发送 `Follow` 才开始位置闭环。以启用跟随时的位置 Pm0、Ps0 为锚点：

```text
位置同步：主轴目标 = Pm0 + 共同位移轨迹
          从轴目标 = Ps0 + offset + ratio × 共同位移轨迹

手动主从：最终目标 = Ps0 + offset + ratio × (主轴实际多圈机械位置 - Pm0)
          在线追赶轨迹 → p_ref / v_ref
          从轴目标速度 = v_ref + P × (p_ref - 实际位置) + damping × (v_ref - 实际速度)
          （受从轴速度上限及原有速度给定斜坡约束）
```

R38 的 4015 电流跟随在整个减速过程中使用同一对参考位置/速度。主轴活动度同时调度输入带宽与快慢通道，平缓手转继续平滑，快速往复由加速度维持快速通道。主轴安静且误差、速度足够小时，沿用 R37 的积分保持及负载补偿，详见 [4015_moving_smoothness.md](4015_moving_smoothness.md)。

设置 Gear Ratio=100、Gear Offset=0，即同方向 1:1 跟随机械角度变化：主轴手转 90°，从轴目标也增加 90°；跨过一圈和反转均使用连续多圈位置。此前保存的自定义比例/偏置继续保留，首次试用请核对这两项。主轴在启用瞬间可以正在手转，从轴必须已静止。重复发送 Follow 不重新抓取零点；Stop 后再次 Follow 才建立新锚点。

这个锚点定义不会在开启时强行将从轴拉到主轴的编码器读数。两轴原始编码器零点和电角度对齐零点不代表同一个外部机械零位；若希望两根轴的指针始终指向同一方向，先手动摆到相同的机械位置，再按 OK 建立共同起点。运行中有真实闭环误差和延迟，不能保证任意负载、任意手转速度下完全重合。

Run 的手动跟随页面显示 `Hand M1 -> M2` 或相反方向、RUN/STOP 和 Error（从轴实际位置减目标位置，rad）。OK 启用/停止跟随，KEY2 停止并返回；方向键不再输入速度、扭矩或位置数值，避免误驱动主轴。Run 自动选择从轴的参数视图，Pos Speed、Speed Limit、Curr Loop 使用从轴自己的设置。Status 可以分别查看两轴，主轴 Drive 显示 HAND MASTER，Aligned=NO 不妨碍手动测量。主轴的 Align / Retry 仅重试编码器，不通电对齐。

同步模式的位置组目标仍是相对启动锚点的共同位移，运行中修改目标保留锚点，同步组以 M1 为参考。

同步组速度受两路 Speed Limit、型号最大速度和比例共同约束；加速度与减速度也按比例缩放，使从轴不超过自身 100/150 rad/s² 斜坡。手动跟随的参考加减速度在电压模式及 2804 电流模式为 40/40 rad/s²。4015 电流跟随按误差连续调整参考加速 55～180、减速 100～500 rad/s²，后级给定斜坡最多 300/600 rad/s²；速度前馈仍来自受约束参考。最终指令受从轴 Pos Speed、Speed Limit 和型号最大速度约束。参考约束不等于实测轴加速度保证，超过从轴能力时仍会产生误差。

同步模式启动 3 秒后，速度关系误差超过 10 rad/s、或位置关系误差超过 1 rad，持续 500 ms 会触发 SYNC ERROR 联动停止。手动跟随允许短暂落后和追赶，单独检测 FOLLOW ERROR：超过一圈偏差，或主轴基本停止且从轴没有朝目标追赶、偏差超过 1 rad，持续 500 ms 停止（仍保留 3 秒启动窗口）。具体速度判据见响应文档。这些是软件监督门限，不是实际机械精度保证。

独立模式的单路电流/编码器故障仅停止该路；同步组任一路驱动故障停止两路。手动跟随中，任一路编码器失联、从轴驱动故障或意外输出状态都会联动停止；主轴保持滑行。公共 ADC、注入配对、母线、VREFINT 等故障停止两路。恢复后不会自动跟随，必须重新按 OK 或发送 Follow 建立新锚点；M2 在作为通电从轴时丢失编码器会取消对齐，需要先 Align / Retry。ADC/过流/过载锁存处理仍需按原故障流程复位，不通过换模式绕开。

M2 W 相电流由 `Iw=-(Iu+Iv)` 重建，并参与瞬时过流和过载判断；该路 IW R/Z 显示 derived，重建 W 相触发时 I Trip 标为 RUN W derived，不将虚构的 ADC 计数显示成实测值。

## SPI2 与保存

M2 启用时由 ADC 节拍调度 3 字节 SPI2 DMA，每帧临时切换 Mode 1、/256，完成后恢复 Flash 参数。DMA 完成回调不自触发下一帧。5 次实际连续失败后停止 DMA；启用的 ENC2 等输出停止并释放 SPI2 后，每 250 ms 探测一帧，有效后恢复 DMA；未启用的 ENC2 继续停止探测。有效帧清除失败连续计数，SPI2 忙不计为采样失败，恢复不自动 Run/Follow。

M2 运行/对齐及整个手动跟随期间独占实时采样，包括 M2 作为不通电主轴的情况；保存和日志排队，Status → Save Bus 显示 DEFERRED，停止后自动继续。空闲时 Flash 每次事务及整个存储服务步骤持有总线锁，编码器让出该时间窗口。数据更新与输出启停仍须通过控制接口，不能从其他中断直接访问 W25Q64。

DMA 超时/错误先让控制环停止，清理和 Abort 放到主循环，清理完成后才恢复片选和总线状态。显式 Align / Retry 可以重新尝试编码器，不需要为了重试一个缺失设备持续占用 SPI2。

设置记录升级到 version 3，A/B 和 CRC 保留，自动迁移 v1/v2；M1 原目标、步长、LED、型号等保留，新增加的 M2 参数采用默认值。配置恢复不启动电机。

## 串口命令

配置命令在主循环执行，最多排队 4 条；配置正在处理时拒绝插入运动命令，Stop 仍可立即处理。成功的配置变更会请求后台保存。

```text
Stop
Motor1:4015
Motor2:2804
Work:Independent
Work:SingleM1
Work:SingleM2
Work:SyncSpeed
Work:SyncPosition
Work:MasterM1
Work:MasterM2
Feedback:Encoder
Feedback:HSO
Gear:-1,0.1
Focus:M1
Focus:M2
Align:M1
Align:M2

M1:Speed:5
M2:Speed:10
M1:Angle:0.5
M2:Torque:0.1
M1:Stop
M2:Stop
Follow
```

示例是命令格式，不要求连续执行全部模式切换。单电机模式中，原 Speed/Angle/Torque 默认发往启用的电机；双独立默认发往当前查看的路，推荐显式 M1:/M2: 前缀。在同步模式，运动命令作用于整个组，不允许单独覆盖从轴；组位置目标按上面的相对锚点定义解释。手动主从仅通过 Follow 启用、Stop 停止，拒绝 Speed/Angle/Torque（不改变正在进行的跟随）。独立 Angle 沿用原有的当前整圈基准语义。OpenLoop 仅允许 Single M1。

手动跟随最短流程：停止两路 → Work:MasterM1（或 MasterM2）→ Gear:1,0 → 等从轴对齐并静止 → Follow → 手转主轴。配置命令需要等待主循环应用，可通过 Status 的 Work、Cmd 核对后再发下一条；不要把配置与 Follow 在同一处理批次内连发。

Status → Cmd：1=接受，0=配置排队中，-1=状态/配置不允许，-2=运动参数格式错误，-3=配置队列忙。拒绝启动时同时核对对应路 Drive、Encoder、Aligned；运动目标遵守对应路 Speed Limit。单 M1 的 HSO 命令和资格条件沿用原文档。

JustFloat 保持 25 个 float + 4 字节帧尾，共 104 字节。Single M1 的映射不变。启用 M2 时，前 16 通道继续为 M1；后 9 通道为：17 Work 枚举、18 M2 实际速度、19 M2 目标速度、20 M2 位置、21 M2 Iq、22 M2 Id、23 最大控制执行时间 us、24 M2 标志（bit0 已对齐，bit1 请求输出）、25 协同误差。此时后 9 通道替代原 HSO 遥测。

## 验证与资源

已通过 `tests/motor_host/run.py`、`tests/ui_host/run.py`、`tests/usb_host/run.py`。新增测试覆盖真实 M1 FOC + M2 控制/协同 + 协议代码中的两种手动主从方向、主轴不通电/不对齐、手转启动、跨圈/反转、启动锚点、速度前馈、比例/偏置、M2 主轴的 SPI2 预留、编码器失联联动停止、恢复不自启和显式再启动；并回归采样保护、MT6701/W25Q64、存储和 USB。UI 覆盖手动跟随页、OK 启停、方向键无运动副作用、KEY2 停止及版面宽度。外设、ST HSO 二进制计算及机械运动使用主机模型，不能代替上板带载验证。

Debug：Flash **130396 B / 131072 B**，余 **676 B**；RAM **21840 B / 32768 B**。Release：Flash **129612 B**，余 **1460 B**；RAM **21744 B**。主循环、外设初始化、USB 及 SPI/GPIO/UART HAL 共享 LTO 优化，采用单一 LTO 分区；中断分发采用 -Os，HSO 初始化/配置及使能入口按大小优化，10 kHz HSO 算法保留原设置。Debug 仍含调试符号，但部分局部变量会被优化。Flash 已接近容量上限。

两种构建的 ELF/BIN/MAP 均已更新，未烧录或驱动实际电机。链接仍有原 RWX LOAD 段警告，LTO 另有串行编译提示；构建均成功，无 Flash 溢出。
