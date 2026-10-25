# M1 独立无感启动

R77 新增单 M1 编码器模式下的[极对数／电阻检测](Motor_detect_R77.md)。检测结果单独保存，HSO 继续使用既有预设及配置参数；进入检测前需关闭 HSO Feedback 和 Six Step。

R76：HSO 最低非零机械速度指令按 5 rad/s 向上取整，2804／4015 均为 5 rad/s，正负方向一致，之后按 Run Steps 调节。观测器实际运行／接管判据不因指令取整而提高。实现和验证见 [R76 最低速度](HSO_speed_floor_R76.md)。

R72 起，HSO 的设置入口为 `Settings → System → Sensorless → HSO Feedback`，六步开关也在同一目录；Status 下 HSO 与 Six Step 分别独立。仅调整界面分类，本页控制流程保留，见 [R72 UI](UI_R72_sensorless_menu.md)。

UI-R72-UI 在 Single M1 提供独立六步无感 Throttle／Speed 驱动及 ESC Auto 通用油门，与 HSO 互斥；ESC Auto 仅作用于六步，不替换本页 HSO 算法及其编码器独立启动规则。选择和验证见 [通用六步](Six_step_ESC_R70.md)。

R67 的 OFF／ON ADC 采样中心切换仅用于六步 SYNC／RUN，HSO 与编码器 FOC 保留谷底采样；退出六步恢复原 CH4 配置。详见 [R67 采样说明](Six_step_R67_sampling.md)。

R66 共用 STM32G431 的 CORDIC 计算正余弦，替代 CMSIS 查表；每次事务设置自身格式与单位模长，恢复原中断屏蔽状态。HSO 启动状态机、参数及主计算 O2 设置保留。原 HSO 主机回归通过，但该三角函数实现替换尚未实板复测。

UI-R59-STOP 增加 KEY1 全局停止，取消 HSO 启动或 CONTROL，松开后不自动重启；重新启动仍检查静止和启动条件。详见 [KEY1 全局停止](KEY1_global_stop.md)。以下 R58 硬件保护与 R53 无感控制继续保留。

UI-R58-IBUS 在既有无感控制上加入独立的 M1 母线硬件过流保护，见 [IBUS 门限与恢复](M1_IBUS_hardware_protection.md)。硬件过流后优先显示 M1 IBUS TRIP；确认清除不自动重启，下次仍需手动进行完整 HSO 启动。以下保留 R53 无感算法和 R57 界面说明。

当前固件为 **UI-R79-ID**，HSO 速度编辑边界以 [R76 最低速度](HSO_speed_floor_R76.md) 为准，Status 分组和辅助单位见 [R57 UI 说明](UI_R57_status_units.md)。HSO 控制沿用 R53：在有效 PWM 高平台上对齐三相电压采样时间，减少错时采样把 PWM 共模变成假差分电压的问题。保留 R52 偏置估计使能修复、R51 固定相位窗口和 dq 电流交接、R49 周期平均磁链学习、R50 有限资格窗口，以及 R48 的 SPI2/保存修复，见 [W25Q64_HSO_save_recovery.md](W25Q64_HSO_save_recovery.md)。

本版把 Single M1 的 HSO Feedback 改为独立无感速度控制。编码器不参与启动、磁链估算、角度资格、速度反馈或故障后回退；允许 MT6701 未连接。双电机模式及 Single M2 继续使用编码器，原有主从跟随算法和 E1/E2 诊断保留。

本次优先面向用户的 4015。ST 核心仍是 MCSDK 6.4.2 的官方 ARM 静态库；启动状态机及 I/F 磁链估算属于本项目外围实现。用户先确认 R52 在 12 V 成功接管、20 V 启动失败，随后确认 R53 下 20 V 也已可以运行；12/20 V 均已有用户实板成功反馈。

## R53：12 V 成功、20 V 接管失败

**实板反馈更新（2026-10-23）：** 用户在 R53 发布后确认“20V也已经可以运行了”。结合此前 12 V 成功的记录，当前两个母线电压均已有独立 HSO 运行成功反馈。本轮测试上下文为 4015、空载、`Speed:50`；尚未提供成功后的 H Off/H R Hz/H R Drift/H Time、重复启动次数或连续运行时长，因此暂不扩展为长时间、带载、全速域或全部启动条件验证。该结果支持采样时间对齐修正的有效性，但不单独证明所有误差都来自这一因素。现有参数和保护阈值保持不变，以下保留失败现场及修复依据。

两次均为 4015、空载、`Speed:50`；20 V 是停止后调整母线，再重新启动，排除了本次是在 CONTROL 运行中改变母线的情形。虽然命令为 50 rad/s，CAPTURE 前虚拟 I/F 速度仍被 15 Hz 电频率限制在约 8.57 rad/s。失败时 H Speed 是失锁观测值，不能直接当实际机械转速。

20 V 记录：H Gate/H Reject=ANGLE，H Reset=950，H R Deg=-82.9°，H R Hz=-11.3 Hz，H R EMF%=78.4%，H R Drift=-91.1°，H Off=57.8 mV、H Lim=59.3 mV，H Lost=LOCK TIMEOUT；H Speed=1.7 rad/s，H Error=-83.6°，H Flux=7545 uWb，H Actual=5.5 mWb，H Time=78 us/0。补偿已接近默认上限，角度、频率和相位都明显不满足接管条件；78 us/0 未报告执行超时。本轮不提高补偿上限、电流、超时或资格容限。

检查代码确认：ADC 同步时钟为 42.5 MHz，Rank1 电流采样 12.5 周期；Rank2/3 都为 47.5 周期采样，加上 12.5 周期转换。V 相电压在 ADC2 Rank2，U/W 相电压在 Rank3，V 比 U/W 早 **60/42.5 MHz≈1.411765 us**。该通道顺序本身与 ST 的双 ADC 建议一致，但 ST 同时要求考虑电流/电压采样间隔。[ST HSO ADC 配置说明](https://community.st.com/stm32-mcus-motor-control-34/hso-adc-config-145373)。单个 KsampleDelay 不能单独消除三相电压之间的 PWM 共模错位。

用户相电压网络为 `(20 kΩ || 2 kΩ) × 22 nF≈40 us`，极点约 25000 rad/s。10 kHz、50% 占空比、三相实际电压相同的连续 PWM/RC 模型中，当前采样时刻计算出假 Clarke 矢量约：12 V 时 `(0.0553,-0.0958) V`、模长 0.1106 V；20 V 时 `(0.0922,-0.1596) V`、模长 0.1843 V。模型说明有一条随母线放大的确定误差路径，并不证明它是实板全部误差的唯一来源。

R53 的处理：

- 采样入口在任何输出重新使能和本周期 FOC 写 CCR **之前**，快照 M1 的 MOE、六路 CCER 和三个 CCR。仅当三相 PWM1 高电平都覆盖整个电压采样窗口（CCR1/2/3 都大于 800，对应 4.71 us，含触发/死区余量）才允许对齐。原始 ADC 值和 `MotorSampleFrame.m1_voltage[]` 均保留，不做零点伪造。
- HSO 输入侧根据当前滤波极点和实测母线，把晚采的 U/W 沿已知 RC 过程退回 V 的时刻：`Vearly = Vlate + (Vlate - VBUS)·(exp(Δt/τ)-1)`。系数在配置时计算，采用 Padé [2/2]，本板参数下其理论逼近误差小于 1 nV，实际精度由 ADC 量化、浮点和元件误差决定；快环只增加两次乘加，没有 expf。
- 高平台条件不成立、输出关闭或边沿附近时，保留原有三相测量输入，不跨未知开关边沿推算；原来的失锁保护继续判断。高调制度下可能落入此回退，不能把本版视为覆盖所有 PWM 波形的全范围补偿。
- 对齐到 Rank2 电压时刻后，使用的 IU/IV 均来自 Rank1；默认 KsampleDelay 对应两者采样结束间隔 1.411765 us，更新为 **0.01411765 PWM 周期**。自定义 `HSO:Filter` 仍可覆盖滤波参数；RC 硬件改变时必须匹配实际极点。
- 没有改 ADC/定时器初始化、PWM/控制频率、双电机有感或主从控制。无需修改 CubeMX。构建检查新增 ADC Rank/采样时间、触发、PWM 平台和时钟一致性校验，重新生成后若不匹配就报错。

独立的连续时间 PWM/RC 解经过 ADC 量化，再输入真实 `st_hso.c`，验证 12/20 V、零差分和不同占空比场景：对齐后的 Clarke 向量误差小于一个折算至相端的 ADC LSB（约 8.76 mV）。测试还覆盖边沿/滑行不补偿、前两帧重新使能不误判为已驱动、无效母线拒绝、原始数据保留，以及旧启动、保护、编码器、SPI2/Flash、UI/LCD 和协议回归。它不是实板波形，也没有模拟全部 ADC 噪声、阻抗误差、开关压降或元件容差；实际 HSO 收敛和时间余量仍需测量。

上板先保持相同的 **4015、空载、Speed:50** 对比 12 V 和 20 V，确认 About 为 **UI-R53-SAMPLE**。若仍失败，优先记录 H Off mV/H Lim mV、H Reject、H R Hz、H R Drift 和 H Time/Ctrl Max；是否显著减小补偿占用可帮助判断此误差路径的影响。相电压分压增益不匹配、RC 容差和 PWM 混叠仍可能需要进一步测量，不能由 12 V 成功就宣称 20 V 必然收敛。

R53 Debug：Flash **131068 B / 131072 B**（余 **4 B**），RAM **22344 B / 32768 B**。Release：Flash **130252 B**（余 **820 B**），RAM **22264 B**。状态快照/停止及电流交接辅助按 -Os 共享，UI 对已核对长度的固定状态文字直接复制；HSO/阻抗补偿算术和官方二进制保留原设置，全部保护保留。两套 ELF/BIN/MAP 已更新；无 Flash 溢出，保留原 RWX LOAD 警告。用户已反馈实板运行成功；代理未连接开发板或代为烧录。

Debug BIN SHA256：`dd9aa0456e3e5faf50c901395dd1416c817840276804a87197699d43cdef48f8`。
Release BIN SHA256：`c324b5ad0dd7236cf8d9d4ef893cfccc37d5c8854da1df327d278734a4fb0362`。

以下 R52 及更早章节保留当时的配置、数据和构建历史；当前行为以上述 R53 为准。

## R52：接管前启用 HSO 自身的偏置估计

R51 实板记录：H Gate/H Reject=PHASE DRIFT，H Reset=682，H R Deg=-25.8°，H R Hz=-2.0 Hz，H R EMF%=81.3%，H R Drift=-20.5°，H Lost=LOCK TIMEOUT；H Error=-25.9°，H Speed=6.8 rad/s，H Flux=7491 uWb，H Actual=5.5 mWb，H Time=77 us/0。当前频差快照在 ±2.25 Hz 内，但原始相位相对窗口冻结起点的变化已超 ±10°。不能再凭这个快照判断只是固定负载角，也不能由此证明真实转子失步。

核对本机 MCSDK 6.4.2 `MotorControl.zip` 的 `templates/mc_tasks_hso.c.ftl`：`MCM_OPEN_LOOP_CURRENT_MODE`（约 812～835 行）开启 PWM 后即调用 `HSO_setFlag_EnableOffsetUpdate(..., true)`；速度、转矩和开环电压模式也开启，STOP 后关闭。原项目适配层却在每个采样强制 `phase==CONTROL` 才开启：CAPTURE 中可能因未补偿的直流反电动势偏差导致磁链/频率/角度波动，一直不能进入 CONTROL，也就始终无法开启补偿。这是已确认的集成差异，实板偏差的具体来源仍未确认。

R52 只在观测器已初始化且正在旋转的阶段（RAMP/CAPTURE/CONTROL）开启原库的偏置估计；DC ALIGN 保持冻结。未知磁链的学习阶段仍不提前运行未配置的 HSO，学习完成进入 CAPTURE 后即可估计偏置。停止后不再运行观测器，故障现场保留；重新配置会清零新诊断，避免学习时显示上一次的补偿量。

没有修改 HSO 的默认补偿幅值上限、增益、交越频率、磁链或官方库；也没有提高 ±60° 启动角差、±10° 相位变化、15% 频差、坏帧预算、2 s 锁定期限、电流或硬保护门限。主从与双有感逻辑不变，编码器仍不参与无感估计和接管。

新增 Status 诊断（均为库自身状态，不是 VREF 或独立电压表测量）：

| 项目 | 含义 |
| --- | --- |
| H Off mV | HSO α/β 两个直流补偿分量绝对值中的较大者，mV，停止后保留。 |
| H Lim mV | HSO 默认单轴补偿上限，mV，随观测器配置而定；没有观测器时为 0。 |

这次新增了 **官方 ARM 库的合成信号测试**，不再仅靠返回理想角度的替身。`tests/motor_host/probe_hso_arm.py` 用 Unicorn 加载最终 ELF，通过公开 API 运行实际链接的 `libHSO.a`；不读取/改写其私有结构字段。只用主机数学替代 CORDIC 的 sin/cos/polar，输入是已知反电动势，不包含 ADC、阻抗补偿、电流环或机械模型，因此不能据此断言实板已经收敛，也不能推断 MCU 执行时间。

以 15 Hz、7.491 mWb、72.6 V 标度为例：无 DC 偏差时两种开关设置都稳定；叠加 0.1 V 的 α 轴 DC 偏差，关闭估计后稳态角差约 -28.9°～+27.0°、频率约 11.0～22.2 Hz；开启后约 -10.8°～+10.5°、13.2～17.3 Hz。此时补偿碰到库默认约 58.83 mV 上限，继续等待到 6 s 也没有进一步改善，说明不能用延长超时解决超出补偿范围的问题。0.08 V 偏差开启补偿后末段角差小于 10°，关闭时大于 15°；0.2 V 偏差仍严重失锁。上述测试复现了相关失真机制，不代表板上的偏差恰好是这些数值。

复现命令（依赖 `unicorn`、`pyelftools`；也可用 `--python-path` 指向安装目录）：

```text
python tests/motor_host/probe_hso_arm.py --python-path build/hso-emulation/python --self-check
```

主机回归另外检查已知磁链时 ALIGN 关闭、RAMP/CAPTURE/CONTROL 开启偏置估计，新诊断的数值显示，以及原有学习/相位漂移/接管/故障恢复、SPI2/Flash、UI/LCD 和协议路径。最终 ELF/BIN、强 MSP/中断入口和版本标识也已核对。

本轮无需改 CubeMX。烧录后确认 **UI-R52-OFFSET**，先保持 4015 空载、`Speed:10`，记录 H Off mV、H Lim mV、H R Drift、H Reject 及 H Time/Ctrl Max。若补偿量持续顶到上限而相位仍不稳定，应核对差分相电压偏差、采样/滤波以及 Rs/Ls，不能据此直接扩大补偿范围或取消失锁门限。

R52 Debug：Flash **130916 B / 131072 B**（余 **156 B**），RAM **22336 B / 32768 B**。Release：Flash **130108 B**（余 **964 B**），RAM **22256 B**。两套 ELF/BIN/MAP 已更新；保留原 RWX LOAD 警告，无 Flash 溢出，未烧录。

Debug BIN SHA256：`910fb66235f509dbf2cf178c26e23e95e88654f53ac7815c47031a7e9ad5e6b3`。
Release BIN SHA256：`d26ee7c5d08b7b606161046eb200053c7326d86eddc4ef9d3da7723371700903`。

以下 R51/R50/R49 章节保留当时的门限和构建历史；当前行为以上述 R52 为准。

## R51：稳定负载角与失步判定分开

R50 实板记录：H Gate/H Reject=ANGLE，H Reset=309，H R Deg=-28.8°，H R Hz=-2.3 Hz，H R EMF%=82.6%，H Lost=LOCK TIMEOUT；H Error=-28.7°，H Speed=6.7 rad/s，H Flux=7480 uWb，H Actual=5.3 mWb，H Time=75 us。这次重置次数明确证明资格窗口反复被清零；重置角差超过原 25° 启动门限。82.6% 能量和约 0.71 倍观测磁链没有越过各自启动幅值范围，75 us 没报告 100 us 超时。

但不能把这组数据解释为“HSO 已经稳定，只差放宽角度”：15 Hz 虚拟频率下原 15% 频差门限为 ±2.25 Hz，记录中的 -2.3 Hz 已略超出；H Speed 还是另一滤波通道/时刻的值。真实转子是否同步、角差是否固定仍需验证。不开编码器参与判定，不取消频率保护，也不把虚拟频率写入 HSO 伪造收敛。

I/F 使用旋转的 d 轴电流矢量，虚拟角度并不是实测转子角度；带转矩时两者可以有稳定负载角。研究资料也强调 I/F 参考系与转子参考系的偏差、恒速阶段的稳定，以及交接时角度和 q 轴电流的连续性。[I/F 启动与平滑接管研究，Nair 等，2020](https://doi.org/10.1049/iet-epa.2020.0387)。以下数值是本项目的工程门限，不是 ST 官方推荐定值，也没有实现论文中的 pulse-off 方法。

- CAPTURE 原始 HSO−I/F 角差改为有界的 ±60°，给旋转 d 轴电流的 ±90° 边界保留余量。更关键的是新增**固定参考的 200 ms 相位稳定窗口**：先用约 20 ms 圆周低通取得起点，窗口开始后冻结它，原始角差相对这个起点的变化不得超过 ±10°。每个采样都参与检查；滤波仍继续用于下一窗口的起点，但不能拖动当前窗口来追随持续漂移。
- 仍先等待 100 ms；200 ms 内最多 50 个软异常，连续 20 个软异常仍重置。重置不会延长独立的 2 s 接管期限。±10° 检查也受相同坏帧预算约束，持续漂移或明显角度摆动不能靠长期零散累加过关。
- ANGLE 表示原始启动角差超过 ±60°；新增 PHASE DRIFT 表示相对冻结参考变化超过 ±10°。CONTROL 的 25° 增量残差、45° 严重跳变、频率/磁链/反电动势、ADC/过流/过载及 100 us 保护保留。原始当前帧频率/幅值合格仍是实际交接的必要条件。
- 角度交接改为 50 ms，启动电流矢量在 100 ms 内与速度 PI 的 q 轴电流交叉渐变。随着参考系旋转，同步计算 `Id = (1-w)·Istart·cos(turn)`、`Iq = w·Iq_PI − (1-w)·Istart·sin(turn)`，其中 `turn = offset·min(t/50ms,1)`、`w = min(t/100ms,1)`。保持原启动矢量的物理方向，再逐步让速度 PI 接管；全部输出继续经过原 dq 电流矢量限幅。当前 PI 状态未直接跳转参考系，50 ms 渐变的实际表现需上板核对。
- 渐变后的快环电流不回写为速度 PI 请求，避免在每个 100 us 周期上递归混合。同型号参数、电流上限、I/F 速度、学习阶段和官方静态库均未修改；双有感和跟随控制保持原逻辑。

Status 新增 **H R Drift**，单位度：最近一次窗口重置时，原始角差相对冻结起点的变化，和 H R Deg 一起保留到下一次显式启动。H R Deg 仍是 HSO−I/F 原始角差，没有改成“误差已归零”。若 R51 再次失败，优先记录 H Reject、H R Drift、H R Hz、H R Deg、H R EMF%、H Reset 和 H Time/Ctrl Max。

新增主机验证：稳定 ±28.8° 且带独立频率纹波时可接管，正反向/跨圈的电流矢量方向、50/100 ms 渐变和电流上限正确；只有 0.25 Hz 的持续相位滑移、15°/5 Hz 的角度摆动、持续 -2.3 Hz 频差仍不能接管。真实 FOC/协调器测试使用编码器断开和 -28.8° 偏置，检查快环不递归覆盖速度 PI 电流请求。原学习/故障/采样/编码器/SPI2/存储、UI/LCD 和 USB 测试通过。ST 二进制边界仍由确定性信号模拟，以上不等于真实电机或真实 ST 观测器已稳定。

R51 Debug：Flash **130716 B / 131072 B**（余 **356 B**），RAM **22296 B / 32768 B**。Release：Flash **129916 B**（余 **1156 B**），RAM **22216 B**。HSO 判定/诊断单独按 -Os 共享，Clarke/阻抗补偿/HSO 调用保留 -O2，未扩大 Flash 或删除保护。两套 ELF/BIN/MAP 已更新，启动/强中断入口、版本和 ELF/BIN 一致性检查通过；保留原 RWX LOAD 警告，未烧录。

Debug BIN SHA256：`ea3fdc1f58175eeb72d7f265c74a8465db3142ecd64f24941c995943155df6ed`。
Release BIN SHA256：`57ce48ea3cfb7ddb33a3fcfbb9d0d321fc9e6c5eb8add4a27940740953e5badd`。

以下 R50/R49 章节保留当时的门限和构建历史；当前行为以上述 R51 为准。

## R50：已完成学习，但 CAPTURE 反复重置

R49 实板数据：H Gate=OK，H Qual=9/200 ms、0/20，H Lost=LOCK TIMEOUT，H Actual=10.5 mWb；补充 H Error=10.1°，H Speed=7.4 rad/s，H Flux=7480 uWb，H Time=74 us/0。LOCK TIMEOUT 表明已有参考磁链，失败在接管资格。H Actual 是 HSO 当时的观测值，H Flux 才是 7.48 mWb 的学习参考；两者不能混称。

若参考仍为 8.57 rad/s，7.4 rad/s 接近原 15% 频差容限的下界；10.5/7.48 约为 1.40，也靠近磁链 1.5 倍的启动上限。10.1° 的当时角差没有越过 25°，74 us/0 没报告执行超时。以上是有限快照，不能证明拒绝必然来自频率，也不能排除其他时刻的角度或幅值异常。

代码中确认：原 CAPTURE 使用瞬时反电动势能量及未经低通的频率判定，连续 20 个坏帧会清零；但稀疏坏帧没有总比例限制，合格帧可能被跨很长时间累加。最后一帧恢复正常还会覆盖拒绝原因，导致超时后只看到 OK。另外学习与 CAPTURE 共用 6 s 截止时间：学习较晚完成时，剩余时间可能不足以观察满 200 ms，因此仅凭 9/200 ms 也不能确认此前发生过重置。

R50 的应用层修改：

- CAPTURE 使用 HSO 自身的反电动势频率做约 20 ms 低通，反电动势能量做约 5 ms 低通；没有用虚拟频率替代观测频率。先稳定 100 ms，再开始资格窗口。
- 至少观察 200 ms，最多 50 个软异常帧（200 ms 的 2.5%），连续 20 个异常帧（2 ms）仍立即重置窗口。不合格帧不累加为好帧，不无限暂停计数。达到窗口长度后，实际交接的当前原始频率/幅值也必须合格；等待当前好帧时坏帧预算仍保留，进度封顶 200 ms。
- 对齐/I/F/磁链学习仍最多 6 s；进入 CAPTURE 后独立计时，最多 2 s，完整启动最多约 8 s，重置资格窗口不重置锁定期限。这样较晚完成学习也有完整的滤波稳定和资格观察机会，不无限延期。
- 保留 25° 原始角差、15% 启动频差、0.5～1.5 倍参考磁链、反电动势门限和原始方向检查。CONTROL 中原逐帧保持判据和全部 ADC/过流/过载/执行时间硬保护保留；不提高电流、速度或盲目放宽阈值。
- CAPTURE 的最近拒绝原因不再被好帧覆盖；重置记录保留到下一次显式启动。启动资格与故障辅助按 -Os 共享代码以适配 128 KiB，HSO 算术和官方二进制的优化设置保留。

新增 Status 记录均可停机后读取，不必在运行中抓瞬时数值：

| 项目 | 含义 |
| --- | --- |
| H Reject | 最近一次资格窗口重置原因；没有重置时 OK。 |
| H Reset | 本次显式启动内的资格重置次数，重新启动清零。 |
| H R Deg | 最近一次重置时的原始 HSO − I/F 电角度误差，单位度。 |
| H R Hz | 当时滤波后的 HSO 电频率 − I/F 电频率，单位 Hz，带符号。 |
| H R EMF% | 当时滤波能量开方 / 预期反电动势幅值，100 表示相等。一般门限约 40～160%，仍有至少 0.2 V 的绝对下限；它不是 H Actual/H Flux。 |

H Gate 保留最近的软判定原因；若进度已到 200 ms、H Reset=0，但始终没有合格的当前原始样本，H Gate 会显示 FREQUENCY/FLUX 并继续等待，仍在独立 2 s 锁定期限内超时。H Reject/H R 数据专指发生过的窗口重置，不等于所有拒绝帧计数。

新增测试在 ST 二进制边界注入 50 Hz 的 ±3 Hz 频率纹波（对应约 ±3.44° 的角度波动），原 ±2.25 Hz 逐点判据会反复经历超过 2 ms 的越界；新持续性判据可接管，模型中的原 CONTROL 判据也继续通过。另覆盖瞬时能量尖峰、少量角度异常、10% 离散坏帧不能靠无限累加接管、连续 20 坏帧、持续频差/角差/幅值错误、滤波均值合格但原始样本始终不合格仍不能接管，以及重置现场/超时首因的保留。还模拟磁链较晚才完成学习、需要超过原共用 6 s 才能观察满 CAPTURE 窗口的启动，验证独立锁定预算有效；没有信号时的学习超时仍为 6 s。完整电机/HSO、SPI2/Flash 保存、UI/LCD、USB 主机测试通过，最终 ELF 启动顺序、强中断入口及 BIN 一致性通过。测试不是实板波形或真实 ST 收敛验证。

R50 Debug：Flash **131068 B / 131072 B**（余 **4 B**），RAM **22272 B / 32768 B**。Release：Flash **130268 B**（余 **804 B**），RAM **22184 B**。两套 ELF/BIN/MAP 已更新，原 RWX LOAD 警告保留。

Debug BIN SHA256：`1f3c0a955f8f3925579cad0e6f010a639e45be666c9ec768b2187aad65879f36`。
Release BIN SHA256：`ba77c1ee9c1a8793b381396a36b5a7bf00d90e5d0c4556983eaa68a44bc99fe1`。

## R49：平稳起转但磁链学习超时

用户反馈 Ref=8.6、I/F FLUX、平稳转动，随后 H Gate/H Lost=START TIMEOUT，H Actual=0.0 mWb，H Qual=1/200 ms。已确认该退出是启动超时，而非尚未起转；平稳转动本身仍不能独立证明没有失步。

R48 逐 100 µs 采样检查反电动势幅值和方向，任何一次不合格就把学习进度清零，要求连续 10000 次全部合格；整段启动最多 6 s。这种逐点判据容易被 PWM 纹波或偶发噪声打断。本轮未测量实板电气信号，不能据此认定具体噪声来源。另两项确认为显示缺口：学习尚未运行 HSO，H Actual 没有被更新；学习超时后 H Qual 分母错用 CAPTURE 的 200 ms。

R49 先等待两个完整电周期，再对阻抗补偿后的反电动势做虚拟 dq 变换，按完整电周期平均，**包含每一个采样，不筛选有利采样**。方向、0.2 V q 轴幅值、约 30° 相位界限改为检查周期均值；仍需累计至少 1 s 合格周期，周期磁链估算的标准差仍须不超过均值 10%。不合格的整周期仍会重置学习；未提高启动电流、速度上限、超时时限或接管/运行保护门限。

- H Actual 在 I/F FLUX 期间显示最近一个完整电周期的**候选磁链**，不是编码器实测值，也不表示已经可以接管；进入 CAPTURE 后改为 HSO 观测磁链。
- H Gate 区分 EMF SETTLE（先稳定两个周期）、EMF LOW（q 轴幅值不足）、FREQUENCY（反电动势方向不符）、ANGLE（平均相位超限）、FLUX（磁链范围不合格）和 FLUX RIPPLE（周期估算不稳定）。当前合格时显示 OK；学习超时后保留最近一次学习拒绝原因。
- H Lost 区分 FLUX TIMEOUT（6 s 内未完成学习）和 LOCK TIMEOUT（已有磁链但未完成接管），采样、过流、执行超时等硬故障仍立即停止。
- H Qual 在学习及学习超时后统一使用 `/1000 ms`，按完整电周期增加；右侧 `/20` 仍是 CAPTURE/CONTROL 的软异常计数，不是学习失败次数。

4015 的 8.57 rad/s 来自 **15 Hz 电频率上限**（约 82 rpm 机械），是虚拟参考，不是实测转速或 HSO 的物理最低速度。先保持相同 Speed:10 对比本轮修改；只有 H Gate=EMF LOW 等数据才能支持进一步判断需要提高信号幅值，不能只看到 Ref=8.6 就认定 HSO 无法低速收敛。

## 和 ST 官方流程的关系

ST 员工说明：提供初始转子位置后，HSO 可以直接闭环启动。RsDC 通过通电对齐得到初始角度，PolPulse 估算初始位置；RsDC 可能引起最多 180° 电角度的对齐运动。必要时可以使用开环启动。[ST 官方启动说明](https://community.st.com/stm32-mcus-motor-control-34/hso-startup-alignment-rotor-movement-and-gearbox-load-considerations-168778)

还对照了本机 `D:/Tools/MC_SDK_6.4.2/Utilities/PC_Software/STMCWB/assets/MotorControlPack/MotorControl.zip` 中的 `rsdc_est.c.ftl`、`speed_pos_fdbk_hso.c.ftl`、`speed_torq_ctrl_hso.c.ftl` 和 `mc_tasks.c.ftl`：固定初始角度、电流环使能、速度环暂停、Id 缓升，以及使用公开的 `HSO_setAngle_pu`、`HSO_setFlag_TrackAngle` 接口。未改 ST 供应商文件或猜测其内部对象布局。

本版采用 RsDC 风格的 **AlignOnly**，不自动修改 Rs。由于磁链未知，额外增加 I/F 起转及磁链估算，再经过资格判断切入 HSO；这一步不是 ST Motor Profiler，也没有加入 PolPulse 或 ZeST。

## 启动过程

| Status → HSO | 行为及通过条件 |
| --- | --- |
| WAIT ADC/STOP | ADC/母线/电流零点有效；连续 200 ms 的差模相电压幅值低于 0.3 V、电流矢量幅值低于 0.1 A。该条件只是电气静止准入，不是精确零速测量。 |
| READY | 已可接收新的 Speed 启动命令；不会自动通电。 |
| DC ALIGN | 固定电角度 0，Id 在 250 ms 内升到预设连续电流上限的 50%，总对齐时间 1 s。4015 为 0.6 A。末尾校验实际电流，达不到条件则停止。 |
| I/F RAMP | 速度环暂停，虚拟电角度以机械加速度 20 rad/s² 起转，保持相同 Id、Iq=0。升到目标速度与 15 Hz 电频率两者中较低者。 |
| I/F FLUX | 仅磁链未知时执行。先稳定两个电周期，再累计至少 1 s 合格的周期均值估算磁链；不读取编码器角度或速度。 |
| CAPTURE | 运行 ST 阻抗补偿与 HSO，先等待 100 ms 滤波稳定，再观察至少 200 ms；有限坏帧预算和当前原始好帧共同决定接管。 |
| CONTROL | 电角度在 50 ms 内平滑交接到 HSO，启动电流矢量在 100 ms 内与速度 PI 的 q 轴请求交叉渐变；速度 PI 使用 HSO 机械速度，电流环持续工作。 |
| START/HSO FAIL | 对齐/I/F/学习超过 6 s，或 CAPTURE 超过独立 2 s，H Lost 区分 FLUX TIMEOUT / LOCK TIMEOUT；或发生观测/采样/执行时间异常。关闭桥臂输出，故障消除并再次满足静止准入后，由新的 Speed 命令重新启动。 |

4015 在目标 10 rad/s 时，虚拟启动速度上限约 8.57 rad/s，未知磁链时名义启动过程约 2.9 s；有磁链时省去周期稳定和磁链学习阶段。实际能否启动仍取决于电机、电路参数与负载。DC 对齐及 I/F 过程会使转子运动。

磁链估算使用阻抗补偿后的反电动势幅值：

```text
Eαβ ≈ Vαβ − Rs·Iαβ − Ls·dIαβ/dt
Edq = rotate(Eαβ, -θ_virtual)
ψ ≈ |mean_over_electrical_cycle(Edq)| / |ωe_virtual|
```

要求周期平均反电动势的 q/d 方向符合虚拟 d 轴、有效 q 轴幅值至少 0.2 V、相位偏离小于约 30°，且至少 1 s 的周期估算标准差不超过均值的 10%。所有采样参与平均，零均值的高频纹波不会因逐点开方而直接抬高磁链均值。估算成立的前提是电机已随 I/F 同步旋转；这些检查不能独立证明转子完全未失步。4015 的 Rs 使用既有 1.8 Ω 星形等效值，Ls=1.3 mH 的厂家测量口径仍待核实，错误参数会影响估算和收敛。

估算结果保存在 RAM，可通过 Status → H Flux 查看。未新增自动写 Flash 的磁链持久化；复位、换型号或 HSO:Learn 后重新估算。不把铭牌 0.16 V/(rad/s) 未核实口径地直接填成物理磁链。

## 首次 4015 测试

本轮 **无需修改 CubeMX**。保留现有 170 MHz、10 kHz 同步 PWM/注入采样、VREFINT、DMA 和中断配置。

1. 烧录 Debug 或 Release 的 `FOC_two.bin`/ELF，确认 About → FW 为 **UI-R54-UI**。
2. 停止输出，Settings → System → Mode（Work Mode 页）选择 Single M1，Axis 选择 M1，再回 Settings → Motor 设为 4015。
3. Settings → System → HSO Feedback 开启，等待 Status 的 HSO=READY。Run 此时只提供 Speed；Settings 的 Curr Loop 显示 ON 并禁用编辑，无感运行强制使用电流环，编码器模式原有设置保留。
4. 先空载用 Run 确认 10 rad/s，或单独发送带换行的 `Speed:10`。观察 DC ALIGN → I/F RAMP → I/F FLUX → CAPTURE → CONTROL。启动中 Run 的 **Ref** 是虚拟参考速度，进入 CONTROL 后 **Act** 才是 HSO 观测速度。
5. 若停在 WAIT ADC/STOP，检查 Drive、Bus、VREF 及电流/相电压；若启动失败，记录 H Gate、H Lost、H Flux、H Actual、H Time、Ctrl Max 以及实际转动现象。相序/采样极性错误不会靠取消编码器校验自动解决。

也可通过 `Work:SingleM1`、`Motor1:4015`、`Feedback:HSO` 配置，但它们经主循环排队应用；每条执行后核对 Status，再发送下一条。换型号或开关 HSO 会重新开始 200 ms 准入窗口。

物理编码器未接时，E1 State=ERROR 和错误计数仍保留作硬件诊断；它们不阻止 HSO 启动。逻辑 Aligned 表示无感路径可执行自己的启动对齐，不代表编码器已经标定。Angle 等编码器位置字段也不能当作无感机械位置。

## 运行、停止和恢复

- 无感仅支持速度模式。目标电频率至少 5 Hz、低于 400 Hz，并继续受当前轴 Speed Limit 与型号最大速度约束。R76 将由 5 电气 Hz 换算的机械指令下限向上取到 5 rad/s 的倍数：4015／2804 均为 **5 rad/s / 47.7 rpm**。这是软件指令门限，不是实测最低稳定速度保证；观测器内部的物理判据保持原值。
- `Speed:0`、Stop、Run 再次按 OK、KEY2 均关闭相输出并滑行停止。普通编码器模式仍保留 Speed:0 的闭环减速语义。
- 本版无零速保持、无自动低速切回编码器、无飞车重启。低于最小非零速度的指令拒绝，并保留正在运行的原目标。
- 运行中同方向改速继续使用 HSO。直接反向指令拒绝且不改旧目标；先 Stop，待电机真正停止并通过电气准入，再发送反向 Speed。
- HSO 失锁关闭输出，不因编码器恢复或 HSO 再次有效自行起转；满足静止准入后新的 Speed 命令会重走对齐/启动。ADC/过流/过载/全局超时锁存仍按原流程处理，不靠新 Speed 清除。
- `HSO:Off` 停止并关闭观测器，静止检测仍保留；`HSO:On` 重新启用。停稳后 `Feedback:Encoder` 或 `HSO:Encoder` 才能切回编码器，随后需要健康编码器及重新对齐。
- `HSO:Learn` 现在要求停止，清空磁链，让下一次 Speed 启动重新估算。`HSO:Use` 保留兼容入口，运行中不重复启动；正常操作直接使用 Speed 即可。
- `HSO:Motor:Rs,Ls,Flux` 和 `HSO:Filter:Pole,Delay` 保留；只接受停止时的有效参数。Flux 单位是物理 Wb，不能与 MCSDK 的 `2πψ` 字段混用。

HSO 靠反电动势观测，不能把本实现解释成 ZeST 全零速域控制。[ST ZeST/HSO 说明](https://www.st.com/content/st_com/en/ecosystems/stm32-motor-control-ecosystem/stm32-zest-and-hso-solutions.html)

## 诊断含义变更

- H Error：启动时为 **HSO 角度 − I/F 虚拟角度**，CONTROL 时为相邻角度增量相对观测频率的残差；不再是编码器角度误差。未知磁链学习时显示 `--`。
- H Qual：I/F FLUX 及 FLUX TIMEOUT 显示学习进度 `/1000 ms`，CAPTURE 显示资格进度 `/200 ms`；右侧为连续软异常数 `/20`。学习期间 H Actual 为候选磁链。
- CONTROL 中连续 20 帧软异常停止；非法数值、严重角度跳变、ADC 越界、采样丢帧及 100 µs 超时直接停止。H Lost 保留停止原因，到下一次有效启动清除。
- 旧混合反馈的 H Jitter/H Reset 定义已删除；R50 新 H Reset 专用于独立启动资格窗口重置次数，R51 增加 H R Drift。遥测的 H Error 同样使用上述新定义，启动阶段的速度通道是虚拟参考。

## 软件验证与边界

Debug 和 Release 链接官方 ST ARM 库。主机测试运行真实 `st_hso.c`、FOC、协调器和协议，但用确定性信号替身模拟 ST 二进制边界；其通过不等于真实 ST 算法已经在该电机上收敛。

覆盖：已保存 HSO 配置的上电路径、编码器始终离线、4015 零磁链估算、正反向独立启动、电流缓升/限幅、资格与接管、无反电动势/对齐失败超时、样本/序号/数值/执行时间故障、失锁无自启、显式恢复、Speed:0、关闭 HSO 后模式退出，以及原有双电机/编码器/存储/UI/USB 回归。R49 新增每个瞬时样本都不满足旧相位判据、但周期均值合格的零均值纹波（正反向和低速）场景；也覆盖持续弱信号、反方向、平均相位超限、周期磁链波动不能误接管，以及学习超时后候选磁链/拒绝原因/1000 ms 分母保留。上板需实测 H Time/Ctrl Max、启动电流、磁链稳定性和失锁行为。

R49 Debug：Flash 130212 B / 131072 B（余 860 B），RAM 22144 B / 32768 B。Release：Flash 129428 B（余 1644 B），RAM 22056 B。两套 ELF/BIN/MAP 均已更新；保留原 RWX LOAD 警告。

R49 Debug BIN SHA256：`a4702b1498fdd95dc38bf207483111d25bbae8a141e500b940f7cde0cdf17c37`。
R49 Release BIN SHA256：`1810616d9555cc84a6fd076eff72d5825d27f0ed61c9edd859e574309ac91ee0`。

以下保留 R44 实现时的历史构建资源。R44 当时已核对强 MSP/中断 handler、HAL_Init 调用顺序、版本标识及 ELF/BIN 字节一致性：

| 构建 | Flash | RAM | Flash 剩余 |
| --- | --- | --- | --- |
| Debug | 130972 B / 131072 B | 21776 B / 32768 B | 100 B |
| Release | 130164 B / 131072 B | 21680 B / 32768 B | 908 B |

Debug 保留调试符号，HSO 算术保持 `-O2`，冷启动辅助按 `-Os`，适配层加入 LTO；未通过扩大芯片容量或取消保护解决空间问题。Flash 余量仍很小，后续新增功能需要继续控制体积。链接器沿用工程已有的 RWX LOAD 段警告。

Release BIN SHA256：`be6f1bcfc01f528ec9dde9e16adb68a2658fa1c54ed69c572ced9a169b6689d9`。固件位于 `build/Debug`、`build/Release`，均保留 ELF、BIN、MAP。未烧录或实际驱动电机。
