# 电机驱动板 UI 操作与验证

当前固件 **UI-R81-CF**：第 4 批整理控制框架，新增 Motion → Input、LOCAL／USB／UART 来源检查与已确认参数发布。双机通信目标必须写 `M1:`／`M2:`，浏览 Run 不打断远程运行；明确的 Back／KEY1 继续停止。规则、构建占用与验证见 [R81 控制框架](Control_framework_R81.md)。无需修改 CubeMX。

当前 R79 参数辨识已增加 Ls／磁链／Ke、左右结果翻页与 v5 保存，详见 [R79 操作与测量定义](Motor_detect_R79.md)。

R78 为 M1 参数检测的 POLES 扫角加入最多 5 ms 的近零全零缺样窗口，保留最后有效角度并等待下一有效帧；正常有感的 3 周期失联门限及其他故障处理继续使用原规则。复现、边界与验证见 [R78 跨零修复](Motor_detect_zero_gap_R78.md)。无需修改 CubeMX。

R77 新增 `Settings → Motor → M1 Detect`：Single M1 编码器模式下，以 100～500 mA 的测试电流检测极对数及等效相电阻。默认 300 mA，正反向长扫描约 36 秒；OK 开始／停止，KEY2 停止返回，DONE 后 OK 单独保存结果。结果不自动改变两套运行预设，操作、定义及验证见 [R77 电机检测](Motor_detect_R77.md)。无需修改 CubeMX。

R76 将 HSO 的最低非零 Speed 向上取到 5 rad/s 的倍数：2804／4015 均为 5 rad/s；后续增减仍按 Run Steps，反向同样处理。Run、Status → HSO 的 H Min 和控制端共用范围，0 仍为停止。详见 [R76 HSO 最低速度](HSO_speed_floor_R76.md)。无需修改 CubeMX。

R75 优化用户协议代码：负指数使用等价的 double 常量倒数，配置命令识别一次后按类型排队执行。保留所有命令、数值精度和界面；本轮再回收 728 B，Flash 余量为 2800 B。对照测试与当前固件见 [R75 协议优化](Flash_protocol_R75.md)。

R74 仅优化应用显示格式化：使用有界的整数／字符串拼接替代 `ui_app.c` 中的 `snprintf`，保持原界面、数值格式和控制功能。HAL、ST 库、中断及 KK_UI 核心文件未修改。两套构建均回收 2064 B，Flash 余量为 2072 B；对照测试与交付见 [R74 用户代码优化](Flash_user_code_R74.md)。

R73 将六步 Speed 的最低非零指令统一向上取到 5 rad/s 的倍数：2804 为 15、4015 为 10；后续增减仍按 Run Steps，正反方向一致。界面与控制端范围同步，详见 [R73 最低速度](Six_step_speed_floor_R73.md)。

**UI-R72-UI** 将两个无感开关归到 `Settings → System → Sensorless → HSO Feedback / Six Step`；Single M2／双机模式可查看目录，但两项禁用。Status 新增与 HSO 同级的 `Six Step`，原 Drive 中的八项六步诊断完整移入，标题固定标注 `M1 Six Step`。返回层级、互斥、保存及原控制行为保留。标题补齐同字体、同字号的 `l/x`，原字形和行框不变。操作、预览和验证见 [R72 菜单说明](UI_R72_sensorless_menu.md)。无需修改 CubeMX。

R71 针对“2804 预设与 ESC 同为 Throttle 20%，只有 ESC 能运行”的反馈，统一两条路径的六步启动信号资格；没有放宽过零或增大电流。Speed 启动参考不主动超过目标，原模式与界面继续保留。对照测试与当前固件见 [R71 启动资格](Six_step_capture_R71.md)。

当前固件 **UI-R79-ID**（2026-10-25）：`Single M1 + Six Step ON` 下，`Settings → Motor` 新增 `ESC Auto`。通用 Run 仅提供油门 0～100%，显示电气 Hz，机械 rpm 为 `--`；左右短按不切换到其他控制模式。确认后清空目标，停止后后台保存，掉电恢复但不自动运行。选择 2804／4015 返回原预设及 Speed 模式。通用启动不读取电机 Rs／Ls／极对数，既有两套预设共用的六步启动也改为固定电流 PI 与实测电压交接；具体电流边界、模型验证和实板复测步骤见 [R70 ESC Auto](Six_step_ESC_R70.md)。无需修改 CubeMX。

此前 R69 为启动时的 W25Q64 识别加入最多三次尝试及错误保留，SPI2 速率／模式／片选保持原值，详见 [R69 Flash 初始化](Flash_init_retry_R69.md)。R68 的无感型号切换修复继续保留。About 的 Flash 是固定型号文字，不是实际 Flash ID；诊断看 Status → Diagnostics 的 Flash Err、SPI2 Fault／Reset。

R68 已修复 Single M1 无感模式下 Motor 菜单先 Stop 清空静止计数、随后必然拒绝切换型号的问题。已停止的六步／HSO 保留静止资格，仍运行时先停止并等待资格后重试；无感模式提示 `Wait for stop`／`Motor set`，不要求编码器或编码器对齐。切换清空目标且不自动运行，后端静止和保护检查保留。详见 [R68 型号切换](Six_step_sensorless.md)。下述 R67 采样行为沿用。

R67 起在六步 SYNC／RUN 中按实际 PWM 占空比选择较长的 OFF／ON 采样窗口，保留 ALIGN→SINE→SYNC→RUN。Status → Drive 的 `6S ZC` 周期比增加一位小数，新增 `6S Cause`（相位过早／过晚、周期比过小／过大）与 `6S Sample`（OFF／ON）；拒绝原因及最近六个候选周期在故障后保留，周期轨迹通过六步 JustFloat 通道 8～13 发布，单位 ms。原 `6S Hold`／`6S Start EMF`、油门／Speed、双机、HSO、保护和按键保留，无需修改 CubeMX。实现、通道表、对照验证及实板复测边界见 [R67 采样说明](Six_step_R67_sampling.md)。

此前 **UI-R59-STOP** 接入 KEY1 全局停止：任意页面下中断关闭 M1/M2 输出，取消对齐、HSO 和主从；松开稳定 30 ms 后仍需手动重新启动。KEY1 不再由 UI 初始化或扫描。本版继续保留并覆盖六步，操作及验证见 [KEY1 全局停止](KEY1_global_stop.md)。

此前 **UI-R58-IBUS** 已接入 M1 母线硬件过流保护。Settings → M1 Protection 调门限（默认 3000 mA，500～4000 mA、步长 100 mA）及确认清除故障；Status → ADC 可查 DAC 门限、比较器状态和锁存计数。清除后保持输出关闭，上电对齐若被中断需手动重试。工作模式／型号切换保留门限，操作与验证见 [M1 IBUS 硬件保护](M1_IBUS_hardware_protection.md)。

此前 **UI-R57-INFO** 完成 B6／B7／B9：HSO 速度编辑跳过不可启动区、跨方向先确认停机；Status 拆为 Drive／Motion／Encoder／ADC／HSO／System；增加 rpm／deg 辅助显示，控制单位仍为 rad/s／rad。入口与详细规则见 [R57 说明](UI_R57_status_units.md)。长按左／右切轴与 KEY1 保留规则见 [R56 按键说明](UI_R56_axis_keys.md)。保留 Run 整体居中与首页原 12px 字形行高，见 [R55 对齐修复](UI_R55_alignment.md)，以及 [R54 第一批界面调整](UI_R54_first_batch.md)。控制算法保留 R53。

R53 在有效 PWM 高平台上对齐三相电压的采样时间，针对 12 V 可接管、20 V 失败时随母线放大的共模错位误差；保留 R52 偏置估计、R51 固定相位窗口及 dq 交接。Status 的 H Off mV（最大绝对补偿分量）、H Lim mV（单轴上限），和 H Reject、H Reset、H R Deg、H R Hz、H R EMF%、H R Drift 均可停机后读取，见 [独立无感启动](HSO_sensorless_startup.md)。保留 SPI2 Fault / SPI2 Reset 和共享总线恢复，以及 Save WAIT 与 BUSY/WEL 检查，见 [保存恢复说明](W25Q64_HSO_save_recovery.md)。

R46 在 Status 增加 Flash Err，显示 Flash 失败的操作/HAL返回值/HAL错误/SR1，区分 Save WRITE 中的写使能确认与命令/数据传输异常。Flash 发送改为同步收发；说明与当前资源见 [保存恢复说明](W25Q64_HSO_save_recovery.md)。

R45 增加 Status → Save Last，并将保存失败提示细分为状态、擦除、写入、读回、校验或忙超时；异步保存最多完整重试一次，见 [保存恢复说明](W25Q64_HSO_save_recovery.md)。R54 在 Settings 另提供持久的 Save 状态行。

R44 的 Single M1 → HSO Feedback 已改为独立无感速度启动：编码器可断开，未知磁链通过 I/F 启动估算，Speed:0 滑行停止。Run 显示启动阶段及 Ref/Act，H Error 不再相对编码器，删除 H Jitter/H Reset。操作及完整限制见 [HSO_sensorless_startup.md](HSO_sensorless_startup.md)。

R43 的 Status 增加 E1/E2 Zero、ZPre、ZPost、ZTime，显示最近全零事件的类别、次数、前后原始角度和毫秒持续时间。NEAR/FAR 是前帧是否接近绝对零点的线索，GAP 表示前样本过期；不能据此直接把全零当成健康帧。完整说明见 [Encoder_zero_event_diagnostics.md](Encoder_zero_event_diagnostics.md)。

R42 对启用的 ENC2 增加故障停机后每 250 ms 重试，恢复后仍须手动 Run/Follow；未启用的 ENC2 保留五次停止。APB1/APB2 恢复 170 MHz、编码器约 664 kHz，其他通信速率不变，配置和上板步骤见 [Encoder_reconnect_clock_restore.md](Encoder_reconnect_clock_restore.md)。

R41 将 Status 的编码器行标注为 E1/E2，并在 Stop Age/Stop Err 中标注实际停机来源；M1 增加 DMA 超时和主循环恢复，见 [Encoder_axis_dma_recovery.md](Encoder_axis_dma_recovery.md)。E1/E2 跟随物理轴，与主从角色无关。

R40 将 Status → Enc Last 的全零/全一原因显示为 `ALL 0` / `ALL 1`；Enc Frame 仍是累计计数，Enc Age 仍是当前/历史最大周期数，恢复通信不会清除历史错误。详见 [Encoder_constant_frame_diagnostics.md](Encoder_constant_frame_diagnostics.md)。

R39 将两路编码器通信时钟从约 664 kHz 降至 332 kHz，已同步初始化代码和 CubeMX 文件；原控制算法、设置和按键操作保留，见 [Encoder_SPI_slow_clock.md](Encoder_SPI_slow_clock.md)。R38 的移动输入整形说明见 [4015_moving_smoothness.md](4015_moving_smoothness.md)。

R33 修复 HAL_Init 被启动注释吞掉造成的上电错误，新增 g_boot_stage / g_error_return_address；详见 [Startup_error_handler.md](Startup_error_handler.md)。

R32 将手动主从改为追赶轨迹 + 速度前馈 + 独立的位置修正/速度阻尼，参数和上板试验见 [Hand_follow_trajectory.md](Hand_follow_trajectory.md)。按键操作不变，无需修改 CubeMX。

R31 将 2804/4015 的位置 P 从 4 提到 6，位置 I/D 继续为 0；当时普通位置与手动跟随均生效，历史记录见 [Position_response_tuning.md](Position_response_tuning.md)。

R30 的失联提示增加 ENCODER1 / ENCODER2 编号，联动停机保留首次检测来源，Stop Age/Err 使用故障来源那一路的数据；见 [Run_stop_diagnostics.md](Run_stop_diagnostics.md)。

R29 的 Pos Speed 范围、增益和手动跟随误差处理已更新，详见 [Position_response_tuning.md](Position_response_tuning.md)。已有 Pos Speed=30 不自动提升；从轴 Speed Limit=120 时可在 Pos Speed 中设置到 120。

**UI-R53-SAMPLE** 起：主从模式手转主轴、从轴位置跟随，主轴关闭功率输出且不自动对齐。Gear Ratio=100、Offset=0 时同向 1:1 跟随机械角度变化；先摆齐两轴再按 OK 建立共同起点。Run 显示 Hand M1 -> M2（或反向）、RUN/STOP 和位置误差，方向键不输入目标，OK 启停，KEY2 停止返回。保留单 M1 / 单 M2 / 双独立 / 速度同步 / 位置同步和逐路 2804/4015 参数。双电机固定编码器反馈；单 M1 可选独立 HSO。System 配置、长按左右切轴、SPI2 保存延期、Follow 命令及遥测映射见 [Dual_motor_control.md](Dual_motor_control.md)。编码器模式保留原 R26 滑行停止和 Speed=0 闭环减速。

R25 让 Run 异常停止显示并保留具体原因，Status 新增 Run Stop、Stop Age/Err 及 M1 编码器分类诊断，详见 [Run_stop_diagnostics.md](Run_stop_diagnostics.md)。

R24 增加 `Settings → Motor → 2804 / 4015` 切换 M1 的控制与保护参数，确认后清零目标并重新对齐；型号后台保存，Status/About 显示实际型号。运动中/编码器异常/对齐中拒绝切换，旧记录默认 2804。具体参数和操作见 [Motor_profiles.md](Motor_profiles.md)。

## 按键

工程 `.ioc` 中 PD8/9/10/11/12/14 均为上拉输入，默认低有效。上、下、左、右、OK 和 KEY2 各占独立键位；KEY1（PD13）上拉、下降沿 EXTI，任意页面全局停止两轴。长按左右切轴保持原规则。

| 场景 | 上/下 | 左/右 | OK | KEY2 |
| --- | --- | --- | --- | --- |
| 首页 | 无动作 | 图标轮播 | 进入 | 根页无动作 |
| Settings | 选择条目 | 无动作 | 打开条目 | 返回 |
| Status / About | 滚动内容 | 无动作 | 返回 | 返回 |
| 整数编辑 | 增加/减少 | 编辑数值时无动作；退出编辑后移动按钮焦点 | 数值编辑→确认按钮→提交 | 取消草稿 |
| 开关编辑 | 无动作 | 左关、右开；退出编辑后移动按钮焦点 | 编辑→确认→提交 | 取消草稿 |
| 确认框 | 无动作 | 取消/确定 | 执行当前选择 | 取消 |
| Run | 增加/减少待确认目标，支持长按 | 短按切模式；双独立时长按左选 M1、右选 M2 | 改值后确认；未改值时启动/停止 | 停止输出并返回 |
| Run 手动主从 | 无动作，手转主轴提供位置目标 | 无动作 | 启用跟随并记录起点 / 停止 | 停止跟随并返回 |
| Run Steps | 选择模式行 | 减少/增加该模式步长 | 保存三行并返回 | 放弃草稿并返回 |

统一使用 20 ms 去抖；快速连续点按每次各走一档，不等待双击窗口。方向键长按 300 ms 后自动连发，间隔由 160 ms 加速到 100 ms、60 ms；OK 和 KEY2 不连发。Run 模式切换只响应短按事件，不随长按反复切换。开机按住的键必须稳定释放后才能操作。显示异常恢复保留方向键的长按计时，不再将其锁到松手；转场期间不执行按下事件；持续按住的方向键会在转场结束后重新等待 300 ms 再恢复连发，OK 和 KEY2 仍需释放后重新按下。相反方向和斜向同时按下不执行方向动作。

## Run

进入时为 STOP；上下键短按或长按都修改待确认目标。数值不同于上次确认值时，OK 一定提交该数值：停止时启动，运行中更新目标并继续运行。提交直接调用电机控制接口，不等待 Flash 保存。数值未变化时，OK 交替执行启动和停止；改值后又调回已确认值，也按未变化处理。

底部随行为显示 `OK:Start` / `OK:Apply` / `OK:Stop`。数值与单位整体居中，状态行显示 `EDIT`（停止中有修改）或 `RUN*`（仍按上次确认目标运行），有修改时第二行显示之前确认的 `Set` 值；确认后显示 `RUN`。双独立模式在当前轴状态后加 `*`。例如 Speed 已确认 0.5 rad/s 并运行，长按修改为 3.5 后仍按 0.5 运行，按 OK 当即更新为 3.5，再按一次 OK 才停止。页首始终区分 ENC / HSO 并显示有效 VBUS。

模式切换先停止输出，保留各模式本次页面内的草稿，新的模式需要再次按 OK 启动。双独立 Run 的短按模式切换延迟到松手后，长按约 600 ms 只选择电机，不先执行短按动作；单机等其他模式保留按下切模式的时机。KEY2 在退出动画开始时即清零输出，放弃未确认的修改，页面离开回调再次兜底。重新进入时读取已确认值，电流目标仍从零开始。编码器异常、失去标定或检测到其他控制模式接管时停止本次运行，编码器恢复后需要重新按 OK。编码器未就绪时确认失败，保留待确认目标供重试。

| 模式 | 目标范围 | 默认步长 | 单位 |
| --- | --- | --- | --- |
| Torque，关闭 Curr Loop | 2804：±6.93；4015：±13.86 | 0.05 | V，q 轴电压，实际还受母线限制 |
| Torque，打开 Curr Loop | 2804：±0.50；4015：±1.20 | 0.05 | A，Iq |
| Speed | -Speed Limit～+Speed Limit，默认 ±100.0 | 0.5 | rad/s |
| Position | -3.14～+3.14 | 0.05 | rad，相对本次启动位置 |

`Settings → Speed Limit` 调节 Speed 模式的目标范围上限：2804、4015 均可设 1～600 rad/s，默认 100 rad/s，每次上下键调整 1 rad/s，支持长按连发。设为 100 时，Run 的 Speed 目标范围为 -100.0～+100.0 rad/s；设为 20 时则为 -20.0～+20.0 rad/s。Run 无草稿修改时 `Max` 显示此值。软件范围不代表电机实际可达到的速度；HSO 的电气频率校验独立保留。

编辑时按 OK 进入确认按钮，再按 OK 提交；KEY2 放弃本次修改。上限提交后立即应用，已有正向或反向目标超出新范围时会收回到对应边界，例如 -80.0 在上限改为 20 后变为 -20.0。返回 Run 仍需 OK 才会启动，不会因改范围自动运行。恢复默认设置时，上限恢复为 100、目标恢复为零。Speed Limit 约束 Run 的目标；此外控制接口统一遵守所选型号的最高机械速度，串口也不能超过该硬上限。Pos Speed、Torque 限幅和 Run Steps 分别设置。

`Settings → Run Steps` 可分别设置三种模式的步长，页面直接显示实际数值和单位：

| 模式 | 可调步长范围 | 每次左右键调整量 |
| --- | --- | --- |
| Torque | 0.01～1.00 V/A | 0.01 V/A |
| Speed | 0.1～10.0 rad/s | 0.1 rad/s |
| Position | 0.01～1.00 rad | 0.01 rad |

上下选择模式行，左右调步长，四个方向均支持长按连发；刚进入页面、动画还没结束就按住方向键也会在转场结束后恢复连发。OK 保存，KEY2 放弃草稿。Run 页右下方 `Step` 显示当前步长；短按和长按连发均使用该步长，目标仍按原有范围限幅。Torque 的单位由 Curr Loop 设置决定。

`Settings → Pos Speed` 调节位置模式速度上限（1～当前轴 Speed Limit，默认 30 rad/s）。降低 Speed Limit 时同步收回超出的 Pos Speed；M1/M2 分别保存，600/120 等大于 63 的数值也能正确重载。位置模式每次启动捕获原点，运行中连续修改目标不会重新按当前位置选圈。再次停止、启动会重新捕获原点。普通位置 P=6，4015 电压模式速度 PI=0.1/1.0；手动跟随按从轴型号及 Curr Loop 选择预设，4015 On 的参考加速为 55～180、减速 100～500 rad/s²，位置修正 P=3，速度 PI 按误差连续调整比例项，见 [4015_fast_follow.md](4015_fast_follow.md)。

电压与电流使用独立草稿，切换电流环不会把 V 的数值解释为 A。速度外环限幅随电压/电流模式分别使用电机电压限幅/最大电流限幅，切换时清空相关 PI 历史。速度显示统一采用控制器实际使用的 rad/s。

已确认的目标速度、电压、相对位置、位置速度上限、Speed Limit、三种步长与设置在停止且至少 2 秒无修改后后台保存，成功后提示 Saved；未确认的 Run 草稿不写入设置，不会恢复自动运行。电流目标不持久化，每次进入 Run 从零开始。位置速度上限继续使用 flags 的既有编码；Speed Limit 使用 flags 的空闲位 14～23，保持 version 2 记录布局不变，旧记录没有此值或值无效时采用默认 100 rad/s。新增步长字段采用 version 2 设置记录；读取 version 1 时先按旧格式校验 CRC，再在 RAM 中补上默认步长，保留原有目标、LED、位置速度上限和序号，下次保存才写 version 2。

## WS2812 状态灯

已移除按键时亮白灯的覆盖逻辑和对应按键自检接口。灯色只由电机/编码器状态决定：编码器异常为红色、未标定为黄色、已标定待机为绿色、有速度时为蓝色；LED 开关和亮度设置继续生效。

## 动画与存储

首页使用连续环形坐标绘制，先计算移动位置再环绕，修复了旧代码在 About↔Run 边界和连续点按时把可见图标重新放到另一侧的跳变；动画中反向和多次点按会从当前可见位置继续。首页轮播为 300 ms。

Run、Settings、Status、About 使用同一套 320 ms 页面进入/返回动画；信息页滚动为 220 ms。仍由 KK_UI 独占完整场景绘制和 DMA 刷新。DMA 忙时继续扫描和处理输入，合并重画请求，空闲后绘制最新状态，避免浪费主循环时间反复绘制发不出去的画面。

ST7735 DMA 完成/出错 IRQ 只记录结果，主循环在每轮开头调用 `OLED_DriverService()` 向图形核心交付一次完成通知。这样即使 DMA 中断抢在 `OLED_UpdateDMA()` 返回前执行，核心也不会丢失完成事件并停留在 Busy。图形核心的双缓冲与提交状态机保持原样。

运行时 W25Q64 保存和日志使用 Begin/Poll 分步状态机，每次主循环只推进一步，不在芯片擦写期间忙等。设置保存使用固定快照、A/B 备份和读回校验；只在校验成功后提示 Saved。日志采用 4 条固定队列，队列满时放弃新日志，不阻塞 UI。启动读取、扫描以及维护用同步接口仍保留。

Debug 保留调试符号；FOC、采样和主循环按既有 -Os／LTO 策略，HSO 算术 -O2。R57 的紧凑数值解析器保留既有数字格式，最终结果仍为控制器使用的 float，详见 R57 的 Flash 与验证说明。

## 固件识别与连发诊断

UI-R23-LCD 按用户提供的 BOE 屏幕示例将可见区偏移从 26/1 修正为 24/0，修复 270° 显示时顶部两行、右侧一列漏刷；开屏前完整清除 ST7735S GRAM，并将 SPI3 降至 10.625 MHz。根因、厂家地址与命令流测试见 [LCD_edge_stripes_fix.md](LCD_edge_stripes_fix.md)。

当前版本增加了 VREFINT 平均与低通滤波，使用测得的 VREF+ 统一换算电流、母线和相电压。Status 新增 VREF（有效估算的电压）、RefState、Ref R/C、RefCnt、RefErr；显示精度不代表测量精度。首个有效平均值完成前不使能电机，参考估计超时锁存 ADC REF。CubeMX 工程和 ADC 初始化均已同步修改，配置、滤波参数与上板核对见 [VREFINT_reference_tracking.md](VREFINT_reference_tracking.md)。

本轮固件在 `About` 首行 `FW` 显示 **UI-R79-ID**。ENC2 停止/重连见 [Encoder_reconnect_clock_restore.md](Encoder_reconnect_clock_restore.md)，USB 修复见 [USB_CDC_tx_recovery.md](USB_CDC_tx_recovery.md)。HSO 的启动、最低速度、停止和失锁恢复采用 [R44 独立无感规则](HSO_sensorless_startup.md)，不再按编码器实测 45rpm 自动回退。H Gate 为当前校验原因，H Qual 为学习/接管进度和保持异常计数，H Lost 保留上次异常，H Actual 为观测磁链。手动加载过载预算见 [2804_current_limits.md](2804_current_limits.md)。Status → ADC 的 I Trip、IU/IV/IW R/Z 和 ChipRev 见 [current_adc_diagnostics.md](current_adc_diagnostics.md)。

`Status → Bus` 为 ADC 实测母线电压。`Drive` 显示采样/电流零点的具体状态或锁存故障，编码器在线与对齐另看 Encoder / Aligned。BusRaw 是最新母线原始值，BusCnt / FrameCnt 是规则组样本数 / 完整注入帧数，正常约每秒增加 2000 / 10000；ADC1Err / ADC2Err 显示 HAL 错误码。Enc2 显示第二路 MT6701 的健康状态和原始角度，Enc2Err 为其错误累计值。第二路是否参与控制由 Work Mode 决定；Single M1 六步／HSO 不依赖编码器2，双有感的控制与故障恢复规则保持。

Run 在采样未就绪时显示具体原因并拒绝确认启动；运行中采样异常则停止。上电没有电机母线电源时仍连续测量，Bus 显示 `0.0 V`，Drive / Run 显示 `WAIT BUS`。稍后接通母线会更新读数，并在电流零点完成、母线稳定后继续初始化，无需因最初 0 V 而复位。没有新样本时 Bus 显示 `NO DATA`。运行后的真实故障显示 `ADC PAIR`、`FRAME TIMEOUT`、`BUS TIMEOUT`、`ADC ERROR`、`BUS LOST` 等首因；功率输出关闭，但母线检测继续。锁存故障不会随母线恢复自动解除，处理后重新启动开发板。详见 [ADC / PWM 应用适配记录](ADC_dual_motor_review.md)。

`Status` 中的 `KeyRpt` 是 Run/Run Steps 收到的连发事件数，持续按住加减键时应增加；`DispErr` 是显示刷新错误/超时进入恢复的次数。两者上电从零计数；Status 数字每 500 ms 更新，离开 Run/Run Steps 后仍保留，方便复查。

本轮实际复现了两条旧代码故障路径：刷新失败恢复后，按住 1 s 的目标仍只有 0.05（正常应为 0.30）；DMA 完成 IRQ 提前发生时，核心会保持 Busy。修复后，相应错误注入测试均通过。尚未取得板上计数或时序观测，因此不把这两条源码缺陷直接等同于已经证实的实物故障原因。

## 已执行验证

运行 `python tests/ui_host/run.py`，需要本机 GCC。测试链接实际 KK_UI、KK_OLED 图形层、应用和字库，使用模拟 GPIO、电机、DMA 与 NOR Flash：

- 六键扫描、独立轴、抖动、连发、开机按住、转场按住、对向键、计时回绕。
- Run 三模式停止中/运行中的上下长按；改值确认、未改值交替启停、改回原值后取消、未确认值退出不保存、确认失败后重试；反向速度、电流限幅、编码器异常、固定位置原点、返回前清零输出。
- 编辑取消与提交、位置速度上限存储、三模式步长的取消/提交/应用/持久化。
- Speed Limit 的取消/确认、方向键长按、1/600 边界、超过原 60 rad/s 的正反向目标、降低上限后收回已有目标、重新加载、恢复默认和旧记录/非法值回退；真实模拟 NOR 保存时的范围快照及读回，位置速度上限字段不受影响。
- 首页四个入口的双向边界，以及动画中每 45 ms 改变目标的连续性；新目标建立的同一时刻画面应逐字节一致。修改前该测试出现 45 字节突变，修改后无突变。
- 连续十次快速点按逐次计数，以及长按 1 s 的连发次数。
- 显示错误注入后，Run/Run Steps 的加减持续连发；测试按真实 GPIO 扫描接口生成输入，并走与 main.c 相同的显示恢复流程。
- 单独链接真实 ST7735 驱动，覆盖启动函数返回前中断完成/出错、正常 DMA、启动失败、终止与阻塞刷新。
- Run Steps 的左右持续增减、上下持续换行；分别覆盖页面稳定后开始长按、进入动画期间开始长按，以及 OK/KEY2 持续按住只执行一次。
- Run、Status、About 多个转场中间帧，以及 30/60 ms 异步 DMA 下传输帧保持不变。
- 异步保存、保存中继续修改、A/B 恢复、编程失败、读回失败、日志跨区轮换、version 1 设置迁移。

固件已在标准目录 `build/Debug`、`build/Release` 重新构建；CMake 每次链接会自动生成同目录的 BIN。独立目录 `build/ui-validation` 保留主机测试与画面预览。

运行 `python tests/ui_host/render_preview.py` 可从测试导出的真实 framebuffer 生成动画（可选，需要 Pillow）：

- [About → Run → Settings](../build/ui-validation/home-forward.gif)
- [Run → About → Status](../build/ui-validation/home-reverse.gif)
- [快速连续点按与反向](../build/ui-validation/home-rapid.gif)
- [步长设置页](../build/ui-validation/run-steps.png)
- [Speed Limit 设置](../build/ui-validation/speed-limit.png)
- [Run 速度范围显示](../build/ui-validation/run-speed-limit.png)

尚未烧录或连接电机验证。实屏帧率、物理按键电平、实际负载下三模式运行和电流环参数需要上板确认；主机模拟不代表这些项目已经验收。

最终构建占用（2026-10-25，UI-R79-ID）：Debug、Release 均为 Flash 131064 B / RAM 23112 B，Flash 余量 8 B。Debug 保留调试符号；两套 ELF/BIN 一致性检查通过，新增 Ls／磁链辨识尚未实板验证。固件仍为 `build/Debug/FOC_two.elf`、`build/Release/FOC_two.elf` 和同目录的 BIN、MAP。参数辨识操作、定义和验证见 [R79](Motor_detect_R79.md)；通用六步操作、电流边界及 Flash 优化见 [R70 ESC Auto](Six_step_ESC_R70.md)；KEY1 见 [全局停止](KEY1_global_stop.md)，硬件保护见 [M1 IBUS 硬件保护](M1_IBUS_hardware_protection.md)，独立 HSO 见 [HSO_sensorless_startup.md](HSO_sensorless_startup.md)，两套电机参数见 [Motor_profiles.md](Motor_profiles.md)。
