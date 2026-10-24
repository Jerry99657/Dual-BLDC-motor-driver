# UI-R58-IBUS：M1 母线硬件过流保护

已接入用户生成的 COMP4／DAC3／TIM1 BRK 配置，无需再改 CubeMX。目标硬件为 STM32G431VBT6；M1 的 5 mΩ 母线分流电阻经 INA240A2（增益 50）送 PE7／COMP4_INP，REF1=AGND、REF2=与 VREF+ 同源的模拟 3.3 V，COMP4_INM 接内部 DAC3_OUT2。

这是正向母线瞬时过流关断。原有相电流峰值、ADC 有效性、持续过载预算、母线与参考电压、控制超时保护继续工作。仅一条上限比较通路不提供负向回馈电流硬件保护；它也不能代替相电流保护。

## 操作

Settings → **M1 Protection** 固定对应物理 M1，与当前 View M1／M2 无关。

| 条目 | 行为 |
| --- | --- |
| IBUS Trip | 默认 **3000 mA = 3.0 A**，范围 500～4000 mA，步长 100 mA；M1 输出请求关闭时才可修改 |
| Clear IBUS Trip | 仅有 IBUS 锁存时可选，必须通过确认框；取消不改变故障 |
| IBUS: READY／TRIPPED／NOT READY | 只读状态 |

编辑器用 mA 显示，Status 用 A 显示。OK 确认后立即应用，Back 取消。即使串口恰好在编辑器打开期间启动 M1，底层也会拒绝门限修改并恢复原显示。当前 0.5～4.0 A 是这次提供的板级调节范围，不随 2804／4015 型号切换；提高 IBUS 门限不会提高现有相电流和 dq 目标限幅。

门限沿用 W25Q64 延迟保存规则。退出 Run、等待 Save: Saved 再断电；M2 占用 SPI2 时仍等待总线释放。切换模式、反馈或两轴型号保留门限，恢复默认设置将门限恢复为 3.0 A。使用既有 v3 记录 control_config 的空闲 bits[13:8]，按 100 mA 编码，不改变记录长度和 CRC 布局；旧 v1／v2／v3 以及该字段为零／越界的记录默认 3.0 A。

## 故障与恢复

正向电流超过阈值时，COMP4 直接触发 TIM1 BRK，硬件清除 MOE。软件再断开 M1 的六个相输出、撤销启动请求和预装载计数，并锁存 **M1 IBUS TRIP**。相输出关闭后保留计数器、CH4／TRGO2 和 ADC 采样流程；不会通过停止 ADC 或停计数器实现关断。

持续故障时关闭的是 Break 中断使能，硬件 Break 本身始终使能，避免高电平故障导致中断风暴。软件在功率输出已断开、故障已记录后确认 BIF，以检测后续短脉冲；确认标志不会恢复输出。

恢复步骤：

1. 排除过流原因，保持 M1 输出关闭。
2. COMP4 返回低电平，期间没有新的 Break 事件，连续保持至少 100 ms；ADC 帧、母线和 VREF+ 必须有效。
3. 在 M1 Protection → Clear IBUS Trip 确认清除。若有新的脉冲、持续高电平、初始化异常或其他 ADC／电流锁存故障，拒绝清除。
4. 再手动启动。清除动作不恢复原速度／电流目标，也不使能 PWM。若上电对齐被中断，先手动 System → Align / Retry，再启动。

故障前若为独立 HSO，成功清除时会同时确认其伴随的停止锁存，保持输出关闭；下一次仍走完整独立无感启动。Drive 优先保留采样／IBUS 根因，不用 HSO STOPPED 遮盖硬件故障。

双独立模式中 M2 可继续运行；同步、主从模式中，由控制协调层关闭两路输出、取消排队／正在执行的对齐，并保留 M1 IBUS TRIP 根因。**M2 的组停机是软件协调，COMP4 的直接硬件关断对象是 TIM1／M1。** 清除后也不会自动恢复组运行。

初始化或运行中的 DAC 更新失败显示 **M1 IBUS INIT**，禁止 M1 输出；它不通过普通 IBUS 清除菜单恢复，应检查配置／硬件后复位。其他原有 ADC、相电流和过载故障保留各自的恢复规则，IBUS 清除不绕过它们。

## 换算与启动顺序

按已确认的接法，INA240 输出为：

`Vout = VREF+/2 + 50 × 0.005 × Ibus = VREF+/2 + 0.25 × Ibus`

DAC 码值按 STM32G4 的 4096 分母计算，使用工程已有的 VREFINT 平均／低通估计：

`DOR = round(4096 × (0.5 + 0.00025 × Itrip_mA / VREF+))`

例如 VREF+=3.26 V、门限 3 A 时，阈值约 2.38 V、码值约 2990。首次有效参考平均完成前禁止 M1 功率输出；后续参考更新在主循环同步刷新 DAC。保留 0.25 V 的输出上端余量检查，不把阈值顶到模拟电源。参考说明见 [RM0440 §22.4.6](https://www.st.com/resource/en/reference_manual/dm00355726.pdf)。

启动时保持全部相输出关闭，先写 DAC、启动 DAC 并等待 1 ms，再启动 COMP4。HAL 等待比较器启动稳定后，检查真实输出状态、确认启动期间的未资格化标志，最后使能 Break 中断。启动后比较器仍为高电平会立即锁存，不能因起始状态被忽略。

DAC/INA 零点换算依赖用户确认的同源参考电压与采样方向。电阻、增益、参考和比较器失调／迟滞均影响实际动作电流，以上为名义换算，不是校准后的精度保证。

## 诊断与代码

Status → Drive 显示故障首因；Status → ADC 新增固定 M1 的 IBUS 状态、IBUS Limit、IBUS DAC、IBUS Thresh、IBUS Trips、COMP4。IBUS Trips 是本次上电的锁存次数，未自动写 Flash；IBUS Thresh 是根据 DAC 码值和参考电压计算的阈值。**工程没有新增 IBUS ADC 通道，这些项目不代表实测母线电流。**

主要代码位于 motor_sampling.c／.h（DAC／COMP 启动、锁存、门限、清除和输出门禁），motor_control.c（组停机与启动／对齐资格），ui_app.c（菜单、确认及保存），app_storage.h（兼容编码）。main.c 载入门限，并在 IBUS 锁存后取消上电自动对齐；JerryFOC_stop 同时取消排队中的对齐。

所有 MOE 重新使能路径均保留 Break 检查。使能事务先断开相通道、写 MOE，再检查一次 BIF，最后才接通相通道。若脉冲在检查与 MOE 写入之间到达，不会短暂接通相输出；若在最终检查后到达，硬件清除 MOE，事务后续不再写 MOE。手动清除不会清 BIF，防止抹掉刚到达的新事件。内部连接和关断机制参考 [ST AN4277](https://www.st.com/content/st_com/en/technical-documents/an4277-how-to-use-pwm-shutdown-for-motor-control-and-digital-power-conversion-on-stm32-mcus-stmicroelectronics_asset/chunk-003-topic-an4277-examples.html)。

## 验证与固件

主机测试通过：门限范围／停机限制、VREF 更新、DAC／COMP 启动顺序、初始化及运行时 HAL 失败、启动瞬态与持续高电平、短脉冲、ADC 先于 Break IRQ、持续故障防中断风暴、M2 独立运行、同步／主从组停机及对齐、静止资格与毫秒回绕、手动清除前的新脉冲、MOE 两个竞态窗口、Stop／Start 不绕过锁存、HSO 根因显示、菜单确认／取消／竞态拒绝、NOR 保存与旧记录兼容。既有控制、采样、HSO、编码器、SPI2、USB、UI 和 LCD 回归通过。

最终 ELF 检查确认 HAL／COMP／DAC MSP、TIM1 Break 中断为实际实现，Break IRQ 通向锁存处理，BIN 与 ELF 匹配。原有 RWX 段链接提示保留。

| 2026-10-24 | Flash / 131072 B | 空余 Flash | RAM / 32768 B |
| --- | --- | --- | --- |
| Debug | 126116 B | 4956 B | 22824 B |
| Release | 125340 B | 5732 B | 22720 B |

About → FW：**UI-R58-IBUS**。Debug／Release 的 ELF、BIN、MAP 已更新。

| 固件 | BIN SHA256 |
| --- | --- |
| Debug | `4d567efab4e2878bbbca1e0f6e1a23e2b3f0485793c0d64e796d8f0d0e200d18` |
| Release | `786d3d22003a5853342bca33b45ebf4bb7bda10aca4cf6e694a2e4e8f2b2f9a7` |

预览：build/ui-validation/r58-preview.png，由 tests/ui_host/render_preview.py 从真实 UI 帧生成。

尚未烧录或实板验证。首次上板需核对零电流 COMP4 为低、正向电流使 INA 输出升高，使用限流供电与可控电流确认实际动作门限、六路 PWM 的关断波形和延迟，以及 Break 后 BusCnt／FrameCnt 继续更新；主机外设模型不能替代这些测量。
