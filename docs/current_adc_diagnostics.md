# UI-R13-SYNC 电流采样故障处理

用户在 R12 上反馈 `Drive=M1 ADC U`、`I Trip=RUN U 0`，实时 IU/IV/IW 为
2045/2044/2042，零点均为 2047。说明零点已完成，随后 U 相采到过 0；停机后恢复正常。
该现象不能单靠停机后的正常读数排除真实电流瞬态，也不能证明只有一帧异常。

## R13 修复：消除规则组与注入组的异步冲突

原先 TIM6 独立定时触发 ADC1 VBUS 规则转换，可能与 TIM1 注入触发重叠。
ST [ES0431 Rev 9 §2.5.10](https://www.st.com/resource/en/errata_sheet/es0431-stm32g431xx441xx-device-errata-stmicroelectronics.pdf)
记录了 Rev Y 的规则/注入冲突可能使首个注入结果错误返回 0；本项目首项正好是 M1 IU。
这与反馈高度吻合，但芯片版本和实板消除效果仍需核验，不认定为已证实的唯一根因。

- TIM6 保持停止，不再启动 500 us 周期计数。
- 每 5 次完整注入帧后，应用通过 TIM6 EGR.UG 产生 UPDATE/TRGO，触发一次 VBUS 转换。
  CubeMX 的 ADC1 TIM6 TRGO 配置及 DMA 循环缓冲保持有效，不需要重新生成。
- 在注入回调早期、读取完两路 ADC 的注入数据后调度；VBUS 转换约 1.42 us。
- 当前 PWM1 CH4=1 的上升触发在计数底部附近，只允许在 TIM1 向上计数的前半段触发规则组，
  为下一次注入保留约 75 us 以上。过晚回调跳过该次 VBUS 触发；连续超时仍按原逻辑停机。
- 启动零点校准前、0V 等待期间、停机故障后均继续按上述方式测量 VBUS。
- 原 ADC 越界、零点检查、2A 过流和过载门限保持不变，不滤掉读到 0 的电流帧。

Status 新增 `ChipRev`：0x2002=Rev Y（本条勘误受影响），0x2001=Rev Z、0x2003=Rev X。
即使是其他版本，同步规则转换也能避免无必要的注入抢占。
如再次出现异常，保留 Drive/I Trip/R/Z 和 ChipRev；还需测量 INA240 OUT、采样引脚和启动波形。

新增主机测试用 JEOS→TIM6 UPDATE→DMA 模型验证约 2kHz 连续采样、先 MCU 后母线上电、
迟到回调跳过、故障后继续更新；模型不模拟芯片内部勘误或 INA240 的物理瞬态。
Debug/Release 及电机、UI 回归通过。Debug Flash 110428 B、RAM 21000 B；
Release Flash 109172 B、RAM 20720 B。链接器原有 RWX LOAD 警告仍存在，未做实板验证。

## R12 原有诊断

用户确认 INA240 REF1 接 AGND、REF2 接 REF3.3V；Status 在点击 Run 前已出现 `M1 CURRENT ADC`。
这不能单独证明故障发生于零点校准：程序上电后还会自动执行编码器对齐并施加电压。
当前 CubeMX 注入组与应用读取一致：U=PA0/ADC1 JDR1，V=PA1/ADC2 JDR1，W=PC0/ADC1 JDR2。

R12 补充首次故障快照和分阶段提示；这些诊断保留在 R13 中。

| Status 项目 | 含义 |
| --- | --- |
| Drive: M1 ZERO U/V/W | 2000 次零点样本均值 <=10 或 >=4085，未允许三相输出 |
| Drive: M1 ADC U/V/W | 允许输出后，该相原始采样 <=10 或 >=4085；也包括上电自动对齐期间 |
| I Trip | 首次电流故障的 ZERO/RUN 阶段、相位和当时原始 ADC 码；ZERO 阈值判定用均值，快照记录最后一帧 |
| IU R/Z、IV R/Z、IW R/Z | 对应相的实时原始 ADC 码 / 校准零点（四舍五入） |

采样保持连续，首次故障快照不会被停机后的正常读数覆盖。故障仍锁存，需排除原因后复位。
若出现过流，I Trip 同样保留触发相位；矢量过载没有单独相位，I Trip 显示 `--`。
新一次采样启动清空旧零点和旧快照，避免显示前次校准残留。

按上述 REF 接法，零电流输出理论约 1.65V。ADC 参考 3.3V 时理论零点约 2048，实际仍采用测得零点。
依据：[TI INA240 数据手册，Output Set to Mid-External Reference](https://www.ti.com/lit/ds/symlink/ina240.pdf)。
如新固件继续报错，应记录 Drive、I Trip 和三行 R/Z，区分持续输入异常与输出开启后的瞬态越界。

验证：采样主机测试覆盖三相两端越界、首次快照保留、零点失败和正常后续采样；UI 测试覆盖新增数值显示。
Debug/Release 和电机/UI 回归测试通过。Debug Flash 110284 B、RAM 20984 B；Release Flash 108988 B、RAM 20704 B。
未进行实板验证。链接器原有 RWX LOAD segment 警告仍存在。
