# Run 停机原因与 MT6701 错误分类（UI-R25-STOP）

当前 **UI-R30-ENCID** 已将编码器失联提示明确为 `ENCODER1 LOST`（M1 / SPI1）和 `ENCODER2 LOST`（M2 / SPI2）。Run 提示、停止原因、启动拒绝和控制层状态使用相同编号。双同步/手动主从联动停止保留控制环首先处理的故障来源，随后另一轴失联或切换查看对象不会改写它；成功重新启动后清除旧控制层停机原因。这个编号表示软件检测来源，不证明电气故障的物理先后。

`Status → Run Stop` 保留编号；`Stop Age / Stop Err` 在 UI 观察停机时读取该故障编码器的诊断，而不是当前查看轴。例如 Run 在 M2 视图、ENCODER1 失联时，仍记录 ENCODER1 LOST 及 M1 的年龄/累计错误。通信恢复后这组 Run 停机快照仍保留。底层 ADC/过流/过载等未就绪原因继续优先显示。

R30 主机回归覆盖两种编码器、同步及两种手动主从方向、先后失联、恢复保留、重新启动清除和 M2 视图中故障来源与计数匹配；屏幕宽度与显示快照已检查，Debug/Release 固件已更新，尚未进行本次实机验证。以下为 R25 调试过程的历史记录。

用户实测：4015、Speed 10 rad/s、母线 20 V，运行片刻提示 `Motor stopped`；停机后 Status 为 Drive=READY、Encoder=OK、Aligned=YES、Errors=443。

这些数据说明读取 Status 时没有锁存的采样/过流/过载故障，编码器也已就绪；不能证明运行中一直健康。Errors 是 M1 编码器自 MCU 复位以来的累计错误数，既不是当前故障码，也不是每秒错误率。短暂编码器数据失效后恢复是优先排查方向，**尚未凭这些数据确定 CRC、磁场、SPI 或其他具体根因**。

## 修复的可观测性问题

旧 `ui_app_update()` 把驱动未就绪、编码器失效、零点未对齐、模式变化、诊断开环接管都转换为同一个 `Motor stopped`。恢复后 Status 只显示当前 READY/OK，失去当时停机的依据。

R25 在调用 `stop_run()` 改写模式/目标之前，短临界区读取停机条件及编码器快照，立即刷新状态并显示具体提示：

| Run Stop | 含义 |
| --- | --- |
| M1 OVERLOAD / M1 OVERCURRENT / ADC REF 等 | 实际底层保护或采样状态，优先于编码器错误 |
| HSO STOPPED | HSO 阻止驱动就绪 |
| ENCODER LOST | Run 观察到 M1 编码器健康检查未通过 |
| NOT ALIGNED | 编码器零点尚未标定 |
| MODE CHANGED | 控制模式与当前 Run 页不一致 |
| OPEN LOOP | 诊断开环接管 |

Run 页的电流/步长行在异常停止后显示保留的原因，Status 同时新增：

- `Run Stop`：上次 Run 异常停止的原因；编码器恢复、返回首页不会清除。
- `Stop Age`：当时的编码器样本年龄，单位为 10 kHz 采样周期（每周期约 100 μs）。
- `Stop Err`：当时的 M1 累计错误数，后续新增错误不会覆盖此快照。

重新成功确认 Run、切换电机型号、恢复默认或 MCU 复位才清除上述 Run 停机快照。失败的启动尝试不会清除。普通主动停止不制造故障记录。

## 编码器分类

M1 的原 CRC/状态校验和样本健康门限保持不变，只将总计数拆分；M2 的有效帧条件与连续 5 次失败停止探测也保持不变。

| Status 项 | 含义 |
| --- | --- |
| Errors | 原有总错误数 |
| Enc CRC | 非全零/全一帧，但 CRC6 不正确 |
| Enc Field | CRC 正确，Mg[1:0] 非零（过强/过弱/保留状态） |
| Enc Track | CRC 正确，Mg[3] 表示失跟踪；与 Field 同时出现时优先计为 Track |
| Enc SPI | DMA 请求失败或 HAL SPI 错误回调 |
| Enc Frame | 全零或全一帧 |
| Enc Age | 当前样本年龄 / 本次上电最大年龄；最大值从首个有效帧后开始统计 |
| Enc Last | 上次错误类别；后来收到有效帧仍保留，不能单独据此断言本次 Run 的根因 |

每次错误只增加一个分类，分类总和等于 Errors。CRC 错误帧中的状态位不可信，不会把它误记为磁场异常。Mg[2] 的按压标志继续允许通过。

状态含义与 CRC 范围见 [MagnTek MT6701 Rev1.8，7.8.2](https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf)。时钟/帧位对齐也已核对：现有路径为上升沿移出、下降沿采样的 24 位读法，本轮没有盲目改变 SPI 模式；采用空闲高的 Mode2 时，需留意额外首位和完整帧长度，参见 [SimpleFOC MT6701SSI 实现说明](https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/mt6701/MagneticSensorMT6701SSI.h)。具体线缆波形仍需示波器确认。

## 板上复测

1. 烧录后 About→FW 应为 `UI-R25-STOP`，Status→Motor 应为 4015。
2. 继续使用已验证相序、Speed 10 rad/s。若再次停机，先保留现场，不复位、不重新点运行。
3. 读取 `Run Stop / Stop Age / Stop Err` 及 `Enc CRC / Field / Track / SPI / Frame / Age / Last`，并记录运行前后的变化。
4. 若原因是 ENCODER LOST，按实际增长的分类检查 SSI 信号完整性/接地/供电，或磁铁强弱、位置与偏心；不能直接提高过流门限或关闭 CRC 作为修复。

本轮没有放宽编码器超时、电流保护，也没有加入失联后自动重新运行。仍由原快速环即时撤下不可靠反馈下的输出，并由 Run 保持停止，直到用户明确重新确认。

## 验证

- 真实 MT6701 驱动主机测试：CRC 正确的强/弱磁和失跟踪帧、CRC 错误优先、全零/全一、按压位允许、SPI 请求失败、诊断快照临界区、原 M1 DMA/M2 共享 Flash 回归。
- UI 主机测试复现“短暂失效→Run 停止→READY/OK 恢复”：Run Stop、Stop Age=4、Stop Err=443 保留，实际目标已清零且不会自动恢复；再次明确运行后清除。另覆盖底层故障优先、对齐失败、模式变化、开环接管。
- 电机/UI/LCD 全部主机测试通过；Debug Flash 117804 B / RAM 21728 B，Release Flash 116276 B / RAM 21416 B，ELF/BIN/MAP 已更新，仍有原 RWX LOAD 段链接警告。
- 尚未进行本次实板复测，不能把主机复现当成对这块板具体错误来源的证明。
