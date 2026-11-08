# R124：双机运行时的外环超时与实时路径精简

固件 `UI-R124-RT`，Native 配置回执编号 124。无需调整 CubeMX。

## 现场数据与结论

双机独立模式下，先运行 M1 `Speed:30 rad/s`，再启动 M2 `Speed:100 rad/s`；不到一秒出现 `Drive OUTER STALE`。

| 现场诊断 | 数值 |
| --- | ---: |
| ADC IRQ Max | 89.8 µs |
| Outer Max | 2179.0 µs |
| Outer Jitter | 1180.7 µs |
| Outer E/G/S | 2 / 0 / 1 |

现有 ADC 周期为 100 µs，外环周期为 1 ms。现场数据已确认两次外环执行超期、一次完成心跳超时；2179 µs 已超过 2 ms 心跳保护期限。`Outer Max` 包含被较高优先级 ADC／SPI 中断抢占的时间，不能直接当作 PID 计算本身耗时。

ADC 单次峰值接近整个周期，加上两路编码器 DMA/SPI 中断，峰值情况下留给 TIM2 外环的时间不足。这是需要处理的实时预算问题；当前最大值不能用于计算整段运行的平均 CPU 占用率。未把保护门限放宽，也没有将 `OUTER STALE` 当作编码器通信故障处理。

## 已保留的修改

### SPI DMA 重复停用

原路径在 `start_dma → quiesce → setup_dma` 重复停用通道，又在 `DMAIRQ → quiesce → Service` 重复停用一次；每次停用还分四次修改 CCR 的 EN／TCIE／HTIE／TEIE。

现在：

- 使用生成句柄的 `Instance->CCR`，一次清除 EN 和中断使能，保留全部 DMA 格式位。IFCR 仍依据 `DmaBaseAddress`、`ChannelIndex` 清除自己的标志。
- `start_dma` 已在 PRIMASK 保护下停用两个通道，`setup_dma` 直接设置本次地址／计数及使能。
- DMA 完成中断已经停用两个通道并清除 SPI DMA 请求；`Service` 确认 TXE、非 BSY、FIFO 空后只发布一次完成回调。
- TE 优先于 TC、迟到中断、错误快照、两个控制器之间的通道重映射、ADC／WS2812 标志隔离及 SPI2 共享规则保持。

实际驱动／ST LL 函数体配合寄存器模型，测得完整一次传输的寄存器写次数：

| 路径 | DMA CCR 修改前 → 后 | IFCR 修改前 → 后 |
| --- | ---: | ---: |
| SPI1 编码器全双工 | 37 → **9** | 8 → **4** |
| SPI2 编码器全双工 | 37 → **9** | 8 → **4** |
| SPI3 LCD 发送 | 19 → **5** | 4 → **2** |

这是寄存器操作次数，不是实板微秒值。两路编码器的读取频率、SPI 分频／相位及 DMA 配置均保持原值。

### FOC 重复计算

- M1 有有效采样时刻时直接选择时间补偿角度，省去随后会被覆盖的普通电角度计算。
- M2 选定普通／补偿角度后只进行一次角度归一化。
- 逆变器损耗的固定 `[-1,1]` 限幅使用局部比较，避免每相绕行通用 Min/Max 调用；非有限结果仍回到中性 PWM。
- 电流前馈的 50 ms 使能增益已经达到 1 时直接使用 1；升幅过程、真实速度前馈及异常值路径保留。
- d 轴请求为零时直接使用额定电流绝对值作为 q 轴电流上限；当前预设／自定义型号继承的额定电流为有限正数。HSO 非零 d 轴请求仍按原电流矢量余量计算。

电流环增益、PI 矢量限幅／回算、逆变器误差补偿及时间补偿公式没有重新整定；HSO 继续采用已验证的 PI 电压路径。

曾试验对调制器和采样入口使用 O2，但采样 O2 试验将 Flash 余量压到 32 B，最终已撤回这些属性修改。交付版本沿用原编译策略，不修改 ST 官方源码、生成初始化、IRQ 优先级或控制频率。

## 保护与资源

`board_diag.c` 的外环监测代码与 R123 完全相同：1 ms 执行超期计数、连续异常处理、2 ms 完成心跳停机、双轴关闭和需复位解除的时序故障锁存均保留。

| 构建 | R123 Flash | R124 Flash | 余量 | R124 RAM 分配／余量 |
| --- | ---: | ---: | ---: | ---: |
| Debug | 130288 B | **130248 B** | **824 B** | 30904 B / 1864 B |
| Release | 130288 B | **130248 B** | **824 B** | 30904 B / 1864 B |

计入版本资源后，Flash 净减少 40 B，RAM 分配净减少 16 B。ELF／BIN／MAP 已更新到标准构建目录。

## 验证

- SPI 原版／新版均通过相同的正常、TE／TC 并发、BSY 延迟、回调内重启、中断抢占及 600 组生成句柄重映射回归；另对比 CCR／IFCR 写入次数。
- 每个实际 ARM 镜像的调制器运行 64 组轴／母线／角度／电流组合，与 R123 的三个 CCR 和 dq 反馈逐位相同。模型替代参数获取、采样就绪与母线读取边界，并模拟 CORDIC；不执行完整 ADC ISR，不是硬件时序测试。在此模型中指令范围由 541～557 降到 448～464，每例至少减少 90 条执行指令。
- 电流环、逆变器模型、两轴控制／Follow、HSO、参数辨识、采样／保护、编码器与 W25Q64、协议、遥测和 JustFloat 回归通过。
- 外环保护的主机与实际 Thumb 测试保持通过，正常、回绕、超期／缺失、两轴停机及锁存策略沿用。CPU 异常保护每个镜像 139 例通过。
- 生成配置检查／55 项配置变更、IRQ 与真实采样集成、ELF／BIN 一致性、UI 对照以及 HTML 格式握手和串口恢复回归通过。
- 207 个受保护文件及官方／核心编译命令相对 R123 基线一致。

基线、构建与回归记录在忽略目录 `build/outer-stale-r124/`；SPI 写次数对照在 `build/spi-validation/register-before.txt`、`register-after.txt`。

```powershell
python -X utf8 tests/spi_host/run.py --baseline build/outer-stale-r124/mycode/spi_port.c
python -X utf8 tests/motor_host/check_modulator_work_arm.py --baseline build/outer-stale-r124/Debug/FOC_two.elf
python -X utf8 tests/diag_host/run.py
python -X utf8 tests/diag_host/check_outer_arm.py
python -X utf8 tests/motor_host/check_startup.py
python -X utf8 tests/flash_report.py final --directory build/outer-stale-r124
```

## 上板复测

本轮未烧录，实际 ADC／外环微秒值尚待验证，不能宣称实板问题已经消除。

1. 烧录后核对 `About → FW = UI-R124-RT`。复位会解除上一轮时序故障的运行锁存；Clear diag 只清记录。
2. 两轴停机时清诊断，复现 M1 30 rad/s，再启动 M2 100 rad/s；保持原电机型号、母线及电流环开关条件。
3. 运行后停止，记录 ADC IRQ Max、Outer Max、Outer Jitter、Outer E/G/S。若再触发，复位前保留这些值和 First 原因，用于判断剩余峰值预算。

本轮目标是减少真正的高频开销；实板效果以更新后的诊断数据为准。
