# R126：控制链数据复用与重复准入查询精简

本轮对应优化清单第 3、4 项。基线为 R125 `UI-R125-DL`，交付为 `UI-R126-CTX`，Native 回执固件编号 126。

## 实际改动

第 3 项：

- `JerryFOC_CurrentControl()` 每次取得一次电机参数，将指针传给内部调制函数。有感电流环的参数查询由 2 次降为 1 次。
- 调制函数返回本次实际使用的母线值，PI 回算的 PWM 量化容差复用该值。母线查询由 2 次降为 1 次，也避免同一次控制在毫秒边界取到不同有效状态的母线值。
- `JerryFOC_run()`、`JerryFOC_OuterIRQ()` 分别复用各自调用内的 HSO 模式选择；动态的 HSO 运行状态仍实时读取。M2 快速环的预测、对齐及电流限制共用本次电机参数。
- 公共电压接口签名、PI、前馈增益、矢量限幅、死区补偿、PWM 更新边界及重试次数保持原有行为。HSO 仍为 PI 电压控制。

第 4 项：

- 快速环已经先调用 `JerryFOC_isDriveReady()`，原先随后调用 `sensored_control_ready()` 又重复查询了一遍相同准入条件。
- 拆出只检查连续采样、编码器健康与对齐的 `encoder_control_ready()`，快速环在刚通过准入后使用它。
- 启动和外环继续使用完整的 `sensored_control_ready()`。电流控制／编码器调制阶段的两次 M1 实时就绪复核、调制前的轴采样检查、KEY1、Break、IBUS 及输出使能检查全部保留。
- 复用结果仅在单次调用内有效；没有跨周期、跨轴或持久化的保护状态缓存。

## 实验与取舍

先试过在 `MotorSampling_Capture()` 中保存母线时效结果，以及合并采样新鲜度判断。直接缓存能将正常 Capture 的时钟读取从 5 次降到 4 次，但实际链接代码的寄存器压力增大：相较仅第 3 项的镜像，多占 80 B Flash，模型中的双轴采样路径还多执行 4 条指令。提取辅助函数、局部栈变量的试验也没有得到合适收益，均未保留。

最终 `motor_sampling.c` 与 R125 基线完全一致。保留的第 4 项处理的是快速环重复的**准入查询**，不合并采样层或最终输出门控的独立复核。

| Debug 测量阶段 | Flash 已用 | RAM 已分配 |
| --- | ---: | ---: |
| R125 基线 | 130184 B | 30904 B |
| 第 3 项，使用 R126 版本标识 | 130080 B | 30936 B |
| 第 3、4 项最终组合 | 130096 B | 30936 B |

最终 Debug、Release 相同：

| 项目 | R125 | R126 | 变化 |
| --- | ---: | ---: | ---: |
| Flash 已用 | 130184 B | 130096 B | 减少 88 B |
| Flash 剩余 | 888 B | 976 B | 增加 88 B |
| RAM 已分配 | 30904 B | 30936 B | 增加 32 B |
| RAM 剩余 | 1864 B | 1832 B | 减少 32 B |

88 B 是最终完整镜像的净差，包含版本标识、资源和链接布局变化，不按源码行数或单个函数大小直接归因。版本标识更新后，压缩资源实际由 2334 B 增为 2335 B，未删除界面内容。32 B RAM 增量来自现有 RAM 执行区中的调制代码及链接布局，没有增加常驻采样或电机状态缓存。

第 4 项比仅第 3 项多用 16 B Flash，换取快速环每次成功准入后少做一遍完整驱动就绪查询。整轮以减少控制路径重复工作为主，Flash 收益有限。

## 验证

- Debug／Release 正常构建与 CubeMX 检查通过；207 个受保护文件和官方／生成代码编译命令与基线一致。
- 192 组真实 FOC 主机轨迹对比覆盖 2804／4015、电流／电压模式、VBUS 0／12／20 V、空闲、未就绪、编码器异常与未对齐。PWM、电流滤波和目标值与 R125 逐项一致；其中 48 组消除了一次重复准入查询。
- 两种镜像各 480 组实际 Thumb 电流控制模型对比通过，PWM 和控制器状态一致。正常 M1 电流调用的两次实时驱动就绪复核仍在，参数和母线查询均为 2→1。
- 在模拟外设边界下，代表性有感电流调用的指令数为 M1 710→693、M2 689→669；部分安全退回支路增加 4 条指令。该模型替代了外设／部分查询，不能视为板上周期或微秒测量。
- 14 组真实采样轨迹覆盖 0～6 ms 超时边界、毫秒计数回绕、读取时钟时 IRQ 发布新数据及随后新出现的硬件故障；帧、故障、输出状态和时钟读取次数保持一致。
- 正常采样的实际 Thumb 模型另核对单轴／双轴、五组原始电流；采样函数的数据和执行工作保持一致。
- 控制、运动、HSO 独立启动、辨识、采样及保护、有感 FOC、电流环、逆变器补偿、编码器恢复、协议、遥测和 JustFloat 回归通过。真实采样与 IRQ 分发集成、外环超期保护、CPU 异常向量与启动产物检查通过。
- UI 的 68 组图像、64857 次文字绘制、870 行诊断格式对比通过。上位机前端及真实 Edge 格式切换、串口恢复回归通过。

新增或扩展的定向检查：

```powershell
python tests/motor_host/check_foc_reuse.py --baseline build/foc-sampling-r126/mycode/JerryFOC.c
python tests/motor_host/check_sampling_reuse.py --baseline build/foc-sampling-r126/mycode/motor_sampling.c
python tests/motor_host/check_modulator_work_arm.py --scope current --baseline build/foc-sampling-r126/Debug/FOC_two.elf --output build/foc-sampling-r126/current-work.json
python tests/motor_host/check_sampling_work_arm.py --baseline build/foc-sampling-r126/Debug/FOC_two.elf --output build/foc-sampling-r126/sampling-work.json
python tests/flash_report.py final --directory build/foc-sampling-r126
```

基线副本和测量日志存于忽略的 `build/foc-sampling-r126/`。标准 `build/Debug/`、`build/Release/` 的 ELF／BIN／MAP 已更新，继续作为备份产物。

尚未烧录。上板仍需用双机独立的 M1 30 rad/s、M2 100 rad/s 工况，以及 Follow 工况，比较 `ADC IRQ Max`、`Outer Max`、`Outer Jitter`、`Outer E/G/S`。本轮未调整任何超期阈值、控制周期或电流保护阈值，也无需重新配置 CubeMX。
