# KEY1 冷启动充电等待（UI-R103-KEY）

现场复现：`Stop Src KEY1 LOW`、`Stop At 0ms S4`（用户写为 54）、`Stop K/P 0/0 Now 1/0`。用户确认 KEY1 只有对地 100nF 电容，没有外部上拉，使用 MCU 内部上拉。

这组记录说明：第一次软件电平检查在外设初始化刚结束、HAL tick 仍为 0 时读到低电平，没有 EXTI 待处理；后来引脚已恢复高电平。它符合内部上拉给电容充电尚未完成的表现。完整实板电压波形尚未测量，不据此排除其他启动电源／引脚波形因素。

## 选用启动等待

[ST 数据手册 DS12589 Rev6](https://www.st.com/resource/en/datasheet/stm32g431vb.pdf)，Table 53、PDF 第 111 页：GPIO 弱上拉等效电阻最小／典型／最大为 25／40／55kΩ，输入高电平规格包含 0.7VDD。按等效 RC、理想阶跃估算，100nF 的时间常数为 2.5～5.5ms，到达 0.7VDD 约为 3.0～6.6ms。实际时间还受电容容差、电源上升、GPIO 接通时刻等影响。

`main.c` 的 `USER CODE 2` 在保留 CPU 异常的启动禁止之后、首次 KEY1 电平检查之前，增加一次 `HAL_Delay(30U)`。这是一次性的冷启动／复位检查等待，给 100nF／内部上拉留出约五倍最大名义时间常数。

- 等待时未启动 PWM 计数、ADC／FOC、编码器或自动对齐，两桥输出尚未使能。
- KEY1 的生成 EXTI 已启用；不屏蔽中断、不清 pending、不清已有停止锁存。
- 等待期间若真实按下、短按、发生下降沿，立即保留原有全局停止。启动前已经记录的边沿、CPU 启动禁止也不会被等待后覆盖或解除。
- 30ms 后仍低电平，照常锁存 KEY1 LOW；按住 KEY1 上电仍禁止对齐／运行。
- 运行期间没有新增按下去抖延迟，仍由 EXTI 立即断开两桥 MOE／CCER，并保留 GPIO／pending 的兜底检查。原有释放稳定 30ms、主循环清理、明确手动恢复均不变。
- 原 R102 的来源、阶段、时间和触发／当前引脚状态诊断保留。

不调整 `.ioc` 或任何生成的初始化参数，不修改 ST HAL／LL／CMSIS／USB，不更改控制算法或电流保护。无需重新操作 CubeMX；重新生成时需继续保留 USER CODE。`scripts/check_cubemx.py` 每次构建检查 CPU 禁止 → 30ms 等待 → 首次电平轮询的顺序，防止重生成丢失这段启动业务。

## 验证与资源

`tests/motor_host/check_key_boot.py --baseline build/key-startup-r103/Core/Src/main.c` 从实际 main USER CODE 提取启动前缀，分别编译修改前／后的代码并使用真实采样／停止实现：旧版在模拟 RC 条件下复现 `0ms S4` 和 `0/0 Now 1/0`；新版覆盖电容上升、启动按住、等待期间 1ms 短按、先前 EXTI、CPU 启动禁止、待处理边沿和 tick 回绕。还验证等待期间无输出／计数、之后运行中的两桥停止无新增 tick 延迟。该用例已接入采样回归。

CubeMX 回归增加两种删改等待的拒绝测试，共 36 种配置变更；链接镜像检查实际 `movs r0,#30 → HAL_Delay → MotorSampling_IsGlobalStopped` 指令顺序。采样／IBUS／双轴停止、UI／存储／显示、诊断及 Native／JustFloat 回归保持。

Debug／Release 均为 Flash **130620 B**，比 R102 增加 **12 B**，剩余 **452 B**；RAM 分配 **29592 B**，无变化。修改前的 ELF／BIN／MAP 及用户代码备份在 `build/key-startup-r103/`，两版新产物继续保存在 `build/Debug/` 和 `build/Release/`。

上位机配置回复版本增至 103，HTML 兼容范围同步更新，界面本身仍为 R102；通信格式和设置结构不变。

上板验证：About 确认为 UI-R103-KEY，完全断电并等待电容放电后重复上电，KEY1 释放时应正常进入原有对齐流程；按住 KEY1 上电应保持 GLOBAL STOP；运行中短按应立即切断两桥，释放后不自动恢复。如仍误停，保留 Stop Src／At／K/P，尤其检查是否仍为启动阶段、或变为 KEY1 EDGE。
