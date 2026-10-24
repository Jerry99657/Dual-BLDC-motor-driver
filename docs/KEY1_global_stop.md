# KEY1 全局停止（UI-R59-STOP）

2026-10-24。KEY1 使用 PD13 / EXTI13，按下接地、上拉、下降沿触发；NVIC 启用 EXTI15_10、优先级 0。CubeMX 已生成正确的初始化和中断入口，应用不再把 KEY1 改回普通输入。

## 操作

- 任意页面、工作模式下，KEY1 同时停止 M1/M2，包括双独立、同步、主从、编码器对齐、HSO 启动和 CONTROL。断开功率输出，采用滑行停止；转子可能因惯性继续转动。
- 停止事件锁存。松开不会恢复；稳定释放至少 30 ms、主循环完成清理后，新的 Run OK 或串口运行指令才允许启动。按住时拒绝运行和对齐；开机按住也生效。
- 未完成的自动对齐被取消。需要对齐时，在 System 的 Align Current Axis 手动请求。停止中切换型号、反馈或工作模式不会自动对齐。
- 双独立模式两轴 UI 运行缓存一起清除，目标草稿保留。长按左／右仍可选轴，KEY2 继续返回。
- 提示、Run Stop 和 Status 的 Drive 显示 GLOBAL STOP。KEY1 不清除 IBUS、ADC、电流等已有故障。
- HSO 重新启动仍需满足静止和启动条件；主动停止导致的观测失锁不会额外记录为新的 HSO STOPPED。

## 实现

`motor_sampling.c` 的 `HAL_GPIO_EXTI_Callback` 只处理 KEY1：锁定并记录待清理事件，关闭两桥 MOE 和六路相输出 CCER，撤销两轴输出请求及预装载计数。允许定时器句柄尚未初始化。中断不做显示、Flash、SPI 或阻塞等待。

`MotorControl_Service` 在主循环清理目标、PI、对齐、主从、HSO 及 SPI2 实时占用。重新启动入口要求停止事件已清理、KEY1 已稳定释放；释放本身不请求输出。采样停止／重启不清除全局锁定。

输出请求与实际使能均检查锁定、GPIO 和 EXTI 待处理位，避免延迟的按键事件被后续 MOE／CCER 写入覆盖。采样回调和主循环也轮询 KEY1，覆盖开机按住。计数器、CH4、ADC、母线诊断和通信继续工作，COMP4→TIM1 Break 保持独立。

KEY1 由 CPU 响应，同优先级控制中断及已有临界区会影响响应时延；实板时延需要测量。

## 验证

- `tests/motor_host/run.py`：实际采样代码的初始化前按下、开机按住、双桥断开、释放消抖／计时回绕、延迟 EXTI、GPIO 兜底、预装载／MOE 竞争、连续 ADC、重启保留和 IBUS 独立锁存；实际控制器的全部工作模式、按住拒绝启动、手动重启、对齐取消；实际 HSO 外围的启动／CONTROL 停止和延迟主循环清理。
- `tests/ui_host/run.py`：KEY1 保留 EXTI、不进入 UI 六键路由；双独立两轴运行缓存清除、停止提示及新的 OK 启动；原有 UI／保存／LCD 回归。
- `tests/motor_host/check_startup.py`：链接后的中断入口、真实停止回调、版本及 ELF/BIN 一致性。

主机测试使用外设及 ST 核心桩，不代表实板响应时延、电机停转时间已经验收。产物位于 `build/Debug`、`build/Release` 的 ELF、BIN、MAP。

最终构建占用：Debug Flash 127144 / 131072 B、RAM 22832 / 32768 B，Flash 余 3928 B；Release Flash 126384 B、RAM 22728 B，Flash 余 4688 B。保留原有 RWX LOAD 段链接警告，无新增编译警告。版本为 UI-R59-STOP，无需继续调整 CubeMX。

BIN SHA256：

- Debug：`4a706c57d82c8a7b36496770f0515359cea0bd4b3002e0217344f6d19e0a6d3f`
- Release：`3b8647006e4bfe8be2d3a8ebc266205e01d0aec5dab82bd44b979b256cebb5b6`

上板依次核对：按住 KEY1 上电；各工作模式运行中停止；对齐／HSO 启动／CONTROL 中停止；按住时 UI／串口拒绝启动；释放后不自动运行；原有保护不被清除；母线和采样计数继续更新。定量时延测量 PD13 到两桥驱动输入关闭的波形。
