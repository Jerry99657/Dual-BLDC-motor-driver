# 上电 GLOBAL STOP 来源诊断（UI-R102-STOP）

## 现场与结论边界

现场：完全断电后上电出现 GLOBAL STOP，Diagnostics 的 CPU 项均为 `--`。
`--` 仅表示没有有效的保留 CPU 异常记录；原有 GLOBAL STOP 显示不能区分 KEY1 中断、KEY1 电平／待处理边沿、软件暂停和 CPU 启动保护。

核对当前 `.ioc` 和生成代码：KEY1 为 PD13、上拉、下降沿 EXTI，EXTI15_10 优先级 0，生成调用经 `cubemx_irq.h` 路由。初始化配置与应用的按下低电平约定一致。本轮没有证据证明 CPU 异常或确认按键线路故障，也没有证明启动误停已经消除，需要实板的新来源记录。

## 新增 Diagnostics 行

位于 Status → Diagnostics 末尾；两轴显示同一全局事件。上位机 `GetRow` 的 Diagnostics 组也可查询，总行数由 37 增至 40。

| 行 | 内容 |
| --- | --- |
| Stop Src | `CPU BOOT`：有效 CPU 记录导致的启动禁止；`KEY1 EDGE`：EXTI 回调；`KEY1 LOW`：轮询发现低电平；`KEY1 PEND`：按键已高但 EXTI 仍待处理；`COMMAND`：界面／上位机的软件暂停。无事件为 `--`。 |
| Stop At | 从本次 MCU 启动起的毫秒数和触发阶段，例如 `4 ms S3`。S3 为外设初始化期间，S4 为初始化结束后的启动检查，S5 为 ADC 采样启动，S6 为外环启动，S7 为主循环。 |
| Stop K/P | `触发K/触发P Now 当前K/当前P`。K=1 表示 KEY1 高／释放，K=0 表示低／按下；P=1 表示 EXTI 待处理。EXTI 分发先清 pending，再进入停止函数，故 EDGE 事件的触发 P 通常为 0。 |

KEY1 EDGE 即使记录时已释放，也不能仅凭高电平判为误触发：真实短按、线路脉冲都可能如此。需结合触发时间、阶段与必要的引脚波形判断。CPU BOOT 在以后被清除 CPU 诊断时仍保留来源，防止清诊断后失去当次停止的证据。

## 实现与保护约束

- 停止入口明确携带原因，原 `MotorSampling_GlobalStopIRQ()` 保留给现有 CPU 启动钩子；其他入口分别记录 KEY1 与 COMMAND。
- 先断开两桥 MOE／CCER、撤销请求，再记录短小的 RAM 信息；中断内不访问 SPI／Flash、不格式化、不收集完整首次故障快照。
- 每次停止锁存只记录首次原因，后续轮询／弹跳／其他停止入口不覆盖。采样 Stop／Start、清除 CPU／首次故障诊断都不抹掉该记录；明确解除停止后，下一次停止可更新。复位后记录归零，不写 W25Q64，不掉电保存。
- 全局停止、KEY1 电平与 pending 检查、短按检测、30 ms 稳定释放、主循环清理、手动恢复和其他保护均保留。没有加入开机忽略窗口、清除启动 EXTI 或自动解除停止。
- 未改 ST HAL／LL／CMSIS／USB、CubeMX 初始化、DMA／NVIC 配置或控制参数。
- 固件标识为 UI-R102-STOP，Native 配置回复版本为 102，上位机扩展接受范围同步增加到 R102；JustFloat 上电默认和所有传输内容保持原样。

## 验证

新增真实采样层来源回归：初始化阶段已释放的边沿、上电低电平、仅 pending、CPU 启动、软件暂停；来源不可被覆盖，采样重启保留，触发／实时按键状态区分，PRIMASK、计时回绕和手动恢复无自动上电。

诊断格式与行边界、UI 本地／远程行数、设置暂停路由、真实 ST LL EXTI／采样集成、双轴控制／HSO／FOC、Native／JustFloat、CubeMX 34 项变更、两种构建实际 Thumb CPU 异常回归、USB／USART 驱动回归、上位机 R102 连接／读回均已通过。211 个受保护文件及其编译选项校验通过。主机测试不代替 KEY1 实板启动波形验证。

Debug／Release 均为 Flash **130608 B**（比 R101 增加 **356 B**，余 **464 B**），RAM 分配 **29592 B**（增加 **136 B**，含原有堆栈预留）。ELF／BIN／MAP 已更新，修改前备份及资源报告位于 `build/global-stop-r102/`。仅保留既有的 RWX 段链接警告。

上板流程：确认 About 为 UI-R102-STOP，完全断电后重新上电；如再出现 GLOBAL STOP，在再次复位前记录 `Stop Src`、`Stop At`、`Stop K/P` 三行。不要先按全局暂停或断电。KEY1 释放后可通过现有 Run 的明确 OK／对齐入口手动恢复，释放本身不会恢复输出；若需对齐，对齐结束后仍须再次确认运行。
