# R97：Run 未对齐时的手动恢复与诊断入口命名

固件 **UI-R97-ALIGN**。用户现场：Single M1、ENCODER，Drive=GLOBAL STOP、Aligned=NO，Reset 后没有自动对齐；重新断电上电后恢复。

## 原因和边界

GLOBAL STOP 会取消排队／正在执行的自动对齐。触发来源可能是 KEY1、上位机全局停止，或者 R95 起新增的“有效 CPU 异常记录在 Reset 后禁止自动对齐”。松开 KEY1 本身不会自动恢复输出。重新上电恢复，不能独自证明之前一定发生了 CPU 异常；本次没有保留断电前的异常寄存器数据。

旧 Run 的 OK 在执行控制层恢复逻辑前检查 Aligned，因此给出 NOT ALIGNED，无法在 Run 内申请重新对齐。旧固件可从 Settings → System → Align / Retry 明确申请对齐。

## 修改

- Run 未对齐时页脚显示 `OK:Align`；按 OK 调用已有 `MotorControl_RequestAlign()`，保留按键释放、停止状态、轴使能、硬件 IBUS 和控制模式检查。
- 只进入页面、松开 KEY1、清除诊断或完成对齐，都不会自动应用 Run 目标。对齐完成后仍需再次按 OK 才能运行。
- 对齐进行中重复 OK 只提示 ALIGNING，不重新开始计时。按键仍被按住或保护不允许时，恢复请求被拒绝。
- GLOBAL STOP 的显示优先于 NOT ALIGNED；正在对齐显示 ALIGNING。成功申请 M1 对齐后清除旧控制层停机提示，诊断首次故障／CPU 保留记录继续保留。
- Status 中原名为 `System` 的诊断入口和页面标题改为 `Diagnostics`。Settings 的 `System` 保持。CPU 异常类型字段的实际名称为 `CPU Fault`，不是 CPU Kind；进入专题后向下滚动查看。协议仍为原专题 6、原 37 行，不新增遥测通道。

本轮没有修改 KEY1 的中断／极性／引脚、CubeMX 初始化、全局停止和异常关断机制、ST 官方库或 KK_UI 核心，也没有加入自动解除暂停或自动重新运行。

## 资源和验证

| 构建 | R96 Flash | R97 Flash | 增加 | Flash 剩余 | RAM 总分配 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 128828 B | 129024 B | 196 B | 2048 B | 29424 B |
| Release | 128828 B | 129024 B | 196 B | 2048 B | 29424 B |

RAM 比基线增加 16 B（显示字符串及对齐），含原有堆栈预留；未分配 3344 B。本批可装下，无需额外压缩。

验证：

- `tests/ui_host/run.py`：先复现旧 Run 缺少对齐操作的场景；覆盖 GLOBAL STOP、禁止恢复、显式申请、对齐中重复确认、完成后不自启、二次确认启动、编码器／ADC 不满足时拒绝；检查 Diagnostics 菜单／信息页真实路由。原 UI、R96 查询隔离、存储和 LCD 回归通过。
- `tests/motor_host/run.py control foc sensorless protocol telemetry`：真实控制层按键暂停／释放／对齐、两轴状态、HSO、Run 准入、协议和遥测；新增恢复后 M1 不再残留 GLOBAL STOP 提示及不回放旧目标的检查。
- `tests/motor_host/check_startup.py`、`tests/diag_host/check_arm.py --both`：实际 Debug／Release 启动和 ELF／BIN 一致性；两种 ELF 各 127 个异常处理 Thumb 仿真场景。
- `tests/pc_tool/test_console.js`、`tests/pc_tool/test_wire.py`：单文件上位机兼容 R97，保留旧版本支持与原有交互／协议。
- `tests/flash_report.py final --directory build/alignment-r97`：211 个受保护文件及官方／Core 编译选项全部保持基线内容。每次构建的 CubeMX 配置检查通过。

Debug／Release 的 ELF、BIN、MAP 已更新；本轮未刷板。再次出现问题时，先在 Status → Diagnostics 保留 CPU Fault／PC／CFSR 和首次故障，再决定是否断电；这些 SRAM 记录不保证断电保留。
