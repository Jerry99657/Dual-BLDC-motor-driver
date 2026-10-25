# R81：控制框架整理

固件 `UI-R81-CF`，2026-10-26。第 4 批将工作模式、驱动方式、输入来源和已确认参数分开管理；保留现有控制算法、保护、参数辨识与 UI 样式。无需重新配置 CubeMX，W25Q64 继续使用 v5 记录。

## 控制边界

`MotorControl` 统一处理命令准入、轴选择、来源检查、模式限制和限幅，再交给各轴原有控制器执行。M1 保留编码器／HSO／六步；M2 与所有双机模式继续只用编码器。HSO 与六步通过单个驱动枚举互斥，保存时仍使用原有配置位。

UI 的 View 只影响查看和本地操作，不再隐式决定双机通信命令的目标轴。两轴的估计器、PI、校准和运行状态仍分别保存；本批没有合并实时环路，也没有改变主从轨迹算法或控制周期。

## 输入来源与停止

成功启动后，该轴记录 `LOCAL`、`USB`、`UART` 或 `FOLLOW`。运行中只有相同来源可以更新目标；其他来源返回 `CONTROL BUSY`，目标与输出保持原状。双独立允许两轴分别由不同来源控制；同步组同时检查两轴。USB／UART 分包接收各自记录来源，不共用临时来源变量。

Status → Motion 新增 `Input`。未请求功率输出时显示 `--`，手动跟随的被动主轴也显示 `--`。来源不写入 Flash；停止或故障后重新启动需要明确命令，恢复通信不会自动运行。切换来源的方法是先停止，再由新来源启动。

`Stop`、KEY1 全局停止、Run 页明确的 Back 停止不受来源限制。KEY1 的中断关输出和松开后手动重启规则保留。浏览 Run、编辑草稿、离开页面或本地确认被拒绝不会取消 USB／UART 的运行；离开 Run 仍清理本地启动的运行。手动 Follow 用原有独立入口启动，不接受普通目标命令接管；运行中再次按 OK 仍可明确停止整组，不受 FOLLOW 来源标记影响。

### 通信命令变化

**所有双机模式的目标命令必须明确写 `M1:`／`M2:`。** 缺少轴前缀返回格式错误，不按 View 猜测。单机模式继续接受无前缀命令。示例：

```text
M1:Speed:20
M2:Angle:0.5
M1:Torque:0.1
M2:Stop
Stop
```

双独立的 `M2:Stop` 只停止 M2；同步／主从组的停止仍停止整组。无前缀 `Stop` 停止全部。指定停止 M2 不会额外清理 M1 的 HSO 状态。同步模式仍只允许向组主轴下目标，不能覆盖从轴。

`Work:`、`Motor1:`、`Motor2:`、`Feedback:`、`Gear:`、`Focus:`、`Align:` 仍为全局配置命令，不加轴前缀；配置在主循环排队执行。`Follow` 仍为整组动作。`OpenLoop:` 仍仅用于 Single M1 编码器方式的诊断，但现在也检查来源。未知命令明确返回格式错误。兼容入口 `Protocol_HandleLine()` 按 UART 来源处理。

## 参数发布与单位

控制环不再直接读取可变的 `AppSettings` 中的限幅、位置速度或齿轮参数。`MotorControl_SetOptions()` 在短临界区发布每轴已确认配置，UI 只有在发布成功后才修改保存记录并标记待保存；拒绝的编辑恢复显示，不触发保存。

| 参数 | 应用时机 |
| --- | --- |
| 有感 Speed Limit／Pos Speed | 允许运行中确认；速度参考继续经过原有斜坡，位置速度不超过 Speed Limit |
| HSO／六步 Speed Limit／Pos Speed | 对应轴停止后修改 |
| Curr Loop | 对应轴停止、未对齐，且没有活动同步／主从组时修改 |
| 型号、工作模式、驱动方式、齿轮比／偏移 | 保留原有停机和静止准入检查 |

串口 `Angle` 的默认位置速度现在与 Settings 的 Pos Speed 一致，并受 Speed Limit 限制。机械速度／位置继续使用 rad/s／rad；Run 的 `Iq(A)`、`Uq(V)`、`Throttle %` 保留。旧命令名 `Torque` 在电流环开启时表示 A，关闭时表示 V，并非 Nm；ESC Auto 仍使用油门与电气 Hz。没有新增转矩换算。

## 构建与验证

| 链接占用 | R80 基线 | R81 |
| --- | ---: | ---: |
| Debug／Release Flash | 130164 B | 131068 B |
| Flash 剩余 | 908 B | **4 B** |
| RAM 分配（含预留堆栈） | 23096 B | 23144 B |

本批净增加 Flash 904 B、RAM 48 B。已合并协议前缀识别与 UI 配置装载等重复代码以容纳本批，但现有芯片已接近容量上限，不能据此承诺后续新增功能仍能直接放入。

ST/HAL、CMSIS、官方 USB、HSO、CubeMX 生成文件、链接脚本、KK_UI／KK_OLED 核心及官方编译选项未修改；205 个受保护文件和构建命令与本批基线核对一致。

验证命令：

```text
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/usb_host/run.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/framework-r81
```

电机主机测试覆盖真实协调器／FOC、双独立与同步／主从、独立 HSO 启动与故障重启、六步、辨识、采样和编码器恢复；新增来源互斥、双轴显式命令、交错分包、草稿隔离、实时限幅、停止优先和位置速度一致性检查。UI 测试覆盖拒绝编辑不保存、Input 显示、浏览及本地确认失败不打断远程运行、Back 明确停止。USB 与链接启动检查同时验证。

这些是主机测试、模型和构建检查，未烧录开发板，也未测量本版实机 Ctrl Max。实板请优先核对双独立的带前缀命令、切换来源、运行中限幅、KEY1／Back 停止，以及原有 HSO／六步启动。

BIN SHA256：

```text
Debug   58151be222f0adffd9f90381b12cb5b30875f92604a15fa13482e464d4f3a503
Release 7913dfb39815574af3fdb905ad459e8d2ed6aa3c71ef2f353be958df4613b4d0
```
