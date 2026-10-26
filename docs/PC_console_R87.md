# R87：单文件上位机，第 1～3 批

固件 `UI-R87-PC`。按本轮确认的范围，仅扩展现有可编辑数据的通信入口；固化 PID／电机预设不新增编辑能力。优先提供工作模式、电机与驱动、Run 控制，保留全部板端页面、控制方式、参数辨识、保护与通信功能。

## 实际交付与范围

`pc_tool/index.html` 内嵌 CSS、JavaScript、Web Serial、二进制解析、绘图、CSV 与配置查询。用桌面 Chrome／Edge 打开这个文件即可连接 USB CDC 或 USB 转 USART；不运行 Python 服务，不访问外部脚本／CDN。`start.ps1` 只打开 HTML。旧 Python 文件保留作脚本和协议测试工具。

连接仅查询配置，不下发运动。七种 Work、M1／M2 型号、Single M1 Encoder／HSO／SixStep、ESC 通用、Speed／Angle／Torque／Throttle、Follow、Stop、Focus 已有控制面板；所有原有文本命令继续保留。`Motor1:ESC` 新增为现有 `MotorControl_SelectGeneric()` 的通信入口，仍必须先满足 Single M1／SixStep／静止条件。

本批查询所有持久化数值字段及保存阶段，并通过现有遥测／按需专题显示运行状态。工作模式、型号、反馈、Gear、Focus 走板端原有控制配置入口并请求保存。其余 Settings（Curr Loop、Pos Speed、Speed Limit、步长、LED、IBUS 等）本批先读回，统一写入属于第 4 批。修改已生效与写入 W25Q64 独立；待保存设置不会借用旧的 Saved 状态显示成功。

**远程运动自动进入 LCD Run 页及编辑草稿同步属于第 5 批，本批没有接入。**辨识远程启动、更完整诊断及参数导入属于第 6 批。原有板端功能不受上述上位机分批范围影响。实际限幅后的运动目标由实时曲线／状态显示，ACK 只表示控制入口受理，不表示已转动、HSO 已锁定或已掉电保存。

## Flash 与 RAM

| 阶段 | Debug／Release Flash | Flash 剩余 |
| --- | ---: | ---: |
| R86 基线 | 131004 B | 68 B |
| 用户代码优化对照，复用 R86 协议 | 130308 B | 764 B |
| R87 完整交付 | **131008 B** | **64 B** |

优化对照实际节省 **696 B**；队列／配置查询／回执等净增加 **700 B**，最终比 R86 增加 4 B。没有达到原先 4～6 KiB 的空间规划目标，但能够容纳本批最小接口。后续板端扩展仍需逐批测量，不能据此承诺全部新增功能都能装入。

RAM 分配含原预留堆栈：25080 → **26208 B**，增加 1128 B；32 KiB 链接区余 **6560 B**。该数值不是实测最大栈余量。新增通信使用静态缓冲，没有堆分配。

## 保留的用户代码优化

- `JerryFOC.c` 的四套 PID 不再保留一份上电立即被型号选择覆盖的初始化镜像；实际 `MotorControl_Init()` 在 ADC／输出启动前调用 `JerryFOC_selectMotor()`，完整装载原有增益和阈值。PID 初始化直接赋值，避免临时完整对象。
- `MotorControl_GetAxis()` 直接填写输出快照，减少临时对象与浮点寄存器保存；保留全部状态字段、保护及 HSO／六步覆盖规则。
- 共享固定 `2π` 归一化，使用二进制尾数整除保持有限输入的 binary32 余数及正负零，非有限值返回 NaN；三处调用不再引入通用双参数 `fmodf`／错误处理实现。没有改变 ST 数学库。
- UI 图标、文字池及可变标签的初始字节合并无损压缩：**1871 → 1235 B**，启动时展开到静态 RAM。字体及原始图标不变。112 条信息行从四字节改为三字节描述；字段对齐和可编码范围由编译期表达式检查，KK_UI 仍接收原有结构。
- 协议用户函数限制克隆、少量用户函数保持共用调用。ST／核心源文件与其编译命令、CMake、链接脚本和 CubeMX 均未改变。

原始文字仍在 `ui_app.c` 的 `UI_STRINGS` 与 `ui_labels.h`，图标仍在 `ui_icons.h`。修改资源后运行：

```text
python scripts/pack_ui_resources.py
python scripts/pack_ui_resources.py --check
```

单独阻止内联、切换 `-Oz`、继续压缩字模、压缩页面路由等无收益试验均已撤回。没有通过删除页面、诊断、保护、控制方式或初始化外设换取空间。

## 命令执行与停止

USB／USART 接收中断只组完整行并放入最多四条的静态 FIFO；普通解析、配置和运动准入在主循环执行。浮点解析不再占用 USB／UART 接收 ISR。

Stop 保持立即执行，同时清空未执行队列。排队超过 500 ms 的请求不执行；Key1 全局停止后，停止前收到的旧运动请求不能释放停止锁存。停止锁存期间收到的运动／对齐请求，等待超过 10 ms 会取消，避免等过原有 30 ms 松手保护后执行。另有停止代数检查覆盖解析期间的 Stop 中断。主循环处理结果采用每次调用返回的局部值，避免另一路串口接收将当前 ACK 的结果覆盖。

两轴输入来源、静止准入、无感互斥、编码器／ADC／电流限制、全局停止及故障后手动重新启动的规则继续由原控制框架判断。断开上位机不自动停机，原有控制行为保持。

## 请求及回复协议

保留原 ASCII 行命令，同时支持：

```text
@1:Get
@2:Work:Independent
@3:Motor2:4015
@4:M2:Speed:20
@5:Stop
```

ID 为 1～65535 的十进制数，换行结束。整行仍最多 63 个 ASCII 字节；HTML 对命令正文采用 52 字节上限。`Get` 不更改配置或运动来源。来源由实际接收端确定，不能通过请求文字伪装。

回复沿用 R86 的 `A5 5A`、版本 1、序号、板端时间及 CRC16/CCITT-FALSE，新增 **kind 32，payload 固定 92 B，完整帧 106 B**。小端结构：

| payload 偏移 | 类型 | 内容 |
| ---: | --- | --- |
| 0 | u16 | 请求 ID |
| 2 | i8 | 1 受理；-1 状态／准入拒绝；-2 格式／参数错误；-3 忙／来源冲突；-4 取消／过期 |
| 3 | u8 | 0 USART1；1 USB CDC |
| 4 | u8 | 配置接口 schema 1，固定定义本批查询／Work／Motor／Run 功能集合 |
| 5 | u8 | 固件修订号 87 |
| 6 | u8 | AppSaveState；尚待提交的新配置覆盖为 Pending |
| 7 | u8 | 板端 View，0 M1／1 M2 |
| 8 | 21×u32 | AppSettings 从 speed_x10 到 motor_detect_flux_uwb 的数值字段，顺序如下 |

```text
speed_x10, voltage_x100, position_x100, flags, led_brightness,
speed_step_x10, torque_step_x100, position_step_x100, control_config,
gear_ratio_x100, gear_offset_x100, axis2_flags,
axis2_speed_x10, axis2_voltage_x100, axis2_position_x100,
axis2_speed_step_x10, axis2_torque_step_x100, axis2_position_step_x100,
motor_detect, motor_detect_l_uh, motor_detect_flux_uwb
```

有符号字段按 i32 解释，位图按 u32 解释。没有传输存储 magic／version／size／seq／CRC，也不允许按内存偏移写参数；字段总长度由静态断言锁定。实际运动模式、有效电流环、反馈来源、故障等仍读取状态帧。界面显示持久化目标时明确标注“保存的”，与实时目标区分。

回执仅发送至请求来源，优先于普通遥测；TX 忙或一次提交失败时保留同一回执和不可变活动缓冲。最长保留 1 s，避免断开的主机冻结命令服务。HTML 一次等待一条普通请求，2.5 s 超时显示结果未知，禁止自动重发运动；Stop 可打断等待。输入 FIFO 超载或超长行可能没有回执，客户端按超时处理，不视为成功。

ASCII 请求没有新增输入 CRC 或幂等重发机制；CRC 保护板端回复及遥测。断线前已执行但未收到回执的命令，需读回确认。新 HTML 在配置握手成功前禁止普通运动；Stop 在未握手时另发兼容的原始停止行。

## 浏览器数据处理

- 任意串口分包、噪声、坏 CRC 和重启重同步；诊断整数按整数解析，不经 float32。
- 每条曲线最多 2200 点；读串口和记录独立于暂停绘图。
- 直接文件记录的未完成写入队列上限 1 MiB，超出后停止记录并提示；内存回退记录上限 16 MiB，自动导出并停止。
- 连接／重连建立新的 CSV epoch，重启再分段；配置／状态 JSON 行不当作波形回放。
- 所有板端文字使用 textContent，配置读回不覆盖输入框尚未发送的运动目标。

## 验证与边界

已完成：

- 完整电机主机回归（有感、双机跟随、独立 HSO、六步、辨识、编码器恢复、采样和保护等）；新定角归一化与独立 libm 对照 **100 万组 binary32**，另保留原 CORDIC／PRIMASK 测试。
- 实际生产 C 代码的 FIFO／逐条回复、USB／UART 来源、TX BUSY／失败保留、Stop 清队列、断开回执超时、旧运动过期及 Key1 取消测试。
- JavaScript 消费实际 C 生成的遥测和 RPC 字节；逐字节破坏 CRC／帧重同步；Web Serial 模型的只读连接握手、拒绝／超时、不自动重发、Stop 优先、重连与挂起写入清理；CSV 内存和写入队列边界。
- UI 3689 次显示提交、真实 LCD 驱动与异步存储回归。使用本轮备份的 **R86 原始 ui_app.c／资源**另行编译对照，62 张静态／过渡图中仅 About 的 FW 文字变化，其余逐字节一致。
- LL 中断 8388608 种 ADC 组合、真实采样／保护集成、USB CDC 及最终 ELF 的 MSP／控制／Break／KEY1 路径检查。
- Debug／Release 干净构建；205 个受保护文件及官方／核心编译命令与本轮基线一致，ELF／BIN 匹配。

未烧录、未打开真实 COM 口；浏览器授权／视觉效果、长时间文件记录和实板控制耗时尚待验证。主机模型不能替代真实 ST HSO 运算精度和电机实测。当前资源余量不足以直接承诺后续所有板端扩展。

```text
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
node tests/pc_tool/test_frontend.js
python tests/pc_tool/test_wire.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/console-r87
```

基线镜像、体积试验、完整日志、C 字节样本及图像对照位于本机忽略目录 `build/console-r87/`。本轮不提交 Git、不操作电机。

最终 BIN SHA256：

```text
Debug   28d35b7ea21e569f61b3b251d199bbf9fe153101d03b6eea1e72b71b029a8ead
Release 3c480393d81eb73785d02ec613c68692ac4bd55ae0fe20e2697f3384feef61f2
```
