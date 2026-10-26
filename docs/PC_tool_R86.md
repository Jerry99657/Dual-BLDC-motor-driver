# R86：精简遥测、HSO 相对位置与电脑端工具

固件 `UI-R86-PC`。落实 [R85 遥测审查](Telemetry_review_R85.md)：常用曲线、低频状态和按需专题诊断分开；新增本地电脑端工具，USB CDC／USART1 共用协议和文本命令。

## 结果与资源

| 项目 | R85 基线 | R86 |
| --- | ---: | ---: |
| Debug／Release Flash | 129164 B | **131004 B** |
| Flash 剩余 | 1908 B | **68 B** |
| RAM 分配，含原预留堆栈 | 24944 B | **25080 B** |
| 链接 RAM 余量 | 7824 B | **7688 B** |

功能增加净消耗 Flash 1840 B、静态 RAM 136 B。当前已接近 128 KiB 上限，后续板端功能必须重新测量体积，不能把电脑端功能的可扩展性当作 MCU 空间充裕。没有调整 Flash 长度或链接脚本。

ST HAL／LL、CMSIS、USB／HSO 官方实现与编译选项均未修改；KK_UI／KK_OLED 核心、字体、图标、CubeMX 初始化和 `.ioc` 未修改。无需重新生成 CubeMX。

## 板端改动

- 移除 `JerryFOC_OuterIRQ()` 的旧 25-float 组帧及按模式覆盖槽位的 `MotorControl_Telemetry()`，改由主循环的 `Telemetry_Service()` 生成有类型的数据包。CRC 和传输复制不在控制 ISR 中执行。
- 参数辨识时的外环提前返回继续保留，通信生成已独立于此路径；辨识过程继续发送状态、心跳、进度字符串和完成后的 PP／R／Ls／磁链结果。
- 增加实测 VBUS 与新鲜度有效性。未启用轴、未使用目标、未合格 HSO 位置和不适用的电流不伪装成有效 0。
- 常用 Iq 在输出未请求或辨识接管时标为无效。FOC 专题的 U／V 相电流直接读取 `MotorSampling_CopyFrame()`，六步和辨识下也不会上传旧 FOC 电流缓存；不适用的 dq 值仍标为无效。
- 变更状态优先，普通心跳约 10 Hz；状态抖动合并到至少 20 ms 间隔，避免所有曲线时隙被状态变化占满。
- 配置采集组使用 `Stream:`，只保存在 RAM，不写 W25Q64、不切 LCD View、不启动电机、不改变运动来源。
- 共用静态类型化组帧工作区；USB 保留最新待发送与独立活动缓冲，UART 仅在 HAL READY 时复用单个静态缓冲。传输引用的缓冲与组帧工作区分离，工作区不直接交给 DMA。
- 发送生成和邮箱消费均在主循环；USB 邮箱复制不再屏蔽控制中断。原有早完成、拒绝提交、断开／重连及活动帧不可修改规则保留。

## HSO 机械相对位置

ST HSO 接口 `HSO_getAngle_pu()` 提供电角度。项目在连续合格的 CONTROL 中记录电角度跨圈数，以

```text
relative_mechanical_position = (electrical_turns * 2π + angle - first_control_angle) / pole_pairs
```

得到本次连续观测的机械相对位移，单位 rad。此计算是本工程外围实现，不是 ST 库提供的机械绝对位置 API。

跨圈计数在原 10 kHz 观测更新中执行，利用既有小于 400 电气 Hz 的限制保证正常相邻角差小于 π。保留整数电气圈数和有界角度，在读取时换算，避免每个采样不断累加 float 微小机械增量造成累计舍入误差；计数溢出前撤销位置有效性。

`StHso_Position()` 和快照新增字段不访问编码器。首次合格 CONTROL 建立相对零点；Stop、拒绝／失锁、断采样、初始化后位置无效，读取返回 NAN；下一次启动重新建立零点。`JerryFOC_getAngle()` 在 HSO 方式下读该位置，电角度显示同样改用合格 HSO 输出。LCD 对非有限诊断值显示 `--`。

这不能恢复上电前的机械零位、辨认静止时的外力位移，也不保证失锁期间的位置连续。估算精度仍受电机参数、相电压／电流精度、噪声与 HSO 锁定质量限制。本次没有开放无感位置闭环、零速保持或双机无感功能。

参考：[ST HSO／ZeST 官方说明](https://www.st.com/content/st_com/en/ecosystems/stm32-motor-control-ecosystem/stm32-zest-and-hso-solutions.html)，本地官方接口 `ThirdParty/ST_HSO/hso.h`。本工程只有已有 HSO 库，没有新增 ZeST。

## 二进制协议 v1

USB CDC 与 USART1 发送相同字节序列；USART1 保持 115200、8N1。所有整数为小端，float 为 IEEE754 binary32。包最大 128 B，不发送本机指针或未初始化填充字节。

| 偏移 | 类型 | 内容 |
| ---: | --- | --- |
| 0 | u16 | 0x5AA5，线上字节 A5 5A |
| 2 | u8 | 版本 1 |
| 3 | u8 | 包类型 |
| 4 | u16 | 生成序号，按 65536 回绕；传输忙跳帧可表现为序号缺口 |
| 6 | u16 | payload 字节数 |
| 8 | u32 | 主循环组帧时 HAL 毫秒时间，按 2^32 回绕 |
| 12 | bytes | payload |
| 12＋长度 | u16 | CRC16/CCITT-FALSE，小端；多项式 0x1021、初值 FFFF、不反射、无最终异或，覆盖整个头和 payload |

曲线 payload 为 N 个 float 加末尾 u16 有效位。低 N 位分别表示相应值有效；无效值同时写 NAN。末尾位 15 在双轴包中表示协同误差为速度误差，在 FOC 包中表示 q 命令单位为 A，在六步包中表示 Speed；六步位 14 表示 ESC 电气 Hz。单位可由该帧自身解释，不依赖状态帧先到达。

| 类型 | payload 长度 | 内容，按发送顺序 |
| ---: | ---: | --- |
| 1／2 | 26 | M1／M2：目标速度、反馈速度、目标位置、反馈位置、Iq、VBUS；6 floats＋有效位 |
| 3 | 50 | M1 的前五项，M2 的前五项，VBUS、协同误差；12 floats＋有效位；双独立的协同误差无效 |
| 4 | 38 | HSO：目标速度、控制参考速度、滤波控制反馈、观测速度、Iq、VBUS、角差、观测磁链、机械相对位移；9 floats＋有效位 |
| 5 | 26 | 六步：目标、估算速度、参考速度、占空比 ratio、过零残差 V、VBUS；6 floats＋有效位；RUN 前估算实际速度无效 |
| 6／7 | 30 | FOC1／FOC2：Iq／Uq 命令、Iq、Id、U 相电流、V 相电流、反馈速度、VBUS；7 floats＋有效位 |
| 8／9 | 48 | E1／E2 整数诊断，见下文 |
| 10 | 44 | HSO 专题诊断，见下文 |
| 11 | 38 | 六步专题诊断，见下文 |
| 16 | 96 | 状态、最近命令结果、辨识结果及文字，见下文 |

类型 8／9：`8*u32 + 7*u16 + 2*u8`。u32 为 errors、CRC、Field、Track、SPI、Frame、ZeroStart、ZeroEnd；u16 为 Age、MaxAge、ZeroBefore、ZeroAfter、ZeroCount、ZeroAge、RawAngle；u8 为磁场 status、last_error。计数保持整数精度，不经过 float。

类型 10：`7*f32 + 3*u32 + 4*u8`。float 为参考磁链、捕获角差、捕获频差、反电势比、相位漂移、电压偏置、偏置限制；u32 为资格样本数、坏样本数、捕获重置数；u8 为入场拒绝、捕获拒绝、位置有效、usable。

类型 11：`f32 + u32 + 13*u16 + 4*u8`。float 为启动反电势；u32 为累计过零数；u16 为 phase_pct、period_pct、period_fraction、startup_ms、sector、quality、gate、六个旧→新候选周期；周期单位 10 µs。最后四字节为 state、sampling_off、speed_control、reject。

### 状态 payload

前 16 个 u8：Work、M1 Drive、M1／M2 ControlMode、M1／M2 Input、M1／M2 Flags、HSO State、HSO Reason、Six State、Six Reject、采集选项、E1／E2 健康位、control_result、hso_result。最后两个按 i8 解释。

每轴 Flags 位 0～7 分别为 ready、反馈健康、aligned、请求输出、实际采用电流环、aligning、功率输出已使能、轴启用。反馈健康在 HSO 下不是 E1 健康；E1／E2 编码器健康另报。

采集选项 bit0 为 ESC Auto，bit1～4 为采集组，bit5 为六步 Speed 类型，bit6 为 VBUS 新鲜，bit7 为 HSO 相对位置有效。M2 固定编码器，故不重复上传 M2 Drive。

接下来的四个 u32 为最大控制耗时 us、已完成辨识的 PP／Rs 编码、Ls µH、磁链 µWb。PP 为第一个编码的有符号低字节，Rs 为 bits[23:8] 的 mΩ。未完成时仍遵循原辨识 Result=0 规则，不自动写入预设。

最后是四个 16 B 字符串：M1 Drive、M2 Drive、协调器命令状态、辨识阶段。每项最多 15 个 ASCII 字符，末尾 NUL 并补零；详细故障仍可看 LCD 诊断或专题包。

结果码沿用原接口：control_result 为 1 接受、0 配置排队、-1 拒绝、-2 格式／参数错误、-3 忙；HSO 结果仍为 1 接受、-1 未知、-2 参数错误、-3 当前不能执行。不是逐条带命令 ID 的可靠应答协议；多来源同时发命令时只能看到最近结果。

## 速率与兼容性

主循环每经过至少 10 ms 产生一个发送时隙，不补发积压的历史帧。状态首帧、100 ms 心跳或变化优先，变化最多每 20 ms 合并一次。稳定时通常约 90 Hz 常用曲线＋10 Hz 状态；专题诊断约 10 Hz。`Stream:Off` 保留状态和心跳。两种传输各自在忙时跳帧，电脑端按序号发现缺口。

完整帧长度已经包含版本、时间、序号、有效位、CRC，和审查阶段只计算 JustFloat 的数字不同。正常稳定时，估算发送量如下，未包含额外状态变化及调度跳帧：

| 常用组 | 曲线包长度 | 90 Hz 曲线＋10 Hz 状态，B/s | UART 理论发送占用 |
| --- | ---: | ---: | ---: |
| 编码器单轴／六步 | 40 B | 4700 | 40.8% |
| 双轴 | 64 B | 6860 | 59.5% |
| HSO，含新增位置 | 52 B | 5780 | 50.2% |

原 25 路为 10400 B/s、约 90.3%。减少传输不会按同比例减少 Flash；新增类型化协议与诊断组织实际占用了剩余空间。

原 JustFloat 接收器需更换解析配置，不能直接混用；文本运动／配置命令仍保留，来源归属、显式 M1／M2 轴、Stop、KEY1 和故障后手动启动规则不变。板端没有新增通信超时自动停机，也不因电脑端连接或切换采集组启动电机。

## 电脑端与测试

启动和操作见 [电脑端 README](../pc_tool/README.md)。提供 COM 口选择、自动／专题采集、按单位分区曲线、失效断线、实时状态／诊断、手动命令／全局停止、CSV 记录及最近历史回放。日志、曲线缓冲和页面状态位于电脑端；未增加板端堆分配。

已通过：

- 原电机主机回归：两轴、主从、FOC、HSO、六步及独立物理模型、辨识、采样／IBUS／全局停止、编码器／SPI2、协议和数字解析。
- HSO 正反向多机械圈跨零、无编码器输入、首次锁定零点、序列缺失后无效、重启归零、中断屏蔽状态保留测试。ST ARM 库以替身建模，不代表实物观测精度。
- 实际 C 组帧的全部 12 种包，USB／UART 字节一致、CRC、采集命令、Off 心跳、辨识期间通信、状态抖动不饿死全部曲线、UART 忙时活动缓冲不被改写、毫秒回绕。
- Python 对 C 导出字节流分包／粘包解析，逐字节破坏后的重同步，非法版本／长度，序号与时间回绕、重启、有效位／单位及大于 2^24 的整数计数；本地 HTTP API 和手动命令约束。
- 原页面 JavaScript 的 DOM／Canvas 模型测试：直播数据、无效值、切组清理、流式 CSV 的内存上限、历史回放拦截普通运动、显式 Stop 保留、初次打开不发送运动命令。
- UI／存储／真实 LCD 驱动、USB CDC、LL 中断与真实采样／保护集成回归；Debug／Release 干净构建及启动、ELF／BIN、205 个受保护文件与官方／核心编译命令审计。

```text
python tests/motor_host/run.py
python tests/pc_tool/test_wire.py
node tests/pc_tool/test_frontend.js
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/telemetry-r86
```

尚未烧录实板或连接实际 COM 口。Windows 界面自动化工具因无法可靠识别浏览器当前 URL 而停止，本轮没有完成真实浏览器的视觉／交互验证，未绕过该限制。上板后应分别验证 USB 与 USART 的长时间收流、Stop、忙时跳帧、参数辨识连续状态和 HSO 脱离编码器后的位移方向／重复性，并重新观察真实 `Ctrl Max`。

本轮基线、构建日志、真实 C 导出字节流和体积／保护边界记录保存在忽略目录 `build/telemetry-r86/`。最终 ELF／BIN／MAP 位于 `build/Debug/` 和 `build/Release/`。

最终 BIN SHA256：

```text
Debug   503c968b796df2ce47a8d373737a7bb06acd7c931ec345e857f88e7b568ed84d
Release 7083f483046ba7ecbbf1c6e2d83aea3d226e2ed97b656b39cf24bad88ff123c4
```
