# R131：共用配置入口与双轴视图同步

对应固件 `UI-R131-CFG`，Native 配置回执编号 131。实施 [架构审查](Flash_architecture_audit_R129.md) 第 3、4 项，在 R130 的 A/B 加载及重复发布精简基础上继续整理用户业务代码。

## 实际资源结果

Debug、Release 使用本轮基线相同的工具链及选项，全量构建结果一致：

| 项目 | R130 | R131 | 净变化 |
| --- | ---: | ---: | ---: |
| Flash 镜像 | 130472 B | 130440 B | 减少 32 B |
| Flash 剩余 | 600 B | 632 B | 增加 32 B |
| RAM 总分配 | 30912 B | 30912 B | 0 B |
| RAM 区剩余 | 1856 B | 1856 B | 0 B |

本轮体积收益很小。新增统一入口的分发、结果处理及草稿保护抵消了大部分回收，不能把删掉的源代码行数或函数尺寸当成净节省。第 1～4 项相较 R129 累计回收 176 B，仍不构成大量新增功能的空间。

首轮只改为按轴同步后，Debug 为 130456 B；接入统一入口的初版为 130552 B，比基线大 80 B，未作为交付结果。随后复用确认值结构、合并初始化／默认值／配置刷新、取消重复型号判断，并保留油门草稿和停止提示处理，最终为上表数值。这些链接差值包含 LTO、常量和对齐影响，不能逐函数相加归因。

## 第 3 项：两个前端走同一条业务路径

新增固定参数接口，无堆分配或通用回调框架：

```c
bool App_Configure(AppConfigAction action, MotorAxis axis,
                   int32_t value, int32_t extra);
unsigned AppSettings_Configure(AppConfigAction action, MotorAxis axis,
                               int32_t value, int32_t extra);
```

- UI 菜单事件与 `Work:`、`Motor1/2:`、`Feedback:`、`Gear:`、`Align:`、`Focus:` 的串口解析均调用 `App_Configure()`。
- `AppSettings_Configure()` 执行业务动作、调用原控制器准入检查、标记保存，并返回接受／重置该轴的结果。正常换型号才清该轴草稿；选择相同型号保留运行状态，不重复标记保存。
- `App_Configure()` 根据结果同步视图、显示提示及读回当前控制状态。去掉串口成功后的第二次 `App_ConfigChanged()`；数值设置继续走已有的 `AppSettings_Apply()`。
- 齿轮命令的比例、偏移仍作为一对原子业务参数提交。`value`、`extra` 为原有整数单位，Gear 分别为比例 ×100、偏移 ×100。
- 内部轴号仍为 `MotorAxis` 的 0／1；线上 `App:` 的全局 0、M1 1、M2 2 约定及所有旧文本命令不变。反馈和六步动作只允许物理 M1。

`App_SelectMotor()`、`App_ConfigChanged()` 和 `AppSettings_SelectMotor()` 兼容接口仍保留，但新配置路径不会再借助临时切换 UI 轴来执行换型号，也不重复发布已接受的修改。

安全边界继续独立：

- 串口接收仍只排队；原有队列取消、命令来源、500 ms 过期和全局停止检查保留。
- 远程对齐保留 `Protocol_RequestCurrent()`、停止状态下的 10 ms 新鲜度和关中断提交。对齐动作不触发设置保存，也不在该临界区执行完整 UI 格式化。
- 原控制器仍决定停机、速度、对齐、当前来源等条件是否允许动作；拒绝时不换参数、不启动输出。
- KEY1、外环保护、电流／IBUS 保护和硬件 Break 均未改动。

## 第 4 项：确认值来源与按轴同步

### 限幅字段的纯解码

`motor_limits.h` 共用 Speed Limit、Pos Speed 的旧版字段解码与默认值规则：Speed Limit 默认 100，Pos Speed 默认 30，零值／非法字段回退规则不变。

设置服务保留整数范围归一化；运行控制保留浮点电机上限及位置速度上限检查。控制环仍读取实时发布的选项，没有增加长期缓存或减少最终输出检查。没有改变 600 rad/s 的现有配置上限。

### 确认值与草稿分离

`AxisView` 直接持有 `AppAxisSettings`，删除提交／读回时逐字段拼装同一数据的过程。Run 速度、位置、电流、相对原点及运行状态继续独立保存，编辑器仍绑定静态变量地址。ARM 上每个视图仍为 64 B，两轴缓存和当前视图的 RAM 预算保持原值。

步长在设置／存储中使用 Speed、Torque、Position 顺序，Run 页面使用 Torque、Speed、Position 顺序。转换统一在明确的模式索引映射中完成，不改变菜单顺序、单位或步长含义。

### 按轴刷新与初始化共用

- `sync_axis_view(axis, view, reset)` 明确指定目标轴和目标缓存；更新另一轴时不再临时修改 `s_view_axis`。
- `sync_views(reset_axes)` 共用于启动、恢复默认和配置变更。只有需要重置的轴才重建停止草稿，最后一次载入实际焦点并更新编辑器范围。
- 降低 Speed Limit 仍限制相应速度草稿；位置速度最大值跟随 Speed Limit；切换电流环会清除旧的电流草稿。
- 外轴／全局设置刷新保留未确认的六步油门，显式保存也不会把该草稿当成确认值。真正重置 M1 型号、恢复默认或重新进入初始化时才相应清空／重建。

## 验证与交付

- 当前 UI 与 R130 渲染器对照：68 份图像／帧流、64836 条文字绘制记录一致；版本标记使用同一当前字体内容归一化。
- 新桥接回归：跨轴型号／焦点操作保留 Run 草稿，确认值才进入保存，相同型号不打断运行；工作模式、齿轮、非法轴、对齐拒绝正确传回。无 UI／无模型初始化时可执行配置业务，不自动运行或立即写 Flash。
- 新油门回归：另一轴及全局设置不清掉 M1 未确认油门，不错误持久化；M1 实际型号重置会清空油门草稿。
- 5120 组实际控制指令与 R130 对照，目标输出／选项逐位一致；速度限幅求值 6400 → 6400，位置速度求值 2560 → 2560，没有增加求值次数。对照工具现在要求次数不增加，也适用于保持次数不变的结构整理。
- 实际控制器、Follow／motion、独立 HSO、协议／JustFloat／Native、保存与旧版本迁移、LCD／六键输入回归通过。解析夹具额外验证传入共用入口的动作、轴和单位，停止后过期的对齐命令不会进入业务入口。
- 两套 ARM 镜像的外环超时保护与 TIM2 六键执行路径检查、启动与 ELF／BIN 一致性检查通过。主机的指令计数不是板上实测微秒。
- HTML 扩展兼容上限至固件 131；真实 C 回执、Edge 格式握手与串口恢复回归通过。
- 207 个受保护文件及官方／Core 编译命令相对本轮基线一致。未改 ST 官方库、CubeMX／DMA／NVIC 配置、编译容量、控制算法或保护阈值。

主要复现命令：

```powershell
python tests/ui_host/run.py --view-baseline build/config-paths-r131/mycode/ui_app.c
python tests/motor_host/run.py protocol justfloat telemetry control motion sensorless
python tests/motor_host/check_limits.py --baseline build/config-paths-r131/mycode/motor_control.c
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/flash_report.py final --directory build/config-paths-r131
python tests/motor_host/check_startup.py
python tests/diag_host/check_outer_arm.py
python tests/ui_host/check_input_arm.py --baseline build/keys-r129/Debug/FOC_two.elf
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/pc_tool/test_serial_recovery_browser.py
```

基线、分步结果、日志及最终尺寸报告位于忽略的 `build/config-paths-r131/`。标准 Debug／Release ELF／BIN／MAP 已更新。无需调整 CubeMX；尚未连接实板或烧录。
