# R130：设置加载与同一事务内的重复发布精简

对应固件 `UI-R130-CFG`，Native 配置回执编号 130。实施范围为 [R129 架构审查](Flash_architecture_audit_R129.md) 的第 1、2 项。保留现有功能和保护，仅修改用户代码；ST 官方库、CubeMX 配置、初始化、编译／链接选项没有改动。

## 结果

相同工具链及配置，Debug、Release 全量重编译结果一致：

| 项目 | R129 基线 | R130 | 净变化 |
| --- | ---: | ---: | ---: |
| Flash 镜像 | 130616 B | 130472 B | 减少 144 B |
| Flash 剩余 | 456 B | 600 B | 增加 144 B |
| RAM 总分配 | 30912 B | 30912 B | 0 B |
| RAM 区剩余 | 1856 B | 1856 B | 0 B |

Debug 分步测量：仅第 1 项时 130480 B，净减少 136 B；加入第 2 项后 130472 B，再减少 8 B。更新版本标记后体积保持该值。以上为完整链接镜像的差值，包含 LTO、常量合并及对齐影响，不代表被删除函数的尺寸。

收益主要是去掉重复工作、明确确认值的发布顺序；这两项没有释放大量空间，600 B 余量仍不足以支持大规模功能扩展。没有调整快速环／外环参数、放宽超时或保护条件。

## 1. 共用 A/B 选择，启动只加载一次

`app_storage.c` 新增内部 `settings_load_banks()`，供 `AppStorage_Init()` 与显式 `AppStorage_Load()` 共用。读取两份记录，校验后选择有效且较新的记录；有效序号相等仍选择 B，序号回绕沿用有符号差值比较。

两个入口的失败行为分别保留：

- Init：两份都无效时使用默认值，将 A 作为后续保存的基准地址；仍扫描日志。
- 显式 Load：设备就绪但两份都无效／读取失败时，保留已确认的 RAM 设置、有效标记和活动地址。设备未就绪仍提供默认值。
- Flash 探测最多三次，两次间隔各 2 ms；没有增加初始化期间的擦写。
- 版本 1～6 的迁移、CRC 校验、v7 结构、A/B 地址、保存序号、异步快照／验证／重试及日志格式保留。

正常启动先由 `AppStorage_Init()` 加载设置，再初始化电机控制和 UI。`ui_app_reset()` 现在只从已确认的 RAM 设置重建视图，不再触发 Flash 读取，也不会用旧的持久化副本覆盖等待延迟保存的已接受修改。需要主动重新读取时，仍可调用 `AppStorage_Load()`；该 API 没有删除。

实际 Debug／Release 反汇编中，`settings_read()` 调用点从四处减到两处，UI 初始化所在的 `main` 不再包含额外的 A/B 读取。这减少启动 I/O，未据此推断板上耗时。

## 2. 一次确认只发布一次

### M2 型号切换

`MotorControl_SelectProfile()` 的 M2 分支保留停止输出、选取参数、重置控制器；删除紧接着的中间 `save_config()`。成功后仍清除该轴旧目标，并由共同路径发布最终配置。

`save_config()` 只重建 RAM 配置字段，没有 Flash I/O。两个原调用位于同一关中断事务内；`reset_m2()` 与 `clear_saved_axis()` 不依赖被删除的中间配置。实际链接函数内的调用点由两处减为一处。

### 本地目标和油门

`ui_store_current()` 原先先通过 `AppSettings_StoreAxis()` 校验／限幅并写目标，又调用 `AppSettings_StoreTargets()` 重写传入的三个目标，附带保存油门。后一次原值复制有覆盖前一次限幅结果的隐患。

现在本地提交保留 `StoreAxis()` 的一次校验和目标写入；成功后仅更新 M1 六步油门。油门位更新共用 `AppSettings_StoreThrottle()`，限制仍为 0～100%，不改动三个目标或其他配置位。

远程运动仍由 `AppSettings_StoreTargets()` 保存控制器接受的目标，并使用同一油门更新实现。M2 更新不会修改 M1 油门。校验／选项应用失败时不提交目标、步长或油门；Run 的未确认草稿仍独立于确认值。

## 验证

- 实际存储状态机与异步 NOR 模型：A／B 单份及双份有效、CRC 无效、序号相等／回绕、单份读失败、显式重新加载失败保留 RAM／活动地址、后续写另一份、无有效记录默认值、三次探测上限；既有旧版迁移、保存／读回／验证失败、不可变快照和日志轮转通过。
- 实际 UI／设置业务：重建不调用 Load，已接受的修改及待保存状态保留，速度目标限幅、选项拒绝不改存储、另一轴和油门隔离；原有按键／长按／重复、显示恢复、页面／模式、对齐、参数辨识／Custom 和 headless 路径通过。
- 实际电机控制：预设／Custom 切换与启动恢复，M2 切换保留 M1 目标、焦点、IBUS 和油门；停止／急停／外环保护、编码器和共享 Flash 仲裁回归通过。
- 协议／JustFloat／Native 通过；HTML 接受固件编号扩展至 130，真实 C 回执、Edge 格式握手与串口恢复回归通过。命令及存储协议结构未变。
- 两套实际 ARM 镜像的启动／ELF-BIN 一致性、外环保护及 TIM2 六键路径检查通过。207 个受保护文件和官方／Core 编译命令与本轮基线一致。

主要命令：

```powershell
python tests/ui_host/run.py
python tests/motor_host/run.py control encoder protocol justfloat telemetry
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/flash_report.py final --directory build/config-dedupe-r130
python tests/motor_host/check_startup.py
python tests/diag_host/check_outer_arm.py
python tests/ui_host/check_input_arm.py --baseline build/keys-r129/Debug/FOC_two.elf
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/pc_tool/test_serial_recovery_browser.py
```

基线、分步尺寸、完整报告及链接调用证据位于忽略的 `build/config-dedupe-r130/`。标准 `build/Debug`、`build/Release` 的 ELF／BIN／MAP 已更新。无需修改 CubeMX，尚未连接实板或烧录；主机模型及反汇编验证不替代上板验证。
