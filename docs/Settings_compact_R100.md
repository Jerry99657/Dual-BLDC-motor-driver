# R100：合并设置校验与 UI 适配路径

固件 **UI-R100-SET**。在 R99 的独立设置服务基础上压缩用户代码，保留菜单、上位机、控制准入、保护、草稿及保存功能。ST 官方源码、KK_UI／KK_OLED 核心、CubeMX 初始化、编译选项和链接脚本均未修改。

## 改动

- 位置速度、速度上限和三种步长统一在 `AppSettings_StoreAxis()` 校验；`Apply()` 只组装候选值。校验及控制器受理均成功后才发布设置，限速降低时同步限制已确认目标。检查极端整数时不会先进行可能溢出的乘法。
- 全局操作的零值载荷共用一处检查，辨识启动继续单独检查测试电流。轴／key 地址校验、HSO／六步的电流环限制、DAC 阈值和硬件拒绝路径保留。
- 本地标量菜单事件使用业务 key 编码，沿用整数绑定的值指针，省去逐项重复提交代码。本地及上位机共用结果提示和设置回读，UI 绑定只承担编辑、显示及交互范围，业务服务仍可独立运行。
- 齿轮比和偏移、LED 亮度和开关各自提交对应 key；修改一个字段不顺带写入另一个 UI 缓存值。
- 型号切换后的目标清零、默认速度／位置速度、电流环偏好及选项发布由控制层的 `clear_saved_axis()` 完成一次。设置服务不再重复读回、重置及写回；本地 UI 同样只回读，并同步型号标记，避免下一个 UI 更新再次重置。
- 启动、恢复默认及型号切换共用停止视图的重建代码；未确认 Run 草稿和已确认设置仍分开，远程标量回读不采用运动目标覆盖草稿。

内部事件编号变化不影响协议。RPC schema 2、App key 0～17、21 个设置读回字段、W25Q64 v5 记录及诊断行号均保持。单文件 HTML 兼容上限更新为固件 100。

## 实测资源

| 构建 | R99 Flash | R100 Flash | 减少 | 剩余 | RAM 总分配 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 130232 B | 129728 B | 504 B | 1344 B | 29424 B |
| Release | 130232 B | 129728 B | 504 B | 1344 B | 29424 B |

RAM 不变，含原堆栈预留。相对分离前 R98 的 129136 B，当前仍多 592 B；本轮收回了 R99 增量的一部分，没有宣称消除全部分层成本。尝试共享完整轴 DTO 的方案收益很小且增加 RAM，未采用。

原 R99 ELF／BIN／MAP 和用户源文件基线保存在忽略目录 `build/settings-compact-r100/`；`final.json` 核对 211 个受保护文件及官方／Core 编译命令完全一致。Debug／Release 新产物已更新，尚未刷板。

## 验证

- `python tests/settings_host/run.py`：独立设置业务、两轴边界与发布拒绝、零载荷操作、静默／强制／延期保存；新增 INT32 极值拒绝、失败不写 RAM／不标脏，以及限速目标夹取。
- `python tests/ui_host/run.py`：菜单、按键、Run 草稿、HSO／六步、型号切换、存储及 LCD 原回归；新增真实编辑器的 LED 两字段及齿轮比／偏移分别提交。R99 跨轴草稿、仅保存确认目标、重建 UI 保留待保存及无 LCD 路径继续通过。
- `python tests/motor_host/run.py control foc sensorless detect protocol telemetry`：实际控制器／FOC／HSO／辨识／协议／遥测；新增两轴两型号的控制层默认值发布、清零、步长与另一轴隔离检查。
- `python tests/diag_host/run.py`、`python tests/diag_host/check_arm.py --both`：首次故障／保留记录，以及两个实际 ELF 各 139 个 CPU 异常关断／栈／下载期签名场景。
- `python tests/cubemx_host/run.py`：34 个配置／DMA／KEY1 映射回归；正常构建继续执行生成配置检查。
- `node tests/pc_tool/test_console.js`、`python tests/pc_tool/test_wire.py`：真实 C 协议数据与单文件上位机，5 项传输测试。
- `python tests/motor_host/check_startup.py`、`python tests/flash_report.py final --directory build/settings-compact-r100`：初始化及中断路径、ELF／BIN 一致性、实际资源与保护边界核对。

板上建议复测一次 Settings 的限速、步长、LED 和齿轮参数确认／取消／掉电保存，再切换 2804／4015／ESC，核对目标清零、步长保留和没有自动运行。无需重新配置 CubeMX。
