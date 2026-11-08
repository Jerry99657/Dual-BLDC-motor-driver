# R132：第 5 项复合诊断行整理

对应固件 `UI-R132-FMT`，Native 配置回执编号 132。本轮覆盖编码器、HSO、六步三组共 9 个复合数值字段；沿用同一 LCD／GetRow 格式化实现，保留现有功能、显示内容和通信协议。

## 实测资源

Debug、Release 结果一致，与 R131 相比：

| 项目 | R131 | R132 | 净变化 |
| --- | ---: | ---: | ---: |
| Flash 镜像 | 130440 B | 130336 B | 减少 104 B |
| Flash 剩余 | 632 B | 736 B | 增加 104 B |
| RAM 总分配 | 30912 B | 30912 B | 0 B |
| RAM 区剩余 | 1856 B | 1856 B | 0 B |

没有扩大 Flash、修改 ST 官方库、CubeMX 配置、编译／链接选项或删减功能。第 1～5 项相较 R129 累计回收 280 B，仍属于小幅空间回收。

分阶段 Debug 实测：

| 阶段 | 镜像大小 | 处理 |
| --- | ---: | --- |
| 编码器零码 4 行 | 130360 B | 保留，较基线减少 80 B |
| 加入 HSO 3 行、六步 2 行 | 130348 B | 保留，再减少 12 B |
| 将复合格式化独立为函数 | 130428 B | 比上一步增加 80 B，撤回 |
| 去掉浮点格式表的无效填充 | 130336 B | 保留，再减少 12 B |

版本标记更新和编译期类型检查没有改变最终大小。以上是完整链接镜像的净差，包含 LTO 与对齐影响，不能当成源码行数或独占函数成本。

## 修改范围与依赖

`mycode/ui_app.c` 已有的 `ValueRow` 保持 3 B 编码：目标文本偏移、格式类型、快照成员偏移。普通数值行继续使用原类型。本轮增加复合类型，并将以下字段放入原有行描述表：

| 字段 | 快照与依赖 | 保留的规则 |
| --- | --- | --- |
| E1/E2 Zero | `MT6701Diagnostics` 的 count、age、before | 未发生时 `--`；NEAR／FAR／GAP 边界和计数不变 |
| E1/E2 ZPre | before、age | age 为 `UINT16_MAX` 时未知；不把合法 before 数值误当成 age 标记 |
| E1/E2 ZPost | after、count | after 为 `UINT16_MAX` 时事件尚未结束 |
| E1/E2 ZTime | after、start、end，必要时当前毫秒 | 已结束用 end，未结束用当前时间；保留无符号回绕计算 |
| H Qual | `StHsoSnapshot` 的 qualified_samples、bad_samples、state、reason | 样本数除以 10；学习／学习超时用 1000 ms，其他用 200 ms |
| H Flux | reference_flux_wb | 原有微韦伯换算、整数截断和单位不变 |
| H Time | max_control_cycles、overruns | 周期数除以 170 的截断及超期计数不变 |
| 6S ZC | `SixStepSnapshot` 的 quality、gate、phase、period、fraction | 空 gate 用 `-`，小数位仍取模 10 |
| 6S Hold | startup_ms | 原有 `/500 ms` 显示不变 |

`refresh_values()` 在读取复合字段的依赖并执行格式化前，先判断是否为当前请求字段。因此查询其他数值行时，也不会再为这些复合行计算并生成随后丢弃的文字。LCD 批量刷新仍生成全部所需内容。

这不是整组快照的惰性采集：本轮没有改变驱动快照 API 或其调用策略，部分其他文字仍保留原有按组处理。没有建立另一套上位机 formatter、动态字段对象、函数指针框架或长期缓存。

## 维护约束

- 复合格式与快照类型通过 C11 `_Generic` 在 `VALUE_ROW` 定义时检查；错误快照类型或未知格式直接构建失败。
- 描述大小、偏移编码范围及复合类型的区间保留静态断言。检查不增加运行时 Flash／RAM。
- 浮点格式表只包含实际浮点格式，增加复合类型不再自动增加未使用的零填充。
- 新增同类复合行时，应在对应快照的描述表及格式分支中明确依赖、单位、无效值和容量，并补对应边界用例。
- 所有处理仍在主循环。编码器驱动、DMA 仲裁、HSO／六步算法、控制中断、保护门限、按键与保存行为均未改动。

## 验证

1. R131 与当前版本的 870 行诊断查询逐字一致，固件标记归一化。
2. 68 份 UI 图像／帧流、64836 条文字绘制记录一致；图像对照中的旧渲染器使用当前版本标记和资源。
3. 新增 110 次边界读回检查，覆盖两轴、无零码事件、前后值未知、NEAR／FAR 边界、GAP、未结束事件、毫秒回绕、HSO 学习窗口／周期数截断／大计数，以及六步空 gate 和小数位。LCD 与 GetRow 一致，查询不改 UI／设置。
4. 只在主机测试副本中对 `UiText_Format()`／`UiText_Fixed()` 计数，24 次代表性查询合计从 174 次降到 116 次。完整 LCD 刷新两版均为 57 次普通格式化、33 次定点格式化；快照读取次数不变。该结果不代表整次查询或控制中断的实测微秒变化。
5. 错配快照／未知格式的负向编译测试通过；普通 UI、输入、设置、对齐、存储迁移及 LCD 驱动回归通过。
6. Native／JustFloat／遥测协议、HTML 与 Edge 格式握手／串口恢复、两套 ELF／BIN 一致性检查通过。
7. 207 个受保护文件和官方／Core 编译命令与本轮基线一致。

主要命令：

```powershell
python tests/ui_host/run.py --formatter-baseline build/diag-fields-r132/mycode/ui_app.c --view-baseline build/diag-fields-r132/mycode/ui_app.c
python tests/motor_host/run.py protocol justfloat telemetry
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/flash_report.py final --directory build/diag-fields-r132
python tests/motor_host/check_startup.py
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/pc_tool/test_serial_recovery_browser.py
```

基线、阶段尺寸、撤回试验及最终报告位于忽略的 `build/diag-fields-r132/`；逐查询工作量记录位于 `build/ui-validation/before-diag-work.txt`、`after-diag-work.txt`。标准 Debug／Release ELF／BIN／MAP 已更新，无需调整 CubeMX。尚未连接实板或烧录。
