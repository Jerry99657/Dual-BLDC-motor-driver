# R96：减少诊断查询开销

固件 **UI-R96-QUERY**，完成第 4 项。所有诊断字段、八个专题、行号、USB CDC／USART 协议与现有界面保留。无需重新配置 CubeMX；本轮未修改 ST 官方库、Core、KK_UI／KK_OLED 核心、初始化、`.ioc` 或编译选项。

## 实现

原 `App_ReadRow()` 每读一行，需要临时载入指定轴的 UI View、完整刷新所有文字，再恢复 View 和编辑变量并完整刷新一次。查询 HSO 时也会读取六步、ADC、两路编码器、Flash 等无关信息。

现在直接以请求的轴号和专题调用共用格式化逻辑：

- Drive、Motion、Encoder、ADC、HSO、Diagnostics、SixStep 只读取该专题需要的 RAM 快照，About 和 HSO 固定策略文字直接复制。
- 数值描述表只转换所请求的数值字段；专题内其他文字写入栈上的 32 B 临时缓冲。LCD 仍使用相同格式化函数及原有长期文本缓冲。
- 查询不切换／恢复 View，不重建菜单标题或标记，不更新位置速度绑定、Run 草稿、Settings 变量或保存状态。修复旧查询会覆盖 IBUS 编辑变量的问题。
- 编码器名称直接标明请求的 E1／E2；不修改 LCD 的编码器名称。HSO／SixStep 专题仍描述物理 M1，E2 Probe／Total 仍描述物理编码器 2，与旧定义一致。
- 远程 Torque 单位来自实际电流环状态，HSO 最大速度来自已生效的 M1 限速；本地未确认的编辑值不会影响诊断读回。
- 沿用 R95 的 26 行扩展诊断直接查询路径，以及现有首次故障／CPU 异常记录布局；Reset 保留的旧记录不会仅因版本升级而失效。

没有引入页面缓存、刷新 TTL 或堆分配。每行读取时取当前快照，因此整页仍为逐行取样，不保证同一瞬间。现有时间戳、分页、返回值、重试和清除条件不变；查询在主循环同步执行，不供中断调用。

## 对照结果

使用同一主机夹具分别编译 R95 基线与 R96 实现，统计从应用格式化层发起的快照 getter 次数，不包含真实 getter 内部的嵌套调用。每组以第 0 行为样本：

| 专题 | R95 M1／M2 | R96 M1／M2 |
| --- | ---: | ---: |
| Drive | 16／17 | 2／2 |
| About | 16／17 | 0／0 |
| Motion | 16／17 | 1／1 |
| Encoder | 16／17 | 3／4 |
| ADC | 16／17 | 1／1 |
| HSO | 16／17 | 1／1 |
| Diagnostics 原有行 | 16／17 | 1／1 |
| SixStep | 16／17 | 1／1 |

HSO 固定策略行不读快照；R95 新增的扩展诊断行继续走直接读取路径。以上是可复现的调用次数，不能当作实板微秒数或 CPU 百分比。

| 构建 | R95 Flash | R96 Flash | 增加 | 剩余 Flash | RAM 总分配（含原堆栈预留） |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 128252 B | 128828 B | 576 B | 2244 B | 29408 B |
| Release | 128252 B | 128828 B | 576 B | 2244 B | 29408 B |

两种构建均可容纳，所以按本次要求暂未开展额外 Flash 压缩。静态 RAM 总分配与基线相同，剩余 3360 B；查询只新增上下文指针和短生命周期的栈上上下文。

## 验证与产物

- `python tests/ui_host/run.py --query-baseline build/diag-query-r96/mycode/ui_app.c`：同一时刻，两轴、三种控制模式、全部信息行与真实本地显示格式逐项一致；交叉查询不改变全部文本、标题、绑定、设置、焦点、编辑框和保存请求。覆盖即时数值更新、E1／E2、M2 派生 W 电流、固定策略、实际电流环单位、未发布限速及 IBUS 草稿；包含原 UI／存储／LCD 回归。
- `python tests/motor_host/run.py protocol telemetry`：真实协议与遥测，包含 USB／USART、回执、队列、Stop、超时和 CRC。
- `node tests/pc_tool/test_console.js`、`python tests/pc_tool/test_wire.py`：单文件 HTML 兼容 R96 和旧版本，设置／诊断／保存／安全导入及原通信功能。
- `python tests/diag_host/run.py`、`python tests/diag_host/check_arm.py --both`：真实诊断器及两个 ELF 的异常关断／保留记录，各 127 个 Thumb 仿真场景。
- `python tests/cubemx_host/run.py`：34 项配置与 DMA／KEY1 改映射回归。正常构建继续执行生成配置检查。
- `python tests/motor_host/check_startup.py`、`python tests/flash_report.py final --directory build/diag-query-r96`：ELF／BIN 一致性、211 个受保护文件与官方／Core 编译选项核对。

Debug／Release 的 `.elf`、`.bin`、`.map` 已更新。对照源码、大小报告和日志在忽略的 `build/diag-query-r96/`，调用统计在 `build/ui-validation/query-metrics-*.txt`。本轮未刷板，尚未测量运行中查询对实板主循环／显示响应的耗时改善；可在运行及本地 Settings 编辑时，用上位机连续读取 M1／M2 专题复测。
