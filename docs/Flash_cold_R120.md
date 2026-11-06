# R120 诊断格式化与命令解析体积优化

固件标识为 `UI-R120-SIZ`，Native 配置回执中的固件编号为 120。本轮落实诊断格式化、命令解析两项优化。以当前 R119 固件为基线，保留源码、ELF／BIN／MAP 和测量报告于 `build/flash-cold-r120/`。

## 实际资源

| 阶段 | Debug Flash | Release Flash | 剩余 Flash | RAM 总分配 |
| --- | ---: | ---: | ---: | ---: |
| R119 基线 | 131060 B | 131060 B | 12 B | 30920 B |
| 诊断格式化 | 130984 B | 130984 B | 88 B | 30920 B |
| 命令解析及 R120 标识 | **130848 B** | **130848 B** | **224 B** | **30920 B** |

诊断阶段净回收 76 B，加入协议改动后再回收 136 B，合计 **212 B**。这次没有达到原定回收 512～1024 B 的目标。各阶段包含整个 LTO 链接结果，不能将单个源码改动的收益简单相加。RAM 数字包括原有堆、栈和诊断保留区域。

## 保留的改动

### 诊断格式化

- `BoardDiag_Row()` 的母线／参考电压、目标／反馈速度、目标／实际位置共用一个有符号成对数值格式化出口。原 milli 换算、截断、单位和 `INVALID` 条件继续逐行对照。
- CPU B/M 行直接格式化 BFAR／MMFAR，省去同一缓冲区先写 BFAR、再被成对结果覆盖的操作。
- UI 的轴选择小函数使用共享调用入口。其参数仍由同步诊断查询上下文或本地视图决定。

### 命令解析

- 命令前缀与长度存为连续描述项。每个长度字节紧邻其前缀，遍历使用一个指针。命令拼写、顺序、数量、长度和描述项大小都由同一份清单及编译期检查约束。
- 前缀数组明确标记为非 NUL 结尾字节串，仅通过有界比较访问。截断命令和相似前缀继续执行原拒绝规则。
- `App`／`GetRow` 在验证 float 参数属于 ±100000 且可精确转换为整数后，保存转换结果。后续行地址校验、应用调用和回执地址直接使用整数，避免重复浮点转换与浮点范围比较。
- 小数、科学计数、十六进制数和负零仍经过原 `Number_Parse()`。非法请求在设置发布前被拒绝。具体轴／key 与设置业务策略由原应用服务处理。
- HTML 的固件兼容上限更新为 120，About 字模资源重新打包以显示新版本。

设置服务经过 R100 等版本的合并后，本轮没有找到额外净收益，因此最终实现集中在协议适配路径。ST 官方源码、生成配置、编译选项、链接容量及控制算法均与本轮基线一致。207 个受保护文件及其官方／核心编译命令通过散列核对。

## 撤回的试验

下表为相对于各次试验起点的完整链接变化，正数表示增大。

| 试验 | Flash 变化 | 处理 |
| --- | ---: | --- |
| 再增加通用整数诊断包装器 | +8 B | 撤回 |
| `handle_line` 与 `AppSettings_Apply` 改为函数级 Oz | +104 B | 撤回 |
| 将 Stop 提前、延迟读取工作模式 | +16 B | 撤回 |
| 把诊断所有分支合为稠密 switch | +32 B | 撤回 |
| 重组数值描述表分派 | +16 B | 撤回 |
| 合并普通目标值格式化调用 | 0 B | 撤回 |

## 验证

- 新增真实诊断格式化对照，**4030 条**名称与数值逐字相同，覆盖两轴、空记录、CPU／全局停止、32 位计数、正负边界、NaN／Inf 和输出缓冲边界。
- UI 与诊断查询回归通过，**870 条**UI 查询结果逐字相同，仅规范化固件标识。菜单、按键、草稿、设置、异步存储和 LCD 驱动回归通过。
- 扩展 `App`／`GetRow` 的整数边界、分数、负地址、科学计数和十六进制参数测试。非法请求同时检查设置适配器未被调用。
- 前后协议实现通过同一回归，**159264 B** 的数值目标轨迹逐字节相同。对照基线只规范化固件回执编号，真实的命令处理逻辑使用基线副本。
- FOC、双轴／Follow、独立 HSO、参数辨识、队列／请求过期、优先 Stop、USB／USART 遥测和 Native／JustFloat 回归通过。
- HTML 前端与实际 Edge Web Serial／Streams 模拟板端握手、格式切换和 Stop 回归通过。
- 52 项 CubeMX 配置变化及 DMA／KEY1 重映射回归通过。Debug／Release 正常构建执行生成配置检查，启动／MSP／Break／IRQ 及 ELF／BIN 一致性检查通过。
- 两个实际 ARM 镜像的 CPU 异常保护和外环超期保护指令仿真通过。

开发板尚未烧录。本轮变更不要求调整 CubeMX。构建产物继续保存于 `build/Debug/` 和 `build/Release/`。

常用复核命令如下。

```powershell
python -X utf8 tests/diag_host/run.py --formatter-baseline build/flash-cold-r120/mycode/board_diag.c
python -X utf8 tests/ui_host/run.py --formatter-baseline build/flash-cold-r120/mycode/ui_app.c
python -X utf8 tests/settings_host/run.py
python -X utf8 tests/motor_host/run.py control foc sensorless detect protocol telemetry justfloat
python -X utf8 tests/cubemx_host/run.py
python -X utf8 tests/motor_host/check_startup.py
python -X utf8 tests/diag_host/check_arm.py --both
python -X utf8 tests/diag_host/check_outer_arm.py
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/flash_report.py final --directory build/flash-cold-r120
```
