# R122：低频用户函数编译策略实测

固件 `UI-R122-SIZ`，Native 配置回执固件编号为 122。本轮落实第 4 项，保留功能及 ST 官方库，只测量低频用户函数的编译属性。无需修改 CubeMX。

## 实际资源

| 配置 | R121 Flash | R122 Flash | 净回收 | 当前 Flash 余量 | RAM 总分配 | RAM 余量 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Debug | 130736 B | **130624 B** | **112 B** | **448 B** | 30920 B | 1848 B |
| Release | 130736 B | **130624 B** | **112 B** | **448 B** | 30920 B | 1848 B |

数字来自实际链接和连续 BIN 镜像，包含 `.data` 的 Flash 初值副本、RAM 中执行函数的加载副本及对齐。RAM 总分配包含现有堆、栈和保留诊断区，并非运行时栈高水位。

本轮没有释放数 KiB；结合 R120 的 212 B 和 R121 的 112 B，第 1～4 项累计回收 **436 B**。

## 保留的编译属性

1. `ui_app.c` 的 `init_ui_resources`、`init_info_rows`、`init_menu_items`、`reload_axis_views`、`seed_run_drafts` 移除单独的 `noinline,noclone` 限制，由现有编译器/LTO 决定是否内联或克隆。文件原有的 `-Oz -flto -fno-inline-functions-called-once` 保留。独立实测净省 16 B。
2. `app_storage.c` 的 `settings_read`、`AppStorage_Init`、`AppStorage_Load`、`AppStorage_ResetDefaults` 标为 `cold`。这些是启动、加载、恢复默认参数路径；运行时保存状态机、SPI2 收发及超时规则继续使用原策略。独立实测净省 64 B。
3. `ui_text.c` 的 `UiText_Fixed`、`UiText_Format` 标为 `cold`，保留它们原有的内联/克隆限制。文本格式化由前台调用，相对于电机控制环执行频率较低。独立实测净省 32 B。

`cold` 是 GCC 的优化提示，通常让编译器偏向体积及低频路径布局；它不改变调度频率、函数调用条件或中断优先级。本次三项组合后实测净省 112 B，恰好等于单项之和；其他组合不保证可相加。

只改变上述属性、说明注释及固件版本。去除编译属性/注释、规范化版本文字后，三个用户 C 文件的代码标记与基线完全一致；`protocol.c` 只修改配置回执版本 121→122。字体、图标、页面、控制参数、保存结构和通信字段完整保留。R122 的 About 资源重新打包，原始 3704 B、压缩 2334 B，版本更新未增加固件体积。

FOC、HSO、测速、控制中断、SPI/DMA 驱动、CubeMX 生成文件及 ST 官方库均未改动。根 CMake 编译/链接选项也未改动；207 个受保护文件及官方/核心编译命令通过基线散列核对。

## 全部试验

共测量 23 组单项和 1 组组合。每次以相同的 R121 源码为起点完成 Debug 链接；最终保留的组合另行构建 Debug/Release。表内正数表示 Flash 增大，负数表示减小。没有收益和增大的方案全部撤回。

| 试验 | Flash 变化 | 结果 |
| --- | ---: | --- |
| 命令解析辅助函数解除 noclone | 0 B | 撤回 |
| 命令解析辅助函数解除 noinline/noclone | +16 B | 撤回 |
| 文本格式化解除 noclone | 0 B | 撤回 |
| 文本格式化解除 noinline/noclone | +32 B | 撤回 |
| 五个 UI 初始化/草稿函数解除 noinline/noclone | -16 B | 保留 |
| UI 数值访问/格式化辅助函数解除内联限制 | +24 B | 撤回 |
| AppSettings_Apply 解除内联限制 | 0 B | 撤回 |
| UI 初始化标 cold | 0 B | 撤回 |
| 命令处理标 cold | +16 B | 撤回 |
| 设置应用标 cold | +16 B | 撤回 |
| 存储加载/初始化标 cold | -64 B | 保留 |
| 文本格式化标 cold | -32 B | 保留 |
| refresh_info 使用函数级 Os | 0 B | 撤回 |
| 自定义页面绘制使用函数级 Os | 0 B | 撤回 |
| 自定义页面输入使用函数级 Os | 0 B | 撤回 |
| 文本格式化使用函数级 Oz | 0 B | 撤回 |
| settings_read 使用函数级 Os | 0 B | 撤回 |
| handle_line 单独使用函数级 Oz | +96 B | 撤回 |
| 五个 UI 初始化/草稿函数仅解除 noclone | 0 B | 撤回 |
| 三项有效改动组合 | -112 B | 保留 |
| UI 配置同步标 cold | 0 B | 撤回 |
| 诊断行查询标 cold | 0 B | 撤回 |
| 保存请求等路径标 cold | 0 B | 撤回 |
| 协议回执构建标 cold | 0 B | 撤回 |

这些结果也说明，已有 `-Os/-Oz/LTO` 的工程不能仅根据函数大小或优化等级推断收益。以最终加载进 Flash 的完整镜像为准。

## 验证

- Debug/Release 正常构建成功，均执行 CubeMX 配置检查；ELF/BIN 一致性通过。
- UI 与基线对照：**68 份 framebuffer 截图/动画流逐字节一致**，**64857 次文字绘制内容、坐标、顺序一致**，交互/刷新回归输出一致。比较统一了 About 固件文字。870 条 UI 诊断查询逐字一致，仅规范化固件标识。
- 文本格式化边界测试、存储异步保存/日志、A/B 恢复、写入故障与历史版本迁移、LCD/DMA 回归通过。
- 两轴设置、主从/启停、命令优先级/过期/分片、USB/USART、Native/JustFloat 和编码器零码/重连回归通过。
- 实际 W25Q64/ENC2 仲裁与 Single M1 + HSO 保存/加载、BUSY/WEL/超时及重试回归通过。
- 4030 条实际诊断格式化输出与基线逐字一致，覆盖边界、无效浮点数、CPU/全局停止及两轴。
- 两个实际 ARM 镜像的启动/MSP/Break/KEY1/IRQ 检查、各 139 项 CPU 异常保护，以及两轴外环超期保护指令仿真通过。
- HTML 兼容上限更新至 122；前端回归和真实 Edge 的模拟 Web Serial/Streams 格式握手、切换、Stop 回归通过。

未烧录开发板。上述结果来自主机回归、绘图缓冲对照和 ARM 链接指令仿真，未测量实板 `Ctrl Max` 或前台耗时。ELF/BIN/MAP 已更新到 `build/Debug/` 与 `build/Release/`。

## 复核与记录

本轮基线、逐项候选源码、构建日志和 24 条试验结果保存在被忽略的 `build/cold-compile-r122/`；`baseline.json` 与 `final.json` 记录资源、散列及受保护文件检查。

```powershell
python -X utf8 tests/ui_host/run.py --view-baseline build/cold-compile-r122/mycode/ui_app.c --formatter-baseline build/cold-compile-r122/mycode/ui_app.c
python -X utf8 tests/settings_host/run.py
python -X utf8 tests/motor_host/run.py control protocol telemetry justfloat encoder
python -X utf8 tests/diag_host/run.py --formatter-baseline build/cold-compile-r122/mycode/board_diag.c
python -X utf8 tests/motor_host/check_startup.py
python -X utf8 tests/diag_host/check_arm.py --both
python -X utf8 tests/diag_host/check_outer_arm.py
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/flash_report.py final --directory build/cold-compile-r122
```
