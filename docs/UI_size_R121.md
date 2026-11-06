# R121 自定义页面绘制与输入体积优化

固件 `UI-R121-SIZ`，Native 配置回执编号为 121。以 R120 为基线完成第 3 项，优化 Run、Run Steps、Follow 和 Detect 的应用层绘制与输入。直接使用新版构建产物，无需调整 CubeMX。

## 实测资源

| 构建 | R120 Flash | R121 Flash | 净回收 | 当前余量 | RAM 总分配 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 130848 B | **130736 B** | **112 B** | **336 B** | 30920 B |
| Release | 130848 B | **130736 B** | **112 B** | **336 B** | 30920 B |

RAM 增量为 0 B，链接余量为 1848 B。分配量包含原有堆、栈与诊断保留区。结合 R120 的 212 B，第 1～3 项累计回收 324 B。函数大小受 LTO 共享与布局影响，下列分阶段值均来自完整链接。

## 保留的实现

1. **一次绘制内复用 Run 草稿**。在普通 Run 分支读取当前草稿和确认值，后续变更标记、主数值、已确认目标、辅助单位与操作提示共用这份值。草稿由主循环持有。
2. **共用页眉绘制**。Detect、Run Steps、手动 Follow 和普通 Run 使用应用层 `draw_header()`，处理标题和 y=14 的分隔线，调用方继续设置各自字体与参考高度。标题文字坐标、线条、宽度和传入裁剪范围通过画面对照。
3. **只格式化当前辨识结果项**。Detect 根据当前页选择极对数／电阻、Ls、磁链、Ke 或应用／Custom 操作名称，省去先生成电阻文字再被其他结果覆盖的工作。缺数据判定和 Ke 的原有换算顺序沿用。
4. **共用视图运行标记清理**。当前视图及两轴缓存视图的运行标记通过同一函数清除。原来各调用位置的控制器停止、输入来源检查和对齐条件继续执行。
5. **复用油门判断与有效限值选择**。一次上／下输入共用油门判断，按当前模式读取需要的速度、位置、电压或额定电流限值。重复按键与编码器批量输入使用相同草稿修改路径。
6. **合并 Run Steps 上下选择**。上下动作共用三项循环选择逻辑。新增实际页面测试，将批量输入后的编辑结果保存到对应模式设置，与逐个消费物理步数的独立预期比较。

字体、字号、图标、文字、按键定义与页面功能完整保留。KK_UI／KK_OLED 核心及显示驱动由既有接口提供。ST 官方源码、CubeMX 生成配置、编译选项和真实链接容量经基线核对一致。

## 对照构建

| 累计改动 | Debug Flash |
| --- | ---: |
| R120 基线 | 130848 B |
| Run 草稿读取复用 | 130832 B |
| 四种页眉共用 | 130800 B |
| Detect 结果格式化分支 | 130792 B |
| 视图运行标记清理共用 | 130768 B |
| Run 输入判断／限值复用 | 130752 B |
| Run Steps 上下选择合并、R121 标识 | **130736 B** |

左侧文字包装函数增大 16 B、集中 Run 弹窗增大 32 B、重组状态行绘制增大 40 B，均已撤回。合并 Detect 动作提交路径没有收益，也已撤回。

## 验证

- **68 份实际 framebuffer 截图／动画流逐字节一致**，**64857 次文字绘制的内容、坐标和顺序一致**。同一套交互及刷新回归输出一致，包含 Run 确认／取消、长按连发、长按切轴、KEY2 返回／停止、双机、手动 Follow、HSO／六步、辨识与 Custom。
- 画面对照将两份应用的固件标识统一为当前 R121，并使用相同打包资源，以包含 About 在内的全部图片进行比较。测试先检查文字清单、字体、图标和动态标签源的一致性。R120 原始固件及资源副本保留在基线目录。
- Run Steps 新测试包含起始三行、上下两个方向以及 0／1／2／3／4／17／1000／65534／65535 步，实际确认保存后逐项检查三种模式的步长。
- **870 条诊断查询**与 R120 一致，仅规范化固件文字。设置、存储 A/B、异步 LCD／DMA 和边缘映射回归通过。
- 真实控制器与协议、队列、USB／USART、Native／JustFloat 回归通过。HTML 前端及 Edge 模拟串口握手、格式切换和 Stop 回归通过。
- 52 项 CubeMX 配置变化／DMA／KEY1 重映射回归通过。Debug／Release 的正常构建均执行配置来源检查。
- 两个实际 ARM 镜像的启动／MSP／Break／IRQ、ELF／BIN 一致性、各 139 项 CPU 异常保护，以及外环超期保护指令仿真通过。
- 207 个受保护文件及官方／核心编译命令与本轮基线一致。

未烧录开发板，本轮验证覆盖主机模型、真实绘图缓冲和链接镜像。ELF／BIN／MAP 已更新到 `build/Debug/` 和 `build/Release/`。基线、试验日志与最终资源报告位于忽略目录 `build/ui-cold-r121/`。

```powershell
python -X utf8 tests/ui_host/run.py --view-baseline build/ui-cold-r121/mycode/ui_app.c --formatter-baseline build/ui-cold-r121/mycode/ui_app.c
python -X utf8 tests/settings_host/run.py
python -X utf8 tests/motor_host/run.py control protocol telemetry justfloat
python -X utf8 tests/cubemx_host/run.py
python -X utf8 tests/motor_host/check_startup.py
python -X utf8 tests/diag_host/check_arm.py --both
python -X utf8 tests/diag_host/check_outer_arm.py
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/flash_report.py final --directory build/ui-cold-r121
```
