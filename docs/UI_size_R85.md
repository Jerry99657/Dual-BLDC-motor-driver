# R85：继续压缩 UI Flash

固件 `UI-R85-SZ`。在 R84 基础上继续压缩应用层 UI，保留现有页面、文字、字号、图标像素、动画、按键、控制及保护行为。无需修改 CubeMX。

## 实际结果

| 链接占用 | R84 基线 | R85 | 变化 |
| --- | ---: | ---: | ---: |
| Debug／Release Flash | 129576 B | **129164 B** | 减少 **412 B** |
| Flash 剩余 | 1496 B | **1908 B** | 约 **1.86 KiB** |
| RAM 分配（含原预留堆栈） | 24432 B | **24944 B** | 增加 **512 B** |
| 链接 RAM 余量 | 8336 B | **7824 B** | 减少 512 B |

以上是最终 ELF／BIN 的实际结果，已经扣除图标解压器、格式描述表等新增代码和数据。RAM 余量是链接统计，不代表已测得运行时最大栈余量；本轮没有增加堆分配或控制环／DMA 缓冲。

## 保留的优化

1. **扩展数值描述表**：将 Status 中速度、RPM、六步占空比／电压／速度及 HSO 速度／角差共八项显示并入原有紧凑描述表。继续使用原定点格式化和四舍五入函数，保留 HSO 非有限角差显示 `--`、ESC 电气 Hz 与预设机械 rad/s 的区别，以及 ESC 的 RPM 不可用标记。
2. **复用文字常量**：菜单、绑定标题、自定义页中与文字池相同的字符串直接引用原常量字段。文字池继续放在只读 Flash，避免增加实时字符串复制。
3. **复用页脚绘制**：Run、Run Steps、Detect 和手动跟随共用原有页脚坐标、横线和裁剪窗口结束逻辑。Run Steps 直接带单位格式化，省去一次中间字符串拼接。
4. **紧凑格式规格**：浮点显示规格使用一字节描述单位、倍率和小数位数，代替重复的格式分支。计数器继续区分 16／32 位读取；描述表大小、文本偏移和源快照大小仍有编译期约束。
5. **首页图标无损压缩**：原四个 32×32 XBM 图标共 512 B，压缩载荷为 **238 B**。`ui_app_reset()` 在 `KK_UI_Init()` 前将其解压到 512 B 静态 RAM，KK_UI 继续接收原有位图指针。解压仅在 UI 初始化时进行，不在刷新或控制中断中执行。

图标原始数据仍完整保存在 `mycode/ui_icons.h`，其像素没有修改。工程附带生成结果 `mycode/ui_resources_packed.h`，正常 CMake 编译不依赖 Python。修改原始图标后执行：

```text
python scripts/pack_ui_resources.py
python scripts/pack_ui_resources.py --check
```

生成器验证全部 512 B 的往返一致性、重叠回引用、源／目标边界及输入消耗；UI 回归也会先检查生成文件是否与原始位图一致。运行时解压器只处理这份经过生成验证的静态 Flash 资源。

## 对照构建

| 累计改动 | Flash |
| --- | ---: |
| R84 基线 | 129576 B |
| 扩展数值描述表 | 129456 B |
| 复用文字常量 | 129356 B |
| 页脚与 Run Steps 格式化复用 | 129348 B |
| 首页图标压缩，初始解码格式 | 129208 B |
| 数值格式规格表 | 129176 B |
| 图标回引用改为一字节短窗口格式 | **129164 B** |

以下试验已撤回：

- 只给 KK_UI／KK_OLED 改为 `-Oz`：129456 B 增至 129800 B，增大 344 B。最终编译配置完全恢复。
- 将文字池和图标一起解压到 RAM：129348 B 增至 129460 B，且额外消耗 RAM。最终只压缩图标，文字池保持只读。
- 再封装设置提交／刷新失效调用：没有实际 Flash 收益，未保留。

## 验证与边界

- 版本号更新前，**62 张静态／过渡图像逐字节一致**；最终仅 `about.pgm` 的 FW 文字变化。
- 112 条信息行、43 个菜单项目继续通过与 R83 的逐项资源比对，155 项标签、动态引用、类型与动作一致，仅规范化版本文字。
- UI 完成 3689 次显示提交，按键连按、长按切轴、Run 确认／停止、远程来源保护、参数范围和保存／恢复回归通过。
- 真实 LCD 驱动初始化、边缘映射、DMA 早完成／异常恢复与异步存储模型回归通过。
- LL 中断 8388608 种 ADC 组合、真实采样／保护集成、USB CDC、最终 ELF 启动与控制路径、ELF／BIN 一致性检查通过。
- Debug／Release 完成干净构建；205 个受保护文件及官方／核心编译命令与本轮基线一致。
- ST HAL／LL、CMSIS、USB／HSO、KK_UI／KK_OLED 核心、字体、原始图标、控制算法、CubeMX 初始化、`.ioc`、链接脚本及编译配置均未修改。

```text
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/ui-r85
```

未烧录实板。回退镜像、62 张基线图像、试验及最终构建日志、占用和受保护文件审计记录位于本机忽略目录 `build/ui-r85/`；ELF／BIN／MAP 位于 `build/Debug/` 和 `build/Release/`。上板建议核对冷启动四个首页图标、Run／Run Steps／Detect 页脚及长按切轴、运行／停止操作。

最终 BIN SHA256：

```text
Debug   23b5d0702dab8d1f45024f2542a630ebec1851ba21f9e4112f87c2b93d04b808
Release 7a35af28ef3a643ac0b2b789ebb6ffea17b584a9ecf458315863647595a4d9ce
```
