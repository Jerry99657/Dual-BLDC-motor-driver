# R84：固定 UI 资源压缩

固件 `UI-R84-SZ`。继续在应用层压缩 Flash，本轮只修改 `ui_app.c`，保留全部页面、信息行、菜单、字体、图标、动画、按键和控制行为。无需修改 CubeMX。

## 实际结果

| 链接占用 | R83 | R84 | 变化 |
| --- | ---: | ---: | ---: |
| Debug／Release Flash | 130252 B | **129576 B** | 减少 **676 B** |
| Flash 剩余 | 820 B | **1496 B** | 约 **1.46 KiB** |
| RAM 分配（含预留堆栈） | 23192 B | **24432 B** | 增加 **1240 B** |

本轮使用约 1.21 KiB 静态 RAM 换取 Flash。链接脚本的 32 KiB RAM 区域还剩 8336 B，以上已使用量包含原先预留的堆栈。本轮没有增加堆分配，也没有扩大控制环或 DMA 缓冲。

## 实现

1. **信息行压缩**：112 条 Status／About 信息行在 Flash 中保存两个 16 位相对引用，启动时展开为 KK_UI 原有的 `KK_UI_InfoRow`，静态 RAM 增加 896 B。
2. **菜单压缩**：43 个菜单项目保存 16 位标签引用、8 位类型和 8 位目标／事件引用，启动时展开为原有 `KK_UI_MenuItem`；当前 ARM 构建的静态 RAM 增加 344 B。
3. **共用文字资源**：字面量集中到只读字符池；动态值仍指向原 `UiTexts`、编码器标签、轴标签和工作模式标签，切轴、模式提示和实时诊断继续更新原缓冲。
4. **共享解码表**：五组资源基地址使用静态常量表，避免每次解码复制到栈上。引用范围、记录大小、菜单引用范围和记录总数均有编译期检查；页面起始下标由前一页条目数推导。
5. **Run 草稿复用**：上电 UI 初始化和进入 Run 共用草稿装载函数，保留原有限幅、电流目标清零、位置原点及停止／确认规则，不发送运动命令。

资源展开在 `ui_app_reset()` 中完成，发生在 `KK_UI_Init()` 之前；以后不在帧刷新中重建。指针及底层对象都是应用静态存储，生命周期覆盖 UI 使用。没有建立动态菜单容器或控件树，KK_UI 收到的仍是原有公开结构和固定项目。

对照构建：

| 增量 | Flash |
| --- | ---: |
| R83 基线 | 130252 B |
| Run 草稿装载复用 | 130212 B |
| 信息行紧凑描述 | 129644 B |
| 菜单紧凑描述 | 129592 B |
| 静态资源基址表 | 129576 B |

曾试验关闭未使用的 Message 编译开关，可减少 72 B；该试验已撤回，本版保留原有库配置和全部库能力。

## 验证与边界

- 112 条信息行与 43 个菜单项目，按标签、动态字段引用、类型、事件／目标逐项与 R83 比较，**155 项完全一致**（仅规范化 FW 版本文字）。比对记录位于本机忽略目录 `build/ui-r84/`。
- 版本更新前 **62 张静态／过渡图像逐字节一致**；最终仅 `about.pgm` 的 FW 显示不同。
- UI 回归完成 3689 次显示提交，按键、长按切轴、Run 确认／停止、远程来源保护、设置与保存／恢复通过；真实 LCD 驱动、异步异常恢复测试通过。
- LL 中断全组合与真实采样／保护集成、USB CDC、最终 ELF 启动／控制路径及 BIN 一致性检查通过。
- Debug／Release 完成干净构建。205 个受保护文件及官方／核心编译命令与 R83 基线一致。
- ST HAL／LL、CMSIS、USB／HSO、KK_UI／KK_OLED 核心、字体／图标、控制算法、`.ioc`、链接脚本和编译配置均未修改。

```text
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/ui-r84
```

未烧录实板。建议实板核对冷启动进入首页、Settings／Status 各目录、长按切轴、Run 确认／停止及掉电保存恢复。回退镜像、资源比对脚本、图像及构建记录保存在本机忽略目录 `build/ui-r84/`；最终 ELF／BIN／MAP 在 `build/Debug/` 和 `build/Release/`。

最终 BIN SHA256：

```text
Debug   4eeacdbc5026dcff1752dff1678939f761b721371a480d1a12be0bca82b2faaf
Release 316ea195968fea468d1f6899bdc5104465b1a6fda8ce50da734af23e3131832c
```
