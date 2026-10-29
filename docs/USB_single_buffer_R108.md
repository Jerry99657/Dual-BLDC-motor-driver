# R108：关闭未使用的 USB 双缓冲支持

固件 `UI-R108-USB`。使用 ST 官方公开编译宏 `USE_USB_DOUBLE_BUFFER=0U`，保留 USB CDC、Native／JustFloat、USB／USART 双向命令、现有缓冲及全部控制保护。CubeMX 无需调整。

## 对照测量

| Debug / Release | Flash 占用 | Flash 剩余 | RAM 总分配 |
|---|---:|---:|---:|
| R107 基线，宏默认为 1 | 130808 B | 264 B | 29984 B |
| 仅将宏设为 0，仍使用 R107 版本资源 | 127992 B | 3080 B | 29984 B |
| R108 最终固件 | 127996 B | 3076 B | 29984 B |

**纯开关收益 2816 B（2.75 KiB）；计入版本资源更新后，最终净减少 2812 B。** RAM 不变。版本字符串压缩和段对齐使最终镜像比纯开关对照大 4 B。

部分链接符号的变化：`HAL_PCD_IRQHandler` 从 4252 B 降至 2800 B，`USB_EPStartXfer.isra.0` 从 764 B 降至 222 B。其余节省来自相关分支和链接共享；这些符号不是全部净收益。

## 当前数据量与缓冲

当前生成的五个 PMA 配置全部使用 `PCD_SNG_BUF`：EP0 OUT／IN、CDC 数据 IN／OUT、CDC 通知 IN。现有固件实际一直使用硬件单缓冲；关闭宏仅裁掉未使用的双缓冲支持，不改变端点的收发方式。

自动遥测最多每 10 ms 产生一帧，即最多 100 帧/s。Native 最大状态帧是 110 B，按每帧都取最大长度作保守估算为约 11 KB/s；JustFloat Auto 的 12 通道加帧尾为 52 B，即约 5.2 KB/s。RPC 查询回复的突发流量另计。当前没有启用硬件双缓冲的需求，应用的 active／pending 发送缓冲、接收缓冲、BUSY 处理和重连机制均保留。

此处的 USB PMA 双缓冲与应用层的两份软件发送缓冲不是同一件事，也与启停保护无关。没有更改遥测频率、通道、包结构或缓冲容量。

## CubeMX 与源码边界

- 根 `CMakeLists.txt` 在 `stm32cubemx` 接口目标上定义宏，使 PCD、USB 底层、协议栈及应用翻译单元使用一致的选项。
- 初始化、PMA 地址、端点模式、时钟和中断配置继续执行 CubeMX 生成结果，未改写生成源码或强制覆盖端点模式。
- 每次正常构建检查全部生成 PMA 调用是否为单缓冲。未来切换 `PCD_DBL_BUF` 或无法识别的模式时明确拒绝构建，应先恢复官方双缓冲支持并重新验证。
- 211 个受保护文件中仅根 CMake 有明确限定的应用配置改动；ST HAL／LL／CMSIS、USB 中间件、`.ioc` 和生成初始化源码均未修改。官方／核心编译命令唯一允许的变化为该公开宏，不改变其余优化参数。

## 验证与限制

- 新增 `tests/usb_host/check_pcd_arm.py`，分别以宏 1／0 编译**未修改的 ST PCD、PCDEx 和 USB 底层源码**，在 Thumb 模拟中执行端点收发与中断。模型实现端点寄存器的翻转／写零清除行为和主机包完成事件。
- 两种构建的单缓冲 Bulk IN／OUT 0～192 B、奇数长度、64 B 边界和多包、STALL／清除、EP0 SETUP／零长度包、复位／挂起／恢复结果一致。
- 实际 CDC 应用接口的忙状态下接收、提前完成、1000 次连续发送、挂起／重连和中断屏蔽状态回归通过。
- Native／JustFloat、USB／USART 命令、FIFO／立即 Stop、数值／格式及 FOC 协议回归通过；UI、存储、LCD 及单文件 HTML／真实 Edge Web Serial 回归通过。
- CubeMX 变更回归由 36 项增至 39 项，新增双缓冲／未知模式拒绝和合法单缓冲 PMA 地址变更；两构建的实际启动、串口初始化及各 139 项异常路径检查通过。

尚未烧录实板。寄存器模型不等于 USB 电气／主机枚举验证，刷入后仍需确认实际 USB 枚举、持续曲线、查询和命令并行、拔插恢复。

基线 ELF／BIN／MAP、用户源码、`measured.json`（纯开关）、`final.json`（最终固件）及日志位于 `build/usb-single-r108/`；最新固件在 `build/Debug/`、`build/Release/`，ELF／BIN／MAP 继续备份。

```powershell
python tests/usb_host/check_pcd_arm.py
python tests/usb_host/run.py
python tests/cubemx_host/run.py
python tests/flash_report.py final --directory build/usb-single-r108 --allow-usb-single-buffer
```
