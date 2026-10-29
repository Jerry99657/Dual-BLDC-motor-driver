# R109：USB LPM 配置测量与一致性整理

固件 `UI-R109-LPM`。将未使用的 USB Link Power Management 扩展设为关闭，通过保存的 `.ioc` 配置和 CubeMX 原生生成落实，保留普通 USB 挂起／恢复、CDC 通信及现有控制保护。

## 实测资源

| Debug / Release | R108 基线 | R109 | 变化 |
|---|---:|---:|---:|
| Flash | 127996 B | 127876 B | −120 B |
| Flash 剩余 | 3076 B | 3196 B | +120 B |
| RAM 总分配 | 29984 B | 29976 B | −8 B |

关闭配置、尚未更新 FW 字符串时测得的结果与最终固件相同，均净省 120 B。RAM 数值包含链接布局及预留堆栈，不是运行时峰值。512 B 字符串缓冲未调整。

收益小于 R108 双缓冲开关，主要价值是消除未使用的协议栈分支和配置不一致。

## 配置与生成结果

原工程 `USBD_LPM_ENABLED=1U`，PCD 的 `lpm_enable=DISABLE`，设备描述符是 USB 2.00，`CDC_Desc` 没有 BOS 回调。

当前 `.ioc` 显式保存 `USB_DEVICE.USBD_LPM_ENABLED=0`，并将该项加入 USB_DEVICE 的 IPParameters。CubeMX 6.17.0／FW_G4 V1.6.3 在隔离副本实际生成的变化为：

- `USB_Device/Target/usbd_conf.h`：`USBD_LPM_ENABLED` 变为 `0U`。
- `USB_Device/Target/usbd_conf.c`：移除 LPM 专用回调注册和扩展回调。普通 Suspend／Resume／Reset／连接断开回调保留；PCD 初始化仍为 `lpm_enable=DISABLE`。
- 其他 Core／USB C、头文件逐字节不变，`usbd_desc.c` 未修改。

两份原生生成对照位于 `build/usb-lpm-r109/cubemx/`：原配置重生成与实际项目完全一致；关闭 LPM 的输出只改变上述两个文件，保存后的 IOC 保留该属性。项目使用这份生成输出，未在初始化后写死覆盖配置，也没有编辑 CubeMX 模板。

每次正常构建增加检查：协议栈和 PCD 的 LPM 设置必须一致、已保存 IOC 必须与生成头文件一致；启用 LPM 时还需生成有效的 BOS 回调。未来不兼容配置会明确拒绝构建。

## 验证

- 新增 `tests/usb_host/check_lpm_arm.py`，以前后两份生成头文件分别编译未修改的 ST USB Device／CDC 库及实际生成描述符，执行 Thumb 指令。USBD_LL 硬件传输以记录桩替代，记录控制响应内容及状态。
- 设备描述符、配置头和完整配置、六类字符串、非法字符串和 BOS 请求的 STALL、设置地址、CDC 配置／控制线状态、配置／状态查询结果逐项一致。普通挂起／恢复仍为 CONFIGURED → SUSPENDED → CONFIGURED。
- PCD／PMA 单缓冲 0～192 B、多包／奇数长度、EP0 SETUP／零长度包、STALL、复位／挂起／恢复的实际官方驱动指令测试通过；该测试与上层 USB 栈测试相互补充。
- CDC 忙时接收、提前完成、连续发送和重连；Native／JustFloat、USB／USART、命令及 FOC 协议回归通过。
- UI／存储／LCD、HTML 和真实 Edge Web Serial 回归通过。
- CubeMX 配置变更回归增至 43 项。两构建实际 ELF 的初始化／串口／中断及各 139 项异常路径检查通过。
- 211 个受保护文件中仅允许 `.ioc` 和上述两个生成 USB 适配文件变化，逐字核对原生输出及限定差异；ST HAL／LL／CMSIS、USB 中间件源文件与全部官方／核心编译参数保持基线。

Debug／Release 的 ELF、BIN、MAP 已更新。基线备份、原生生成副本、`measured.json`、`final.json` 及日志位于 `build/usb-lpm-r109/`。尚未烧录实板，模型验证不能替代实际主机枚举、持续通信和拔插验证。

```powershell
python tests/usb_host/check_lpm_arm.py --baseline build/usb-lpm-r109/USB_Device/Target/usbd_conf.h
python tests/flash_report.py final --directory build/usb-lpm-r109 --allow-usb-lpm
```
