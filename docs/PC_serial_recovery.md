# HTML 上位机串口异常处理与定位

## 本次结论

现象为浏览器显示 `The device has been lost.`，设备管理器仍显示 STM32 端口；用户确认重新连接即可恢复，开发板 UI 正常。

已确认并复现 HTML 的异常恢复缺陷。最初使 Windows／浏览器将串口会话判为丢失的原因尚未确定；现有信息不能证明 MCU 复位、USB 电气瞬断、Windows 驱动异常或浏览器设备服务异常中的哪一种发生了。设备管理器中存在 COM 端口，只说明该设备当前可枚举，不能证明先前打开的会话持续有效。

Chromium 将接收端的 `DISCONNECTED`／`DEVICE_LOST` 映射为 `NetworkError: The device has been lost.`；溢出等错误属于另一类可恢复错误。页面原先只显示 `e.message`，没有保存错误名称与阶段，妨碍了区分。

该 HTML 修复仅修改单文件 HTML、测试及说明，交付时 STM32 固件仍为 `UI-R123-ADC`，Flash 余量 **784 B**。此后 R124 调整了双机实时路径并同步更新 HTML 的兼容版本范围，见 [双机实时预算修复](Dual_outer_budget_R124.md)。串口恢复功能本身仍不增加固件占用，打开更新后的 `pc_tool/index.html` 即可使用。

## 可确认的旧版缺陷

1. 读取循环只有一层。一次缓冲溢出、帧错误等异常，也会结束整个连接；浏览器新建的可恢复接收流没有被重新读取。
2. 读取退出只释放 reader，仍保留 port 和 writer。页面显示未连接，但资源仍被锁定，必须先点“断开通信”才能重新选择串口。
3. 写入失败没有统一终止接收与清理连接；连接握手超时后也可能保留半初始化会话。
4. 断开时先等待 `writer.abort()` 和发送链。真实 Streams 的底层 write 未完成时，abort 也可能一直等待，导致无法进入端口关闭阶段。
5. 读取和数据处理处于同一个异常捕获块。界面／解析处理抛错也会被显示为“通信中断”，难以区分来源。

在同一 Edge 浏览器、同一个模拟 SerialPort 上对比修改前后实际 HTML，结果保存于忽略目录 `build/serial-recovery/before-after.json`：

| 注入情况 | 修改前 | 修改后 |
| --- | --- | --- |
| 接收溢出，浏览器仍提供新接收流 | `connected=false`，port／writer 保留 | 保持连接，读取新接收流 |
| `NetworkError: The device has been lost.` | port／writer 保留 | 释放 reader／writer／port，允许直接重新连接 |

这是浏览器模型复现的软件缺陷，不是对开发板首次丢失 USB 会话的实物复现。

## 修改后的行为

- 接收异常后先释放旧 reader；浏览器确实提供另一个接收流时才继续读取。连续三次恢复仍未收到字节，则结束连接，避免反复读取错误流。
- 恢复时丢弃半帧、重置 JustFloat 探测，并建立新的采样分段，避免跨越缺失数据进行测量。错误计数保留。
- 未完成的命令回执标为“结果未知”，取消批量操作及尚未提交的旧发送任务。已经交给驱动的写入无法撤回，不自动重发。
- 清除未经重新确认的配置快照；接收恢复后点击“读取全部设置”确认状态，再继续控制。恢复过程不发送运动指令。
- 真正失去接收流、写入失败、接收结束、数据处理失败以及握手失败均进入统一清理；并发的清理请求合并。
- 关闭前取消接收、释放两个流的锁，再调用 `SerialPort.close()`；不先等待挂起的写入或 abort。若驱动拒绝关闭，则保留端口句柄供“断开通信”再次清理，不伪装成已关闭。
- 真正断连不会自动重开串口；点击“选择串口并连接”即可重新授权选择原端口。连接时仍只执行 `Format:Native`、`Get`，不重放运动／设置指令。
- Web Serial 缓冲区从 8 KiB 提高至 64 KiB，给浏览器短暂繁忙留出余地。该设置不能消除 USB 电气问题，也不能修复驱动句柄已失效的问题。

## 新增诊断

页面原有“浏览器运行诊断 → 导出诊断 JSON”新增 `transport`，会话检查点也保存此字段：

- `connections`、`readErrors`、`writeErrors`、`recoveries`、`osDisconnects`；
- 累计字节、最后字节时间／间隔、最后有效帧间隔；
- 最近成功连接的 VID／PID 与固件编号；
- 当前是否连接、是否仍保留端口、是否正在打开／关闭；
- 最近 64 个传输事件，包括错误名称、原始消息、处理阶段、当时的收包数量／板端时间与发送格式。

`os-disconnect` 表示浏览器报告设备断开事件；`read-error`／`writer-error`／`write-error` 记录流和 I/O 错误；`processing-error` 表示页面处理错误。`read-recovered` 表示新接收流确实再次交付了字节。没有 OS disconnect 事件也不能排除 USB 瞬断或驱动异常。

再次出现问题时，先在当前页面导出诊断 JSON，保留断连前后事件。Chrome 还可查看 `chrome://device-log`（Edge 对应 `edge://device-log`）中的串口／USB 事件时间，结合板端是否复位、是否只有开功率输出时发生继续定位。当前未增加固件端 USB 复位／错误计数，不能从浏览器 JSON 单独推断设备内部状态。

## 验证与边界

- Edge 的真实 ReadableStream／WritableStream，配合模拟 Windows 行为的已授权端口：读取溢出恢复、半帧丢弃、命令结果未知及未提交任务取消；同端口 device-lost 后重连；写入侧失败、EOF、处理错误、打开失败、握手超时；恢复预算、挂起写入时断开、并发断开、关闭失败重试；额外连续 35 次恢复。无未处理页面异常，无运动指令写入。
- 原前端协议／设置联动、Native／JustFloat 格式切换和 Stop、浏览器性能与诊断导出回归通过。
- 现有实际 C CDC 接口、固件协议／遥测、USB／USART 一致性测试通过。
- 原版 ST PCD／PMA／IRQ 的 ARM 模型单缓冲与双缓冲编译开关对照通过，包含多包、奇数长度、STALL、EP0、复位和挂起恢复。

以上使用本机浏览器与寄存器／接口模型；未连接开发板，尚不能宣称已消除实板的偶发断连。

```powershell
python -X utf8 tests/pc_tool/test_serial_recovery_browser.py
node tests/pc_tool/test_frontend.js
python -X utf8 tests/pc_tool/test_format_browser.py
python -X utf8 tests/pc_tool/test_performance_browser.py
python -X utf8 tests/usb_host/run.py
python -X utf8 tests/usb_host/check_pcd_arm.py
python -X utf8 tests/motor_host/run.py protocol telemetry justfloat
```

## 依据

- [Chrome 官方 Web Serial：可恢复读取错误与流关闭](https://developer.chrome.com/docs/capabilities/serial)
- [Chromium：接收／发送错误与 DOMException 映射](https://chromium.googlesource.com/chromium/src/+/db623cd54325fa0a14c2fb002afeee49b1b4b494/third_party/blink/renderer/modules/serial/serial_port.cc)
- [Chromium：Windows 串口 I/O 错误处理](https://chromium.googlesource.com/chromium/src/+/refs/tags/142.0.7444.20/services/device/serial/serial_io_handler_win.cc)
