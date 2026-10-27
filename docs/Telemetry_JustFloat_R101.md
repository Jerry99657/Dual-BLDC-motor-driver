# R101：标准 JustFloat 与 Native 共存

固件标识 `UI-R101-FMT`。MCU 上电／复位默认 **JustFloat + Auto**；USB CDC 与 USART1 共用一份输出格式和采集组。任何接口或板上菜单修改后，两路同时生效。格式只保存在 BSS/RAM，不进入 AppSettings、W25Q64、配置导入或保存流程。

## 切换与连接

使用单行 ASCII 命令，行尾 LF 或 CRLF：

```text
Format:Native
Format:JustFloat
Stream:Auto
Stream:M1
```

命令区分大小写，不能加 `M1:`／`M2:` 前缀。Settings → System 最后一项 `TX Format: JustFloat/Native`，按 OK 直接切换；查看菜单和切换格式不会启动／停止电机或更改目标。格式切换不请求保存，重新构建 UI 保留当前 RAM 格式。MCU 复位后重新成为 JustFloat。

HTML 连接后第一条命令为 **不带请求编号的 `Format:Native\n`**，随后发送带编号的 `Get`。只有校验通过、请求匹配的配置回执到达后，才确认 Native 并开放完整控制。无需固定延时，也不自动运行电机。R87～R100 忽略不支持的普通 Format 命令，仍通过 Get 保持旧版兼容。

HTML 顶部新增发送格式选择；高级原始命令也能使用上述两条 Format 命令。主动切到 JustFloat 使用不带编号的命令，不等待不存在的 Native 回执。控制台暂停设置／运动／诊断查询，全局 Stop 仍发送普通 `Stop\n`；它只表示请求已发送，不能声称板端已确认。切回 Native 后重新 Get 确认实际状态。

如果从 LCD 或另一路接口切换为 JustFloat，HTML 通过两个连续、同宽的合法浮点帧识别格式变化，取消尚未完成的查询并提示“先前命令结果未知”。不会自动覆盖用户选择；需要时选 Native 并点击切换，或断开后重新连接。该检测不把 JustFloat 当作带 CRC 的可靠数据协议，也不在 JustFloat 下提供完整 HTML 控制台或 CSV 采集。

同一 COM 口不能同时由 HTML 和 VOFA+ 占用。两路输出共用格式，因此 HTML 将板子切到 Native 时，另一接口上的 VOFA+ JustFloat 显示也需相应停止／切换；没有接口独立格式。

## 标准结构与时序

```text
N × 小端 IEEE754 float32 + 00 00 80 7F
```

没有自定义头、有效位、序号、时间戳、CRC 或额外状态帧。所有无效值、NaN 和 ±Inf 标准化为 `0x7FC00000`（quiet NaN），避免把 +Inf 的 `0x7F800000` 误当作帧尾；不以零代替失效测量。VOFA+ 对无效点的绘制仍取决于其版本和显示配置，需实机确认。

采样／组帧仍在主循环，不改变控制 ISR、控制频率、ADC 调度、时钟和串口参数。JustFloat 最大约 100 Hz，每帧为 4N+4 B；繁忙时只保留最新待发送 USB 帧，UART 忙时跳过。协议自身不报告丢帧或精确板端时间，VOFA+ 的时间尺度需按实际采样率设置。

切换保留已交给 USB／UART 的完整活动帧直到正常完成，丢弃未发送的旧格式 USB 数据与旧 Native 回复；不终止 DMA／TC 中的活动帧，不改外设配置。新格式从下一条完整帧开始。

## JustFloat 通道顺序

以下通道编号从 0 开始；`Stream:` 仅选择输出内容，不改变电机工作模式。Auto 固定 12 路，不随 Single／双机／HSO／六步重新排列。未启用轴保留 NaN 槽位。

| 采集组 | N | 通道顺序 |
| --- | ---: | --- |
| Auto | 12 | M1 目标速度、实际速度、目标位置、实际位置、Iq；M2 同样五项；VBUS；协同误差 |
| M1 / M2 | 6 | 选中轴目标速度、实际速度、目标位置、实际位置、Iq、VBUS |
| FOC1 / FOC2 | 7 | Iq/Uq 指令、Iq、Id、U 相电流、V 相电流、实际速度、VBUS |
| HSO | 9 | M1 目标速度、HSO 控制参考速度、控制反馈速度、观测速度、Iq、VBUS、角差、电角磁链、相对机械位置 |
| Six | 6 | 目标、估算速度、参考速度、占空比、反电势残差、VBUS |
| Off | 0 | 完全停止遥测，仍可接收命令；无心跳 |

速度 rad/s、位置／角差 rad、电流 A、母线／反电势 V、磁链 Wb；占空比为 0～1。FOC 指令依电流环为 A 或 V；Six 目标依 Speed／Throttle 为 rad/s 或 %，ESC 速度为电气 Hz；协同误差依同步速度／同步位置为 rad/s 或 rad。JustFloat 不传单位，改变相关控制方式时需自行更新 VOFA+ 通道标签。

HSO/六步使用专用曲线时手动发送 `Stream:HSO`／`Stream:Six`。JustFloat 不接收 ENC1／ENC2 整数诊断组；请求会拒绝并保留原组。若从 Native 的 ENC1／ENC2 切到 JustFloat，则回到固定 Auto，避免输出被错误解释。Native 的 Auto、HSO／Six／ENC 诊断、Off 心跳、整数精度、设置回执及全部既有功能保持。

## 构建与验证

R100 → R101，Debug／Release：

| 项目 | R100 | R101 | 变化 |
| --- | ---: | ---: | ---: |
| Flash 使用 | 129728 B | 130252 B | +524 B |
| Flash 剩余 | 1344 B | 820 B | -524 B |
| RAM 总分配（含既有堆栈预留） | 29424 B | 29456 B | +32 B |

没有修改 ST HAL／LL／CMSIS／USB 中间件、CubeMX 生成代码、`.ioc` 或编译选项。正常构建继续执行 CubeMX 配置检查；ELF／BIN／MAP 保留。改动前产物、保护边界及体积记录位于忽略目录 `build/justfloat-r101/`。

验证使用真实 C 组帧及真实 HTML 代码，覆盖双源命令、标准帧尾、固定槽位、全部曲线组、非有限数、Off 静默、两路格式一致、活动帧不变／待发帧清理、RPC 排除、tick 回绕、Stop、菜单和无保存行为；Edge 通过模拟 Web Serial/Streams 及真实 C 字节验证 Native-first 握手、板上切换检测／查询取消／恢复，以及原有画布和控制联动。控制算法／驱动及构建保护检查另行回归。

尚未连接实板、未烧录，也未启动实际 VOFA+ 接收。板上长时间 USB／USART 联调和 VOFA+ 实收波形仍需验证。

标准依据：[VOFA+ JustFloat 官方说明](https://www.vofa.plus/docs/learning/dataengines/justfloat/)。实现独立编写，未复制协议引擎源码。
