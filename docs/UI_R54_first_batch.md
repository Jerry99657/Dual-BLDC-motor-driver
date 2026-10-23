# UI-R54-UI：第一批界面调整

R55 已修复 Run 居中与首页字体行高，最新固件、资源和预览见 [R55 对齐修复](UI_R55_alignment.md)。以下保留 R54 的功能和构建记录。

本批完成 A1～A6、B1、B2、B4、B5、B8。无需重新配置 CubeMX。HSO 启动、观测、主从跟随及各项保护沿用 R53；工作模式和电机型号切换的停机条件保留。

## 操作变化

| 位置 | 当前行为 |
| --- | --- |
| Settings 首项 | `System: 当前工作模式`，恢复默认移到末项 `Restore Defaults`，仍需确认 |
| System → Mode | 进入 Work Mode；`*` 标记实际已应用模式，移动光标不会改变标记 |
| System → Axis M1 / Axis M2 | 选择参数、诊断和双独立模式无前缀命令的当前轴；`*` 标记当前轴 |
| Settings / Motor / Steps 标题 | 显示当前轴和电机型号；型号入口统一保留在 Settings → Motor |
| Single M1 + HSO 的 Curr Loop | 显示 `ON`，禁止关闭；保留该轴原先的编码器电流环偏好，关闭 HSO 后恢复显示该偏好 |
| Speed Limit | 两种型号、两路均可设 1～600 rad/s；Pos Speed 上限跟随当前 Speed Limit |
| Settings 底部 Save 行 | 持续保留保存状态，可以在短暂提示消失后查看 |

既有 Speed Limit 值不自动提高，默认仍为 100 rad/s。600 是软件可配置范围，不是额定转速或保证可达到的速度；4015 额定速度参数仍为 104.72 rad/s。HSO 的 5～小于 400 电气 Hz 目标校验，以及母线、电流、过载和控制超时保护继续独立生效。

## Run

页首显示当前轴、控制方式和 `ENC` / `HSO`，右侧始终显示实测 VBUS。母线采样失效时显示 `--V`，不把旧数值当作有效测量。母线与 Status 共用约 500 ms 的文本更新；运行数据约 100 ms 刷新。

Torque 根据电流环显示为 `Uq(V)` 或 `Iq(A)`。HSO 页仅允许 Speed，状态行明确显示 DC ALIGN、I/F RAMP、I/F FLUX、CAPTURE、CONTROL 等实际阶段。Ref 表示启动阶段的参考速度，Act 表示运行反馈速度。

大字显示当前草稿：`EDIT` 表示停机中已修改，`RUN*` 表示仍按之前确认的目标运行。存在修改时，第二行同时显示 `Set` 的之前确认值。OK 文案随行为变化：停机时 `Start`，运行且已改值时 `Apply`，运行且未改值时 `Stop`。方向键的加减和长按连发规则保留。

双独立模式状态行同时显示 `M1:RUN/STOP`、`M2:RUN/STOP`，当前轴反白，状态取实际输出请求而非缓存的页面状态。KEY1 切换当前轴，保持另一台运行；未改值时 OK 仅停止当前轴。KEY2 或离开 Run 仍停止两路。手动主从页也显示 ENC 和 VBUS，OK 显示 Start / Stop。

## 保存与兼容性

双独立模式的当前轴与设置一起延迟保存，菜单选择和 Run 的 KEY1 切换都有效。停止调整至少 2 秒、当前 Run 停止且显示空闲后请求保存；SPI2 被 M2 实时采样占用时等待总线释放。建议退出 Run 后等待 `Save: Saved` 再断电。重新上电恢复当前轴及其参数，仍须手动启动。

| Save 行 | 含义 |
| --- | --- |
| Defaults | 尚无有效的持久化记录，当前使用默认值 |
| Pending | 有待保存修改或请求正在排队 |
| Saving | 正在擦除、编程或读回校验快照 |
| Wait bus | 保存等待 M2 释放 SPI2 |
| Saved | 有效记录已保存，最新请求已通过读回和 CRC 校验 |
| Failed | 无 Flash 或保存最终失败；具体阶段仍在 Status → Save Last / Flash Err |

使用 version 3 的 control_config 空闲 bit 5 保存双独立当前轴，无需改变记录大小或 CRC 布局。旧 v1/v2/v3 记录可继续读取，未写入该位的记录默认为 M1。切到单机或同步/主从模式不会覆盖双独立的轴偏好。

## 构建与验证

2026-10-23：Debug Flash 130372 / 131072 B，RAM 22696 / 32768 B；Release Flash 129660 B，RAM 22592 B。Debug 保留调试符号。通过合并 UI 轴状态拷贝、重复数值格式化和标题字库缩减收回空间；未删减诊断，也未调整控制 ISR 的优化选项。原有链接器 RWX 段提示保留。

标题继续使用 [LEDFont](https://ledfont.botelvdong.com/agent-api.md) 的 `wenquanyi_9pt` 12px，按实时目录生成 36 字符子集，1166 B 减至 472 B；正文 14px 和大号数值字库保留。标题、全部正文 ASCII、Run 各阶段及正负范围边界已用真实字形宽度检查。

主机回归覆盖真实 KK_UI 输入和异步绘制、修改后 Apply、双机状态、KEY1/KEY2、HSO ON 与编码器偏好、模式标记、保存状态、NOR 异步写入和旧记录迁移，以及真实控制后端的轴恢复和 4015 超过 120 rad/s 的命令限幅。控制/采样/HSO/编码器/SPI2/USB 回归与最终 ELF 初始化顺序、BIN 对应关系均通过。截图由 `tests/ui_host/render_preview.py` 从测试帧生成，输出 `build/ui-validation/r54-preview.png`。

| 固件 | BIN SHA256 |
| --- | --- |
| Debug | `ba9d6dbed63ae2e0378359986d6334816e20e012e49da4e988547fe76037f2dd` |
| Release | `6632501c2aa9020ae9759ebdd8684c21f514aa4b7441ea59896f92844e3be323` |

尚未烧录或实板验证。本批未实施 KEY2 停止规则变更、Status 重分组、HSO 最低速度编辑器或额外 rpm/角度显示。
