# UI-R57-INFO：B6、B7、B9

本批完成 HSO 调值边界、Status 分组、rpm／角度辅助读数。无需修改 CubeMX。原有首页字号、Run 大字居中、长按左右切轴、KEY2 返回与停止规则继续使用；KEY1 仍保留，尚未接入全局暂停。

## B6：HSO 速度编辑

仅作用于 Single M1 + HSO 的 Run 页。上下键及长按连发使用 `负速度有效区 / 0 / 正速度有效区`，跳过不足 5 电气 Hz 的不可启动区。

| 电机 | 最小非零目标 | 观测器允许的最大 UI 目标 |
| --- | --- | --- |
| 2804，7 极对 | 4.5 rad/s | 359.0 rad/s |
| 4015，11 极对 | 2.9 rad/s | 228.4 rad/s |

最大目标还受到当前 Speed Limit 的限制；这些边界由 `StHso_MinSpeed()`、电机极对数和严格小于 400 电气 Hz 的条件计算，并非提高电机额定能力。Speed Limit 低于最小值时只允许 0，调值提示 `Raise Speed Limit`。控制后端的全部校验和保护继续独立生效。

从 0 加／减一步进入正／负最小速度。向零调节时先落到 0，即使大步长也不直接跨到反向。运行中尝试反向提示 `Stop before reverse`，原已确认目标及输出保持；先把 0 通过 OK 应用，或按停止键，等待静止后再反向启动。停止后的非零启动还检查 HSO 的静止资格，未满足时提示 `Wait for stop`。

0 表示滑行停止，不是零速保持。Run 的零草稿将 OK 标为 Stop，确认后显示 `0=Coast`；Status → HSO 前部新增 H Min、H Max、H Zero、H Reverse。进入 Run 时，旧编码器目标超出 HSO 范围或处在低速空档，会调整当前草稿，仍须按 OK 才提交和保存。

## B7：Status 分组

首页 Status 进入分组菜单。上下选组、OK 进入；信息页中上下滚动，OK／KEY2 返回分组菜单，再按 KEY2 返回首页。

| 入口 | 内容 |
| --- | --- |
| Drive | Drive、Output、Feedback、Bus 优先显示，其后为工作模式、型号、对齐和停机首因 |
| Motion | SI 目标／反馈、辅助单位、电流与电角／机械角 |
| Encoder | 当前物理 E1／E2 的状态、CRC／Field／Track／SPI／Frame、Age、Last、零帧事件及 E2 Probe／Total |
| ADC | I Trip、IU／IV／IW 原始／零点、母线和注入帧计数、ADC 错误、VREFINT 原始／校准及计数 |
| HSO | M1 的启动／观测阶段、速度边界、资格、拒绝原因、磁链、偏置和执行耗时 |
| System | 命令结果、两台型号、控制耗时、Flash 保存与 SPI2、KeyRpt／DispErr |

原 Status 的 80 项均保留。分组菜单末尾的 Axis M1／Axis M2 与 System 中的轴选择一致，在双独立模式下继续延迟保存选择。各组标题显示当前轴和型号；HSO 始终明确显示 M1，System 为全局信息。

## B9：辅助单位

控制、设置和串口命令继续使用 rad/s、rad。Run 未修改草稿时，在 Act／Ref 后补充实际速度的整数 rpm 或相对本次运行原点的整数 deg；修改草稿时优先保留 Act／Ref 与之前确认的 Set。单轴编码器 Speed／Position 的状态行右侧另显示草稿对应的 rpm／deg；若空间不足则保留运行状态并省略该行 Iq。双独立的两路运行状态、HSO 的启动阶段和故障文字继续优先显示。

Status → Motion 提供 0.1 rpm／0.1 deg 读数：Target rpm、Speed rpm、Elec 0-360、Mech 0-360；Position 的目标辅助行名称为 Tgt 0-360。Status 的度数统一表示一圈内的 0≤角度<360°，负角和多圈角先取模；原有 Target／Mech 的 rad 值仍保留多圈含义。Run 的相对位置 deg 则保留正负方向。数值精度不代表传感器精度。

## Flash 与验证

新增页面最初使 Debug 超出真实的 128 KiB Flash。链接分析发现 `strtof` 连带引入了较大的双精度字符串转换和任意精度支持，现改为无堆分配的有限浮点数解析器。保留正负号、前导 ASCII 空白、小数、科学计数法、C 十六进制浮点格式和既有 63 字节命令帧限制；仍拒绝残留字符、缺失指数、NaN、Infinity 和溢出，不改变命令派发和 USB 收发流程。中间使用 double 防止小电感／磁链在指数应用前下溢，最终仍为 float；极端舍入中点容许一个 float ULP 的差别，不将其作为任意精度转换库。

标题字模通过 LEDFont 实时目录生成，同为 wenquanyi_9pt 12px，增加分组所需的 5 个字符，共 41 字符、545 B。保留字形与原 R53 字节相同，ALL 行高仍为原来的 13px。正文和大号数字字模保留。

主机验证覆盖真实按键及连发、两种电机 HSO 最小／最大／跨零／反转／静止门限、旧目标草稿适配、六组路由与返回栈、轴标识、正负 rpm 和角度取模、原有首页及 Run 像素居中、异步显示、NOR 保存和 LCD 驱动。数值解析另覆盖边界、错误输入、次正规数、科学／十六进制表示及 20000 个 binary32 样本；完整控制／采样／HSO／编码器／SPI2 与 USB 回归均需通过。预览由 `tests/ui_host/render_preview.py` 生成至 `build/ui-validation/r57-preview.png`。

UI、完整控制／采样／HSO／编码器／SPI2、USB 主机回归全部通过；最终 ELF 另核对 HAL 初始化顺序、强回调与 BIN 一致性。原有链接器 RWX 段提示保留。

| 构建 | Flash / 131072 B | 空余 Flash | RAM / 32768 B |
| --- | --- | --- | --- |
| Debug | 121524 B | 9548 B | 22592 B |
| Release | 120784 B | 10288 B | 22496 B |

| 固件 | BIN SHA256 |
| --- | --- |
| Debug | `b241086f97f5c51569f7688322f8ad3e8ae50e0b96485a536db76dcf9ccc882a` |
| Release | `7ce4f1b93a2bba9ba07d8b92e724b915aac6e77db5984655315c37069d768249` |

About → FW 为 `UI-R57-INFO`。Debug／Release 的 ELF、BIN、MAP 均已更新，尚未烧录或实板验证。
