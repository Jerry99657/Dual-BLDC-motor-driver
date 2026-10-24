# UI-R48-SPI2：共享 SPI2 的切换与故障恢复

2026-10-25: **UI-R69-6STEP** adds bounded startup detection retries; SPI2 mode/rates unchanged. See [R69 Flash initialization](Flash_init_retry_R69.md).

当前 UI-R53-SAMPLE 保留本页全部 SPI2/保存修复；磁链学习、CAPTURE 资格修改与当前固件资源见 [独立无感启动](HSO_sensorless_startup.md)。本页以下保留 R48 及此前版本的修复与构建记录。

## R48：Save ERASE、4/0/00/00 与 ENC2 同时失联

用户报告 R47 实板变为 **Save ERASE、Flash Err=4/0/00/00**，并且持续 ENCODER2 LOST。末项 00 表示连续两个 SR1 字节没有共同确认 WEL=1；HAL 返回 OK 不能证明芯片实际收到了写使能，也不能单凭这个值确认 PCB 或软件根因。本轮没有连接实板抓取波形。

已核对 SPI2：PB13=SCK、PB14=MISO、PB15=MOSI；ENC2 片选 PB12，W25Q64 片选 PD15；RX/TX 分别为 DMA2 Channel1/2，SPI2 和 DMA 中断入口对应正确。未发现引脚分配冲突。

代码中确认并修正以下缺口：

- 原仲裁通过保存/还原 CR1、CR2 切换设备，HAL Init 的相位和分频始终留在 Flash 设置。现在每次取得总线明确配置设备的 8 位、双线、MSB、软件片选、模式和分频，同时更新 HAL 的相位/分频；不把传输前的可变寄存器快照当成恢复依据。
- SPI2 原错误路径只 Abort/DMAStop 后还原寄存器，没有外设复位。现在发现超时、传输错误或无主的 HAL busy / BSY / FIFO / OVR / MODF / FRE 异常时，阻止新事务，由主循环停止两路 DMA、拉高两个片选、仅复位 SPI2 并重新 HAL_Init。初始化失败保持总线不可用，迟到回调不能发布样本；成功初始化也必须收到新有效编码器帧才恢复编码器健康状态。
- 增加 DMA 的毫秒时间看门狗：关闭 M2 实时采样、ADC 调度停止或 tick 回绕时，挂起的传输仍可清理，不再仅依赖 FastService 的周期计数。
- W25Q64 传输失败或不忙但 WEL 未确认时，先记录错误，再交给同一主循环恢复流程；恢复期间拒绝新的嵌套事务，Flash 外层事务未释放前不复位。继续保留 BUSY 等待、有限保存重试和读回 CRC；未放宽写入条件或自动解除芯片保护。
- CubeMX `.ioc` 与生成的 SPI2 Init 统一为 **NSS Pulse Disabled**。以前仅在 main 清硬件 NSSP，HAL Init 中仍为 Enable；现在重新初始化也保持 Disabled。该不一致本身尚不能证明是此次实板故障原因。
- DMA 回调发布样本期间不再被 ADC 的传输超时计数误判为仍在收帧；Flash 嵌套锁的释放也加短临界区。

时钟保持：APB1/2=170 MHz，ENC1/2=Mode 1、/256、约 664.06 kHz；W25Q64=Mode 0、/8、21.25 MHz；其他外设速率未改。未启用 ENC2 仍五次实际失败后停止；需要 ENC2 时仍在输出释放后每 250 ms 单次探测，通信恢复不自动 Run。无需手动重新配置 CubeMX，后续重新生成时确认 SPI2 的 NSS Pulse 为 Disabled。

Status 新增：

| 项目 | 含义 |
| --- | --- |
| SPI2 Fault | 最近一次申请总线恢复时的 `CR1/CR2/SR`，均为四位十六进制，在清理前记录；不是 Flash 的 SR1。没有恢复事件时全零。 |
| SPI2 Reset | 本次上电以来成功复位并重新初始化的次数，16 位计数；不等于编码器成功重连次数。 |

上板确认 About → FW 为 **UI-R48-SPI2**，保持停止重复 Single M1 → HSO Feedback 的保存操作，再检查 ENC2。若仍失败，记录 **Save Last、Flash Err、SPI2 Fault、SPI2 Reset、E2 Last 和 E2 SPI/Frame**；寄存器正常但反复读到 00 时，仍需结合片选/SCK/MISO 波形定位，不能把硬件低电平误判为写入成功。

新增 `m2-recovery` 故障注入：设备切换前模式漂移、HAL Init 与硬件一致性、关闭实时采样/ADC 停止/tick 回绕、Abort 失败且伪 READY、粘滞 BSY/FIFO、HAL_Init 失败、迟到/重入回调、Flash 锁内禁止复位、重新有效采样和恢复后 Flash 读取。原五次停止、250 ms 重试、真实 W25Q64 驱动配 NOR 模型的保存重载/故障/忙等待测试通过；完整电机/HSO、UI/LCD、USB 主机测试通过。最终 ELF 启动顺序、强中断入口、控制分发路径与 ELF/BIN 一致性检查通过。模型没有验证实板电气波形或实际中断耗时。

| R48 构建 | Flash / 131072 B | RAM / 32768 B |
| --- | --- | --- |
| Debug | 129820 B（余 1252 B） | 22128 B |
| Release | 129036 B（余 2036 B） | 22040 B |

Debug BIN SHA256：`71ba73ed9f2df6703a55a7b0f4a83c98904a7e0e5930e6d42af73a4378a9006a`。
Release BIN SHA256：`a8cd0460f9c5f8583b540a6a924909b2a1061a1935cf85e137af28b70be215b3`。
两套 ELF/BIN/MAP 已更新；原 RWX LOAD 警告保留。尚未烧录实板。

## R47 历史记录：4/0/00/03 的 BUSY 等待修复

用户在 R46 上报告 **Save WRITE、Flash Err=4/0/00/03**。按 R46 的单字节诊断定义：写使能确认阶段的状态传输成功、HAL 无错误，读取的 SR1=03，即 BUSY=1、WEL=1。这不是 WEL 未置位，也没有证据表明发生了 HAL OVR。BUSY 期间不能继续发编程命令，状态寄存器可以连续读取；WEL 在相应擦写完成后清除。[Winbond 数据手册 Revision M，7.1、8.2](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/908/W25Q64JV-RevM-12242024-Plus.pdf)

数据仍不能证明为什么状态机前一次空闲判断与随后 SR1=03 不一致。可能涉及状态切换窗口或读取异常，尚未实测波形。本次确认并修正的是：旧状态机在 BeginPageProgram 被接受之前先切到 PROGRAM；Begin 因 BUSY 被拒绝后，它直接报写失败、消耗一次重试并重新擦除，没有保留原步骤继续等待。

R47 修改：

- WREN 前先读取状态，确认不忙；若 WREN 前后读到 BUSY=1，返回“暂缓”，不绕过 BUSY/WEL 条件。
- 异步设置保存遇到此类暂缓，恢复原状态、原计时起点和同一保存快照，暂显 **Save WAIT**，等待后继续；不弹保存失败，不消耗重试次数，也不重复擦除。只有实际写入并读回/CRC 校验成功才显示 Saved。
- 等待保留原期限：页编程内部忙 10 ms，其余设置保存等待 1000 ms；持续忙仍报 **Save BUSY TMO**，并保留忙状态诊断，例如 `4/0/00/03`，不无限延期。
- 计时改用存储服务当下的 HAL tick，并在命令返回后记录开始时间。已发出命令后至少跨过两个 tick 才再次轮询，保证至少一个完整毫秒的启动间隔；主循环较早保存的时间戳不再参与阶段计时。
- 一个 05h 事务在同一 CS 下连续读两个状态字节。BUSY 任一字节为 1 就等待，WEL 两字节都为 1 才认为已使能；其他位保留后一个字节。R47 的 Flash Err 末项显示这个**保守判断值**，不再表示一个原始单字节；不会多数表决放行忙状态。

无需修改 CubeMX、SPI 速率、编码器恢复或 HSO 控制参数。开启 HSO Feedback 后可暂显 Save WAIT；正常应随后 Saved。若最终仍是 Save BUSY TMO，请记录它和 Flash Err，说明状态没有在原期限内解除，仍需实板定位；不要将持续 BUSY 强行判定为写成功。

新增 `busy-admission` 故障注入场景，模拟前序空闲读取后 WREN 被忙芯片忽略、随后确认 SR1=03：测试保存保持等待，解除后仅一次擦除和一次设置编程，同一快照正确重载，期间新 RAM 编辑保留。还覆盖持续 `00/00/03` 状态竞态不会重置超时、tick 回绕、陈旧主循环时间戳，以及两个连续状态字节中仅一个忙仍不能放行。模型复现的是该诊断分支及恢复逻辑，不是已证实实板产生异常读取的电气机制。

完整电机/HSO、编码器、Flash/存储、UI/LCD、USB 主机测试通过；最终 ELF 的启动顺序、强中断入口、LTO 后控制分发路径及 ELF/BIN 一致性检查通过。未烧录或实板验证。

| R47 构建 | Flash / 131072 B | RAM / 32768 B |
| --- | --- | --- |
| Debug | 129388 B（余 1684 B） | 22096 B |
| Release | 128612 B（余 2460 B） | 22008 B |

About → FW：**UI-R47-BUSY**。Release BIN SHA256：`61c779356a0569b720ae9aa0dfb123ebdbf8f8fbf2ba6038db297efc8598c649`。ELF、BIN、MAP 已更新；原 RWX LOAD 警告保留。

## R46 历史记录：实板 Save WRITE 与同步收发修改

用户在 R45 上继续复现：Single M1 → 开启 HSO Feedback，电机未运行，弹出 **Save WRITE**；Status → Save Last 同样为 Save WRITE，Save Bus=AVAILABLE。因而这不是仅观察到保存中的中间状态，也不是 ENC2 实时预留导致的延期。该信息把失败范围缩到页编程入口，但仍不能区分写使能确认、命令或数据传输，更不能直接证明某个 HAL 错误或硬件根因。

R46 修改 `w25q64.c` 的发送：命令、地址和页数据改用 **HAL_SPI_TransmitReceive** 同步接收并丢弃输入。原代码使用双线 SPI 的 HAL_SPI_Transmit，该 HAL 在发送结束后清理接收 FIFO/OVR；新路径在发送过程中同步消耗接收数据，避免依赖这种末尾清理。它是针对该发送路径的修改，**尚无实板错误寄存器证据证明 R45 的失败就是 OVR**。

新增独占 Flash 事务使用的 256 B 接收丢弃缓冲，支持完整页发送；不关控制中断，不新增 DMA 占用，不修改 SPI2 的 Mode 0、/8 或编码器的临时 Mode 1、/256。原有仲裁、WEL 检查、失败清理、一次重试、A/B 副本及读回 CRC 继续保留。HSO 启动与电流/母线保护未调整。

Status 新增 **Flash Err**，显示 `操作/HAL返回值/HAL错误/SR1`。前两项为十进制，后两项为十六进制；在 Abort 清理之前记录 HAL 错误，保存最终失败后保留。SR1 是此前最后一次成功读取的状态值，不是失败时的同时采样。无底层错误时显示 `0/0/00/00`，例如保存成功、纯读回校验失败或内部 BUSY 超时；这不能单独证明所有硬件状态正常。

| 操作号 | 失败位置 |
| --- | --- |
| 1 | 06h 写使能命令发送 |
| 2 / 3 | 05h 状态命令发送 / 状态字节接收 |
| 4 | 状态传输成功，但 WEL/BUSY 检查不通过 |
| 5 / 6 | 页编程命令与地址 / 页数据发送 |
| 7 | 扇区擦除命令与地址发送 |
| 8 / 9 | 普通读取或 JEDEC 命令发送 / 数据接收 |
| 10 | 参数越界，未发送命令 |
| 11 | 未取得 SPI2 事务锁 |

HAL 返回值：0=OK，1=ERROR，2=BUSY，3=TIMEOUT。HAL 错误为 STM32 HAL 位图，例如 04=OVR，20=FLAG。举例：`4/0/00/00` 是传输成功但 WEL 未置位；`6/1/04/02` 是页数据传输报 ERROR/OVR，最后成功读取的 SR1=02。这两种情况在故障注入中都能产生 Save WRITE，新诊断已经区分它们。

上板先确认 About → FW 为 **UI-R46-FLASH**，保持电机停止，再重复原操作并等待 Saved。若仍失败，记录 **Flash Err 的四项完整数值**，同时保留 Save Last；不要只记录 Save WRITE。本轮未烧录实板，是否消除用户的触发问题仍待验证。

R46 验证：完整电机/HSO/编码器回归、真实 Flash 驱动配 NOR/SPI 模型、UI/存储/LCD、USB 测试均通过。新增测试要求 Flash 命令和页数据使用同步收发，覆盖完整 256 B 页读写、跨页及超长拒绝、WEL 未置位、命令/数据失败、清理前错误码保留、有限重试及 A/B 副本保留。它不是实板中断时序或电气波形的验证。

TIM HAL 加入既有 LTO 优化以保留修复和诊断所需空间；未修改初始化参数。最终 ELF 检查除启动顺序、强中断入口和 BIN 一致性外，另确认 LTO 后 ADC/TIM 分发函数仍含 HSO/FOC 与位置/速度控制路径。

| R46 构建 | Flash / 131072 B | RAM / 32768 B |
| --- | --- | --- |
| Debug | 129300 B（余 1772 B） | 22096 B |
| Release | 128524 B（余 2548 B） | 22008 B |

Release BIN SHA256：`2047f116e6252a1d5646024e13c6af35edf8b9afec8bc6b5d41b2a736f1d678a`。ELF、BIN、MAP 均已更新；沿用原有 RWX LOAD 警告。无需修改 CubeMX。

## R45 历史修改与验证记录

用户实板现象：切到 Single M1、开启 HSO Feedback，电机尚未运行，界面显示 Save failed。本轮没有连接实板抓取 SPI 波形或底层 HAL 错误，**尚不能确认 HSO 开关触发首次通信失败的根因**；未将其归因为 PWM 干扰或直接占用 SPI2。

代码核查中确认：HSO 使用 M1 的电流/相电压和 CORDIC，不访问 SPI2；Single M1 不预留 ENC2 的实时总线。已有保存流程却缺少 SPI 失败后的清理，以及写使能的芯片端确认；一次传输/读回异常会立即结束保存，UI 仅报泛化的 Save failed，无法区分失败阶段。

## 本次修改

- `w25q64.c`：每个实际传输失败都先拉高 Flash 片选，再由主循环执行 HAL_SPI_Abort 清理，最后释放 SPI2 仲裁锁；不屏蔽 ADC 控制中断，不修改 SPI 分频和 CubeMX。
- 发出 06h 后读取状态寄存器，要求 BUSY=0、WEL=1 才允许擦除或编程。HAL_OK 只证明 MCU 发出了数据，不能证明 Flash 已接受写使能。WEL/BUSY 定义及写指令前置条件见 [Winbond W25Q64JV 原厂数据手册 Revision M，7.1 与 8.2（TI 托管副本）](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/908/W25Q64JV-RevM-12242024-Plus.pdf)。未自动解除芯片写保护。
- `app_storage.c`：对异步设置保存的 SPI/命令/读回/校验异常，保留同一不可变快照，最多完整重试一次擦除→编程→读回→CRC；重试仍写原先选定的备用扇区，不覆盖上次有效 A/B 副本。重试期间的新 RAM 编辑仍交由下一次保存。
- 持续 BUSY 超时不重试，不无限擦写；程序忙阶段仍保留 10 ms、擦除阶段 1000 ms 的门限，重试空闲等待也有 1000 ms 上限。SPI2 被实时编码器预留时继续延期，不因延期直接报保存失败。
- UI 失败提示改为具体阶段，Status 新增 **Save Last**，便于弹窗消失后继续核对。Save Bus 仍显示 AVAILABLE/DEFERRED；AVAILABLE 只代表没有实时预留，不代表芯片通信正常。
- H Error 在启动时为 NaN 时直接显示 `--`，避免先做无效的浮点到整数转换。

没有关闭保存校验，也没有移除或放宽 ADC、电流、母线、编码器或 HSO 保护。本轮未继续调整主从跟随与编码器判帧算法。

## 上板核对

1. 烧录 `build/Release/FOC_two.bin` 或对应 ELF，确认 About → FW 为 **UI-R45-SAVE**。
2. 保持电机停止，重复 Single M1 → 开启 HSO Feedback，停止按键约 2 秒，等待后台保存。
3. 正常应出现 Saved，Save Last=Save OK。复位后核对 Single M1 和 HSO Feedback 是否保留；不要只以弹窗判断持久化成功。
4. 若仍失败，记录以下 **Save Last 的完整内容**，并记录 Save Bus。无需先启动电机才能定位。

| Save Last | 含义 |
| --- | --- |
| Save NO FLASH | 初始化时未识别到 Flash，不能保存。 |
| Save STATUS | 状态寄存器的 SPI 发送或接收失败，两次保存尝试仍未恢复。 |
| Save ERASE | 写使能确认或擦除命令未成功，重试仍失败。 |
| Save WRITE | 写使能确认或页编程命令/数据发送未成功，重试仍失败。 |
| Save READ | 编程后读回的 SPI 事务失败，重试仍失败。 |
| Save VERIFY | 读回内容、记录结构或 CRC 校验不通过，重试仍失败。 |
| Save BUSY TMO | 芯片内部 BUSY 未在相应阶段的时限内释放。 |
| Save OK | 初始化尚无异步保存错误，或最近一次异步保存已验证成功；不能单凭此字段证明刚改动已落盘，应同时等待 Saved。 |

保存执行中该字段会显示当前阶段；以最终 Saved/错误弹窗之后的值为准。真实总线故障、供电/布线问题或芯片保护不会被强行改判成功；如果实板继续失败，需要依据上述阶段继续定位。

## 验证

`tests/motor_host/run.py` 新增真实 W25Q64 驱动 + SPI2 仲裁器 + AppStorage 的 `hso-save` 场景：保存 Single M1/HSO 位并重载；ENC2 缺失五次后停止；拒绝未接受的 WREN；分别注入状态、擦除、编程、读回的单次 SPI 超时，模拟未 Abort 前持续锁住的外设；验证一次重试可恢复。另覆盖两次校验失败、持续 SPI 失败和 BUSY 超时，确认不会假报成功、无限重试或损坏原有效副本。它是故障注入测试，不是实板 HSO 触发条件的复现。

电机/HSO、双电机协调、编码器恢复、UI/存储/LCD、USB 主机测试均通过。检查了最终 ELF 的初始化顺序、强 MSP/中断入口、R45 标识及 ELF/BIN 一致性。

| 构建 | Flash / 131072 B | RAM / 32768 B |
| --- | --- | --- |
| Debug | 131068 B（余 4 B） | 21800 B |
| Release | 130308 B（余 764 B） | 21728 B |

为容纳修复，ADC HAL 与既有应用初始化共享 LTO，libc 平台桩按 -Os 编译；保留 Debug 符号、现有控制保护和 HSO 算术优化设置。Flash 余量极小，后续扩展仍需继续优化。链接沿用原有 RWX LOAD 警告。

Release BIN SHA256：`d3dc29e4eb5fbb7e766acc451ca93754147324f6aef7e27fce8e3352a305c80e`。本轮未烧录或实机验证。
