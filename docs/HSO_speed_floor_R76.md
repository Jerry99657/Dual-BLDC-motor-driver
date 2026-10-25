# R76：HSO 最低速度向上取到 5

固件 `UI-R76-HSO`，2026-10-25。Flash 优化停在 R75，本轮只调整 HSO 的最低非零 Speed 指令。

机械速度下限为 `ceil((2π × 5 Hz / 极对数) / 5) × 5 rad/s`。2804（7 极对）和 4015（11 极对）均为 **5 rad/s**，反向为 **−5 rad/s**。

Run Steps 若设为 0.1 rad/s，调值为 `0 → 5.0 → 5.1 → …`；向下调到下限后再一步回到 0。反向为 `0 → −5.0 → −5.1 → …`。0 仍为滑行停止，修改草稿仍须 OK 确认。

`StHso_MinSpeed()` 统一提供 Run 和 Status → HSO 的 H Min；`StHso_ValidTarget()` 同步校验控制／串口指令。非零指令绝对值小于 5 rad/s 会被拒绝并保留原目标；不会因无效指令启动或停止电机。Speed Limit 低于下限时只允许停止。旧保存值低于下限时只调整 Run 草稿，等待确认后再提交。

本轮没有修改观测器内部的电气频率、角度、磁链、接管及失锁判据，也没有修改 HSO 启动流程、六步下限、HAL、ST 库或 KK_UI 核心。无需修改 CubeMX。

验证包括实际 HSO 适配器的两套预设／正反转边界、紧邻下限的浮点值拒绝、串口拒绝后原目标与输出保留、Run 的 0／±5／±5.1 操作及原控制／UI 回归。ST 核心由测试桩模拟，尚未上板验证。

Debug／Release 均构建成功，Flash **128304 / 131072 B**（余 **2768 B**），RAM **23080 / 32768 B**。相比 R75，范围校验增加 32 B Flash，RAM 不变。原 RWX LOAD 链接警告保留。固件为 `build/Debug/FOC_two.elf/.bin/.map` 和 Release 同名文件。

BIN SHA256：

- Debug：`ee6a2fc645c790687cd0da25ad5fab37b6c01817ddf9377764e6e503942adbdc`
- Release：`b2781e0f7e81a8086bfbe9114a00f21a289c28282548ca9a0410d7540dd85b16`
