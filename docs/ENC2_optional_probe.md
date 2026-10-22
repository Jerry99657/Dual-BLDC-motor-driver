# ENC2 可选设备探测（UI-R21-ENC2）

本页为 R21 历史记录。当前 R42 保留未启用 ENC2 的五次停止，但对启用的 ENC2 增加故障停机后每 250 ms 单帧重试；也保留显式 Align / Retry。当前固件、总线恢复和时钟配置见 [Encoder_reconnect_clock_restore.md](Encoder_reconnect_clock_restore.md)。

第二路 MT6701 通过 SPI2 / MT6701_CSN2 与 W25Q64 共用总线。原代码每毫秒发起一次读取，即使编码器未连接也持续累加错误，没有停止条件。本次将可选 ENC2 的轮询限制为连续失败 5 次后停止。

## 规则

- 只有实际发起的读取失败才计数，包括全零/全一帧、CRC 或状态校验失败，以及 HAL 返回错误/超时。
- 一个有效样本清除连续失败次数，累计错误数保留。
- SPI2 被占用、硬件仍忙或 W25Q64 片选为低时，只跳过当前轮询，不计失败。
- 连续第 5 次失败后，立即停止后续 ENC2 读取，直到 MCU 复位。后来插上编码器也不会自动重新占用总线，连接完成后复位即可重新探测。
- 每次已开始的 ENC2 事务仍先拉高片选；HAL 传输失败时执行 Abort，再恢复 SPI2 原 CR1/CR2，最后记录失败。达到上限以后不再切换 SPI2 配置或 ENC2 片选。
- 没有关闭 SPI2 外设或取消它的初始化，W25Q64 仍通过同一 SPI2 工作。M1 的 SPI1/DMA 编码器采样不受该上限影响。

ENC2 从启动起未接时，Status 的 Enc2 仍显示 ERROR，Enc2Err 应停在 5。若此前有成功采样或零星错误，则显示累计错误数，而非固定为 5。

## 验证

`python tests/motor_host/run.py` 已覆盖：未接时全一帧、超时/Abort、有效样本清零失败次数、运行中掉线、总线占用不计失败、计时回绕，以及停止后收到有效信号仍不自动探测。

扩展的共享总线测试同时编译真实 `MT6701.c`、`w25q64.c`、`app_storage.c`，通过模拟 SPI 和 NOR Flash 检查：探测期间和停止后保存均成功，实际进行擦除/页编程/读回校验及重新加载；每个 Flash 事务都校验编码器片选已释放、SPI2 配置已恢复。UI、存储和显示驱动回归亦通过。

2026-10-20 构建：Debug Flash 114004 B / RAM 21344 B；Release Flash 112660 B / RAM 21048 B。保留既有 RWX LOAD segment 链接警告，无 Flash 溢出。

固件为 `build/Debug/FOC_two.elf`、`build/Release/FOC_two.elf` 及同目录 `.bin`，About 显示 `UI-R21-ENC2`。

本次确认并修复了 ENC2 无限轮询，但尚未在实板上复现 Saved failed。共享 SPI2 在原设计中已逐事务恢复，因此不能仅凭错误计数就认定保存失败的唯一原因；若烧录后 ENC2 错误已经停止而保存仍失败，需要继续检查 Flash 的识别、擦写和读回校验结果。
