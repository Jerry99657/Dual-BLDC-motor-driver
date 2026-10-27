# R90：HAL＋LL 第5批，USART1 运行时迁移

固件 `UI-R90-UART`。USART1 固定使用 115200、8N1、无硬件流控、FIFO 关闭；保留 CubeMX 生成的 HAL 初始化，将运行时收发、中断和接收错误恢复交给用户模块 `mycode/uart_port.c/.h`。本轮无需调整 CubeMX。

## 实测净收益

以本轮开始时 R89 的实际 ELF／BIN／MAP 为基线，两种模式均全量重编译后对照：

| 构建 | R89 Flash | R90 Flash | 净节省／当前余量 |
| --- | ---: | ---: | ---: |
| Debug | 131072 B | 128688 B | **2384 B，约2.33 KiB** |
| Release | 131072 B | 128688 B | **2384 B，约2.33 KiB** |

已分配 RAM 从 27832 B 变为 27840 B，增加 8 B；链接未分配 RAM 为 4928 B。这里的占用包括原有链接栈／堆预留，不代表实测峰值栈余量。

净收益包含新增 LL 收发、故障行丢弃、固件版本和配套协议更新，并非仅统计被移除的 HAL 函数。没有删减任何现有产品功能，也没有改变官方库编译选项。

实际 ELF 已移除 `HAL_UART_IRQHandler`、`HAL_UART_Receive_IT`、`HAL_UART_Transmit_IT` 及通用 `UART_RxISR*`／`UART_TxISR*`，包括 FIFO／16位收发分支。USART1 的 HAL 初始化及其必要辅助函数仍然保留。本轮没有迁移 SPI／DMA 或初始化流程。

## 收发和错误行为

- `Uart1_Init()` 在原先启动串口接收的位置调用，启动持续 RXNE 和错误中断。接收无需逐字节重新调用 HAL，也不会在 ORE 后退出接收状态。
- `Uart1_TrySend()` 只由主循环调用。接受最多128 B，复制到自有静态缓冲后发布；忙时拒绝并保留在途数据。TXE逐字节发送，最后一个字节写入后转为等待 TC，最后停止位发完才释放发送资格。
- USART IRQ 每次最多处理一个接收字节和一个发送事件，保持非阻塞。PE／FE／NE／ORE 仅通过对应 W1C 位清除；接收故障不取消正在发送的帧，也不对接收寄存器执行可能丢弃后续新字节的延迟 flush。
- 接收出错时调用 `Protocol_DiscardLine(PROTO_SRC_USART)`，丢弃该通道当前残缺命令，直到后续收到干净换行。另一通道的半行数据不受影响。错误字节本身不作为命令或换行处理；必要时先发送一个空行恢复行边界，再查询状态。
- 普通命令仍在主循环执行；干净的 `Stop` 仍在 RX 中立即处理并取消待执行队列。遥测忙时跳过当帧，RPC 回执等待发送资格及原有超时处理继续沿用。

USB CDC 的实现及协议帧布局保持；schema 2、92 B配置回执、12类遥测和 W25Q64 保存格式保持。单文件 `pc_tool/index.html` 更新到R90，兼容R88／R89的完整功能及R87原有优先功能。

HAL 只负责 USART1 初始化；启动本用户模块后，应用不得再向同一外设调用 HAL 中断收发接口或以 `huart1.gState` 判断当前发送忙闲，运行时所有权由新模块统一管理。

## 源码边界与重新生成

`Drivers/`、ST USB中间件、HSO、KK_UI／KK_OLED 核心、`.ioc`、链接脚本及 `Core/Src/usart.c` 均未修改。GPIO、时钟、NVIC优先级和所有通信速率保持。

仅在 `main.c` 的 USER CODE 区域接入模块、删除旧应用回调／单字节缓冲；在 `stm32g4xx_it.c` 的 USER CODE 区域转入 `Uart1_IRQHandler()` 后返回。生成的 HAL IRQ 调用保留在原位置，但不可达。CubeMX 的 Keep User Code 应继续开启；重新生成后运行链接检查。

根 CMake 仅增加新用户模块及其 `-Os;-flto`，所有原有文件编译命令不变。`tests/flash_report.py --allow-uart-user-code` 只允许这三个文件中精确列出的应用变更；205个受保护文件中其余202个文件逐字节保持，官方／核心编译命令全部与基线一致。该开关不会放开任意 Core 或库修改。

## 验证

新增 `tests/uart_host` 从本工程未经修改的 ST LL 头文件提取实际函数体及芯片寄存器常量，只模拟硬件副作用。验证全部字节值、15种PE／FE／NE／ORE组合、W1C、禁用中断源、错误后继续接收、RDR读出后新字节保留；1～128 B发送、全双工、发送中接收错误、BUSY缓冲不变、TC前禁止复用、返回前即完成及PRIMASK保持。

真实协议测试另验证受损 `Speed` 不执行、跨通道碎片保留、恢复后命令有效和立即 Stop。完整电机／六步机电模型／辨识／HSO（ST核为mock）／编码器／采样保护／UI／存储／LCD／IRQ／USB／单文件上位机回归通过。Debug／Release全量构建、真实向量分发、HAL运行时依赖移除及ELF／BIN一致性检查通过。

```powershell
python tests/uart_host/run.py
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/usb_host/run.py
node tests/pc_tool/test_frontend.js
python tests/pc_tool/test_wire.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/uart-r90 --allow-uart-user-code
```

本机基线及报告在忽略目录 `build/uart-r90/`；最终可备份产物位于 `build/Debug/`、`build/Release/`。审计命令依赖本轮原始baseline，不要覆盖它。

Debug BIN SHA256：`55148dba482ba7b39373438afc5beed45e2ef38f299d40556a407641efd57f5b`

Release BIN SHA256：`93ffca8db39805d9b669032a305f6613b06186502eb34915314ca90e87d16f9f`

本轮未烧录或连接真实串口。实板复测先核对 FW 为 `UI-R90-UART`，分别使用 USART1／USB 查询与修改设置，连续接收遥测，验证运动／Stop和KEY1，再测试发送过程中持续下发查询。电机算法未改变；Ctrl Max和实际串口误码恢复仍需板上测量。
