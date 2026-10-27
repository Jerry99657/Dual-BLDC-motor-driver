# R91：HAL＋LL 第6批，SPI／专用DMA运行时迁移

固件 **UI-R91-SPI**。SPI1 编码器1、SPI2 编码器2／W25Q64、SPI3 LCD 的运行时收发和五个专用DMA通道使用新的用户模块 `mycode/spi_port.c/.h`。HAL继续负责初始化／恢复时重新初始化；本轮无需修改CubeMX，也未推进第7批初始化替换。

## 净收益

以第5批完成时的真实R90 ELF／BIN／MAP为基线，Debug／Release均全量构建后比较：

| 构建 | R90 Flash | R91 Flash | 本批净节省 | 当前余量 |
| --- | ---: | ---: | ---: | ---: |
| Debug | 128688 B | **126236 B** | **2452 B** | **4836 B，约4.72 KiB** |
| Release | 128688 B | **126236 B** | **2452 B** | **4836 B，约4.72 KiB** |

RAM分配仍为27840 B，未新增静态缓冲或持久状态；链接未分配4928 B。分配量包含原有堆／栈预留，不是实测峰值栈余量。第5、6批相对R89累计释放4836 B。净收益包含新收发、错误处理、固件版本和协议配套更新，不是HAL符号大小之和。

同一版用户底层按迁移范围分别构建的Debug对照：

| 阶段 | Flash | 相对R90累计净节省 |
| --- | ---: | ---: |
| 6.1：仅SPI1 | 129724 B | −1036 B |
| 6.2：加SPI2与Flash | 128372 B | 316 B |
| 6.3：加SPI3，最终版本 | 126236 B | 2452 B |

单独迁移SPI1会暂时同时链接两套底层。三路完成后，实际ELF已移除HAL SPI IRQ、Transmit／Receive／TransmitReceive、DMA启动／结束／Abort等通用运行时及SPI DMA回调辅助函数。`HAL_SPI_Init`保留。ADC和WS2812仍需要HAL DMA，其通用DMA IRQ、启动及初始化依赖继续保留。

## 保留的功能与固定配置

| 外设 | 专用DMA | 时钟／用途 |
| --- | --- | --- |
| SPI1 | DMA1 CH1 RX、CH2 TX | MT6701，664062.5 Hz，CPHA=2EDGE |
| SPI2 | DMA2 CH1 RX、CH2 TX | ENC2同上；W25Q64为21.25 MHz、1EDGE |
| SPI3 | DMA1 CH4 TX | LCD，10.625 MHz、1EDGE |

APB1／APB2仍为170 MHz，现有NVIC／DMA优先级、引脚、8位全双工主机配置、软件片选及LCD初始化表保持。DMA1 CH3（WS2812）和CH5（ADC）继续交给HAL，用户分发器不清除它们的标志。

- 编码器CRC、状态、全零／全一判定、样本年龄和零点诊断保持。M1控制周期／毫秒超时看门狗、主循环SPI1复位恢复保持。
- ENC2未启用时五次失败停止；所需ENC2故障后250 ms低频重试、恢复后手动Run保持。SPI2预约、嵌套Flash片选区间、速度／相位切换、故障快照及复位恢复仍由原仲裁层负责。
- W25Q64命令／数据写出时同时读取RX，读取使用0xFF虚拟时钟；WEL、双次BUSY确认、A/B保存、验证、重试、JEDEC启动探测和错误编码保持。
- LCD仍使用8行RGB565缓冲、脏矩形、完整GRAM初始化和原偏移。正常DMA、提前完成、错误通知、超时中止及全屏重绘规则保持。LCD通信错误后在主循环复位／重新初始化SPI3，丢弃待发送字节，不复位其他SPI或电机。

## 中断与并发边界

编码器先准备并使能RX DMA，再使能TX请求；只有RX TC才有资格完成全双工帧。LCD由TX TC确认DMA已投递完数据。TE优先于TC，同时检查两路TE和剩余计数，不发布半帧。禁用／迟到向量不产生重复完成回调。

**DMA完成不等于最后一位已经离开SCK。** 只有TXE置位、BSY清零且发送FIFO空才发布完成／释放片选。IRQ不循环等待BSY：未满足条件时，由编码器周期／主循环Service或LCD主循环Service继续检查。BSY卡住仍进入现有编码器看门狗／显示刷新超时恢复，不无限占用中断。

停止DMA请求与两个通道属于同一短临界区；故障先标记错误状态，再停止请求。高优先级ADC因此不能把错误清理误当作成功完成，也不能在旧DMA IRQ尚未清理完时开启新一帧后又被旧IRQ关闭。完成回调在恢复原PRIMASK后执行，原应用层发布／片选所有权保持。

LCD是TX-only DMA，物理配置仍为全双工，未使用的MISO接收可能产生OVR。该端口不因预期OVR发起错误中断风暴；最终发送完成后排空RX并按DR／SR顺序清除OVR。MODF／FRE和DMA TE继续作为错误处理。编码器OVR依然是故障。

`SpiPort_*`是当前板级专用接口，不是所有HAL SPI模式的兼容实现。迁移后应用不得向这些SPI调用HAL运行时收发／Abort或HAL DMA启动接口；初始化／复位后重新初始化仍使用HAL。模块维护`hspi.State/ErrorCode`供现有仲裁和诊断读取，不依赖HAL DMA运行时状态机。既有`HAL_SPI_*Callback`名称保留为应用回调入口。

## 源码边界与验证

205个受保护文件中，仅有三个精确限定的应用变更：根CMake增加用户模块及其`-Os;-flto`；`stm32g4xx_it.c`的Includes及八个SPI／DMA USER CODE入口；用户LCD驱动替换底层调用并接入Service。其余202个文件逐字节一致，官方／原Core／图形核心的编译命令全部与R90一致。ST HAL／LL、CMSIS、USB、HSO、KK_UI／KK_OLED核心、外设初始化、链接脚本和`.ioc`未修改。

`tests/spi_host`使用本工程ST LL原函数体和G431寄存器常量；在主机低地址映射寄存器，仅用wrapper模拟DR读写、OVR及DMA W1C硬件副作用。覆盖五个真实通道索引／地址／计数／优先级、三端口并发、TE／TC／BSY、半帧拒绝、ADC抢占清理后立即开启下一帧、错误时不得误发布、重复／禁用中断、作用域中止、LCD恢复、1～1280字节轮询收发及tick回绕。静态检查实际CubeMX映射与固定格式，重新生成后应再运行。

既有编码器／真实Flash仲裁与保存、完整电机／六步机电模型／HSO（ST核为mock）／辨识、UI／LCD、IRQ／保护、USART、USB和单文件上位机回归通过。最终链接检查确认八个向量进入用户分发器，ADC／WS2812仍进入HAL DMA，并核对ELF／BIN一致。

```powershell
python tests/spi_host/run.py
python tests/motor_host/run.py
python tests/ui_host/run.py
python tests/irq_host/run.py
python tests/uart_host/run.py
python tests/usb_host/run.py
node tests/pc_tool/test_frontend.js
python tests/pc_tool/test_wire.py
cmake --build --preset Debug --clean-first
cmake --build --preset Release --clean-first
python tests/motor_host/check_startup.py
python tests/flash_report.py final --directory build/spi-r91 --allow-spi-user-code
```

本机R90基线、分步Debug产物和报告在忽略目录`build/spi-r91/`，不要覆盖原始baseline。最终ELF／BIN／MAP位于`build/Debug/`与`build/Release/`，继续参与Git备份。

最终BIN SHA256：

```text
Debug   e7a66bea1a2037083765d1b65668ef48b22643a13b5c43004968a45495b1dd53
Release eb8283e8e41f31f7f3eb4f6ad7a05052c19fb469fb8f81a003203d12b9cecd53
```

未烧录实板，未测量新Ctrl Max、实际DMA尾部延迟或LCD吞吐。板上先核对About FW，再复测Single M1／M2、双独立／Follow、运行中LCD刷新、停止后保存／重启读回，最后验证ENC1／ENC2断接后的原重连规则、KEY1／Break保护及USART／USB遥测。编码器线速不变；新增Service和短临界区的实际控制耗时应由板上Ctrl Max及必要的逻辑分析仪测量确认。
