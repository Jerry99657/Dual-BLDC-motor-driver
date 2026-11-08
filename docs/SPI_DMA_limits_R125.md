# R125：SPI DMA 使能合并与限幅重复求值清理

固件 `UI-R125-DL`，Native 配置回执编号 125。本轮范围为两项用户代码精简；无需修改 CubeMX。

## 实际资源变化

| 阶段 | Debug / Release Flash | Flash 余量 | RAM 总分配 |
| --- | ---: | ---: | ---: |
| R124 基线 | 130248 B | 824 B | 30904 B |
| SPI DMA 使能合并 | 130240 B | 832 B | 30904 B |
| 限幅求值复用及 R125 资源 | **130184 B** | **888 B** | **30904 B** |

净节省 **64 B Flash**，RAM 分配不变，RAM 区链接余量 1864 B。分配包含既有堆、栈和保留区，不代表运行时栈高水位。标准 Debug／Release 的 ELF、BIN、MAP 已更新；R124 基线与报告保存于忽略目录 `build/dma-limits-r125/`。

## SPI DMA 使能合并

`spi_port.c` 的 `setup_dma` 原先分别开启 TE 中断、可选 TC 中断、DMA EN。现在先在 EN=0 时设置 CPAR／CMAR／CNDTR，再通过一次 CCR 写入开启本次需要的使能位。

- 继续使用生成句柄的实际通道、地址、格式和优先级。DMAMUX 映射不变。
- 全双工继续先使能 RX 通道／请求，再准备 TX；最后才发布 SPI TX DMA 请求。
- 每条通道从 EN=0 变为 EN=1 的时刻，地址／长度和所需错误／完成中断均已就绪。
- 原 TE 优先处理、迟到中断清理、BSY 检查、回调重启和 SPI2 编码器／Flash 仲裁保留。

实际运行驱动的寄存器模型统计完整一轮传输：

| 通道 | CCR 写入：R124 → R125 | IFCR 写入 |
| --- | ---: | ---: |
| SPI1 全双工 | 9 → **6** | 4 → 4 |
| SPI2 全双工 | 9 → **6** | 4 → 4 |
| SPI3 发送 | 5 → **3** | 2 → 2 |

计数包含启停和中断使能，不是实板耗时。新增测试在每一次 EN 上升写入时检查地址／计数、TE／TC 归属、RX 先于 TX 及格式位保留。

## 限幅求值复用

`motor_control.c` 仅在本次调用内用局部变量保存上限：

- M1 Position 默认／显式位置速度上限。
- M2 Speed 的对称速度限幅，以及 Position 的默认／显式位置速度上限。
- 六步 Speed 的有符号速度裁剪。
- HSO 命令的速度裁剪，以及普通 Speed 命令入口。

没有引入跨周期缓存或新增持久状态。后续命令／外环仍读取已发布的新设置；原输入准入、逐层限幅、默认值与异常值处理保持。位置速度仍受 Speed Limit 及电机型号最大速度约束，M2 原有 0.1 rad/s 下限保留。

测试对 R124 与当前 `motor_control.c` 的独立主机副本插入入口计数，不修改固件代码。5120 组实际命令比较结果：

| 求值 | R124 | R125 |
| --- | ---: | ---: |
| `speed_limit()` | 10880 | **6400** |
| `position_speed()` | 3200 | **2560** |

这组次数属于测试输入集合，不代表每秒运行次数或整板 CPU 占用率。两版本的目标速度、目标位置、Iq、运行状态、控制模式输出逐位一致；同时断言边界结果及设置状态未被修改。

## 验证

- 5120 组命令涵盖两轴、2804／4015、零／非法／默认／扩展设置字段、正负目标与正负零、位置速度默认／显式／超范围值及运行中发布新限速。
- SPI 原版／新版通过同一正常、TE／TC、BSY 延迟、回调内重启、抢占、迟到 IRQ、ADC／WS2812 隔离与 600 组 DMA 重映射测试，另比较写入次数。
- 双轴控制／Follow、外环故障锁存、HSO、编码器／W25Q64、协议、遥测、Native／JustFloat 回归通过。
- CubeMX 的 55 项变更检查、实际 ELF／BIN 启动检查、两轴外环 Thumb 仿真、两镜像各 139 项 CPU 异常保护通过。
- UI 与 R124 的 68 份画面／动画流、64857 次文字绘制及 870 条诊断输出一致；统一比较固件版本文字。HTML 前端、格式握手、串口异常恢复测试通过。
- 207 个受保护文件与官方／核心编译命令相对本轮基线一致。没有修改 ST 库、生成初始化、IOC、控制频率、保护门限或控制增益。

```powershell
python -X utf8 tests/spi_host/run.py --baseline build/dma-limits-r125/mycode/spi_port.c
python -X utf8 tests/motor_host/check_limits.py --baseline build/dma-limits-r125/mycode/motor_control.c
python -X utf8 tests/motor_host/run.py control motion sensorless encoder protocol telemetry justfloat
python -X utf8 tests/motor_host/check_startup.py
python -X utf8 tests/diag_host/check_outer_arm.py
python -X utf8 tests/flash_report.py final --directory build/dma-limits-r125
```

本轮未烧录，实际微秒收益待上板验证。可沿用 M1 30 rad/s 后启动 M2 100 rad/s 的条件，记录 `ADC IRQ Max`、`Outer Max` 和 `Outer E/G/S`；同时核对编码器、LCD 刷新和 W25Q64 保存。
