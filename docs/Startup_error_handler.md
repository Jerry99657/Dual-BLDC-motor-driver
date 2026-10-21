# UI-R33-BOOT：上电立即进入 Error_Handler

本次在 `Core/Src/main.c` 找到被破坏的启动注释：

```c
/* Reset of all peripherals, Initializes the Flash interface and the Systick. *  HAL_Init();
```

缺少 `*/`，导致 `HAL_Init();` 一直被注释到下一个 USER CODE 标记的结束符。旧 R32 ELF 中也没有 `HAL_Init / HAL_MspInit` 符号，确认调用没有进入固件。

本项目的 HAL 将 `uwTickPrio` 初始设为无效值 16（4 个优先级位）。正常由 `HAL_Init()` 调用 `HAL_InitTick(TICK_INT_PRIORITY)` 设置有效值，并调用板级 `HAL_MspInit()` 打开 PWR、SYSCFG 时钟。跳过这一步，`SystemClock_Config()` 内的 `HAL_RCC_ClockConfig()` 最终调用 `HAL_InitTick(uwTickPrio)` 会返回 HAL_ERROR，随后进入 Error_Handler。上述调用关系已对照工程内 HAL 源码核查，不能仅凭编译通过排除此问题。

R33 恢复独立、可执行的 HAL 初始化，并检查返回值：

```c
if (HAL_Init() != HAL_OK) Error_Handler();
SystemClock_Config();
```

两者之间仍保留 CubeMX USER CODE 区域。主从轨迹算法沿用 R32，无需修改 CubeMX。Error_Handler 继续先关闭 TIM1/TIM8 主输出，再关中断进入死循环；没有绕过错误检查。

## 后续错误定位

在调试器 Watch 中查看以下全局变量：

| 变量 | 含义 |
| --- | --- |
| g_boot_stage | 最后进入的启动阶段，见下表 |
| g_error_return_address | 进入 Error_Handler 时的 LR 返回地址 |

| g_boot_stage | 阶段 |
| --- | --- |
| 1 | HAL_Init |
| 2 | 系统时钟及 CRS 配置 |
| 3 | CubeMX 外设初始化 |
| 4 | 编码器、存储、控制器、LCD/UI 初始化 |
| 5 | MotorSampling_Start |
| 6 | TIM2 外环启动 |
| 7 | 启动流程结束，进入主循环 |

阶段号不是“该阶段已成功”的标志。采样启动失败仍沿用原来的禁用功率输出、界面报错策略，之后可以进入主循环，因此阶段 7 也不表示所有外设/电机健康。

Error_Handler 的 noinline 声明及诊断变量放在 USER CODE 中。LR 可结合当前 ELF 的反汇编或 `arm-none-eabi-addr2line` 查询；应清除 Thumb 地址最低位，并查看返回地址前的调用指令。尾调用优化时地址可能指向上层调用者，不是每个 HAL 错误都有唯一源码行。必须使用与板上固件匹配的 ELF。

## 验证

本页记录 R33 的启动修复，当前 R36 继续保留；Debug / Release 已重新生成 ELF、BIN、MAP，About → FW 应为 **UI-R36-SMOOTH**。尚未烧录或实机验证。

构建后执行 `python tests/motor_host/check_startup.py`，检查最终 ELF 中 HAL_Init 在时钟/外设调用之前、HAL_MspInit 为强符号、诊断变量存在、固件版本正确及 ELF/BIN 一致。工具从 Debug 的 CMakeCache 或 PATH 查找，也可用 `--toolchain-bin` 指定 GNU ARM 工具目录。新增检查不依赖在文本中搜索 `HAL_Init`，能够发现函数名字仍在注释内、实际调用消失的问题。

UI/存储/LCD 主机回归通过。R33 历史资源为 Debug Flash 131044 B、RAM 21552 B；Release Flash 130300 B、RAM 21456 B；当前资源以 [双电机文档](Dual_motor_control.md) 为准。两种构建均未溢出，原有链接警告仍保留。启动顺序的静态验证不代替晶振、供电和所有外设的板上验证。
