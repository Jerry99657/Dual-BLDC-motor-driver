# R88：单文件上位机第 4～6 批

固件 `UI-R88-PC`，入口为 [pc_tool/index.html](../pc_tool/index.html)。延续 R87 的 USB CDC／USART1、遥测、曲线、CSV、模式和 Run 控制；本次补齐可编辑 Settings 写入、LCD 联动、辨识、校准、专题诊断及配置导入。无需重新配置 CubeMX，不修改 ST HAL／LL／CMSIS／USB／HSO 官方源码。

## 设置和保存

上位机与本地 Settings 共用 `ui_app.c` 的事件提交、范围校验、`MotorControl_SetOptions`、DAC 阈值设置和存储路径。修改拒绝时返回错误，不宣称已应用。可编辑项：

| 范围 | 项目 |
| --- | --- |
| M1／M2 各自 | Curr Loop、Pos Speed、Speed Limit、Speed／Torque／Position 步长 |
| 全局 | LED 开关／亮度、跟随比例／偏移、M1 IBUS 阈值 |
| 原有配置入口 | 工作模式、型号、Encoder／HSO／SixStep、ESC、View |

Pos Speed 上限与 Speed Limit 联动；最大范围保持板端现有 600 rad/s。HSO 下显示有效电流环 ON，六步下 OFF；导出／导入保留编码器模式的原电流环偏好。固定 PID、型号参数和辨识结果不提供任意写入。

远程型号选择也调用本地 `select_motor()`，执行相同的草稿清零和配置提交；切换型号／ESC 后不会带入旧运动目标。本地编辑框或 Run Steps 正在编辑时拒绝远程设置，避免其未提交草稿反向覆盖电脑修改。

保存采用原有约 2 秒静默合并，SPI2 忙时仍由存储仲裁延期。增加“立即请求保存”，并接入原来本地 Save 状态菜单项。受理、RAM 已更新、等待总线和 Saved 分别显示；只有 W25Q64 完成写入／校验才显示 Saved。LCD 初始化失败时，协议主循环仍可维护设置模型与保存。Run 目标在停止后按相同保存规则提交；电流目标仍遵守原有不持久化的设计。

## LCD、运动和操作

接受 Speed／Angle／Torque／Throttle／Follow 或 HSO Use 后切到 Run，显示对应轴、控制模式和已接受目标；仅修改 Settings 不跳页。位置显示使用当前机械圈次基准；双机同步显示主轴已接受目标，保持原轨迹参考／遥测含义不变。

跨自定义页或有本地编辑框时，只通过 KK_UI 公共导航、普通 Back 输入及现有去抖退出，再进入目标页；不访问 KK_UI 私有状态。自动 Back 使用 40 ms 按下／40 ms 释放脉冲，不参与物理长按左／右分类。Stop 清除待执行导航，KEY1 保持原 EXTI 全局停止。本地 Run OK 在目标未改变时能明确停止远程持有的轴；Back 仍停止输出。来源仲裁保持，改动草稿不代表自动接管 USB／USART 控制。

辨识：100～500 mA 的现有 Single M1 编码器检测，明确确认后启动，进入 Detect 页查看进度；其 OnEnter 不再清掉已经运行的检测。停止仍走立即执行的 Stop，完整成功结果才允许保存到原 AppSettings 检测字段。保存结果不会自动覆盖电机预设。重新对齐、IBUS 清除、恢复默认设置和现有 HSO 操作也有显式按钮；校准／辨识可能使转子转动。

专题诊断复用板端八组信息行：Drive、About、Motion、Encoder、ADC／VREF、HSO、Diagnostics／Flash、SixStep；可查询 M1／M2。临时生成文字后恢复原 View 和草稿，不改变 LCD 页面或电机焦点。各行带采样时的板端时间，整页不是原子快照。

## 导入／导出

导出的 JSON 包含 `format: "FOC-console-settings"`、`version: 1`、`editable`、原配置和状态快照。仅 `editable` 中的白名单设置允许导入；不会执行文件中的文本命令、运动目标或只读辨识值。

导入先校验全部范围和模式组合、预览差异并确认。随后 Stop，再锁存与 KEY1 相同的全局暂停，临时转 Single M1／Encoder，选择两轴型号，依次写入限速／位置速度／电流环偏好／步长／全局设置，恢复目标反馈与工作模式、View，最后请求保存。暂停屏蔽模式／型号切换附带的自动对齐，导入结束保持 GLOBAL STOP；检查状态，需要时显式请求对齐，再手动运行。任何拒绝或超时都停止后续命令；Stop 可中断批量流程。它不是原子事务，已生效的项不自动回滚；发生失败后读取实际状态，静止条件未满足时等待停稳再重新操作。连接和导入不会自动启动电机。

R88 HTML 保留对 R87 的优先控制／读回支持；新增操作要求 R88。配置导入使用本版导出格式。

## 增量协议

ASCII 请求、ID、长度限制、FIFO、即时 Stop、超时不重发规则沿用 R87。

```text
@31:App:1,2,240
@32:App:1,1,150
@33:App:0,5,2400
@34:App:0,13,300
@35:GetRow:2,4,0
```

`App:<axis>,<key>,<value>`：axis 0 为全局，1／2 为 M1／M2。三个参数必须为有限整数；它们不是存储偏移。

| key | 功能 | value 单位／范围 | axis |
| ---: | --- | --- | --- |
| 0 | LED 亮度 | 0～100 % | 0 |
| 1 | Pos Speed | 1～Speed Limit，rad/s | 1／2 |
| 2 | Speed Limit | 1～600，rad/s | 1／2 |
| 3 | 跟随比例 | ×100，−1000～1000，非零 | 0 |
| 4 | 跟随偏移 | ×100 rad，−6283～6283 | 0 |
| 5 | IBUS 阈值 | mA，500～4000，100 的整数倍 | 0 |
| 6 | 电流环偏好 | 0／1 | 1／2 |
| 7 | LED 开关 | 0／1 | 0 |
| 8 | Speed 步长 | ×10 rad/s，1～100 | 1／2 |
| 9 | Torque 步长 | ×100 V／A，1～100 | 1／2 |
| 10 | Position 步长 | ×100 rad，1～100 | 1／2 |
| 11／12 | 恢复默认／清除 IBUS | 0 | 0 |
| 13 | 启动辨识 | 100～500 mA | 0 |
| 14／15／16 | 锁存全局暂停／保存完整结果／请求保存设置 | 0 | 0 |

网页停止辨识使用优先 Stop。key 14 同时终止两轴及辨识，保持全局暂停；Run 面板另有“锁存暂停 · KEY1”，先发优先 Stop 再锁存。对齐和 HSO Use 等启动操作与普通 Run 共用请求代数／全局停止／新鲜度检查，并在实际启动临界区再次确认。

kind 32 保持 92 B payload，schema 改为 **2**、firmware **88**，21 个数值字段顺序不变。`GetRow:<axis>,<group>,<row>` 使用 1／2 轴、0～7 组和从零开始的行号，回复 kind **33**，payload **64 B**：

```c
uint16_t id;
int8_t result;
uint8_t source, axis, group, row, count;
char label[24], value[32];
```

固定缓冲清零、限长终止；帧头、序号、时间、CRC 和请求来源定向发送沿用原实现。USB／UART BUSY 时保留同一回复，不与普通遥测混写。诊断 JSON 也可记入 CSV，但不作为数值曲线回放。

## Flash 优化与验证

| 镜像 | Flash 占用 | Flash 剩余 | RAM 分配（含原预留堆栈） |
| --- | ---: | ---: | ---: |
| R87 基线，Debug／Release | 131008 B | 64 B | 26208 B |
| R88 完整交付，Debug／Release | **131060 B** | **12 B** | **27736 B** |

集成第 4～6 批后净增 Flash 52 B、RAM 1528 B。32 KiB RAM 链接区剩 5032 B；这不是实测最大栈余量。Flash 已接近满载，后续新增仍需逐项测量。

完整扩展依赖应用层无损文字压缩、原 LZSS 编码的最优字节成本选择、四个冷用户模块 Oz、LTO 独立段回收／对齐排序，以及精简的编译器 `__aeabi_f2d` 辅助入口。原 libgcc 将 float→double 与双精度加减放在同一对象；本项目只需前者。用户实现按位精确扩展 binary32，避免链接未调用的加减主体；未替换 ST 函数，也未改 ST 源文件或其编译命令。字号、原字模和图标保持原字节。

基线、试验、镜像和日志在忽略目录 `build/console-r88/`。正式源码只保留验证有效的方案；字模二次压缩试验已撤回。当前空间及最终 SHA256 以该目录 `final.json` 为准。

本次 Debug BIN SHA256：`3ffe5521b935a9bbbdbe686a20ddf758a35b982565cdb94147791c78d1d75004`；Release：`5e1858294af619c82b35f9d74cf4a41b19bee5e7afe98f72692bc5fbc388ff0d`。两个构建的 ELF／BIN／MAP 均已更新。

验证包含：完整电机主机回归；单独 100 万组随机 float 位模式、零／次正规数／无穷大／NaN payload 对照；真实 UI 设置校验／发布／保存、两轴 Run、编辑框／自定义页导航、停止、型号草稿清零、检测及无 LCD 路径；真实 C 回复供 JavaScript 消费、设置单位／范围、导入中止／Stop、CRC／分包／BUSY／断开；LL 中断、采样保护、USB、LCD 驱动；Debug／Release 干净构建与最终启动／MSP／向量检查。保护审计允许的唯一构建例外是准确匹配的用户 CMake 改动，其余 204 个文件及全部官方／核心编译命令严格比较。

尚未烧录、打开实际 COM 口或实测电机；真实控制耗时、浏览器授权／视觉布局与长时间通信仍需板上验收。
