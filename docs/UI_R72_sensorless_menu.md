# R72：Sensorless 设置目录与独立 Six Step 诊断

R73 保留本页菜单布局，将六步最低 Speed 向上取到 5 rad/s 倍数：2804 为 15、4015 为 10；当前固件及验证见 [R73 最低速度](Six_step_speed_floor_R73.md)。下文保留 R72 的菜单修改和构建记录。

2026-10-25，固件 **UI-R72-UI**。本轮只调整 KK_UI 应用层菜单、信息页和标题字库；电机控制、R71 六步启动资格、保护、采样、SPI 及保存格式继续沿用。

## 设置入口

```text
Settings
└─ System
   ├─ Work Mode
   ├─ Axis M1 / Axis M2
   ├─ Sensorless
   │  ├─ HSO Feedback
   │  └─ Six Step
   ├─ Gear Ratio
   ├─ Gear Offset
   └─ Align / Retry
```

`HSO Feedback`、`Six Step` 从 System 的直接条目移入同一个 Sensorless 子菜单，页标题为 `M1 Sensorless`。开关继续使用原绑定和业务事件，开启一种会关闭另一种；停止／静止准入、设置保存及取消语义不变。两个开关均关闭时沿用编码器反馈。

目录在所有工作模式下可查看；只有 **Single M1** 允许修改开关，Single M2 和全部双机模式下两项均禁用，仅 Back 可选。这样可明确无感功能属于 M1。KEY2 从 Sensorless 返回 System，再逐级返回 Settings／首页。

原 `Settings → Motor → 2804 / 4015 / ESC Auto` 的位置不变。ESC Auto 仍须先在 Single M1 下开启 Six Step。

## Status 分组

```text
Status
├─ Drive
├─ Motion
├─ Encoder
├─ ADC
├─ HSO
├─ Six Step
├─ System
└─ Axis M1 / Axis M2
```

原来放在 Drive 末尾的八项诊断完整移到 `Status → Six Step`，与 HSO 同级：

| 项目 | 含义 |
| --- | --- |
| 6S ZC | 连续有效过零／12、检测门槛字母、相位／周期比 |
| 6Step Duty | 实际命令占空比 |
| 6Step EMF | 最后一次浮相反电势残差，V |
| 6Step Speed | 预设为机械 rad/s；ESC Auto 为电气 Hz |
| 6S Hold | 起转稳定资格保持时间／500 ms |
| 6S Start EMF | 滤波后的启动 Uq 强度代理，V |
| 6S Cause | 最后一次过零相位／周期拒绝原因 |
| 6S Sample | OFF／ON 采样窗口 |

信息页固定标题为 `M1 Six Step`。即使 Status 当前查看 M2，该页仍明确显示物理 M1 的六步诊断，与固定属于 M1 的 HSO 诊断一致。Drive 保留通用驱动状态、输出、反馈、VBUS、型号及停止原因。数据源、刷新周期、单位与故障后保留行为不变。上下键滚动，KEY2 返回 Status 并保留选中项。

## 字库与排版

预览发现旧标题子集没有小写 `l`、`x`，缺字会以空格回退；仅判断 `OLED_GetGlyphAdvance()>0` 无法发现该问题。

按照 [LEDFont 官方 Agent API](https://ledfont.botelvdong.com/agent-api.md)，本轮实时查询目录并选用原 `wenquanyi_9pt`、12 px，生成实际标题所需的 44 个 ASCII 字符。解码器支持 BMP，本次字符均为 ASCII。只向接口提交显示字符及字体参数，未上传源码或工程路径。

标题数组从 559 B 增至 576 B，仅新增 `l`、`x`；原 42 个字形记录逐字节一致，保留原 13 px 行框，正文及大数字字体不变。服务响应未提供字体许可字段，本说明不为生成字模推定许可。覆盖检查现在直接核对字形记录，避免将空格回退当作字形存在。未修改 KK_UI／KK_OLED 核心。

## 验证与交付

- `python tests/ui_host/run.py` 通过：真实菜单进入与返回、编辑／取消、HSO 与 Six Step 互斥、后台保存、所有非 Single M1 模式禁用、七个 Status 分组及 M2 视图下的 M1 诊断、上下滚动、字形存在性，以及原按键／Run／存储／LCD 回归。
- `python tests/ui_host/render_preview.py` 生成真实 framebuffer 预览，并检查新页面标题、数据和禁用状态：[R72 预览](../build/ui-validation/r72-preview.png)。它使用主机测试桩，不是实板照片。
- Debug／Release 构建、链接后的强初始化／中断入口检查、ELF 导出 BIN 与交付 BIN 的一致性检查通过。本轮未更改控制实现，未烧录实板。

**无需更改 CubeMX。** 两套构建均为 Flash **131064 / 131072 B（余 8 B）**，RAM **23104 / 32768 B**。Debug 保留调试符号；保留原 RWX LOAD 段链接警告。没有删除功能或放宽保护以节省空间。

| 构建 | 文件 | BIN SHA256 |
| --- | --- | --- |
| Debug | `build/Debug/FOC_two.elf`、`.bin`、`.map` | `066267174d28cdb931a3e04264882bfca11f4da07286d53ca895943ecc449f32` |
| Release | `build/Release/FOC_two.elf`、`.bin`、`.map` | `1d304d5c7e8919f51aac0693c3fd8e8384bb47df71a9a2a6851c17fd42a45dc5` |
