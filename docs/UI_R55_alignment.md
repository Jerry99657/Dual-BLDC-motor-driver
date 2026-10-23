# UI-R55-ALIGN：Run 居中与首页文字位置修复

后续 R56 保留本页全部对齐修复，并将双独立切轴改为长按左右键，见 [R56 按键说明](UI_R56_axis_keys.md)。以下为 R55 记录。

Run 的数值与单位恢复为整体水平居中。RUN / STOP / EDIT / RUN* 移至状态行，避免大数值与左侧状态重叠；双独立模式在当前轴状态后用 `*` 标记未提交的草稿。VBUS、反馈方式、已确认目标、步长和动态 OK 均保留。

首页仍使用原来的 12px 文泉驿字形。已对比 R53 的完整字库与 R54 子集，全部 36 个保留字形的编码逐字节一致；问题是生成子集后全局包围盒高度由 13px 变成 10px，KK_UI 的 ALL-height 顶部定位因而上移 3px。本次恢复原字体的 13px 行高元数据，不放大或缩小字形，也不修改 KK_UI 核心。Run Steps 的紧凑页头单独使用 TEXT 参考高度，保留其原有位置。

UI 主机回归通过，新增检查直接读取真实 framebuffer 的四个首页标签，验证框内水平、垂直边距以及原始 12px 字形高度；同时验证电压、电流、速度、位置模式的正负边界、数值与单位的整体中心及屏幕边界。原有长按、动画、双轴、HSO ON、保存、NOR 和 LCD 驱动回归保留通过。

| 构建 | Flash / 131072 B | RAM / 32768 B |
| --- | --- | --- |
| Debug | 130460 B | 22696 B |
| Release | 129748 B | 22592 B |

About → FW 为 `UI-R55-ALIGN`。无需修改 CubeMX，未烧录或实板验证。`tests/ui_host/render_preview.py` 生成 `build/ui-validation/r55-preview.png`，两套 ELF/BIN/MAP 位于各自的 Debug/Release 目录。
