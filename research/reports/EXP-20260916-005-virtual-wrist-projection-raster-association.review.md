# EXP-20260916-005-virtual-wrist-projection-raster-association: Technical Review Note

技术校验：`py -3 -m experiment_management validate
research/runs/EXP-20260916-005-virtual-wrist-projection-raster-association.json` 通过；650 个配置—杯位键
严格一对一连接。

观察（候选模型内）：位置不变量保护的当前 raster 重检中有 30 条可见、620 条不可见。30 条可见记录中，
29 条的杯体中心仍位于候选透视视锥外，1 条中心在候选视锥/裁剪范围内；10 条可见记录同时为射线命中。
620 条不可见记录中，228 条中心在相机后方，392 条中心在候选透视视锥外，6 条在候选深度裁剪范围外，
30 条为射线命中。保留条件允许重叠。

解释边界：杯体中心的投影分类不足以作为完整圆柱体 raster 可见性的充分代理，因此不应再将中心位置或
`mj_ray` 代理扩大为腕部相机覆盖结论。该关联不区分圆柱边界、面朝向、遮挡、GPU/驱动、渲染器或其他
因素，不能给出因果解释。

复核状态：`pending_human_review`；未采纳为能力或安全边界。进一步研究真实相机能力前，需要正式的
Orbbec capture smoke test 和随后标定；在用户暂不启用硬件的边界内，不继续从候选 raster 推断真机。
