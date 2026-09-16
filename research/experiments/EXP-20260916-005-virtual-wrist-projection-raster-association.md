# EXP-20260916-005-virtual-wrist-projection-raster-association: Experiment Plan

问题：`EXP-20260916-004` 的当前位置不变量保护 raster 重检出现 30 条可见记录。将其同
`EXP-20260916-002` 的候选相机中心投影条件逐条关联后，当前可见/不可见条目在相机后方、候选深度
裁剪范围外、候选透视视锥外/内以及射线命中条件中的分布是什么？

输入和控制项：只读取 002 的投影表与 004 的 raster 重检表；按 `(configuration_index, cup_index)`
严格一对一连接 650 条。保留所有原始布尔条件，分别统计当前 raster 可见和不可见子集；条件允许重叠。
禁止模型加载、渲染、感知、actuation、控制器、动力学、IK、轨迹优化和硬件 I/O。

判据：恰好连接 650 条唯一索引，输出可见/不可见子集规模和每个保留条件的计数；索引缺失或重复即为
运行失败。计数差异不是因果检验、通过阈值或能力阈值。

局限：中心投影和 segmentation 标签均为候选模型内部定义，关联不证明视锥、裁剪、遮挡或任何其他
因素导致 raster 结果；不代表实体相机、感知、可达性、控制或安全。

预期产物：不可覆盖的 formal run manifest、逐条关联表、结果与配对 experiment record。结果保持
`pending_human_review`。
