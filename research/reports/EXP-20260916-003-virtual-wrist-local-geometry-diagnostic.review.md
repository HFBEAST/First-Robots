# EXP-20260916-003-virtual-wrist-local-geometry-diagnostic: Technical Reproducibility Note

manifest 完整性校验通过，且选择和杯体位置不变量均通过。它记录的单一组合为配置 64、杯位 6：相机
中心不在虚拟杯体圆柱内，距圆柱表面约 0.00210 m，新的 320×240 segmentation render 出现 22,248 个
目标像素。

但该组合是依据 `EXP-20260902-003` 的 `raster_visible=false` 选择的；后者记录它的目标像素为零。
用与 003 相同的模型构造、`qpos`、杯位、目标 geom ID、320×240 renderer 和 segmentation 计数公式
进行最小独立复现，仍得到 22,248 个目标像素。因此上游“该组合 raster 为零”的观察在当前固定模型和
运行环境下没有复现。

影响：本 run 不能诊断 003 的零像素原因，因为其选择前提不稳定；也不能用其新 render 值替换或否定
003 的完整 650 组合表。保留产物以展示矛盾，并对全部组合创建带目标位置不变量的独立 raster 重检。

不能推出：此矛盾不证明任何渲染 API、候选模型、实体相机、感知、控制或安全结论。复核状态保持
`pending_human_review`。
