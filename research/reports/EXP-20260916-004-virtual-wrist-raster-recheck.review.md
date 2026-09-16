# EXP-20260916-004-virtual-wrist-raster-recheck: Technical Review Note

技术校验：`py -3 -m experiment_management validate
research/runs/EXP-20260916-004-virtual-wrist-raster-recheck.json` 通过。650 条均通过杯体位置不变量。

观察（候选模型内）：原 `EXP-20260902-003` 表中 650 条均为 raster 不可见；本次重检有 620 条仍为
不可见，30 条变为可见，最大目标像素数为 39,060。没有原来可见而本次不可见的条目。因此原“全部零像素”
观察在当前固定模型、模型哈希、Python/MuJoCo 版本和 320×240 单 renderer 重检中不完全可复现。

影响：003 的历史产物和 manifest 保留，但不能再作为候选腕部相机 raster 覆盖的独立结论。当前重检也
不替代硬件观察，且本 run 没有归因 GPU、驱动、渲染器、模型或历史环境状态。下一步应将重检 raster
结果与已验证的位置/投影条件重新关联，而不是继续使用 003 的全零表。

复核状态：`pending_human_review`；未采纳为能力或安全边界。
