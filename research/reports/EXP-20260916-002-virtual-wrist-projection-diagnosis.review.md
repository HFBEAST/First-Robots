# EXP-20260916-002-virtual-wrist-projection-diagnosis: Technical Review Note

技术校验：`py -3 -m experiment_management validate
research/runs/EXP-20260916-002-virtual-wrist-projection-diagnosis.json` 通过。manifest 记录干净起始
提交、固定模型 revision、两个上游表和全部产物哈希。

观察（候选模型内）：650 个配置—杯位组合全部满足“前向运动学后杯体世界中心等于输入位置”的不变量。
其中 228 个杯体中心位于腕部相机后方，421 个在候选透视视锥外，6 个在候选渲染深度裁剪范围外。
这些是保留的独立布尔条件，故计数可以重叠。只有配置 64、杯位 6 的杯体中心同时在候选视锥和裁剪
范围内；该组合仍是 `mj_ray` 命中而 segmentation raster 无目标像素。

解释边界：这说明杯体中心投影能为大部分候选 raster 全零组合提供几何筛选信息，但不能解释剩余一个
组合。中心在视锥内并不表示整个圆柱体的表面、近裁剪关系、面朝向或渲染可见性。不能由此归因遮挡，
更不能推出实体腕部相机不可见、感知失败、控制可行性或安全性。

复核状态：`pending_human_review`；未采纳为能力或安全边界。下一步仅针对该固定组合记录杯体相对
相机的完整几何体边界距离、相机是否位于圆柱体内部/附近和渲染诊断，不接入硬件。
