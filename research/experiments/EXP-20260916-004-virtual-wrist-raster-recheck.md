# EXP-20260916-004-virtual-wrist-raster-recheck: Experiment Plan

问题：`EXP-20260902-003` 记录 650 个组合均无目标 segmentation 像素，但对其中配置 64、杯位 6 的
最小独立复现得到 22,248 像素。保持同一模型、相机、65×10 配置—杯位集合和单一 segmentation renderer，
在每个组合验证杯体位置后，当前运行能否逐条重现原 raster 表？

输入和控制项：固定 Menagerie revision、`EXP-20260902-002` 的配置段/杯位表、003 的原 raster 表。
使用一个 320×240 `wrist_cam` segmentation renderer 遍历全部组合；每条在 `mj_forward` 后断言
`geom_xpos` 等于输入杯位。记录当前目标像素、当前 raster 布尔值、原 raster 布尔值和四种一致性计数。
禁止 RGB/深度输出、感知、actuation、控制器、动力学步进、IK、轨迹优化和硬件 I/O。

判据：完整记录 650 个组合，全部满足位置不变量，并输出同原 raster 表的一致性计数。位置不变量失败
即为运行失败；原/当前结果一致与否均为观察结果，不是 run 成功阈值。

局限：当前重检只能说明在记录的候选模型/软件环境下是否复现原表，不能定位 GPU、驱动、渲染器状态或
历史环境差异的因果；MuJoCo segmentation 不是 RGB 感知或实体相机观测。

预期产物：不可覆盖的 formal run manifest、逐组合重检表、结果与配对 experiment record。结果保持
`pending_human_review`。
