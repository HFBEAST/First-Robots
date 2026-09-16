# EXP-20260916-003-virtual-wrist-local-geometry-diagnostic: Experiment Plan

问题：`EXP-20260916-002` 只剩一个“杯体中心在候选视锥和裁剪范围内、射线命中、raster 无像素”的
组合。该组合中，候选相机中心相对杯体圆柱局部坐标的径向/轴向距离、是否位于圆柱体内，以及同一
候选腕部相机的 RGB、深度和 segmentation 产物分别是什么？

输入和控制项：从 `EXP-20260916-002` 自动选择且仅选择同时满足 `ray_visible`、
`inside_candidate_frustum_and_clip` 与 `not raster_visible` 的组合；断言恰有一个。使用其配置和杯位，
在编译前初始化杯体并在前向运动学后断言世界位置。记录圆柱局部相机坐标、径向/轴向距离、到圆柱
表面的有符号距离、几何体内判定，并渲染 320×240 RGB、深度、segmentation。禁止感知模型、
actuation、控制器、动力学步进、IK、轨迹优化和硬件 I/O。

判据：恰有一个选择组合，位置不变量通过，局部几何度量和三种渲染产物完整写出。任何选择不唯一或
位置不变量失败均使运行失败；不设目标像素、距离或安全阈值。

局限：圆柱近似只描述虚拟目标几何，不解释渲染管线中的所有裁剪、背面剔除或遮挡因素；渲染 RGB
不是实体腕部相机或感知模型输出。结果不能推出真实相机可见性、感知能力、碰撞安全、控制可行性或
硬件能力。

预期产物：不可覆盖的 formal run manifest、单样本局部几何 JSON、RGB/深度/segmentation 产物、
结果与配对 experiment record。结果保持 `pending_human_review`。
