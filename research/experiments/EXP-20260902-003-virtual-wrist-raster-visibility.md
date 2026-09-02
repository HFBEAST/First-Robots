# EXP-20260902-003-virtual-wrist-raster-visibility: Experiment Plan

问题：前序 run 用 `mj_ray` 把目标几何体作为腕部虚拟相机的几何可见代理，但射线命中不一定表示目标
在相机投影、裁剪和遮挡后的图像栅格中占有像素。对 `EXP-20260902-002` 已记录的 65×10 个
配置—杯位组合，两种候选模型内部定义分别给出什么结果？

输入和控制项：固定候选 MJCF、前序配置段表及其杯位；对每个组合作 `mj_forward`。几何定义沿用
`mj_ray` 首次命中杯体；图像定义使用 MuJoCo `wrist_cam` 的 320×240 segmentation render，统计
目标 geom ID 的像素数。禁止 RGB 感知模型、RGB 训练、actuation、控制器、动力学步进、IK、轨迹优化和
硬件 I/O。

判据：完整记录 650 个组合的射线结果、目标分割像素数和二值化 raster 结果，以及四种一致性计数。
仅以完整产物作为 run 通过条件；不预先设定通过率、像素阈值或能力阈值。

局限：MuJoCo segmentation 是候选模型的真值标签，而非 RGB 物体检测；320×240 分辨率及该相机内参
不是项目实体腕部相机标定。比较结果不能推出相机质量、感知能力、真实可见性、控制可行性或安全性。

预期产物：不可覆盖的 formal run manifest、逐组合比较表、结果与配对 experiment record。结果保持
`pending_human_review`。
