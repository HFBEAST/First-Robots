# EXP-20260923-026-virtual-depth-surface-point-projection: Experiment Plan

目的：只复用 025 已冻结的深度前景 mask、current depth 和相机合同，将每个组件的质心像素及其局部
候选深度中位数沿相机射线投成一个 world **surface-point candidate**。B 只用于记录该点到杯中心的偏差。

本 run 明确不把表面点称为杯中心：圆柱可见表面本来就会偏离中心。通过只检查深度样本、射线方向和输出
坐标有限；不拟合圆柱、不融合两相机、不设误差阈值、不向 Coordinator 或执行层发布。
