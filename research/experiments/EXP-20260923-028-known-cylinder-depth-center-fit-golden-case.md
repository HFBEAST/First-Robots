# EXP-20260923-028-known-cylinder-depth-center-fit-golden-case: Experiment Plan

目的：在用户授权的“已知圆柱杯”前提下，把 025 的两台深度前景组件重建为可见 surface points，并拟合
圆柱的 XY 中心。圆柱被固定为竖直、半径 `0.035 m`、半高 `0.055 m`、落在 `Z=0` 的虚拟桌面；输出中心
Z 因此为 `0.055 m`。为减少顶面内部点对圆拟合的影响，只使用合并点云中 world Z 严格低于其样本中位数的
下半部分。

通过仅要求点云有限、下半部至少三点、代数圆拟合有限且半径为正。记录拟合中心、半径、残差及相对 B 的
评估偏差，不设可接受误差，不向 Coordinator 或 Execution Agent 发送结果。
