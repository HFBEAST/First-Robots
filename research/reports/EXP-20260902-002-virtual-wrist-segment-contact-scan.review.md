# EXP-20260902-002-virtual-wrist-segment-contact-scan: Technical Review Note

技术校验：`py -3 -m experiment_management validate
research/runs/EXP-20260902-002-virtual-wrist-segment-contact-scan.json` 通过。manifest 记录干净
起始提交 `b8061a0`、候选模型版本、两个上游输入表和全部产物哈希。

观察（候选模型内）：从默认 `qpos` 到 `EXP-20260901-002` 样本 166 的 65 个等间隔关节配置，
全部位于候选 MJCF 关节限位内；每个配置在 10 个固定杯位下的 MuJoCo 静态接触数均为零。该配置段
的腕部虚拟相机几何可见杯数最大为 8。

不能推出：这不是连续碰撞检测、时间参数化轨迹、速度/加速度/力矩验证、IK、控制器测试、真实相机
观测、真实碰撞安全或 SO-101 真机能力。65 个离散点的零接触不能证明点之间零接触，也不构成安全
阈值或可执行动作。

复核状态：`pending_human_review`；未采纳为能力或安全边界。建议下一步只在同一候选配置集合中比较
几何射线可见与虚拟图像栅格中的目标像素可见，仍不接入感知模型或硬件。
