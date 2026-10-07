# EXP-20260923-030-known-cylinder-depth-x-step-agent-chain: Experiment Plan

问题：在 029 的固定圆柱金样例后，只改变圆柱 world X（`0.125`、`0.150`、`0.175 m`），双 depth 感知到
mock 状态复核的同一条虚拟链是否仍按合同运行？

阻塞前提：029 的结果哈希必须匹配且为 `passed_pending_human_review`。控制项为已知直立圆柱、Y/Z、尺寸、
桌面、默认机器人关节、两台相机位置与目标、渲染合同、背景差分、拟合办法、消息版本、初态 A，以及所有
禁用的 actuation/dynamics/hardware I/O。唯一变化是列出的圆柱 world X。

每个 X 独立重建空桌 reference 与有圆柱 current 的两路 virtual `DepthFrame`；每路必须有一块正前景组件，
再以两路点云拟合已知圆柱中心。随后分别执行完整的 Coordinate→Coordinator→table-only Capability→mock
Executor→Reviewer 链。记录每点相对配置坐标的误差仅供评估，不设置精度阈值。

通过只说明这些固定虚拟条件和这三个 X 点的处理/消息/内存状态合同可运行；任一深度组件、有限性、结构消息、
门禁、状态或复核不满足即记录为 blocked，并停止扩大变量范围。结果不证明真实深度相机、可视范围、覆盖率、
准确率、可达性、碰撞、抓取、规划、安全或硬件能力。
