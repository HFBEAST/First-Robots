# EXP-20260921-018-coordinate-agent-fault-injection: Experiment Plan

目的：在 017 已通过的正确合成坐标合同之后，隔离验证 Coordinator 的拒绝行为。对固定的 25 个 B，先输入
两条相同的 world-frame 坐标消息，随后分别构造两类故障：第二条坐标加 `[0.001,0,0] m`，或只将第二条
的 scenario version 改为旧版本。每条故障都必须在 mock Execution Agent 之前拒绝，且内存中 A 不变。

`0.001 m` 只是一个明确的非零反例，不是相机噪声、容差或安全阈值。该 run 不重新评价视觉、外参、桌面
包含、可达性或抓取；也不编译机器人、发送控制命令或使用硬件。
