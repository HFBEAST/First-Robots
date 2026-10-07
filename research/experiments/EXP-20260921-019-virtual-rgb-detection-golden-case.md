# EXP-20260921-019-virtual-rgb-detection-golden-case: Experiment Plan

目的：在无挡板、默认关节和固定 B=`[0.15,0,0.055] m` 的单一金样例中，确认 Detection Agent 可以从
MuJoCo 渲染的 RGB 帧而非 `simulator_truth` 坐标产生候选杯像素与图像质心。帧适配器输出固定合同：
`source_kind`、`frame_id`、`scenario_version`、RGB 编码、尺寸和像素数组；未来真实数据适配器只能替换
帧来源，必须输出同一合同。

控制项：候选模型、无挡板、桌、杯 B 的人工洋红显示色、两台固定相机、渲染尺寸和候选 RGB 签名固定。
检测函数只接收 `RgbFrame`，不接收杯世界坐标、相机外参或 MuJoCo model/data。通过只要求两帧均存在
正的候选签名像素，并保存原图、mask、overlay 和结构化输出。世界 B 仅在 run 的评估输出中记录，不传入
检测函数。

本 run 不反投影像素、融合双目或选择误差阈值，因此不把质心视为 world pose，也不驱动 mock 或真实执行。
