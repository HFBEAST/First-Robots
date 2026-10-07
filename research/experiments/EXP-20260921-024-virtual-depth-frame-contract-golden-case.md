# EXP-20260921-024-virtual-depth-frame-contract-golden-case: Experiment Plan

目的：为两台未来固定深度相机建立独立于 RGB 的可替换帧入口。以无挡板、固定 B、默认关节的单点
虚拟场景，`DepthFrameSource.capture(camera_name) -> DepthFrame` 从 MuJoCo renderer 输出两个 depth array。
合同携带 `source_kind`、`frame_id`、`scenario_version`、编码、尺寸和数据；未来真实深度适配器必须输出同一合同。

通过只检查数组类型、尺寸、有限正值与非均匀性，并保留 `.npy` 原始深度和归一化审图。没有杯深度分割、
深度单位外推、像素到 world、双目融合或执行。
