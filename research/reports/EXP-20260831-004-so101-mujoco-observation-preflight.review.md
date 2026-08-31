# EXP-20260831-004-so101-mujoco-observation-preflight: Technical Review Note

技术校验：manifest 与 result/record 配对已通过；人工采纳状态仍为 `pending_human_review`。

## 观察到的事实

- MuJoCo 3.12.0 成功加载 Menagerie `robotstudio_so101` commit `da76818e269b82289eba39808e2fb91d679d6994`。
- 固定 seed 的 10,000 个关节限位采样得到最大径向 `gripperframe` 距离 `0.47835743587233487` 模型单位；
  本 run 的虚拟杯子工作区半径是其一半 `0.23917871793616743`。
- 10 个 seed 都写出了 RGB 和深度产物；最终相同 wrist RGB 的重复哈希相同。
- 基于 MuJoCo 射线的几何可见性为：`fixed_depth_01` 9/10、`fixed_depth_02` 9/10、`wrist_cam` 0/10。
- 人工查看 seed 00 RGB：固定相机画面包含杯体；腕部画面主要包含夹爪和地面，和几何可见性记录一致。

## 不作出的结论

- 不把默认腕相机 0/10 外推为实体腕相机的实际覆盖率；它只描述这个候选模型、默认姿态、虚拟杯子平面和
  虚拟相机条件。
- 不修改已验证 run 的相机参数来追求更高覆盖率。若后续研究需要评估虚拟腕相机位姿、工作姿态或相机布局，
  必须使用新 experiment ID、新配置和新 run。
- 不声明真机标定、感知质量、抓取、IK、碰撞、控制或多机器人能力。
