# 状态

## 当前阶段

`planning`。开发优先项目骨架和可拆卸研究旁车已经建立；当前只有可验证的状态入口，
尚未实现共享信念、能力注册、任务规划、机器人仿真或多 Agent 调度。

研究状态：`optional / active`。已完成 formal virtual run
`EXP-20260831-004-so101-mujoco-observation-preflight`；已采纳 Menagerie `robotstudio_so101`
作为仅限研究旁车的临时虚拟模型，见 `docs/decisions/DEC-20260831-001-adopt-virtual-so101-model.md`。
尚无真机、相机标定、感知、抓取、控制或多机器人能力证据。

最新 formal virtual run `EXP-20260902-002-virtual-wrist-segment-contact-scan` 已验证其 manifest：
在候选模型的默认配置至静态候选样本 166 的 65 个离散关节配置中，全部位于候选关节限位内，且对
10 个固定虚拟杯位均为零 MuJoCo 静态接触，几何可见数最高为 8。该结果保持
`pending_human_review`，不代表连续无碰撞、轨迹、控制、安全或真机能力。

`EXP-20260902-003-virtual-wrist-raster-visibility` 曾记录 40 个组合射线命中杯体、全部无 raster
像素；但 `EXP-20260916-004-virtual-wrist-raster-recheck` 在杯体位置不变量通过的 650 条重检中发现
30 条当前有像素、620 条仍无像素。故原全零表不完全可复现，两个结果都保持 `pending_human_review`，
且均不代表实体腕部 RGB 覆盖。

修正后的 `EXP-20260916-002-virtual-wrist-projection-diagnosis` 已验证 650/650 个杯体中心确实位于
输入位置：228 个位于候选相机后方、421 个位于候选透视视锥外、6 个超出候选深度裁剪范围（条件可
重叠），仅一项中心同时在视锥和裁剪范围内且仍无 raster 像素。该结果保持 `pending_human_review`，
不构成对遮挡、实体相机、感知或安全的结论。

## 已知风险与阻塞

- 最终机器人数量、型号、安装布局和共享工作区尚未冻结；目前只知道研究从 SO-101 能力开始。
- 两台固定深度相机和腕部相机的型号、标定、同步、噪声与实际覆盖尚未登记。
- `research/hardware/HW-20260903-001-orbbec-astra-pro-observation-stack.json` 已登记用户提供的
  Orbbec Astra Pro/OpenNI2 信息和本机只读预检：SDK 文件、设备功能枚举及指定 Conda 环境的
  导入链可用，但尚未初始化设备或读帧，且未确认它们如何对应两台实体固定深度相机。
- 已采纳候选模型默认姿态下，虚拟腕部相机对本 run 的半可达范围杯位姿为 0/10 几何可见；这是虚拟
  观察，不是实体腕相机覆盖率。
- “多 Agent 优于单调度器”只是待验证假设，不能作为架构事实。
- 多机器人交接需要共同可达抓取位、物体所有权、资源锁和同步执行，目前均未实现。
- 外部论文/开源项目只完成资料筛选，尚未在本项目中复现。

## 下一个可验证步骤

先冻结六个核心协议的 v0 草案和行为测试；随后在新 run 中把经过位置不变量保护的 raster 重检结果与
候选相机投影条件重新关联，并保持它和真机标定分离。MRBTP-demo 已完成上游 exploratory 自检；任何
外部项目在与本项目协议的正式证据建立前不得集成进产品源码。
