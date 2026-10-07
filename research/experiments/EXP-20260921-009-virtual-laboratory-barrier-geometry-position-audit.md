# EXP-20260921-009-virtual-laboratory-barrier-geometry-position-audit: Experiment Plan

问题：008 的人工色彩 RGB 遮挡测量是否使用与 006 相同的物理几何，且该杯位是否处于冻结的候选模型
半可达位置筛查内？

本 run 不渲染、不运动、不读硬件。它比较 006 与 008 的桌面、杯体中心/尺寸、挡板中心/尺寸、两台相机、
渲染尺寸、关节、dynamics、actuation 和 hardware I/O 字段；允许的差异仅为 display RGBA 与 RGB 测量定义。
随后从候选模型的 `baseframe` 计算杯体 XY 半径，并与已冻结的 `0.23917871793616743 m` 比较。

通过只表示：008 的色彩探针使用了冻结的 006 几何，且该位置在历史虚拟半可达筛查内。它不重新解释 005/006
的 segmentation，不证明 IK、无碰撞轨迹、抓取或真机可达性。
