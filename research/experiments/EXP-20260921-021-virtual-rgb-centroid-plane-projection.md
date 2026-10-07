# EXP-20260921-021-virtual-rgb-centroid-plane-projection: Experiment Plan

目的：只在 020 已隔离的单一虚拟 RGB 杯子金样例上，检验图像质心是否能作为 Coordinate Agent 的输入。
每台固定相机将 RGB component 质心转为相机射线，再用已冻结的杯中心高度平面 `Z=0.055 m` 得到一个
world 点。B 的真值只在结果中计算偏差；不设误差阈值，不融合两相机，也不向 Coordinator 或 Execution
Agent 发送该坐标。

控制项：模型、无挡板、B、两台相机、渲染合同和修复后的 RGB 谓词均冻结。唯一新增的计算为像素质心射线
与固定高度平面的交点。通过只要求射线交点有限且在相机前方，并完整记录两个估计值和它们相对 B 的偏差。
