# EXP-20260923-025-virtual-depth-background-detection-golden-case: Experiment Plan

目的：在 024 的单点深度帧合同后，测试固定深度相机的最小 Detection Agent：同一固定机器人/桌/相机
场景先渲染无杯 background reference，再仅加入 B=`[0.15,0,0.055] m` 杯体渲染 current depth。对每像素计算
`reference - current`；大于 `1e-6 m` 的正值构成候选前景，使用四邻域检查每相机为一个组件。

控制项：模型、默认关节、桌、相机、渲染大小和 depth 合同均冻结；唯一变量为杯体是否存在。`1e-6 m`
只用于虚拟数组中的数值不变性，不是物理噪声门限。通过不评价 pose 或尺寸，只确认两台相机能从深度差中
得到单个候选前景组件，并保存 reference/current/mask。
