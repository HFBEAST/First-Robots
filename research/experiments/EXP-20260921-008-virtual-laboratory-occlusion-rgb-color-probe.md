# EXP-20260921-008-virtual-laboratory-occlusion-rgb-color-probe: Experiment Plan

007 已显示：在挡板遮挡边界，simulator segmentation 标签不能直接解释为 RGB 可见性。此 run 不改变 006 的
任何几何、相机、渲染尺寸或执行设置，只把杯体设为洋红、挡板设为青色，以直接测量固定相机 RGB 栅格中是否
留下杯体颜色。

预先定义的杯体颜色签名为 `red > green` 且 `blue > green`。两台固定相机的签名像素均为零时，终态为
`not_observable_by_nominal_fixed_rgb_color_probe`；任何非零值则为
`rgb_color_probe_candidate_remains_observable`。这是静态虚拟 RGB 栅格标签，不能称为实体相机盲区或感知结果。

此 run 不测量 IK、碰撞、轨迹、抓取、控制、负载或硬件可达性；位置筛查仍须与抓取级可达性分开报告。
