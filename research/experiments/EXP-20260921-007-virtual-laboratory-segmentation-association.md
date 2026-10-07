# EXP-20260921-007-virtual-laboratory-segmentation-association: Experiment Plan

问题：006 使用的 MuJoCo segmentation geom ID 是否与其对应的 RGB 像素正确关联？默认杯体和挡板都为灰色，
不能靠 RGB 外观复核，所以本 run 仅给杯体指定红色、给挡板指定蓝色显示色。

控制项：006 的模型、关节、桌面、杯体与挡板几何、相机、渲染器和无硬件 I/O 全部冻结。唯一变化是两个
不透明 display RGBA；它不改变几何、碰撞、运动或相机位姿。

通过条件：对每台固定相机，所有 segmentation 标为杯体的像素必须满足 red > blue；所有标为挡板的像素
必须满足 blue > red；RGB 与 segmentation 产物齐全。任一条件不满足即停止，不把 005/006 的像素数解释为
杯体可见性证据。

本 run 仅验证渲染标签关联，不代表实体感知、标定、遮挡、可达性、碰撞、抓取、控制或 sim-to-real 结论。
