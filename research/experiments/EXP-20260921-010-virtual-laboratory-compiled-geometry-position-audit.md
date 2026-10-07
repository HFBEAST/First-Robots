# EXP-20260921-010-virtual-laboratory-compiled-geometry-position-audit: Experiment Plan

008 的配置未显式重复 006 中的挡板 `name/type`，但其执行脚本固定以相同名称和 box 类型构造挡板。此 run
不修改任何已冻结文件，而是各自按 006 与 008 的执行构造规则编译两个 MuJoCo 场景，比较实际编译得到的
桌、杯、挡板与固定相机属性。

通过条件为：两场景的关节、三项 named geometry 的世界位置/尺寸/type，以及两台固定相机的 position、
quaternion、fovy、resolution 全部相等；并且 probe 杯位的 XY 半径不大于冻结半可达筛查半径。

本 run 不渲染、不运动、不读硬件。通过只确认 008 的 RGB 色彩探针采用与 006 相同的编译几何，并确认
位置筛查；它不证明真实可达性或真实遮挡。
