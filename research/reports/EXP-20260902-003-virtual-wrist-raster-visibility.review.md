# EXP-20260902-003-virtual-wrist-raster-visibility: Technical Review Note

技术校验：`py -3 -m experiment_management validate
research/runs/EXP-20260902-003-virtual-wrist-raster-visibility.json` 通过。manifest 记录干净起始
提交 `c3140b9`、候选模型版本、配置段输入与产物哈希。

观察（候选模型内）：对来源配置段的 650 个配置—杯位组合，`mj_ray` 首次命中杯体 40 次；腕部
虚拟相机的 320×240 geom segmentation 中，杯体目标像素为零的组合为 650 次，最大目标像素数为 0。
这表明在这个明确的候选模型/渲染定义下，射线命中不应被当作图像栅格可见的同义词。

实现健全性检查（非 formal run）：用相同场景、杯体 geom ID 和 segmentation 计数逻辑，将 MuJoCo
自由相机明确指向杯体时获得 1,776 个目标像素。因此该 run 的全零 raster 计数不是目标 ID 通道或
像素计数公式的常量错误。该检查只验证 renderer 计数机制，不解释腕部相机全零的原因。

不能推出：本 run 不区分腕部候选相机的投影、近远裁剪、姿态、场景设置或其他渲染因素，不能把
全零 raster 归因于其中任何一项；更不能推出真实腕部 RGB 相机不可见、深度相机可用性、感知失败、
控制可行性或任何真机安全结论。

复核状态：`pending_human_review`；未采纳为能力或安全边界。若继续虚拟研究，下一步应只读取和记录
腕部相机模型参数、把杯体中心投影到相机坐标系，并以此区分视锥外、近远裁剪与遮挡；保持不接入硬件。
