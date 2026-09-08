# 三维整车接入回归

日期：2026-09-07。名义模型、三维 0.25 m 网格；并非实车标定或最终 A→B 验收。

- `yaw0_perception.txt`：直向三维整车 + 观察雷达；XY 单元与表面高度匹配后的射线检查。
- `legacy_physics.txt`：旧一维整车兼容回归。

均包含 8 秒模拟时间延迟启动、反馈恢复斗姿、驱动铲料、举升、倒车、制动、卸料和急停检查。
指标及后续任务统一维护于 `docs/next_stage_3d_ab.md`。

- `yaw45_physics.txt`、`yaw_minus45_physics.txt`：斜向整车挖卸。
- `sparse_perception.txt`：动态生成新卸料单元表面，61 条雷达回波匹配新增土堆。
- `transfer_failed_return.json`：保留早期前进返回失败轨迹，不能当作通过结果。
- `transfer_passed.json`：空载倒车/前进转弯与停车通过。
- `ab_cycle_passed.json`、`ab_cycle_passed.txt`：首轮带料 A→B 及返回完整通过，显式真值调试。
  四段 RMSE 最大 0.099759 m、停车误差最大 0.224090 m、朝向最大 3.345050°；
  装入与卸出 0.707638 m³，B 区材料占比 100%，报告的守恒误差 0。

轨迹图由 `tools/ros/render_ab_report.py` 从通过记录生成。以上尚不替代 10 次或 100 循环验收。


## 侧面地面卸料（新默认场景）

`side_dump_cycle_passed.json/.txt` 为新路线首次整车记录：12.5 cm XY 网格，A=(7,0)，后退到 (-8,0)，前进左转至后轴 (4,12)，斗口目标 B=(4,17.8)，然后返回。
装卸 0.764412 m³，B 内比例 100%，四段最大 RMSE 0.0616 m；有真实 Gazebo 相机帧 `docs/assets/side_yard_complete.png`。
旧 `ab_cycle_passed` 文件保留原反向大转弯基线，不与新路线混用。

`side_dump_default_passed.json/.txt`：最终默认演示入口全循环通过，含真实相机渲染、Foxglove 桥接和按载荷创建/移除斗内显示；装卸 0.706626 m³，B 内比例 100%。
