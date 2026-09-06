# 信号可视化收尾与第四阶段首轮基线

本机 Windows 10 / WSL Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 实测。
原始完整日志仍在 `~/loader_sim_runtime/log/` 与 `results/`；本目录保留可审阅摘要。

- `foxglove_bridge_*_smoke.txt`：实际 WebSocket/CDR、字段路径、TF、网关命令回路。
- `sensor_effects_smoke.txt`：同源扫描 10 Hz、丢点/旋转效应、IMU 和安装扰动。
- `localization_metrics.json`、`localization_trajectory.csv`：历史固定高度裁剪后的行驶基线。
- [l580_adaptive/README.md](l580_adaptive/README.md)：新外观、车顶外参和自适应前处理的三轮行驶结果，RMSE 0.100 / 0.116 / 0.106 m，均通过名义目标。
- `localization_before_ground_crop.json`：相同场景、未做地面裁剪时的失败对照。

KISS-ICP 固定提交和参数见 `../../localization.md`。两次行驶的启动时间、实际行程和采样数
略有不同；历史对照用于排查配准问题，不是统计性能结论，历史版本未通过 0.15 m 精度目标。
新版本通过范围及限制见 l580_adaptive。网页仍需登录并重新导入六页布局做目视验收。
