# 信号可视化收尾与第四阶段首轮基线

本机 Windows 10 / WSL Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 实测。
原始完整日志仍在 `~/loader_sim_runtime/log/` 与 `results/`；本目录保留可审阅摘要。

- `foxglove_bridge_*_smoke.txt`：实际 WebSocket/CDR、字段路径、TF、网关命令回路。
- `sensor_effects_smoke.txt`：同源扫描 10 Hz、丢点/旋转效应、IMU 和安装扰动。
- `localization_metrics.json`、`localization_trajectory.csv`：当前平地裁剪后的行驶基线。
- `localization_before_ground_crop.json`：相同场景、未做地面裁剪时的失败对照。

KISS-ICP 固定提交和参数见 `../../localization.md`。两次行驶的启动时间、实际行程和采样数
略有不同；此对照用于排查配准问题，不是统计性能结论。定位链路已跑通，0.15 m 精度目标
仍未通过。网页仍需登录并重新导入六页布局做目视验收。
