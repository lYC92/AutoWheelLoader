# 固定运行配置

三种配置互相隔离，目的是在 RTX 2070 8GB 上保持可预测的显存和实时性。

- `control_realtime`：Gazebo 车辆、激光定位和规划闭环；BEV 与 Chrono::Dem 关闭。
- `bev_capture`：Gazebo 锁步生成四路鱼眼数据；BEV 网络在 Gazebo 关闭后回放运行。
- `dem_offline`：Gazebo 和 BEV 都关闭，Chrono::Dem 独占 GPU 回放铲斗轨迹。

配置位于 `simulation/config/profiles/`。`validation_status` 为 `nominal` 或
`not_installed` 时，只能用于开发和数据通路验证，不能宣传为实车精度结果。
