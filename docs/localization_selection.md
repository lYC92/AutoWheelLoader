# 激光定位开源方案选型（M2 spike 结论）

日期：2026-09-05。环境：WSL2 Ubuntu-24.04、ROS 2 Jazzy、纯 CPU（RTX 2070 仅用于 Gazebo 渲染）。
输入：车载 1024×32 点云 10 Hz（`/loader/sensors/lidar/scan/points`）+ IMU 100 Hz
（`/loader/sensors/imu`）。硬标准：Jazzy 可构建、CPU 实时、开源许可兼容、无需先验地图。

三个候选均在本机从源码实测构建并启动节点验证（工作区 `~/lio_spike_ws`，与项目隔离）：

| | KISS-ICP | DLIO | GLIM |
|---|---|---|---|
| 类型 | 纯激光里程计（无 IMU） | LIO（IMU 辅助 deskew/初值/姿态观测器） | LIO 紧耦合因子图 + 全局建图 |
| 许可证 | MIT | MIT | MIT |
| Jazzy 构建 | 零改动，47 s | 需源码补 `pcl_ros`（上游无改动） | 需源码补 GTSAM + gtsam_points，须关 CUDA/viewer |
| 节点初始化 | 通过 | 通过 | 通过（须改 CPU 配置） |
| 调参复杂度 | 低（约 15 参数） | 中（约 40，含 IMU 标定/外参） | 高（整套 JSON 模块配置） |
| 维护 | 活跃（2.3k★，2026-06 仍在更新） | ROS 2 分支停滞（2024-11） | 极活跃（2026-09 当天仍有提交） |
| 构建输入 commit | kiss-icp `1ffa7d7` | dlio `c8acc37`（feature/ros2） | glim `9ad7444` + glim_ros2 `4d4ec52` |

## 结论

- **首选 DLIO**：装载机低速、高振动、扬尘场景下 IMU 融合对去畸变和短时特征退化有实际
  价值；CPU 实时性口碑最硬之一；本环境已实测构建与初始化通过。接入 = launch remap 到
  现有雷达/IMU Topic + 填传感器外参和 IMU 标定。风险是 ROS 2 分支维护慢，但代码量小、
  可自持。
- **快速基线 KISS-ICP**：构建最干净、参数最少。接入当天可跑通，作为精度和算力基线；
  不用 IMU 是鲁棒性上限的短板，仿真点云无点级时间戳时 deskew 需关闭（本项目的旋转
  畸变由 M1 效应节点在建模侧施加，不受影响）。
- **进阶备选 GLIM**：功能最全（子图、全局位姿图闭环、可扩展 GPS 因子），维护最活跃，
  已验证 CPU-only 可运行；代价是依赖链重、调参复杂、CPU 占用最高。DLIO/KISS-ICP 精度
  不达标或需要闭环建图时再升级。

## 下一步（M2 接入）

1. 把 DLIO 与 KISS-ICP 以固定 commit 纳入 WSL 构建流程（复用 spike 的依赖处理方式：
   DLIO 补 `perception_pcl` jazzy 分支源码；两者都不改上游）。
2. 新建 `loader_localization` 包承载 launch 与外参/标定配置；算法消费
   `/loader/sensors/lidar/scan/points_effect`（效应通道）与 `/loader/sensors/imu`。
3. 真值对比评测：Gazebo 位姿真值 vs `/odom`，输出平移/旋转 RMSE 和最大误差，
   固定场景自动铲装全程回放，名义门槛（平移 RMSE ≤ 0.15 m）实测后回填。

## 遗留事项

- spike 未做真实点云端到端精度/耗时测试（当时无 bag），接入后用评测脚本补。
- WSL 内 spike 残留 `~/lio_spike_ws`、`~/lio_deps`、`~/lio_deps_src`（数 GB），接入流程
  固化后清理。
- 若恢复 sudo 可 `apt install ros-jazzy-pcl-ros` 替代 perception_pcl 源码构建。
