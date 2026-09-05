# 激光雷达与 IMU 基础链路

第 4 阶段的第一条传感器链路已经在名义装载机上建立。传感器由 Gazebo Sim 产生，经
`ros_gz_bridge` 转成标准 ROS 2 消息，算法进程只消费 ROS 2 接口和仿真 `/clock`。

## 当前配置

| 传感器 | Gazebo 输出 | ROS 2 类型 | 名义配置 |
|---|---|---|---|
| 3D GPU 激光雷达 | `/loader/sensors/lidar/scan/points` | `sensor_msgs/PointCloud2` | 1024×32，10 Hz，360°，-25°～15°，0.5～120 m，距离噪声 σ=0.02 m |
| IMU | `/loader/sensors/imu` | `sensor_msgs/Imu` | 100 Hz，名义中档 MEMS 噪声：陀螺 σ=0.0017 rad/s、零偏 σ=0.00017 rad/s，加速度计 σ=0.017 m/s²、零偏 σ=0.0017 m/s²（Gazebo 内建噪声模型，可经 xacro 参数替换） |
| 仿真时钟 | `/clock` | `rosgraph_msgs/Clock` | Gazebo 仿真时间 |

传感器通过 Xacro 参数 `enable_lidar_imu:=true` 启用。雷达和 IMU 的固定安装关节被显式
保留，防止 URDF 转 SDF 时合并到 `base_link`；自动测试会检查消息 `frame_id` 分别包含
`lidar_link` 和 `imu_link`。

## 名义效应通道与标定扰动（M1）

算法不得直接消费理想传感器流。`loader_sensor_effects` 包提供
`lidar_effects_node`：订阅理想点云 `/loader/sensors/lidar/scan/points`，发布扰动后的
`/loader/sensors/lidar/scan/points_effect`。理想流保留用于真值对比。当前实现的效应：

- 随机丢点：`dropout_probability` 参数，丢点写为 NaN 以保持 1024×32 有组织布局，
  `random_seed` 固定时逐帧可复现；
- 旋转扫描运动畸变：按列采集时刻与最新 IMU 角速度做 Rodrigues 旋转，把所有点统一到
  扫描起始帧（名义近似：陀螺轴与雷达轴对齐，平移畸变暂未建模）；
- IMU 噪声由 Gazebo 内建模型直接产生（见上表），xacro 参数
  `imu_gyro_noise_stddev`/`imu_gyro_bias_stddev`/`imu_accel_noise_stddev`/`imu_accel_bias_stddev`
  控制，置 0 即恢复理想测量；
- 传感器安装标定扰动：xacro 参数 `lidar_mount_offset_xyz/rpy`、`imu_mount_offset_xyz/rpy`
  叠加在名义安装位姿上。

验收（2026-09-05 本机通过）：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_sensor_effects.sh
```

结果：丢点率 0.101（目标 0.10），静止畸变最大 0.0141 m（陀螺噪声在 120 m 量程端的
物理一致表现），IMU 噪声实测标准差与配置一致（加速度计 0.01621 m/s²，陀螺
0.001664 rad/s），安装扰动正确写入 URDF；单元检查覆盖丢点确定性和 Rodrigues 旋转。
证据：`/home/lyc/loader_sim_runtime/results/sensor_effects_smoke.txt`。

## 验证

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_sensors.sh
```

测试会启动带目标物的 500 Hz 世界、生成装载机、启动 ROS 桥，然后检查点云尺寸/字段、
抽样点有限性、IMU 数值、坐标系、消息频率以及传感器和 `/clock` 时间戳单调性。

2026-09-04 本机回归结果：点云 9.66 Hz、IMU 92.29 Hz，点云 1024×32，抽样 4096 点
全部有限。证据保存在：

- `/home/lyc/loader_sim_runtime/results/loader_sensors_smoke.txt`
- `/home/lyc/loader_sim_runtime/log/loader_sensors_smoke_gazebo.log`
- `/home/lyc/loader_sim_runtime/log/loader_sensors_smoke_bridge.log`

完整控制配置性能测试：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/benchmark_loader_control_profile.sh
```

该测试同时启用正式 `ros2_control` 力级控制链、3D 雷达和 IMU。当前结果为平均实时系数
0.990584、雷达 9.98277 Hz、显存峰值 723 MiB，通过 `RTF >= 0.9` 门槛。结果位于
`/home/lyc/loader_sim_runtime/results/loader_control_profile_baseline.csv`。

动态土料射线验收：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_soil_perception.sh
```

固定 512×32 GPU 观察雷达会在完整铲取、转运和卸料前后比较有组织点云，并用雷达外参把
变化射线约束到变化最大料柱附近。当前结果为 2,948 条全局变化射线、其中 265 条位于目标
地形列附近，证明 10 Hz 动态料柱已经进入 Gazebo 的渲染/射线场景。这个测试验证的是地形
可观测性，不替代车载雷达的定位精度测试。

当前动态土料联合性能测试由 `benchmark_loader_soil_profile.sh` 执行：500 Hz 车辆、正式
控制链、280 格动态土料、车载 1024×32 雷达和 IMU 同时运行时，平均 RTF 0.978491、雷达
9.93322 Hz、显存峰值 663 MiB，已通过实时门槛。加入鱼眼相机和 BEV 推理后仍需重新测量。

## 尚未完成

- IMU 温漂、轴不正交和动态零偏（当前为零均值固定零偏）；
- 雷达平移运动畸变（需接入轮速/车速真值）、强度/反射率、雨尘衰减；
- 传感器标定真值的独立发布通道（当前扰动直接写进 URDF/TF）；
- 点云定位算法接入、地图和自动定位误差评测；
- 使用目标实车雷达/IMU 的扫描模式、噪声和安装参数替换当前名义值。

因此当前链路可用于接口开发和基本算法冒烟测试，尚不能作为目标传感器的统计仿真模型。
