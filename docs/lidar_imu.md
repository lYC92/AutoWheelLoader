# 激光雷达与 IMU 基础链路

第 4 阶段的第一条传感器链路已经在名义装载机上建立。传感器由 Gazebo Sim 产生，经
`ros_gz_bridge` 转成标准 ROS 2 消息，算法进程只消费 ROS 2 接口和仿真 `/clock`。

## 当前配置

| 传感器 | Gazebo 输出 | ROS 2 类型 | 名义配置 |
|---|---|---|---|
| 3D GPU 激光雷达 | `/loader/sensors/lidar/scan/points` | `sensor_msgs/PointCloud2` | 1024×32，10 Hz，360°，-25°～15°，0.5～120 m，距离噪声 σ=0.02 m |
| IMU | `/loader/sensors/imu` | `sensor_msgs/Imu` | 100 Hz，当前为理想测量 |
| 仿真时钟 | `/clock` | `rosgraph_msgs/Clock` | Gazebo 仿真时间 |

传感器通过 Xacro 参数 `enable_lidar_imu:=true` 启用。雷达和 IMU 的固定安装关节被显式
保留，防止 URDF 转 SDF 时合并到 `base_link`；自动测试会检查消息 `frame_id` 分别包含
`lidar_link` 和 `imu_link`。

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

## 尚未完成

- IMU 零偏、随机游走、温漂和轴不正交模型；
- 雷达随机丢点、强度/反射率、雨尘衰减和旋转扫描运动畸变；
- 传感器内外参扰动与标定真值接口；
- 点云定位算法接入、地图和自动定位误差评测；
- 使用目标实车雷达/IMU 的扫描模式、噪声和安装参数替换当前名义值。

因此当前链路可用于接口开发和基本算法冒烟测试，尚不能作为目标传感器的统计仿真模型。
