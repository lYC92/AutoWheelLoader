# ROS 2 可观测与操作界面

## 方案选择

本项目采用 Foxglove 作为主工作台，`foxglove_bridge` 直接连接 WSL2 中的 ROS 2 Jazzy。
它同时承担实时曲线、仪表、Topic 原始数据、ROS 计算图、三维点云和控制消息发布。这样算法、
控制和仿真调试使用同一个时间轴与界面，不需要维护一套自研 Web 前端。

PlotJuggler 适合后续做数十条高速时序信号的精细对齐、变换和离线分析，但不作为当前主
界面，因为它不能在一个工作区内同等完整地覆盖 ROS 拓扑、三维点云和操作面板。

## 界面结构

预制布局位于 [`../foxglove/loader_simulation_layout.json`](../foxglove/loader_simulation_layout.json)，
分为六页（更新后需要重新导入布局文件）：

| 页面 | 主要内容 | 用途 |
| --- | --- | --- |
| 整车总览 | 车速/轮速、举升/翻斗、液压压力、土体侵入/流量/切削力、载荷仪表 | 一眼判断动作链和负载是否合理 |
| ROS 节点与消息 | Topic Graph、`VehicleCommand`、`VehicleState`、土体交互和地形状态 | 检查节点、Topic、服务及消息字段 |
| 手动控制 | 两个 Teleop 十字键、使能、禁用、急停、急停复位、网关状态 | 手动驾驶和工作装置操作 |
| 感知与三维 | 观察雷达/车载雷达点云、机器人模型、IMU 曲线和原始消息 | 调试激光雷达、定位和后续 BEV 感知 |
| 第四阶段·传感器 | 丢点/旋转畸变后的点云、IMU、消息时间戳 | 检查算法实际输入，与理想点云对照 |
| 第四阶段·定位 | odom 坐标系下的车辆、点云、估计位置曲线和里程计 | 使用 `-Mode perception -Localization kiss_icp` |

Foxglove Bridge 默认向界面公布当前 ROS 图中的全部 Topic、参数和服务，因此预制面板不是
信号白名单。要观察任意新节点或新消息，可在 Foxglove 中新增 Plot、Raw Messages、Topic
Graph、Parameters 或 Service Call 面板，无需修改桥接代码。

## 首次使用

在普通 Windows PowerShell 中运行：

```powershell
cd "C:\Users\Liyangchuan\Documents\ChatGPT\New project"
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode physics -ControlMode auto
```

启动器会同时打开 Gazebo 和 Foxglove。Foxglove 网页首次使用需要你本人登录或创建免费账号；
完成后数据源会通过下列本机地址连接：

```text
ws://localhost:18765
```

随后只需导入一次布局：

1. 打开左上方 **Layouts**。
2. 选择 **Import from file...**。
3. 选择
   `C:\Users\Liyangchuan\Documents\ChatGPT\New project\foxglove\loader_simulation_layout.json`。
4. 导入后选择“整车总览”或其他标签页。

浏览器会保存当前个人布局。若自动打开的页面没有连接数据，可在 Foxglove 的数据源菜单中
手动选择 **Foxglove WebSocket**，地址仍填写 `ws://localhost:18765`。

## 三种启动方式

自动铲装演示与总线监视：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode physics -ControlMode auto
```

手动驾驶和铲斗操作：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode physics -ControlMode manual
```

包含车载与固定观察激光雷达、IMU 的感知演示：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode perception -ControlMode auto
```

自动和手动模式互斥，目的是保证 `/loader/command` 同一时刻只有一个控制源。

## 手动操作

手动页面发布两个 `geometry_msgs/msg/Twist` Topic：

| Topic | 字段 | 动作 |
| --- | --- | --- |
| `/loader/manual/drive` | `linear.x` | 正值前进、负值倒车 |
| `/loader/manual/drive` | `angular.z` | 铰接转向 |
| `/loader/manual/hydraulics` | `linear.z` | 动臂举升/下降 |
| `/loader/manual/hydraulics` | `angular.y` | 收斗/卸料 |

`tools/ros/loader_manual_gateway.py` 将这些归一化输入转换为正式的
`loader_sim_msgs/msg/VehicleCommand`。它包含三层防护：

- 只有 `-ControlMode manual` 才启动，避免与自动脚本抢占命令。
- Teleop 输入超过 0.35 秒没有刷新，就将行驶扭矩归零并施加制动。
- `/loader/manual/emergency_stop` 优先级最高；`/loader/manual/enable` 可整体禁用控制。

开始驾驶前，先按住工作装置“上”和“左”，举升并收斗使铲刃离地。车辆出生点朝向料堆，
前进会很快进入土体阻力区；要先测试空载行驶，使用“下”键倒车离开料堆。

## 重点信号

| Topic | 内容 |
| --- | --- |
| `/loader/command` | VCU 边界输入：挡位、轮端扭矩、制动、铰接目标、举升/翻斗阀命令、急停 |
| `/loader/state` | 车速、轮速、关节状态、液压压力、载荷质量和控制状态 |
| `/loader/bucket_interaction` | 侵入深度、切削力/力矩、入斗/排料流量、斗内体积 |
| `/loader/terrain_state` | 土体网格、挖除/卸料体积与守恒记账 |
| `/joint_states` | 完整关节位置、速度和力/力矩 |
| `/tf`、`/tf_static` | 车辆与传感器坐标变换 |
| `/loader/sensors/lidar/scan/points` | 车载 32 线雷达点云，仅 perception 模式 |
| `/loader/sensors/lidar/scan/points_effect` | 算法输入：名义 10% 丢点与旋转畸变，保留扫描时间戳 |
| `/loader/localization/points` | KISS-ICP 实际配准点；效应流经名义平地裁剪后的 XYZ |
| `/loader/localization/odometry` | KISS-ICP 估计，仅启用定位时存在；不是地图全局定位 |
| `/loader/ground_truth/odometry` | world 下的仿真真值，只供评测，不作为算法输入 |
| `/loader_soil/observer/scan/points` | 固定观察雷达点云，仅 perception 模式 |
| `/loader/sensors/imu` | 车载 IMU，仅 perception 模式 |
| `/loader/manual/status` | 手动使能、急停、输入新鲜度和最终网关输出摘要 |

Plot 面板支持直接输入消息路径，例如
`/loader/state.longitudinal_speed_mps` 或
`/loader/bucket_interaction.bucket_wrench.force.x`。数组字段可用切片，例如
`/loader/state.wheel_speed_radps[:]`。曲线默认显示最近 20 秒，可在面板设置中调整窗口或暂停
时间轴做局部检查。

## 运行边界与安全

- Bridge 只监听 WSL2 的 `127.0.0.1:18765`，用于本机 Windows 浏览器，不对局域网开放。
- Foxglove 的 Publish 与 Teleop 能向 ROS 回写消息，只在仿真模式使用；接真实 VCU 前必须增加
  独立的模式仲裁、鉴权和硬件急停，不能把网页按钮当作安全功能。
- 当前预制 3D 页需要 perception 模式的数据。physics 模式下点云 Topic 不存在属于正常现象。
- Foxglove 账号只用于界面与个人布局；ROS 数据通过本机 WebSocket 直连，不需要上传到云端。

## 自动化验收

监控链路检查包括 ROS 话题/字段、真实 WebSocket 协议协商、CDR 消息解码、URDF、
动态关节与 TF、手动网关超时/急停，以及 WebSocket 发布工作装置命令后的回读。
perception 模式额外强制检查原始/效应点云、IMU 和通向 base_link 的 TF；缺失即失败。

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_foxglove_bridge.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_foxglove_bridge.sh perception
```

检查结果写入 WSL 的 `~/loader_sim_runtime/results/foxglove_bridge_{physics,perception}_smoke.txt`。
测试使用独立 ROS domain 和 Gazebo partition；8765 端口已占用时会退出。
首次运行在运行目录创建独立 Python venv 并安装固定版本 websockets 15.0.1。

2026-09-06 复核发现并修复：旧启动器遗漏 `joint_state_broadcaster`，导致 `/joint_states`
和 `/tf` 只有订阅端、没有实际消息。当前正式启动器和测试均显式激活该广播器。
本机 Bridge 3.4.1 使用 `foxglove.sdk.v1`；测试同时提供 SDK 和旧 v1 子协议供协商。
高频回调可能造成相邻时间戳轻微乱序；首轮实测 2–4 ms，最终感知回归最大 16 ms。
验收要求源时间持续推进、乱序不超过
100 ms，Plot 使用消息 header 时间戳。协议验收不替代登录后对预制布局的目视检查。

## 故障检查

日常启动器使用 `18765`，以避开本机 Windows 系统保留的 `8679–8778` 端口范围。
旧 Foxglove 页面如果仍连接 `ws://localhost:8765`，请改为 `ws://localhost:18765`。
保留端口可能没有任何监听进程，但仍禁止程序绑定，表现为 WSL 内服务正常、Windows
连接被拒绝。可用 `netsh interface ipv4 show excludedportrange protocol=tcp` 查看范围。
独立 WSL 冒烟测试仍使用 `8765`，不经过 Windows 转发。

如果页面没有数据，保持仿真 PowerShell 窗口运行，并执行：

```powershell
Test-NetConnection localhost -Port 18765
wsl -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash; ros2 node list; ros2 topic list"
```

第一条应显示 `TcpTestSucceeded : True`，第二条应至少包含 `/foxglove_bridge`、
`/loader_command_controller`、`/loader/state` 和 `/loader/command`。桥接日志位于 WSL：

```text
/home/lyc/loader_sim_runtime/log/loader_soil_demo_foxglove.log
```

感知模式日志文件名中包含 `_perception`。
