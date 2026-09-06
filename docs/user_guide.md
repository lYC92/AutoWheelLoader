# 使用简介：启动项目与观测变量

如果还不清楚各个软件和代码模块分别做什么，请先读 [零基础软件说明书](beginner_guide.md)
和 [代码地图](code_map.md)；本文主要用于日常操作速查。

本文面向日常使用。环境安装和验收脚本见 [README.md](../README.md)，
界面设计细节见 [observability.md](observability.md)。

## 1. 启动项目

在普通 Windows PowerShell 中进入项目目录：

```powershell
cd "C:\Users\Liyangchuan\Documents\ChatGPT\New project"
```

三种启动方式，按需选一种：

| 命令 | 用途 |
| --- | --- |
| `powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1` | 自动演示：低速铲取 → 举升 → 倒车转运 → 制动 → 翻斗卸料，你只观察 |
| `... -Mode physics -ControlMode manual` | 手动模式：你在 Foxglove 里驾驶和操作铲斗 |
| `... -Mode perception` | 感知演示：自动动作 + 车载/固定激光雷达和 IMU 数据 |

启动后会自动打开两个窗口：Gazebo 三维界面和浏览器中的 Foxglove 数据工作台。
结束仿真：关闭 Gazebo 窗口，或在 PowerShell 中按 `Ctrl+C`。

自动演示会先制动并调整动臂、铲斗，打印 `PASS cutting pose ready` 后才前进。
这是为了避免窗口加载期间铲斗在重力作用下落地、把车顶住。行驶和转运期间会保持工作装置姿态。
如果姿态未就绪，程序会停止并报告关节角，而不是继续施加牵引扭矩。报错与正常输出均保存到演示日志。

如果启动器提示 `[WARN:COPY MODE]`，说明 WSLg 图形异常，运行
`.\scripts\windows\repair_wslg_gui.ps1` 自动修复并继续（会重启 WSL）。

## 2. 连接 Foxglove 并导入布局（只做一次）

1. 浏览器自动打开 `app.foxglove.dev`，首次使用需要登录或注册免费账号。
   ROS 数据走本机 WebSocket 直连，不会上传到云端。
2. 数据源选择 **Foxglove WebSocket**，地址 `ws://localhost:18765`（自动打开的链接已带此参数）。
3. 左上角 **Layouts → Import from file...**，选择
   `C:\Users\Liyangchuan\Documents\ChatGPT\New project\foxglove\loader_simulation_layout.json`。

## 3. 观测变量

预制布局六个标签页；升级后重新导入 JSON 布局：

- **整车总览**：车速/四轮轮速曲线、牵引扭矩、动臂角/铲斗角、液压压力、土体侵入深度、
  入斗/卸料流量、切削力；右侧车速和斗内载荷仪表、`VehicleState` 全量原始消息。
- **ROS 节点与消息**：节点/Topic/Service 拓扑图，`VehicleCommand`、`BucketInteraction`、
  `TerrainState` 原始消息。
- **手动控制**：见第 4 节。
- **感知与三维**：车载雷达和固定观察雷达点云、车辆模型、IMU 曲线
  （只有 `-Mode perception` 启动时有点云数据）。
- **第四阶段·传感器**：查看带名义丢点/旋转畸变的算法输入点云。
- **第四阶段·定位**：查看里程计估计和 odom 下的车辆/点云，需启用定位。

首次构建定位基线：

```powershell
wsl -d Ubuntu-24.04 -- bash '/mnt/c/Users/Liyangchuan/Documents/ChatGPT/New project/scripts/wsl/bootstrap_localization.sh'
```

之后使用以下命令运行带激光里程计的行驶试验；动作结束自动保存定位误差报告：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode perception -Localization kiss_icp -Scenario localization
```

报告和当前基线限制见 [localization.md](localization.md)。此里程计暂未融合 IMU、未做闭环建图。

常用观测手段：

- Plot 面板可直接输入消息路径，如 `/loader/state.longitudinal_speed_mps`、
  `/loader/bucket_interaction.bucket_wrench.force.x`；数组字段用切片，
  如 `/loader/state.wheel_speed_radps[:]`、`/loader/state.joint_state.position[6]`。
- 曲线默认跟随最近 20 秒，面板设置里可改时间窗或暂停时间轴。
- 想看布局之外的信号：新增 Plot 或 Raw Messages 面板直接输入 Topic 名即可，
  Foxglove Bridge 会公布 ROS 图中的全部 Topic、参数和服务，无需改任何代码。

重点 Topic 一览：

| Topic | 内容 |
| --- | --- |
| `/loader/command` | 控制命令：挡位、轮端扭矩、制动、铰接、举升/翻斗阀、急停 |
| `/loader/state` | 车速、轮速、关节状态、液压压力、斗内载荷质量和质心 |
| `/loader/bucket_interaction` | 侵入深度、切削力/力矩、入斗/排料流量、斗内体积 |
| `/loader/terrain_state` | 土体高度剖面、挖除/卸料体积、体积守恒误差 |
| `/joint_states` | 全部关节位置、速度、力/力矩 |
| `/loader/sensors/lidar/scan/points`、`/loader_soil/observer/scan/points`、`/loader/sensors/imu` | 传感器数据（仅 perception 模式） |
| `/loader/manual/status` | 手动网关状态摘要（JSON） |

## 4. 手动驾驶

用手动模式启动后，在 Foxglove“手动控制”页操作：

1. 先按住工作装置面板的“上”和“左”，举升并收斗，让铲刃离地。
2. 再用行驶面板方向键驾驶：上/下 = 前进/倒车，左/右 = 铰接转向。
3. 方向键是**按住持续动作**，松开后 0.35 秒内自动回中并制动。
4. 红色“急停”锁存急停；恢复时先点“释放急停”，再点“启用手动控制”。
   “禁用并制动”会整体关闭手动输出。

自动演示和手动模式不要同时向 `/loader/command` 发命令。

## 5. 出问题怎么查

页面没有数据时，保持仿真窗口运行，在 PowerShell 执行：

```powershell
Test-NetConnection localhost -Port 18765
wsl -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash; ros2 node list; ros2 topic list"
```

第一条应显示 `TcpTestSucceeded : True`；第二条应包含 `/foxglove_bridge`、
`/loader/state`、`/loader/command` 等。

整条监控链路的自动化验收（桥接端口、布局字段与真实消息逐一核对、
手动网关闭环与急停）：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_foxglove_bridge.sh
```

日志位置（WSL 内 `~/loader_sim_runtime/log/`）：

- `loader_soil_demo_foxglove.log`：Foxglove Bridge
- `loader_soil_demo_gazebo.log`：Gazebo 服务器
- `loader_soil_demo_manual_gateway.log`：手动控制网关

perception 模式的日志文件名带 `_perception` 后缀。
