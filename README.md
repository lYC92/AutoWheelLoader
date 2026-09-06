# Loader Simulation Platform

Windows 10 + WSL2 上的无人装载机仿真平台。基础环境、GPU 图形、CUDA/Chrono DEM、
名义车辆模型和第一版力级动力学闭环已经部署并通过自动化冒烟测试。

完整路线见 [windows_loader_simulation_plan.md](windows_loader_simulation_plan.md)。

**第一次接触这个项目？** 先读 [零基础软件说明书](docs/beginner_guide.md)，再按需查
[代码地图](docs/code_map.md)。也可以用浏览器直接打开 [带目录的阅读版](docs/beginner_guide.html)，
无需启动仿真。

## 一键运行可视化演示

在普通 PowerShell 中进入项目目录，运行：

```powershell
cd "C:\Users\Liyangchuan\Documents\ChatGPT\New project"
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1
```

启动器会打开 Gazebo，生成 `soil_loader`，界面就绪后立即自动执行“低速铲取 → 举升 →
倒车转运 → 制动 → 翻斗卸料”。动作结束后 Gazebo 保持打开，直到关闭窗口或在 PowerShell
中按 `Ctrl+C`。如果默认镜头没有对准车辆，在左侧 Entity tree 选择 `soil_loader` 后按 `F`。
仿真服务器继续使用 NVIDIA D3D12；交互界面单独使用 llvmpipe 软件 OpenGL，以规避当前
Windows 10 / WSLg 中 Qt 与服务器共享 D3D12 上下文时的窗口崩溃。

带固定观察激光雷达的感知演示使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode perception
```

启动器还会打开 Foxglove 数据工作台并连接 `ws://localhost:8765`。首次使用需要在浏览器中
登录 Foxglove，然后从 **Layouts → Import from file...** 导入
[`foxglove/loader_simulation_layout.json`](foxglove/loader_simulation_layout.json)。预制布局包含
整车/液压/土体曲线、仪表、原始消息、ROS Topic 拓扑、激光点云、手动操作，以及第四阶段
传感器效应和定位，共六页。更新后请重新导入布局。

第四阶段已接入日常感知模式：原始点云经过丢点/旋转畸变后发布到 `points_effect`。
激光里程计基线与真值误差评测见 [docs/localization.md](docs/localization.md)。首次运行
`scripts/wsl/bootstrap_localization.sh` 构建后，可启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode perception -Localization kiss_icp -Scenario localization
```

需要从界面手动驾驶、转向、举升和翻斗时运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode physics -ControlMode manual
```

手动模式下先在“手动控制”页举升并收斗，使刃口离地，再操作行驶方向键。方向键为按住
持续动作，松开后控制网关会在 0.35 秒内回中并制动；急停和控制使能也在同一页。自动模式
与手动模式不要同时向 `/loader/command` 发布命令。详细用法和信号表见
[docs/observability.md](docs/observability.md)，日常使用简介见
[docs/user_guide.md](docs/user_guide.md)。

验证 Foxglove 桥接、预制布局字段和手动控制网关的完整监控链路（无 GUI）：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_foxglove_bridge.sh
```

启动器会自动检查 WSLg。若发现任务栏处于 `[WARN:COPY MODE]`（进程存在但窗口不可见），
它会修复共享内存、重启 WSL 后继续启动 Gazebo。重启会停止其他正在运行的 WSL 进程。
也可以单独执行修复启动器：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\repair_wslg_gui.ps1
```

该命令会挂载 WSLg 所需的 `/mnt/shared_memory`、执行 `wsl --shutdown`，并立即启动整车演示，
因此会停止其他正在运行的 WSL 进程。当前 WSL 2.7.12 的上游缺陷可能在 WSLg 完全退出后
再次出现；遇到同一提示时重新运行该命令，等待包含 DeviceHost 1.2.62 的 WSL 正式版更新。

## 当前部署方式

所有 ROS 2、Gazebo 和后续算法进程均运行在 `Ubuntu-24.04` WSL2 中。Windows 只承载开发工具、资产工具和未来的 CAN 网关。

本机 Mesa 需要显式选择 WSLg 的 D3D12 后端；安装脚本会把
`config/wsl/loader-sim-wslg.sh` 安装到 `/etc/profile.d/`，使 Gazebo 使用 NVIDIA GPU 而不是 `llvmpipe`。

### 安装基础环境

在管理员 PowerShell 中执行：

```powershell
wsl -d Ubuntu-24.04 -u root -- env LOADER_SIM_USER=lyc bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/bootstrap_ros_gazebo.sh
```

### 验证基础环境

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/verify_environment.sh
```

### 验证 ROS 2 节点通信

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_ros.sh
```

### 验证 Gazebo 无 GUI 渲染

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_gazebo.sh
```

也可在项目目录用 PowerShell 一次运行以上三项验收：

```powershell
.\scripts\windows\run_phase1_checks.ps1
```

采集 500 Hz 无 GUI 场景的实时系数与 GPU 基线：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/benchmark_gazebo.sh
```

需要可视窗口时，可运行 GUI 基线（会打开并自动关闭 Gazebo 窗口）：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/benchmark_gazebo.sh gui
```

构建 ROS 2 工作区（源码保留在项目中，构建产物写入 WSL ext4）：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/build_workspace.sh
```

验证 nominal 装载机拓扑并在 Gazebo 中生成、落地和稳定运行：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/validate_description.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_model.sh
```

安装并验证 Chrono::Dem 所需的 CUDA 13.2（只安装工具包，不安装 Linux NVIDIA 驱动）：

```powershell
wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/bootstrap_cuda.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/verify_cuda.sh
```

构建、运行并安装固定版本的 Project Chrono DEM：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/bootstrap_chrono_dem.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/verify_chrono_dem.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/install_chrono_dem.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/verify_chrono_install.sh
```

首次 Chrono 构建耗时较长，并固定使用两个并行编译任务以适应当前约 8GB 的 WSL 内存。
源码、构建、安装和测试证据均保存在 WSL 的
`/home/lyc/loader_sim_runtime`，详细记录见
[docs/chrono_dem.md](docs/chrono_dem.md)。

生成并验证 nominal 举升/翻斗降阶运动学查表：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/validate_linkage_kinematics.sh
```

查表文件写入 WSL 的
`/home/lyc/loader_sim_runtime/results/linkage/nominal_linkage_table.csv`。模型定义和厂家数据
替换要求见 [docs/linkage_kinematics.md](docs/linkage_kinematics.md)。

构建并验证 Gazebo 名义动力学插件：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/build_workspace.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_dynamics.sh
```

当前插件涵盖轮端扭矩/制动、铰接转向、油缸压力、连杆雅可比力映射、状态反馈、命令超时
和急停；它仍是 `nominal` 开发模型，边界和验证证据见
[docs/loader_dynamics.md](docs/loader_dynamics.md)。

验证公共 `VehicleCommand` 经 `controller_manager` 和 `GazeboSimSystem` 到 effort 接口的正式
控制链：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_ros2_control.sh
```

链路、控制器生命周期和当前边界见 [docs/ros2_control.md](docs/ros2_control.md)。

验证 3D 激光雷达、IMU、ROS 2 桥和仿真时钟：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_sensors.sh
```

验证名义传感器效应通道（随机丢点、旋转扫描运动畸变）、IMU 噪声模型和传感器安装
标定扰动：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_sensor_effects.sh
```

对动力学、32 线雷达和 IMU 的完整实时控制配置做性能验收：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/benchmark_loader_control_profile.sh
```

当前平均实时系数为 0.990584，雷达为 9.98277 Hz，显存峰值为 723 MiB。接口、坐标系和
未完成项见 [docs/lidar_imu.md](docs/lidar_imu.md)。

验证二维干砂切削、体积守恒、作用/反作用与 Gazebo 碰撞掩码：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_soil_slice.sh
```

验证整车、正式 `ros2_control` 与二维名义土料模型的闭环耦合：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_soil_coupling.sh
```

当前整车验收已覆盖“低速铲取 → 举升 → 倒车转运 → 制动 → 翻斗卸料”：最大侵入
0.672 m、峰值名义阻力 33.83 kN、铲取阶段装入 0.559095 m³（894.55 kg），完整过程
体积守恒误差为 0。载荷质量已反馈到统一 `VehicleState`，280 个 5 cm 料柱以 10 Hz
同步高度场；抽检料柱与 `TerrainState` 的高度误差为 0。公式、实测证据和现阶段限制见
[docs/soil_slice.md](docs/soil_slice.md)。

验证 GPU 激光雷达能观察到同一轮挖除/卸料后的动态几何：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_soil_perception.sh
```

当前固定观察雷达在变化最大料柱附近检测到 265 条变化射线，排除了只看内部高度数组而未
更新渲染场景的情况。

对 500 Hz 车辆、正式控制链、动态土料、车载 32 线雷达和 IMU 做联合性能验收：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/benchmark_loader_soil_profile.sh
```

当前完整动态工况平均实时系数为 0.978491，车载雷达为 9.93322 Hz，显存峰值 663 MiB，
通过 `RTF >= 0.9` 和雷达 `>= 9 Hz` 门槛。

验证三维干砂高度场的横向挖痕、体积守恒、六维方向记账和休止角卸料：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_soil_heightfield_3d.sh
```

当前 150×100 网格原型完成 3.0 m³ 铲取和异地卸料，体积误差为
`-2.132e-14 m³`，并输出连续三角网格 OBJ。详细边界见
[docs/soil_heightfield_3d.md](docs/soil_heightfield_3d.md)。

部署状态见 [docs/deployment_status.md](docs/deployment_status.md)。
