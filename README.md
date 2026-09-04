# Loader Simulation Platform

Windows 10 + WSL2 上的无人装载机仿真平台。基础环境、GPU 图形、CUDA/Chrono DEM、
名义车辆模型和第一版力级动力学闭环已经部署并通过自动化冒烟测试。

完整路线见 [windows_loader_simulation_plan.md](windows_loader_simulation_plan.md)。

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
