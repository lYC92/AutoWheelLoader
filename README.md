# Loader Simulation Platform

Windows 10 + WSL2 上的无人装载机仿真平台。当前工作集中在第一阶段：部署并验证 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 和 WSL GPU 图形链路。

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

部署状态见 [docs/deployment_status.md](docs/deployment_status.md)。
