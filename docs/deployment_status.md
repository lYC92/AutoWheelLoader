# 部署状态

更新时间：2026-09-04

## 已确认

- Windows 10 22H2，Build 19045.6466。
- WSL 2.7.12，默认发行版为 `Ubuntu-24.04`，WSL版本为2。
- 同机还存在 `Ubuntu-22.04`，本项目不使用该发行版。
- Ubuntu 24.04.1 用户为 `lyc`。
- WSLg 的 `DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0` 已配置。
- WSL 内可以通过 `nvidia-smi` 识别 NVIDIA GeForce RTX 2070 8GB。
- Windows NVIDIA 驱动版本为 595.95。
- WSL 当前可用内存约 7.7GB，swap 2GB。
- WSL 根文件系统剩余约 947GB；Windows C盘剩余约324GB。
- ROS 2 Jazzy Desktop `0.11.0`、`ros_gz 1.0.22`、`ros2_control 4.45.2`、
  `ros2_controllers 4.40.1` 和 `gz_ros2_control 1.2.19` 已安装。
- Gazebo Sim 8.11.0、Mesa 25.2.8、GCC 13.3、CMake 3.28.3、Python 3.12.3 已安装。
- Blender 5.2.1 LTS 已安装并可从 Windows 命令行调用。
- NVIDIA CUDA Toolkit 13.2（`nvcc 13.2.86`）已安装在 WSL；未安装 Linux 显卡驱动，
  继续使用 WSL 提供的 Windows 驱动接口。
- CUDA 实机内核已在 RTX 2070 上通过：计算能力 7.5，测试结果 `42`。
- Project Chrono 已固定到官方提交
  `583b8e6f48600699f2084154a31742261d28a7c7`，以 CUDA 13.2、CUDA 后端和
  `sm_75` 架构完成 Release 构建。
- Chrono 官方 DEM 场景已在 GPU 上推进 0.02 秒并输出 376 行颗粒状态。
- Chrono core/DEM 已安装到 `/home/lyc/loader_sim_runtime/install/chrono`；项目外部
  CMake 工程已通过 `find_package(Chrono COMPONENTS DEM)` 链接，并完成 400 颗粒摩擦接触运行。
- Mesa 自动选择错误已通过 `GALLIUM_DRIVER=d3d12` 和
  `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 修复并固化。
- WSLg OpenGL 使用 `D3D12 (NVIDIA GeForce RTX 2070)`，硬件加速开启，核心版本为 4.6。
- ROS 2 C++ talker 与 Python listener 已完成 DDS 实际收发。
- Gazebo 无 GUI GPU 相机话题 `/loader_sim/smoke/camera` 已发布并验证为 `gz.msgs.Image`。
- 500 Hz 最小场景基线：无 GUI 平均实时系数 0.985740，相机 10.0976 Hz，显存峰值 645 MiB；
  GUI 平均实时系数 0.990778，相机 9.99661 Hz，显存峰值 819 MiB。
- `loader_sim_msgs` 已生成 C/C++/Python/Rust 接口并通过接口与 Python 导入验证。
- `loader_description` 已构建，Xacro/URDF 零警告解析，8 个关键关节齐全；模型已在 Gazebo
  中完成生成、6 秒落地稳定和有限位姿/关节状态检查。

## 当前阶段

- [x] 主机与 WSL2 盘点
- [x] 固化 ROS 2 / Gazebo 安装脚本
- [x] 固化环境验证脚本
- [x] 创建 Gazebo GPU 相机冒烟测试世界
- [x] 安装 ROS 2 Jazzy 与 Gazebo Harmonic
- [x] 验证 ROS 2 通信
- [x] 验证 WSLg OpenGL 硬件渲染
- [ ] 验证 Vulkan 硬件渲染（当前非 Gazebo OGRE2 阻塞项）
- [x] 运行 Gazebo 无 GUI GPU 相机冒烟测试
- [x] 采集第1轮 GUI/无 GUI 实时系数、相机频率和显存基线
- [x] 建立 ROS 2 消息契约和运行配置骨架
- [x] 建立装载机 Xacro/URDF 骨架并完成 Gazebo 落地冒烟测试
- [x] 安装 CUDA Toolkit 13.2 并运行 RTX 2070 CUDA 内核验证
- [x] 构建并验证 Project Chrono DEM
- [ ] 建立举升/翻斗降阶运动学表和 CAD 对照测试

第 1 阶段 ROS/Gazebo 基础链路已完成，第 2 阶段的消息契约、运行配置和装载机模型骨架
已经落地，Project Chrono DEM 的构建、官方场景和外部链接链路也已通过。下一项是建立
举升/翻斗降阶运动学表和 CAD 对照测试，再开始铰接转向、工作装置动力学和土料耦合接口。
当前性能结论只适用于最小场景；完整车辆与传感器接入后必须复测。

## 已知风险

- Windows 10 22H2 已结束常规支持；接入办公网、互联网或实车HIL前，需要升级、ESU或隔离网络。
- RTX 2070 只有8GB显存，Gazebo多相机、BEV和Chrono DEM必须分时运行。
- WSL目前只分到约7.7GB内存，首次构建大型C++依赖时需要监测OOM和swap抖动。
- Chrono 官方文档提示 WSL 的统一内存和锁页内存能力有限；当前 400 颗粒测试已通过，但在
  宣称大规模 DEM 可用前，仍需按目标粒子数做显存、锁页内存和长时间稳定性测试。
- Vulkan 当前只枚举到 `llvmpipe`。Gazebo Harmonic 的 OGRE2 已通过硬件 OpenGL 验证，
  因此不阻塞车辆仿真；在确认 Mesa DZN 与当前 NVIDIA/WSLg 版本稳定前不引入第三方 Mesa 源。
