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
- `loader_dynamics` 已完成构建并安装；WSL 构建脚本已隔离 Windows Anaconda 路径，避免
  Windows Protobuf 头文件污染 ROS/Gazebo C++ 构建。
- 500 Hz Gazebo 力级闭环冒烟测试已通过：收到 212 帧状态，铰接/举升/翻斗均由力或扭矩
  产生运动，举升/翻斗压力峰值为 16.25/12.50 MPa；命令饱和、显式急停和 0.5 s 看门狗
  超时均已验证。
- 3D GPU 雷达和 IMU 已挂接到独立 `lidar_link`/`imu_link`，通过 `ros_gz_bridge` 输出标准
  `PointCloud2`、`Imu` 和仿真 `/clock`；点云尺寸 1024×32，坐标系、有限值和时间单调性
  已自动验证。
- 完整 `control_realtime` 名义组合已通过门槛：500 Hz 物理、车辆动力学、10 Hz 32 线雷达
  和 100 Hz IMU同时运行时，正式 `ros2_control` 链路的平均 RTF 0.990584、雷达
  9.98277 Hz、显存峰值 723 MiB。
- `loader_command_controller` 和 `joint_state_broadcaster` 已由 `controller_manager` 激活；
  `/loader/command` 经 `GazeboSimSystem` 的 effort 接口驱动车辆，`/joint_states` 和
  `/loader/state` 均已端到端验证。

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
- [x] 建立 nominal 举升/翻斗降阶运动学表和解析自洽测试
- [x] 建立 nominal 轮端/铰接/液压动力学插件并通过 ROS 2 闭环冒烟测试
- [x] 接入 3D GPU 雷达、IMU、ROS 2 桥和 `/clock` 并通过坐标系/频率测试
- [x] 完整 nominal `control_realtime` 配置通过 RTF 0.9 性能门槛
- [ ] 用厂家 CAD/实测铰点替换 nominal 参数并完成全工作区对照
- [x] 接入 `loader_command_controller`/`ros2_control` 正式控制链
- [ ] 将旧直连动力学插件降为仅限显式 A-B 测试的内部开发入口

第 1 阶段 ROS/Gazebo 基础链路已完成，第 2 阶段的消息契约、运行配置和装载机模型骨架
已经落地，Project Chrono DEM 的构建、官方场景和外部链接链路也已通过。nominal 举升/
翻斗降阶运动学表已生成并通过解析自洽测试；轮端、铰接和液压名义动力学也已经由力级闭环
跑通。3D 雷达、IMU 和仿真时钟也已进入 ROS 2，并通过完整控制配置性能测试。厂家 CAD
对照仍等待实车几何输入。正式 `ros2_control` 力级控制链也已贯通。下一项是轮胎/液压细化
和“单铲斗 + 二维土堆切片”耦合原型；传感器侧继续补运动畸变和标定扰动。
当前性能结论只适用于最小场景；完整车辆与传感器接入后必须复测。

## 已知风险

- Windows 10 22H2 已结束常规支持；接入办公网、互联网或实车HIL前，需要升级、ESU或隔离网络。
- RTX 2070 只有8GB显存，Gazebo多相机、BEV和Chrono DEM必须分时运行。
- WSL目前只分到约7.7GB内存，首次构建大型C++依赖时需要监测OOM和swap抖动。
- 当前动力学参数均为 `nominal`，闭环冒烟结果不能解释为实车精度或性能指标。
- Chrono 官方文档提示 WSL 的统一内存和锁页内存能力有限；当前 400 颗粒测试已通过，但在
  宣称大规模 DEM 可用前，仍需按目标粒子数做显存、锁页内存和长时间稳定性测试。
- Vulkan 当前只枚举到 `llvmpipe`。Gazebo Harmonic 的 OGRE2 已通过硬件 OpenGL 验证，
  因此不阻塞车辆仿真；在确认 Mesa DZN 与当前 NVIDIA/WSLg 版本稳定前不引入第三方 Mesa 源。
