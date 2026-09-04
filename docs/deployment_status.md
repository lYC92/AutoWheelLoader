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
- “单铲斗 + 二维干砂切片”原型已通过：1 cm 子步、3.0 m³ 铲取/卸料、体积守恒误差 0、
  作用/反作用残差 0 N；当前名义峰值阻力 176.97 kN。
- Gazebo/DART 碰撞掩码已用自由落体实测：铲斗穿过 2.5 m 高松散料代理，仍在刚性地面
  z≈0.245 m 处停住，解析土料力不会与代理网格原生接触重复计数。
- `loader_soil` 已接入整车 Gazebo 模型：斗刃扫掠会更新二维内部高度场，解析切削阻力施加
  在斗刃，斗内物料重力施加在载荷质心，并发布 `BucketInteraction`/`TerrainState`。
- 整车 + 正式 `ros2_control` + 土料闭环已通过“低速铲取 → 举升 → 倒车转运 → 制动 →
  翻斗卸料”：车辆速度 0.966 m/s、最大侵入 0.672 m、峰值名义阻力 33.83 kN、铲取阶段
  载荷 0.559095 m³ / 894.55 kg、全过程体积账本误差 0；土料载荷质量已反馈到统一
  `VehicleState` 并通过一致性检查。
- `TerrainState` 现包含 280 格、5 cm 分辨率的完整高度剖面；280 个仅含可视几何的料柱以
  10 Hz 跟随挖除和异地卸料。变化最大料柱与状态剖面的实测位置误差为 0。
- 固定 512×32 GPU 观察雷达已完成动态料堆端到端检测：2,948 条射线变化超过 5 cm，
  其中 265 条位于变化最大料柱附近，证明更新后的 3D 几何进入了 Gazebo 渲染/射线场景。
- 完整动态土料性能工况已复测：500 Hz 物理、正式 `ros2_control`、280 格土料、10 Hz
  几何更新、车载 1024×32 雷达和 100 Hz IMU 同时运行并执行完整铲装/转运/卸料时，平均
  RTF 0.978491、车载雷达 9.93322 Hz、显存峰值 663 MiB，通过 RTF 0.9 和雷达 9 Hz 门槛。
- 独立三维干砂高度场门槛已通过：15 m × 10 m、10 cm 分辨率、15,000 格，2.7 m 宽斗刃
  以不大于 1 cm 子步形成 28 行横向挖痕；完成 3.0 m³ 铲取/异地卸料，体积误差
  `-2.132e-14 m³`，卸料最大坡角 33.985°，并输出连续三角网格 OBJ。

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
- [x] 完成单铲斗 + 二维干砂切片的守恒/受力/碰撞掩码原型
- [x] 把二维名义土料切片接入整车并反馈切削力、载荷质量和质心
- [x] 完成举升、倒车转运、翻斗卸料和休止角沉积的整车自动验收
- [x] 建立 10 Hz 动态料柱代理并与 `TerrainState` 高度剖面逐格核对
- [x] 验证 GPU 雷达实际观察到挖除和异地卸料后的动态料堆变化
- [x] 完整车辆、动态土料、车载雷达和 IMU 联合工况通过实时性能门槛
- [x] 建立三维干砂高度场独立原型并通过横向扫掠、守恒、反力和休止角门槛
- [x] 将旧直连动力学插件降为仅限显式 A-B 测试的内部开发入口

第 1 阶段 ROS/Gazebo 基础链路已完成，第 2 阶段的消息契约、运行配置和装载机模型骨架
已经落地，Project Chrono DEM 的构建、官方场景和外部链接链路也已通过。nominal 举升/
翻斗降阶运动学表已生成并通过解析自洽测试；轮端、铰接和液压名义动力学也已经由力级闭环
跑通。3D 雷达、IMU 和仿真时钟也已进入 ROS 2，并通过完整控制配置性能测试。厂家 CAD
对照仍等待实车几何输入。正式 `ros2_control` 力级控制链也已贯通，二维名义土料已经接入
整车并形成切削力、体积守恒、载荷反馈、异地卸料和动态可视几何闭环，GPU 雷达也已观察
到对应场景变化。三维高度场独立原型也已生成连续三角网格并通过守恒门槛；下一项是把该
三维内核迁入 Gazebo C++ 插件，再增加溢料和载荷惯量，并以 Chrono DEM 和实测数据标定；
轮胎/液压仍需细化，传感器侧继续补运动畸变和标定扰动。
性能结论已经覆盖当前二维动态土料完整工况；换成连续三维网格、鱼眼相机或更大规模 DEM
后必须重新复测。

## 已知风险

- Windows 10 22H2 已结束常规支持；接入办公网、互联网或实车HIL前，需要升级、ESU或隔离网络。
- RTX 2070 只有8GB显存，Gazebo多相机、BEV和Chrono DEM必须分时运行。
- WSL目前只分到约7.7GB内存，首次构建大型C++依赖时需要监测OOM和swap抖动。
- 当前动力学参数均为 `nominal`，闭环冒烟结果不能解释为实车精度或性能指标。
- 当前 176.97 kN 铲掘峰值来自未标定的二维 Rankine 近似，只能验证方向和记账，不能作为
  实车设计或精度结论。
- 整车场景中的料堆已能以 5 cm 料柱同步几何并被 GPU 雷达观察，但它仍是二维剖面外挤且
  存在可见接缝；33.83–36.35 kN 的近期峰值和 3 m³/s 排料率都是未标定名义值。
- Chrono 官方文档提示 WSL 的统一内存和锁页内存能力有限；当前 400 颗粒测试已通过，但在
  宣称大规模 DEM 可用前，仍需按目标粒子数做显存、锁页内存和长时间稳定性测试。
- Vulkan 当前只枚举到 `llvmpipe`。Gazebo Harmonic 的 OGRE2 已通过硬件 OpenGL 验证，
  因此不阻塞车辆仿真；在确认 Mesa DZN 与当前 NVIDIA/WSLg 版本稳定前不引入第三方 Mesa 源。
