# 代码地图：看到一个文件名，先查它的用途

版本：2026-09-06。配套主文档是 [零基础说明书](beginner_guide.md)。这里用于查文件，不要求按顺序读完。

**路径均从项目根目录开始理解，点击文件名可打开源码。** “正式”表示当前演示使用；“原型”表示独立验证或对照；“计划配置”表示文件描述了目标，但不等于功能全部实现。模型正在由另一位开发者更新，具体数值以最新模型代码为准。

## 1. 先认识目录和后缀

| 目录 | 放什么 |
| --- | --- |
| `ros_ws/src/` | 核心 ROS 包：消息、车辆、控制、砂土、传感器效应 |
| `scripts/windows/` | 你在 Windows 里调用的入口 |
| `scripts/wsl/` | Ubuntu 里的安装、构建、启动和验收流程 |
| `tools/` | Python 实验程序、数据处理、独立原型与验证工具 |
| `simulation/` | Gazebo 场景，以及运行、材料、定位等配置 |
| `foxglove/` | 浏览器监控面板的布局 |
| `config/wsl/` | Linux 环境变量设置 |
| `docs/` | 说明、开发记录、验收证据与本代码地图 |

| 文件后缀或名称 | 怎么理解 |
| --- | --- |
| `.cpp`、`.hpp`、`.cu` | C++ 源码、C++ 头文件、CUDA GPU 代码，需要相应构建工具 |
| `.py` | Python 程序，多用于流程、数据处理和实验 |
| `.ps1`、`.sh` | Windows PowerShell 脚本、Linux Bash 脚本，负责组织命令 |
| `.msg` | ROS 数据格式定义，构建后生成可供程序使用的类型 |
| `.yaml`、`.json` | 配置或数据；是否生效取决于谁读取它 |
| `.xacro`、`.sdf` | 车辆描述模板、Gazebo 模型/世界文件 |
| `.obj`、`.stl`、`.dae` | 常见三维网格资产格式，用来提供物体表面形状 |
| `package.xml` | 包的名称、依赖关系和基本说明 |
| `CMakeLists.txt` | 告诉构建工具编译什么、安装到哪里 |
| `.md`、`.html`、`.svg` | 文档源文件、网页阅读版、可缩放插图 |

## 2. 日常启动入口

| 文件 | 用途与使用时机 |
| --- | --- |
| [run_loader_soil_demo.ps1](../scripts/windows/run_loader_soil_demo.ps1) | Windows 正式入口。解析模式选项、检查 WSLg，再调用 Linux 主启动器 |
| [run_loader_soil_demo.sh](../scripts/wsl/run_loader_soil_demo.sh) | 总调度：生成模型，启动桥、物理世界、控制器、传感器和可选定位，运行场景并清理进程 |
| [repair_wslg_gui.ps1](../scripts/windows/repair_wslg_gui.ps1) | 修复特定 WSLg 显示问题后继续演示；会重启 WSL，停止其中的进程 |
| [verify_host.ps1](../scripts/windows/verify_host.ps1) | 盘点 Windows 主机和 WSL 环境信息 |
| [run_phase1_checks.ps1](../scripts/windows/run_phase1_checks.ps1) | 从 Windows 调用第一阶段基础检查 |
| [run_phase1_checks.sh](../scripts/wsl/run_phase1_checks.sh) | 在 Ubuntu 中组织环境、通信和 Gazebo 基础检查 |

启动器不负责在每一步计算液压或砂土公式。它的主要工作是把相应程序按正确方式启动起来。

## 3. ROS 核心包

### 3.1 数据格式：loader_sim_msgs

这四个文件位于 `ros_ws/src/loader_sim_msgs/msg/`。

| 文件 | 读它时关注什么 |
| --- | --- |
| [VehicleCommand.msg](../ros_ws/src/loader_sim_msgs/msg/VehicleCommand.msg) | 命令有哪些字段、合法挡位和单位 |
| [VehicleState.msg](../ros_ws/src/loader_sim_msgs/msg/VehicleState.msg) | 状态字段、轮速顺序、压力和载荷信息 |
| [BucketInteraction.msg](../ros_ws/src/loader_sim_msgs/msg/BucketInteraction.msg) | 铲斗受力与材料交换的报告格式 |
| [TerrainState.msg](../ros_ws/src/loader_sim_msgs/msg/TerrainState.msg) | 砂土高度与体积账本的报告格式 |

### 3.2 车辆模型：loader_description

| 文件 | 用途 |
| --- | --- |
| [loader.urdf.xacro](../ros_ws/src/loader_description/urdf/loader.urdf.xacro) | 正式模型模板：部件、质量、关节、传感器及插件开关。模型更新重点入口 |
| `ros_ws/src/loader_description/meshes/` | 正在更新的模型网格资产目录；网格用于外观、碰撞还是传感器几何，取决于模型文件怎样引用它 |
| [nominal_linkage.yaml](../ros_ws/src/loader_description/config/nominal_linkage.yaml) | 名义连杆几何与采样范围，供运动学验证工具读取 |
| [model_data_requirements.yaml](../ros_ws/src/loader_description/config/model_data_requirements.yaml) | 从简化模型走向实车模型所需数据的清单 |
| [display.launch.py](../ros_ws/src/loader_description/launch/display.launch.py) | 启动 RViz、模型发布和关节滑块，检查模型显示；区别于整车力级演示 |

### 3.3 正式控制：loader_control

| 文件 | 用途 |
| --- | --- |
| [loader_command_controller.cpp](../ros_ws/src/loader_control/src/loader_command_controller.cpp) | 正式执行器控制：接收统一命令，计算关节受力，检查超时/急停，发布整车状态 |
| [loader_controllers.yaml](../ros_ws/src/loader_control/config/loader_controllers.yaml) | 声明控制管理器频率、控制器和关节状态广播器类型 |
| [loader_control_plugins.xml](../ros_ws/src/loader_control/loader_control_plugins.xml) | 告诉 ROS 插件系统去哪个库里找到控制器类，不是控制算法本身 |

### 3.4 整车砂土：loader_soil

| 文件 | 用途 |
| --- | --- |
| [loader_soil_slice_system.cpp](../ros_ws/src/loader_soil/src/loader_soil_slice_system.cpp) | 正式二维土料插件：侵入、扫掠、阻力、体积转移、载荷重量、卸料与可视料柱更新 |

代码中的 `Configure` 用于读取配置、找到实体和准备通信，`PreUpdate` 参与每次仿真步的计算。它是 Gazebo 中被加载的插件，不是需要你单独双击运行的程序。

### 3.5 传感器效应：loader_sensor_effects

| 文件 | 用途 |
| --- | --- |
| [effects.py](../ros_ws/src/loader_sensor_effects/loader_sensor_effects/effects.py) | 丢点掩码、旋转数学和按扫描列分配的时间偏移 |
| [lidar_effects_node.py](../ros_ws/src/loader_sensor_effects/loader_sensor_effects/lidar_effects_node.py) | 接收点云与 IMU，调用效应计算，保留时间戳并发布效应点云 |
| [nominal.yaml](../ros_ws/src/loader_sensor_effects/config/nominal.yaml) | 正式演示使用的名义丢点概率、旋转效应、种子和扫描周期 |
| [__init__.py](../ros_ws/src/loader_sensor_effects/loader_sensor_effects/__init__.py) | Python 包标识，不是另一套传感器算法 |

### 3.6 早期对照：loader_dynamics

| 文件 | 用途 |
| --- | --- |
| [loader_dynamics_system.cpp](../ros_ws/src/loader_dynamics/src/loader_dynamics_system.cpp) | 早期直接驱动 Gazebo 关节的原型，保留作独立测试；当前正式演示默认关闭 |

六个包各自还有 `package.xml` 和 `CMakeLists.txt`，作用与第 1 节一致。修改业务逻辑通常先找上面的源码，构建或依赖失败时才重点查看这些装配文件。

## 4. ROS 辅助程序与定位

以下程序都放在 `tools/ros/`。

| 文件 | 用途与边界 |
| --- | --- |
| [loader_manual_gateway.py](../tools/ros/loader_manual_gateway.py) | 正式手动模式输入翻译器；合并驾驶/液压输入，处理使能、超时和急停 |
| [test_loader_soil_coupling.py](../tools/ros/test_loader_soil_coupling.py) | 既是整车土料验收，也是默认自动演示的动作脚本，包含铲取、举升、转运与卸料 |
| [run_localization_scenario.py](../tools/ros/run_localization_scenario.py) | 专用定位行驶动作，先通过液压控制收斗举升，再倒车和前进 |
| [generate_localization_world.py](../tools/ros/generate_localization_world.py) | 从既有感知世界生成带固定标志物的测试世界，输出到运行目录 |
| [filter_localization_cloud.py](../tools/ros/filter_localization_cloud.py) | 效应流下游的名义平地裁剪，压紧为有限 XYZ 点，供 KISS-ICP 配准 |
| [evaluate_localization.py](../tools/ros/evaluate_localization.py) | 接收里程计和独立真值，按时间插值、初始对齐并输出 JSON/CSV 报告 |
| [test_localization_metrics.py](../tools/ros/test_localization_metrics.py) | 检查裁剪、初始对齐、插值和漂移统计是否符合已知答案 |
| [test_loader_dynamics.py](../tools/ros/test_loader_dynamics.py) | 检查车辆动力学响应、命令限幅、急停和超时；可被不同控制链测试调用 |
| [test_loader_sensors.py](../tools/ros/test_loader_sensors.py) | 检查原始雷达、IMU、时钟、字段、坐标名称和消息频率 |
| [test_sensor_effects.py](../tools/ros/test_sensor_effects.py) | 检查扫描前后时间匹配、丢点、旋转、消息内存布局、IMU 噪声与安装扰动 |
| [test_foxglove_bridge.py](../tools/ros/test_foxglove_bridge.py) | 读取布局，核对话题和字段，验证手动网关，并调用真正的数据传输检查 |
| [foxglove_wire_check.py](../tools/ros/foxglove_wire_check.py) | 模拟 Foxglove 客户端接收 WebSocket 消息，核对解码、TF 和命令回读 |

**KISS-ICP 的核心算法代码来自第三方。** 本项目内的构建入口是 [bootstrap_localization.sh](../scripts/wsl/bootstrap_localization.sh)，源码位于 WSL 的 `/home/lyc/loader_sim_runtime/localization/src/kiss-icp/`。`docs/localization_selection.md` 是其他候选算法的选型记录，不能把源码构建记录当成实测精度。

## 5. 独立原型和生成工具

| 文件 | 用途 |
| --- | --- |
| [generate_linkage_table.py](../tools/kinematics/generate_linkage_table.py) | 从连杆配置计算角度、油缸长度和运动关系，输出表格并作解析检查 |
| [soil_slice_model.py](../tools/soil_slice/soil_slice_model.py) | Python 二维砂土数学原型，便于独立研究和检查守恒 |
| [run_soil_slice_smoke.py](../tools/soil_slice/run_soil_slice_smoke.py) | 给二维原型提供轨迹并验证受力、容量和材料账本 |
| [generate_soil_proxy_world.py](../tools/soil_slice/generate_soil_proxy_world.py) | 生成由料柱组成的 Gazebo 土堆世界，可配置是否带传感器 |
| [verify_soil_proxy_pose.py](../tools/soil_slice/verify_soil_proxy_pose.py) | 把料柱实际姿态与内部高度数据对照，检查画面几何是否真的更新 |
| [heightfield_model.py](../tools/soil_heightfield_3d/heightfield_model.py) | 独立三维高度场模型，按 x-y 网格保存高度并计算横向扫掠 |
| [run_heightfield_smoke.py](../tools/soil_heightfield_3d/run_heightfield_smoke.py) | 检查三维原型守恒、挖痕、卸料坡角，输出网格和结果 |
| [verify_cuda.cu](../tools/cuda/verify_cuda.cu) | 极小 GPU 计算例子，用于确认 CUDA 能真正执行 |
| [Chrono main.cpp](../tools/chrono_dem_smoke/main.cpp) | 独立的小规模颗粒接触试验，验证项目能链接并调用 Chrono |
| [Chrono CMakeLists.txt](../tools/chrono_dem_smoke/CMakeLists.txt) | 构建上面的外部 Chrono 验证程序 |

三维高度场和 Chrono 都有独立运行结果，但目前没有自动替换正式演示中的二维土料插件。

## 6. 世界文件与配置

世界文件相当于“试验场布置图”，不是驾驶策略。车辆通常由启动器另外生成并放入世界。

| 文件 | 用途 |
| --- | --- |
| [loader_soil_slice.sdf](../simulation/worlds/loader_soil_slice.sdf) | 默认整车二维砂土演示世界 |
| [loader_soil_perception.sdf](../simulation/worlds/loader_soil_perception.sdf) | 带感知设施、用于观察地形变化的砂土世界 |
| [loader_soil_control_profile.sdf](../simulation/worlds/loader_soil_control_profile.sdf) | 用于动态土料联合性能测试的世界 |
| [loader_sensors.sdf](../simulation/worlds/loader_sensors.sdf) | 有固定目标物的雷达/IMU 基础测试世界 |
| [loader_kinematics.sdf](../simulation/worlds/loader_kinematics.sdf) | 用于车辆模型与相关基础试验的简化世界 |
| [soil_slice_collision_masks.sdf](../simulation/worlds/soil_slice_collision_masks.sdf) | 专门验证刚性地面和松散料代理的碰撞分层 |
| [loader_smoke.sdf](../simulation/smoke/loader_smoke.sdf) | 最小 Gazebo/GPU 相机启动与性能检查场景 |
| [loader_demo.config](../simulation/config/gui/loader_demo.config) | Gazebo 演示窗口和初始视图设置 |
| [kiss_icp.yaml](../simulation/config/localization/kiss_icp.yaml) | 当前定位算法和名义平地裁剪参数，正式定位启动器会加载 |
| [dry_sand_nominal.yaml](../simulation/config/materials/dry_sand_nominal.yaml) | 二维砂土原型的材料与几何参数；不自动覆盖 C++ 土料插件 |
| [dry_sand_3d_nominal.yaml](../simulation/config/materials/dry_sand_3d_nominal.yaml) | 独立三维砂土原型配置 |
| [dem_smoke.json](../simulation/config/chrono/dem_smoke.json) | Chrono 小规模颗粒冒烟场景参数 |
| [smoke.yaml](../simulation/config/scenarios/smoke.yaml) | 基础场景配置骨架；具体启动行为仍需看调用脚本 |
| [control_realtime.yaml](../simulation/config/profiles/control_realtime.yaml) | 实时控制运行方案和已有性能记录；不是统一自动加载的总开关 |
| [bev_capture.yaml](../simulation/config/profiles/bev_capture.yaml) | 四鱼眼数据采集的目标配置，不能据此认为 BEV 已实现 |
| [dem_offline.yaml](../simulation/config/profiles/dem_offline.yaml) | 离线 DEM 运行方案，完整回放和标定仍需开发 |
| [loader_simulation_layout.json](../foxglove/loader_simulation_layout.json) | Foxglove 六页标签、面板、话题和颜色；与车辆物理模型分开 |

部分 SDF 文件由生成工具产生。修改这类场景前，先看文件开头是否注明生成来源，否则下次重新生成可能覆盖手工改动。

## 7. Linux 安装和构建脚本

以下文件位于 `scripts/wsl/`。日常使用不需要逐个运行。

| 文件 | 用途 |
| --- | --- |
| [bootstrap_ros_gazebo.sh](../scripts/wsl/bootstrap_ros_gazebo.sh) | 安装 ROS/Gazebo 及基础开发环境 |
| [bootstrap_cuda.sh](../scripts/wsl/bootstrap_cuda.sh) | 安装 CUDA 工具包 |
| [bootstrap_chrono_dem.sh](../scripts/wsl/bootstrap_chrono_dem.sh) | 取得固定版本 Chrono 并构建 DEM 所需部分 |
| [install_chrono_dem.sh](../scripts/wsl/install_chrono_dem.sh) | 安装已构建的 Chrono，供外部程序链接 |
| [bootstrap_localization.sh](../scripts/wsl/bootstrap_localization.sh) | 单独取得并构建固定版本 KISS-ICP |
| [build_workspace.sh](../scripts/wsl/build_workspace.sh) | 构建项目 ROS 包，输出到 WSL 运行目录 |
| [verify_environment.sh](../scripts/wsl/verify_environment.sh) | 检查 Linux 工具、图形和基础环境 |
| [verify_cuda.sh](../scripts/wsl/verify_cuda.sh) | 编译并执行小型 CUDA 验证 |
| [verify_chrono_dem.sh](../scripts/wsl/verify_chrono_dem.sh) | 运行 Chrono DEM 基础验证 |
| [verify_chrono_install.sh](../scripts/wsl/verify_chrono_install.sh) | 检查安装后的 Chrono 能否被项目外部 CMake 程序找到并调用 |
| [validate_description.sh](../scripts/wsl/validate_description.sh) | 展开并检查车辆描述 |
| [validate_linkage_kinematics.sh](../scripts/wsl/validate_linkage_kinematics.sh) | 组织连杆几何表生成和一致性验证 |
| [inspect_loader_sensor_topics.sh](../scripts/wsl/inspect_loader_sensor_topics.sh) | 用于查看 Gazebo 传感器消息通道 |

`config/wsl/` 下的 [loader-sim-wslg.sh](../config/wsl/loader-sim-wslg.sh)、[loader-sim-cuda.sh](../config/wsl/loader-sim-cuda.sh)、[loader-sim-chrono.sh](../config/wsl/loader-sim-chrono.sh) 分别设置图形、CUDA 和 Chrono 的环境变量。它们帮助程序找到正确的运行库，不是车辆算法。

## 8. Linux 验收与性能脚本

| 文件 | 检查范围 |
| --- | --- |
| [smoke_test_ros.sh](../scripts/wsl/smoke_test_ros.sh) | ROS 发布端和接收端能否实际通信 |
| [smoke_test_gazebo.sh](../scripts/wsl/smoke_test_gazebo.sh) | Gazebo 和 GPU 相机基础链路 |
| [smoke_test_loader_model.sh](../scripts/wsl/smoke_test_loader_model.sh) | 模型生成、落地与有限状态 |
| [smoke_test_loader_dynamics.sh](../scripts/wsl/smoke_test_loader_dynamics.sh) | 早期直接动力学原型 |
| [smoke_test_loader_ros2_control.sh](../scripts/wsl/smoke_test_loader_ros2_control.sh) | 正式控制器、关节状态与力级执行通路 |
| [smoke_test_loader_sensors.sh](../scripts/wsl/smoke_test_loader_sensors.sh) | 原始雷达、IMU 和时钟桥接 |
| [smoke_test_sensor_effects.sh](../scripts/wsl/smoke_test_sensor_effects.sh) | 雷达丢点/旋转、传输频率、IMU 噪声和安装扰动 |
| [smoke_test_soil_slice.sh](../scripts/wsl/smoke_test_soil_slice.sh) | 独立二维砂土原型和碰撞掩码 |
| [smoke_test_loader_soil_coupling.sh](../scripts/wsl/smoke_test_loader_soil_coupling.sh) | 整车铲装、载荷反馈、受力和守恒 |
| [smoke_test_loader_soil_perception.sh](../scripts/wsl/smoke_test_loader_soil_perception.sh) | 固定雷达是否真的观察到料堆挖除/卸料后的变化 |
| [smoke_test_soil_heightfield_3d.sh](../scripts/wsl/smoke_test_soil_heightfield_3d.sh) | 独立三维高度场的守恒与横向挖痕 |
| [smoke_test_foxglove_bridge.sh](../scripts/wsl/smoke_test_foxglove_bridge.sh) | 布局字段、真实 WebSocket 数据、坐标链路、手动命令回路 |
| [smoke_test_localization.sh](../scripts/wsl/smoke_test_localization.sh) | 无界面的定位行驶与独立真值误差报告 |
| [benchmark_gazebo.sh](../scripts/wsl/benchmark_gazebo.sh) | 最小世界的实时系数、GPU 和相机性能 |
| [benchmark_loader_control_profile.sh](../scripts/wsl/benchmark_loader_control_profile.sh) | 正式车辆控制加车载传感器的性能 |
| [benchmark_loader_soil_profile.sh](../scripts/wsl/benchmark_loader_soil_profile.sh) | 正式控制、动态砂土和传感器联合工况性能 |

`.sh` 一般准备环境、启动场景、记录日志，再调用 Python 或 C++ 检查程序。所以会同时看到 `smoke_test_*.sh` 与 `test_*.py`，它们是外层组织与内层检查的配合关系。

## 9. 文档、记录和生成文件

| 文件或目录 | 用途 |
| --- | --- |
| [README.md](../README.md) | 项目入口和安装、运行、验收命令索引 |
| [beginner_guide.md](beginner_guide.md) | 零基础主说明书，从软件用途讲到代码和数据流 |
| `beginner_guide.html`、`code_map.html` | 可直接用浏览器阅读的版本，由对应 Markdown 生成 |
| [user_guide.md](user_guide.md) | 日常按钮、启动和信号查看方法 |
| [deployment_status.md](deployment_status.md) | 当前完成状态和已知边界 |
| [windows_loader_simulation_plan.md](../windows_loader_simulation_plan.md) | 长期总体路线图，包含尚未实现的目标 |
| [localization_selection.md](localization_selection.md) | 定位候选方案的选型草稿，不代表候选算法都已正式集成 |
| [baselines/2026-09-06/README.md](baselines/2026-09-06/README.md) | 已保存的验收日志、定位指标和逐帧轨迹 |
| [host_crash_2026-09-06.md](host_crash_2026-09-06.md) | 主机蓝屏事件的已知事实与未确认事项 |
| [build_beginner_docs.py](../tools/docs/build_beginner_docs.py) | 把两份 Markdown 转成离线 HTML 阅读版，不启动仿真 |
| [software_overview.svg](assets/software_overview.svg) | 主说明书的软件分工插图 |
| [.gitignore](../.gitignore) | 指定 Git 不跟踪哪些临时或生成文件 |
| [LICENSE](../LICENSE) | 本项目许可证文本 |

你可以先选一个熟悉的动作，再沿“启动器 → 命令来源 → 正式控制器 → 物理世界 → 状态 → 面板”查文件。每次只读一条链，比同时打开所有目录更容易理解。
