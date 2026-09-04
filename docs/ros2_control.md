# `ros2_control` 正式控制链

公共控制入口已经从 Gazebo 直连原型迁移为：

`/loader/command → loader_command_controller → controller_manager → GazeboSimSystem → 关节 effort`

`loader_command_controller` 是 `controller_interface::ControllerInterface` 插件，运行在
`controller_manager` 中。它声明 7 路 effort 命令接口：四个车轮、铰接、举升和翻斗；并读取
8 个关节的位置/速度状态（额外包含后桥摆动）。算法不能直接写 Gazebo 关节或车辆位姿。

## 功能

- `VehicleCommand` 数值/档位检查、范围饱和和故障位；
- 四轮扭矩分配与速度相关制动；
- 铰接角闭环；
- 举升/翻斗名义油压动态和连杆雅可比力映射；
- 0.5 s 命令看门狗与急停；
- `/loader/state` 50 Hz 状态和标准 `/joint_states`；
- 控制器和 `robot_state_publisher` 统一使用 Gazebo `/clock`。

Xacro 参数 `enable_ros2_control:=true` 启用这条链路。不得同时启用旧的
`enable_dynamics:=true`，否则两套执行器会竞争同一批关节。

## 端到端验收

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_ros2_control.sh
```

脚本会启动 `/clock` 桥和 `robot_state_publisher`，生成车辆，激活
`loader_command_controller` 与 `joint_state_broadcaster`，核对公共命令话题订阅者，再执行
动力学、安全和标准关节状态测试。

2026-09-04 本机结果：两个控制器均为 `active`；收到 211 帧 `VehicleState`；铰接、举升、
翻斗运动量分别为 0.0375、1.1071、0.9668 rad；液压峰值为 16.25/12.50 MPa；饱和、急停、
超时和 `/joint_states` 均通过。证据位于：

- `/home/lyc/loader_sim_runtime/results/loader_ros2_control_smoke.txt`
- `/home/lyc/loader_sim_runtime/results/loader_ros2_control_controllers.txt`
- `/home/lyc/loader_sim_runtime/log/loader_ros2_control_smoke_gazebo.log`

## 当前边界

控制循环由 Gazebo 仿真步触发，可作为 SIL 的确定性软实时链路；WSL2/Windows 不宣称硬实时。
当前控制器中的车辆、液压和几何参数仍为 `nominal`，并且状态发布仍可能产生动态内存分配。
在 HIL 和定量标定前，需要把参数移入受版本控制的配置、改用完全实时安全的状态发布路径，
并增加接口占用冲突、生命周期异常和超时恢复测试。

旧 `loader_dynamics` Gazebo 插件保留为独立的开发/A-B 对照通路，默认关闭，不再作为算法的
正式入口。
