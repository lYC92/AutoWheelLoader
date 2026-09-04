# 装载机名义动力学插件

`loader_dynamics` 是第 2 阶段的力级闭环原型。它在 Gazebo 每个物理步读取
`VehicleCommand`，以关节力/扭矩驱动车辆和工作装置，并发布 `VehicleState`。插件不会直接
设置车体、动臂或铲斗位姿。

## 当前实现

- 四轮总牵引扭矩分配、速度相关制动力；
- 铰接转向角 PD 和力矩限幅；
- 举升、翻斗油路的一阶压力响应和泄压限幅；
- 通过降阶连杆解析雅可比 `dL/dq` 将油缸力转换为关节广义力；
- 50 Hz 状态反馈，包括轮速、关节状态、油缸位置、压力和故障位；
- 非有限数拒绝、非法档位拒绝、命令饱和标志、0.5 s 看门狗和急停。

动力学插件只在 Xacro 参数 `enable_dynamics:=true` 时加载。默认关闭，避免影响纯模型和
传感器测试。

## 构建和验证

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/build_workspace.sh
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_dynamics.sh
```

闭环冒烟测试会启动 500 Hz Gazebo Server，生成启用动力学的装载机，并验证：

1. 未收到命令时进入看门狗急停；
2. 连续命令可驱动铰接、举升和翻斗；
3. 油缸压力和全部关节状态为有限值；
4. 越界命令触发饱和故障位；
5. 显式急停和停止发送后的超时急停均有效。

2026-09-04 本机结果：收到 212 帧状态；铰接、举升和翻斗的最大运动量分别为
0.0170 rad、1.1067 rad、0.8316 rad；举升和翻斗压力峰值分别为 16.25 MPa、
12.50 MPa。证据保存在 WSL：

- `/home/lyc/loader_sim_runtime/results/loader_dynamics_smoke.txt`
- `/home/lyc/loader_sim_runtime/log/loader_dynamics_smoke_gazebo.log`

## 准确度边界

当前结果只证明软件链路、力/扭矩驱动方式和安全状态机成立，不证明实际车辆动力学精度。
质量、惯量、轮胎、压力、缸径、阻尼和阀响应均为 `nominal`。动臂/铲斗的大位移仅用于确认
执行器方向和数值稳定，不能与实车动作时间比较。

进入 `validated` 前至少需要厂家 CAD/称重数据、铰点与油缸行程、轮胎试验、液压原理图和
实车阶跃/斜坡响应数据。之后应将参数移入版本化 YAML，通过单位测试、静力平衡、动作时间和
实测轨迹逐项标定。

## 与正式控制链的关系

- `loader_command_controller`/`ros2_control` 正式链路已经接入并通过端到端测试，详见
  [ros2_control.md](ros2_control.md)。本插件直接订阅 `/loader/command` 的路径仅保留作独立
  开发/A-B 对照，默认关闭；测试和算法集成应优先使用 `enable_ros2_control:=true`；
- 尚未加入轮胎滑移、滚阻、阀死区、流量/负载耦合、端止挡和倾翻保护；
- `VehicleState` 的铲斗载荷目前为零，待实时土料模型接入；
- 尚未执行参数化的 100 次作业稳定性和实车误差验收。
