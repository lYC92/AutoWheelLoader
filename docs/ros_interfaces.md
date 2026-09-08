# ROS 2 公共接口约定

`loader_sim_msgs` 是规划、VCU 适配、控制器、车辆动力学和散料插件之间的稳定边界。
所有连续物理量使用 SI 单位；归一化命令的范围在消息注释中固定。

## 消息

- `VehicleCommand`：档位、牵引扭矩、制动、目标铰接角、两路液压阀和急停。
- `VehicleState`：车速、四轮轮速、关节状态、两路油缸位置/压力、斗内载荷和故障状态。
- `BucketInteraction`：土壤作用在铲斗上的六维力、侵入/切削状态和物料流量。
- `TerrainState`：版本化的高度场原点/分辨率/宽度、剖面或 XY 网格、物料体积账本、质量/体积守恒
  误差和更新序号。`remaining_volume_m3` 包含已卸回地形的物料，`dumped_volume_m3` 是累计
  转移量而不是第二个独立储仓，因此守恒式为 `remaining + bucket - initial = 0`。

## 约束

- `header.stamp` 必须来自 `/clock`，不得填 Windows 墙钟时间。
- `header.frame_id` 必须明确给出坐标系；当前 `BucketInteraction.bucket_wrench` 使用 `world`。
- `brake_command` 限定为 `[0, 1]`，液压阀命令限定为 `[-1, 1]`。
- 无效档位、NaN、无穷值、超范围命令和超时命令由 `loader_command_controller` 拒绝或进入安全状态。
- 具体车辆允许的牵引扭矩、铰接角和油缸行程不写死在消息中，由已标定的车型配置提供。


## 三维地形与作业控制

`TerrainState.schema_version=1` 使用 `height_profile_m` 沿 y 拉伸；版本 2 使用
`height_grid_m`，行优先索引为 `row * columns + column`，格角原点为
`(domain_min_m, origin_y_m)`，两个方向均使用 `cell_size_m`，高度相对世界 z=0。
版本 2 的旧 `height_profile_m` 留空，不能再把它当作三维地形。
完整三维网格目前以 10 Hz 发布，`BucketInteraction` 保持 50 Hz；几何只更新变化单元。
消息定义变更后需重建工作空间并重启订阅进程，避免旧类型缓存。

`/loader/planned_path` 是后轴路线（`nav_msgs/Path`，world）；
`/loader/task_state` 是 A/B 当前作业阶段与定位来源（`std_msgs/String` 内含 JSON）。
当前 `ab` 演示明确使用 `/loader/ground_truth/odometry` 调试；估计定位入口不会在反馈失效后切回真值。
A/B 的装料阈值、卸料区域和中间姿态在 `simulation/config/ab_task.yaml`。


### 2026-09-08 三维扩展

- `BucketInteraction.payload_inertia`：铲斗坐标系下的载料质量、质心及惯性矩；实际载料刚体以 20 Hz 离散更新。`dynamic_payload_body_active` 表示当前正在使用动态载料刚体。
- `VehicleState.bucket_payload_center_of_mass_m` 从土料模块接收，不再重复计算或固定为常量。旧的重力载荷模式仍报告实际施力点。
- `/loader/localization/odometry`：融合估计，`world -> base_link`，同时发布对应 TF。`/loader/localization/health` 给出时间、定位有效性、接受/拒绝计数和重定位次数。
- `/loader/navigation/obstacles`：JSON `{frame: "world", stamp: 仿真秒, polygons: [[[x,y],...],...]}`；只接受有序凸多边形、当前时间和明确世界坐标。过期或非法观察导致停止。静态障碍来自任务 YAML，与 Gazebo 障碍共用配置。
- 四相机原始话题 `/loader/cameras/{front,left,rear,right}/raw/{image,depth_image,camera_info}` 及 `semantic/labels_map`。后者使用 Gazebo panoptic 编码：第 2 字节为类别，第 1/0 字节为实例高/低位；实例编号只在当前帧有效。统一重投影后输出等距鱼眼 RGB、径向深度、类别、实例和可见框。
