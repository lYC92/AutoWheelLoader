# ROS 2 公共接口约定

`loader_sim_msgs` 是规划、VCU 适配、控制器、车辆动力学和散料插件之间的稳定边界。
所有连续物理量使用 SI 单位；归一化命令的范围在消息注释中固定。

## 消息

- `VehicleCommand`：档位、牵引扭矩、制动、目标铰接角、两路液压阀和急停。
- `VehicleState`：车速、四轮轮速、关节状态、两路油缸位置/压力、斗内载荷和故障状态。
- `BucketInteraction`：土壤作用在铲斗上的六维力、侵入/切削状态和物料流量。
- `TerrainState`：高度场原点/分辨率/宽度、完整高度剖面、物料体积账本、质量/体积守恒
  误差和更新序号。`remaining_volume_m3` 包含已卸回地形的物料，`dumped_volume_m3` 是累计
  转移量而不是第二个独立储仓，因此守恒式为 `remaining + bucket - initial = 0`。

## 约束

- `header.stamp` 必须来自 `/clock`，不得填 Windows 墙钟时间。
- `header.frame_id` 必须明确给出坐标系；当前 `BucketInteraction.bucket_wrench` 使用 `world`。
- `brake_command` 限定为 `[0, 1]`，液压阀命令限定为 `[-1, 1]`。
- 无效档位、NaN、无穷值、超范围命令和超时命令由 `loader_command_controller` 拒绝或进入安全状态。
- 具体车辆允许的牵引扭矩、铰接角和油缸行程不写死在消息中，由已标定的车型配置提供。
