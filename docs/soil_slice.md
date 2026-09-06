# 单铲斗 + 二维干砂切片原型

这是三维实时土料模型之前的强制门槛，目标是把几何更新、受力方向、材料转移、作用/反作用
记账和碰撞分层先做成可重复测试。它不是经过实物标定的铲掘力模型。

2026-09-07 修复了慢启动后铲斗落地、轮端有扭矩但车不动的问题：自动流程先恢复铲装姿态，
再保持工作装置进行铲取和转运。基础/带雷达两种模式的延迟启动回归均通过，
见 [故障证据与结果](baselines/2026-09-07/README.md)。

## 分层与碰撞

- 刚性地面使用位 `0x01`；
- 松散物料代理使用位 `0x02`；
- 铲斗使用位 `0x01`。

因此铲斗与刚性地面有原生接触，而与松散物料代理没有原生接触。松散物料的受力只由解析
模型计算，避免与 Gazebo 接触力重复。自动测试让一个 0.5 m 立方铲斗从 z=4 m 自由下落，
穿过高度 2.5 m 的代理体，最终在刚性地面 z≈0.245 m 处静止，证明当前 DART/Gazebo
组合实际执行了掩码，而不是只检查 XML。

## 几何和守恒

- 2D 高度列分辨率 5 cm，代表 2.7 m 斗宽；
- 切削轨迹内部细分不超过 1 cm；
- 每个格子的下降体积直接加入斗内有效载荷；
- 斗容达到 3.0 m³ 后停止继续转移；
- 卸料使用不超过 34° 休止角的离散三角包络；
- 几何转移与力学公式分离，未来替换力学模型不会破坏质量守恒。

每一步检查：

`V_terrain + V_payload - V_initial = 0`

当前 12.968813 m³ 初始料堆完成一次 3.0 m³ 铲取和卸料后，体积守恒误差为
`0.000e+00 m³`。

## 名义切削阻力

当前使用可审计的 Rankine 被动土楔近似：

`Kp = (1 + sin(phi)) / (1 - sin(phi))`

`F / width = 0.5 * gamma * depth² * Kp + 2 * cohesion * depth * sqrt(Kp)`

再乘以名义速度修正项。方向始终与水平切削速度相反，竖直分量由斗刃角与土—斗摩擦角的
差确定；土体反力在同一步记为铲斗力的严格相反数。当前干砂名义参数下峰值合力为
176.97 kN，作用/反作用记账残差为 0 N。

这只是结构正确的初值。Rankine 二维近似没有描述三维侧向流动、斗内堆积、齿尖离散作用、
应变软化、密实度历史和非均匀含水率；峰值力不得用于实车选型或性能承诺。下一步应使用
Chrono DEM 同轨迹回放和实测数据拟合阻力模型。

## 运行

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_soil_slice.sh
```

证据位于 `/home/lyc/loader_sim_runtime/results/soil_slice/`：

- `soil_slice_smoke.txt`：验收摘要；
- `interaction_trace.csv`：每个不大于 1 cm 子步的侵入、力、反力、转移量和守恒误差；
- `terrain_profile.csv`：初始、铲取后、卸料后三条高度剖面；
- `collision_mask_bucket_pose.txt`：Gazebo 碰撞掩码实测最终位姿。

材料和网格配置在
`simulation/config/materials/dry_sand_nominal.yaml`，所有未实测字段均标为 `nominal`。

## 整车闭环耦合

同类二维高度列和 Rankine 名义阻力已经接入 `loader_soil` Gazebo 模型插件。插件在每个
物理步中跟踪斗刃扫掠，把解析力施加在斗刃点，把斗内物料重力施加在名义载荷质心。翻斗
负向阀命令和负向关节角同时达到阈值后，斗内体积以不超过 3 m³/s 的名义流量回到地形，
并按 34° 休止角形成离散堆体。

插件以 50 Hz 发布 `BucketInteraction` 和 `TerrainState`；后者包含完整 280 格高度剖面和
网格元数据。`loader_command_controller` 订阅斗内载荷，将质量和质心并入统一
`VehicleState`。场景中的 280 个 5 cm 料柱以 10 Hz 跟随高度场，料柱只有可视几何、不参与
刚体接触，避免车轮或铲斗收到重复力。

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_soil_coupling.sh
```

2026-09-04 本机结果：完成“低速铲取 → 举升 → 倒车转运 → 制动 → 翻斗卸料”；车轮速度
1.288 rad/s、车辆速度 0.966 m/s、最大侵入 0.672 m、峰值名义阻力 33.83 kN、铲取阶段
斗内 0.559095 m³ / 894.55 kg，累计卸料 0.678007 m³，地形最大变化 0.427 m，完整过程
体积账本误差为 0。累计卸料量大于铲取阶段末值，是因为随后的举升扫掠又收入了少量物料。

测试同时检查 `VehicleState`/`BucketInteraction` 载荷一致性、高度剖面积分、异地卸料后的
地形变化，以及一个变化最大料柱与 `TerrainState` 的逐格位置；本次抽检料柱 106 的中心
高度期望值和 Gazebo 实际值均为 -0.657084 m，误差 0。证据位于：

- `/home/lyc/loader_sim_runtime/results/loader_soil_coupling.txt`
- `/home/lyc/loader_sim_runtime/results/loader_soil_coupling_pose.txt`
- `/home/lyc/loader_sim_runtime/results/loader_soil_proxy_expectation.txt`
- `/home/lyc/loader_sim_runtime/results/loader_soil_proxy_column_poses.txt`
- `/home/lyc/loader_sim_runtime/log/loader_soil_coupling_gazebo.log`

GPU 雷达动态几何验收：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/smoke_test_loader_soil_perception.sh
```

该场景从料堆侧面用固定 512×32 GPU 雷达采集铲取前后点云。2026-09-04 本机结果：共
2,948 条配对射线的量程变化超过 5 cm，其中 265 条落在变化最大料柱附近；该料柱的
`TerrainState` 期望中心高度与 Gazebo 实体高度均为 -0.654923 m。证据位于：

- `/home/lyc/loader_sim_runtime/results/loader_soil_coupling_perception.txt`
- `/home/lyc/loader_sim_runtime/results/loader_soil_proxy_perception_expectation.txt`
- `/home/lyc/loader_sim_runtime/results/loader_soil_proxy_perception_column_pose.txt`
- `/home/lyc/loader_sim_runtime/log/loader_soil_coupling_perception_gazebo.log`

完整动态工况性能验收：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/Liyangchuan/Documents/ChatGPT/New\ project/scripts/wsl/benchmark_loader_soil_profile.sh
```

该测试同时运行 500 Hz 车辆物理、正式控制链、280 格土料、10 Hz 几何更新、车载 32 线
雷达和 IMU，并执行完整铲装/转运/卸料。当前平均 RTF 0.978491、车载雷达 9.93322 Hz、
显存峰值 663 MiB，证据位于
`/home/lyc/loader_sim_runtime/results/loader_full_soil_profile_baseline.csv`。

## 进入三维高保真土料前仍需完成

- 独立三维高度场原型已经通过；仍需将其与 C++ 整车插件统一为一套共享、可回放的内核；
- 从二维斗刃点扫掠升级为真实斗齿/斗壁三维扫掠和完整六维力；
- 在当前载荷质量/质心反馈基础上，增加随装料分布变化的惯量反馈；
- 用连续三角网格替代当前可见接缝的 5 cm 料柱；
- 增加斗内溢料、三维侧向流动和非平地休止角重排；
- 对同轨迹 Chrono DEM 的力曲线、装料量和颗粒流动做标定；
- 增加多次切削、回切、溢料和非平地卸料测试。

当前料柱能够同步挖除和异地沉积，并已通过 GPU 雷达变化检测，但仍是二维剖面沿 2.7 m
斗宽外挤的阶梯几何。土体反力只进入解析守恒账本，没有施加给一个动态土体刚体；翻斗
触发角和 3 m³/s 排料率也都是名义值。现有数值只能说明软件闭环、渲染可观测性和守恒
成立，不能说明实车铲掘力或排料流量精度。
