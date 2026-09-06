# 第四阶段：激光里程计基线

当前接入 KISS-ICP 作为 M2 的首条端到端基线。它使用点云，不融合 IMU，也不提供闭环建图或
既有地图重定位。已有 DLIO/GLIM 选型草稿保留在 `localization_selection.md`；该草稿中的
源码构建记录不等于算法精度验收。

## 构建与启动

WSL 中运行（路径带空格，请保留引号）：

```bash
bash '/mnt/c/Users/Liyangchuan/Documents/ChatGPT/New project/scripts/wsl/bootstrap_localization.sh'
```

脚本固定官方 [PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp) 提交
`1ffa7d7512f10bfc8b1185095011fa31184019e3`，在
`~/loader_sim_runtime/localization/` 单独获取和构建源码，不修改 `~/lio_spike_ws`。
依赖沿用已安装的 ROS 2 Jazzy、Eigen、Sophus、TBB 和 robin-map；离线误差评测使用 NumPy/SciPy。
编译限制两个并行任务，算法限制两个线程。

Windows 启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\windows\run_loader_soil_demo.ps1 -Mode perception -Localization kiss_icp -Scenario localization
```

重新导入 `foxglove/loader_simulation_layout.json` 后选择“第四阶段·定位”。
原始点云在“感知与三维”，实际算法输入在“第四阶段·传感器”。
定位行驶试验结束后保存误差报告，Gazebo 保持打开；手动模式只运行估计器，不自动执行短时评测。

定位测试场景在既有感知世界外围增加 8 个固定几何标志物，用于建立可观测的首轮基线。
它不代表只有平地/动态料堆时的定位精度。行驶前用阀命令和关节反馈举升收斗，避免
传感器初始化等待期间铲刃落地，使试验依赖偶然的出生姿态。默认 `-Scenario soil`
保留铲装演示；本次 M2 验收使用独立行驶工况，不宣称动态铲装定位已经验收。

## 数据与坐标契约

| 通道 | 含义 |
| --- | --- |
| `/loader/sensors/lidar/scan/points` | Gazebo 原始 1024×32 点云 |
| `/loader/sensors/lidar/scan/points_effect` | 算法输入；10% 名义丢点、旋转畸变，保留源时间戳 |
| `/loader/localization/points` | 效应流经扫描时刻车体过滤和地面分割后的 XYZ 点云，KISS-ICP 的直接输入 |
| `/loader/localization/odometry` | `odom → base_link` 激光里程计 |
| `/loader/ground_truth/odometry` | Gazebo `world → base_link` 三维真值，仅评测器消费 |
| `/tf`、`/tf_static` | 里程计、关节和传感器外参；真值不写入该坐标树 |

原始扫描没有点级时间字段，因此配置显式关闭 KISS-ICP deskew。
名义 3 m 最小量程保留为近场裁剪，另用正式模型每个可见部件的网格包围盒过滤自身回波。
扫描先等待同时间戳的 TF，最多等待 0.5 秒墙钟、队列最多 5 帧；没有对应 TF 的帧会丢弃并记录警告，
不会用最新关节角代替过去的扫描姿态。启动关节广播之前可能出现少量丢帧。
包围盒向外扩展 0.10 m，能覆盖动臂和铲斗运动，但也可能滤掉紧贴车体的外部物体点。

地面识别在每帧有效点中用固定随机种子的 RANSAC 与平面最小二乘细化，最多采样 768 点、64 个假设。
只接受向上法向、倾角不超过 30°、传感器正下方高度 0.8–6 m 且有足够二维支撑的平面。
去掉平面上方 0.18 m 以内及其下方点，保留突出的静态标志物；没有可靠平面时保留非车体点并警告。
这些假设仍需坡道和铲装场景验证，当前不能声称对任意地形都适用。
过滤器只读取传感器点云、模型与关节 TF，不订阅 Gazebo 车辆真值。

旧的 `ground_mode: fixed` 保留用于对照，`min_z: -2.7` 只在该模式生效。
当前默认 `ground_mode: adaptive`、`self_filter: true`。L580 外观接入后雷达安装点从
`rear_frame` 的 z=2.25 m 改到 z=2.85 m，避免被驾驶室包住；车身稳定时雷达离地约 3.75 m。
新旧报告同时存在模型、外参和前处理差异，不能把误差变化全部归因于某一个算法改动。
当前模型的外参来自 URDF；安装误差尚未拆分成真实外参与估计外参。

## 无界面验收

```bash
bash '/mnt/c/Users/Liyangchuan/Documents/ChatGPT/New project/scripts/wsl/smoke_test_localization.sh'
```

一次只运行一个 Gazebo 测试。脚本使用独立 ROS domain/Gazebo partition、降低 CPU 优先级，
执行举升收斗、倒车、制动、前进和停车，并记录算法输出及独立真值。
需要留出 `127.0.0.1:18765` 给本次仿真的 Foxglove Bridge。

输出位于 `~/loader_sim_runtime/results/localization/`：

- `poses.json`：带源时间戳的原始估计与真值，失败时也保留，支持复查；
- `trajectory.csv`：真值插值后的位置、姿态误差和仿真时间延迟；
- `metrics.json`：匹配帧数、运动距离、位置/角度 RMSE、最大误差、频率、实际前处理配置与模型哈希；
- `status.json`：记录中、完成或评测失败；之前一轮的结果移动到 `previous/`，避免误读旧的通过报告。

每次启动的 URDF 和定位世界保存到 `results/runs/<模式_场景_时间_进程号>/`，防止另一次启动覆盖模型文件。
Windows 启动器在检测到活动仿真时拒绝自动重启 WSLg，避免中断正在进行的无界面评测。

评测只用第一对位姿做坐标对齐，不拟合整条轨迹、不消除后续漂移。
真值使用线性位置插值和姿态 SLERP；不外推，跨越大于 100 ms 真值缺口的样本被排除。
静止、少于 50 个匹配位姿、短于 5 秒或真实行程少于 0.5 m 均不能通过链路验收。
`nominal_accuracy_pass` 单独记录位置 RMSE 是否不超过 0.15 m；链路跑通不代表精度达标。

## 历史基线：几何体外观与固定高度裁剪（2026-09-06）

旧版本在本机无界面行驶场景中的结果：

| 指标 | 结果 |
| --- | --- |
| 匹配位姿 / 仿真时长 / 真值行程 | 185 / 18.7 s / 16.697 m |
| 位置 RMSE / 最大误差 | 0.536 m / 1.189 m |
| 姿态 RMSE / 最大误差 | 2.056° / 4.053° |
| 里程计频率 / 仿真时间延迟 P95 | 9.840 Hz / 0.054 s |
| 0.15 m 名义位置目标 | **未通过** |

[完整指标](baselines/2026-09-06/localization_metrics.json)、
[逐帧轨迹](baselines/2026-09-06/localization_trajectory.csv) 和
[裁剪前对照](baselines/2026-09-06/localization_before_ground_crop.json) 已存入项目。
裁剪前位置 RMSE 为 6.239 m；改进证实这条前处理有作用，但尚有漂移，不能称为定位精度达标。
这些是单次名义行驶基线，未覆盖动态铲装、坡地或重复运行的统计分布。

## 后续工作

1. 地面分割和扫描时刻铰接车体过滤已接入；继续验证坡道、转向和动态料堆，并比较同一份录制数据上的不同配置。
2. 接入 DLIO 与 IMU 融合，在相同数据和真值指标下与 KISS-ICP 对比，补动态料堆工况。
3. 增加地图与重定位，再进入自主作业规划闭环。
4. 单独推进鱼眼 RGB/深度/语义预研和多相机同步采集，之后接 BEV。

这些任务和第三阶段三维散料整车迁移均未因接入里程计而自动完成。
