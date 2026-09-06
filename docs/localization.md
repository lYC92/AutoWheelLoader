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
| `/loader/localization/points` | 效应流经平地裁剪后的 XYZ 点云，KISS-ICP 的直接输入 |
| `/loader/localization/odometry` | `odom → base_link` 激光里程计 |
| `/loader/ground_truth/odometry` | Gazebo `world → base_link` 三维真值，仅评测器消费 |
| `/tf`、`/tf_static` | 里程计、关节和传感器外参；真值不写入该坐标树 |

原始扫描没有点级时间字段，因此配置显式关闭 KISS-ICP deskew。
名义 3 m 最小量程用于减少近场车体回波，不等同于完整车体滤除。
首轮平地试验中，大量地面回波重复出现相同的射线采样图案，点到点配准几乎报告静止，
行驶约 17 m 的位置 RMSE 达 6.24 m。增加近场滤除到 8 m 没有改善，配置已恢复 3 m。
当前在效应流下游单独裁剪 `lidar z < -2.7 m` 的地面回波并压紧为有限 XYZ 点；保留源时间戳
和坐标系，不读取车辆真值。阈值来自本场景约 3.15 m 的雷达离地高度，是可配置的名义
平地前处理；坡道、颠簸和姿态变化场景需要替换成地面分割，不能照搬本次结果。
当前模型的外参来自 URDF；安装误差尚未拆分成真实外参与估计外参。

## 无界面验收

```bash
bash '/mnt/c/Users/Liyangchuan/Documents/ChatGPT/New project/scripts/wsl/smoke_test_localization.sh'
```

一次只运行一个 Gazebo 测试。脚本使用独立 ROS domain/Gazebo partition、降低 CPU 优先级，
执行举升收斗、倒车、制动、前进和停车，并记录算法输出及独立真值。
需要留出 `127.0.0.1:8765` 给本次仿真的 Foxglove Bridge。

输出位于 `~/loader_sim_runtime/results/localization/`：

- `poses.json`：带源时间戳的原始估计与真值，失败时也保留，支持复查；
- `trajectory.csv`：真值插值后的位置、姿态误差和仿真时间延迟；
- `metrics.json`：匹配帧数、运动距离、位置/角度 RMSE、最大误差和频率。

评测只用第一对位姿做坐标对齐，不拟合整条轨迹、不消除后续漂移。
真值使用线性位置插值和姿态 SLERP；不外推，跨越大于 100 ms 真值缺口的样本被排除。
静止、少于 50 个匹配位姿、短于 5 秒或真实行程少于 0.5 m 均不能通过链路验收。
`nominal_accuracy_pass` 单独记录位置 RMSE 是否不超过 0.15 m；链路跑通不代表精度达标。

## 首轮实测（2026-09-06）

当前代码在本机无界面行驶场景中的结果：

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

1. 分析剩余姿态/位置漂移，替换固定高度裁剪为地面分割和铰接车体滤除；再做重复运行。
2. 接入 DLIO 与 IMU 融合，在相同数据和真值指标下与 KISS-ICP 对比，补动态料堆工况。
3. 增加地图与重定位，再进入自主作业规划闭环。
4. 单独推进鱼眼 RGB/深度/语义预研和多相机同步采集，之后接 BEV。

这些任务和第三阶段三维散料整车迁移均未因接入里程计而自动完成。
