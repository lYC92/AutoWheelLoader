# 六项交付续做记录

2026-09-08 用户主动要求暂停，以节省 token。不要自动启动测试；等用户恢复工作。总计划为 `docs/next_stage_3d_ab.md`。

## 已验证成果

1. 三维斗刃精确扫掠，包括横滚两端高差 0.162 m；内核及整车守恒通过。
2. 实际动态载料刚体的质量、质心、惯量更新；500 kg 动力学探针和 A/B 装卸通过。
3. 常规障碍绕行完整循环通过；动态障碍停车通过。原窄通道最后入位仍失败，独立保存在 `simulation/config/ab_tight_obstacle_task.yaml`。
4. 地图点到平面匹配 + 15 状态激光/IMU 融合驱动完整 A/B 循环通过；复用地图、运动中中断 2 秒并停车恢复的完整循环通过（RMSE 6.61 cm）。没有控制真值回退。
5. 移动作业 115 组四相机各 12 路同步数据通过；RGB/深度/语义/实例/标定、鱼眼重投影、BEV、原始及导出 MCAP 回读通过。115 组是每 2 秒采样，不能称连续 10 Hz。
6. 同一世界四轮真实转运（两个方向各两轮）通过，无重置/补料/瞬移。新 Foxglove 第七页、ROS 网格高度和 WebSocket 实际传输检查通过。

## 下次按顺序处理

- 修复 `run_ab_cycle.py` 收到 SIGINT 后 rclpy context 关闭、finally 再 spin 导致报告未完整落盘；本次 250 Hz 中断标记单独保存，不能冒充测试失败/通过。
- 验证刚改的起铲稳定条件（0.008 rad、1 秒，最长 30 秒）和载荷接近目标减速（0.18–0.35 m/s）；位于 `run_transfer_scenario.prepare_pose` 与 `run_ab_cycle.execute_cycle`，尚无新整车结果。
- 验证 250 Hz 物理备用配置完整循环和精度。启动环境 `LOADER_PHYSICS_HZ=250`；默认仍为 500。每次生成独立 controllers.yaml/runtime_profile.json。压力环用解析指数更新，土料按扫掠几何；备用配置文件原订 1 ms 内部子步尚未落实/核对，不应宣称完全满足该配置契约。
- 查明 250 Hz 时融合输出约 50 Hz 的调度问题（IMU发布节拍/`fuse_localization.py` 的 0.01 秒阈值），500 Hz 时约 100 Hz。
- 完整执行 10 次固定种子统计。清理后两轮虽完整成功，质量 CV 2.04%、力 CV 6.46%、航向离散 0.738°、最小 RTF 0.7024 未达目标；最后的低速/姿态改动正是针对重复性，未验证。不可放宽原门槛冒充通过。
- 完整执行 100 次同一世界连续 A/B 转运；先用最新代码跑 4–8 轮检查。`run_continuous_cycles.py` 每两个循环换向，起铲 lift/tilt=-0.18，B 接近后轴 (4,9,pi/2)。长时间回收矮土堆/局部耗尽仍有风险。
- 相机最新改动待验证：NPZ 从压缩改为无压缩，避免阻塞图像回调；图像关联 0–0.15 秒内先前地形；新增实际帧率/最大帧间隔/连续 10 Hz 指标；MCAP 新增 world→base 和 base→bucket。普通 CaptureOnly 20 组先测；实验性 `--lockstep` 以前失败，别默认启用。
- 坡面定位整车、不同随机种子、最新综合回归、Foxglove 登录后目视体验仍待做。同步更新小白文档及计划勾选。

## 环境与操作

Windows 工作区 `C:\Users\Liyangchuan\Documents\ChatGPT\New project`；WSL `Ubuntu-24.04`；运行文件 `/home/lyc/loader_sim_runtime`。ROS Jazzy / Gazebo Harmonic；RTX 2070 经 WSL D3D12 正常加速。没有提交 Git，大量先前修改必须保留。无需新子代理。

不要编辑正在执行的 Bash 脚本，先复制到运行目录并通过 `LOADER_PROJECT_ROOT="$PWD"` 指明项目根目录。也不要在统计批次中途改控制代码，否则结果不再是同一版本。

```bash
cp scripts/wsl/run_loader_soil_demo.sh /home/lyc/loader_sim_runtime/run_frozen.sh
LOADER_PROJECT_ROOT="$PWD" LOADER_HEADLESS=1 LOADER_PHYSICS_HZ=250 bash /home/lyc/loader_sim_runtime/run_frozen.sh perception auto lio_map ab
```

固定种子统计：`python3 tools/ros/run_acceptance_batch.py --runs 10 --output /home/lyc/loader_sim_runtime/results/新目录`。可以在该输出目录创建 `STOP` 文件，使其当前一轮结束后停止，勿直接杀父进程遗留当前仿真。

连续作业：上述演示命令增加 `LOADER_CONTINUOUS_CYCLES=100 LOADER_EVALUATION_DURATION=21600`。新进程树清理 `tools/ros/stop_process_tree.py` 针对拥有的启动 PID 及其子进程；不要全局杀其他项目进程。长循环与摄像采集分开跑，避免 GPU 争用。

Foxglove 只读测试使用 `/home/lyc/loader_sim_runtime/venv/observability/bin/python tools/ros/test_foxglove_bridge.py --port 18765 --mode perception --read-only --yard`，不能在自动循环中运行会发送手动命令的默认测试。

文档重建：Windows `py -3 tools/docs/build_beginner_docs.py`。

## 最近运行与证据

工作区证据根 `docs/baselines/2026-09-08/six_features/`。

- `navigation_static/`：`physics_ab_20260908_153241_542`，0.744098 m³ 全部在 B，作业 RTF 0.9892，进程平均 0.9121，RSS 峰值约 615 MiB。
- `estimated_clean_500hz/`：`perception_ab_20260908_154400_604`，完整通过，RMSE 0.09119 m，作业 RTF 0.7024。
- 第二个干净 500 Hz 通过运行：`perception_ab_20260908_154723_2249`，原始结果在 WSL。父批次中断使其服务器短暂遗留，已明确清理。
- `acceptance_initial_failed/`：重复发布/端口冲突污染的失败批次，保留，不纳入成功统计。
- 最后用户中断：`perception_ab_20260908_155111_4057`，250 Hz，48.18 m、RMSE 0.10253 m、输出 49.87 Hz，未完成返程。中断清理报错导致 cycle JSON 可能仍显示中途 running；以 user_pause.json 为准。
- `map_reuse_dropout/`、`continuous_four/`、`surround_moving/`、`terrain_display_verification.json`、`foxglove_yard_verification.log` 为其他实测证据。

最近 C++ 四项回归均通过（不同朝向、倾斜切削、卸料支撑域对照、1000 次内核稳定性）。250 Hz 与最近 Python 改动不因这些测试通过而自动通过。
