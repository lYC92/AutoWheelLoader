# 第 1 阶段性能基线

采集日期：2026-09-04

测试环境：Windows 10 22H2 + WSL2 Ubuntu 24.04.1、ROS 2 Jazzy、Gazebo Sim 8.11.0、
Mesa 25.2.8、NVIDIA GeForce RTX 2070 8GB。OpenGL 通过 WSLg D3D12 后端运行。

场景为 `simulation/smoke/loader_smoke.sdf`：DART 物理引擎、2 ms 主步长（500 Hz）、
一个静态箱体和一路 320×240、10 Hz OGRE2 相机。实时系数采用 12 秒窗口的平均值判定，
最低值包含启动瞬间，不单独用于 go/no-go。

- `phase1_gazebo_headless_baseline.csv`：无 GUI 标准运行方式。
- `phase1_gazebo_gui_baseline.csv`：WSLg 可视调试方式。

这两项只证明基础链路满足第 1 阶段门槛。加入完整装载机碰撞体、激光雷达、多鱼眼相机和
散料模型后必须按同一方法重新建立负载基线。
