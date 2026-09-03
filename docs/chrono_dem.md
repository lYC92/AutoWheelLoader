# Project Chrono DEM 部署记录

## 固定版本与构建参数

- 上游：`https://github.com/projectchrono/chrono.git`
- 提交：`583b8e6f48600699f2084154a31742261d28a7c7`
- 构建类型：`Release`
- GPU 后端：`CUDA`
- CUDA Toolkit：`13.2.86`
- GPU 架构：`75`（GeForce RTX 2070，计算能力 7.5）
- 并行度：2
- Chrono 源码：`/home/lyc/loader_sim_runtime/src/chrono`
- 构建目录：`/home/lyc/loader_sim_runtime/build/chrono-dem`
- 安装目录：`/home/lyc/loader_sim_runtime/install/chrono`
- CMake 包：`/home/lyc/loader_sim_runtime/install/chrono/lib/cmake/Chrono/ChronoConfig.cmake`

源码固定到提交哈希，避免上游 `main` 更新后出现不可复现结果。Chrono 源码位于 WSL ext4
文件系统；Windows 工作区只保存项目自身的脚本、配置和耦合代码。

## 已通过验证

1. CUDA 最小内核在 RTX 2070 上返回预期结果 `42`。
2. Chrono 官方 `demo_DEM_movingBoundary` 使用项目的小型 JSON 场景推进 0.02 秒，输出
   376 行颗粒状态。
3. Chrono core 与 DEM 共享库及 CMake 配置安装成功。
4. 项目外部 CMake 工程通过 `find_package(Chrono REQUIRED COMPONENTS DEM CONFIG)`
   找到安装包，创建 400 颗粒摩擦接触床并完成 20 个时间步；最终动能和颗粒位置为有限值。

对应脚本：

- `scripts/wsl/bootstrap_chrono_dem.sh`
- `scripts/wsl/verify_chrono_dem.sh`
- `scripts/wsl/install_chrono_dem.sh`
- `scripts/wsl/verify_chrono_install.sh`

## 数值与平台边界

- DEM 输入必须使用一致的单位制和合理的刚度、阻尼、重力及 `psi_T`/`psi_L` 缩放参数。
  仅证明程序可以启动并不足以证明数值有效。
- 当前验证规模仅为数百颗粒，只是部署冒烟测试，不代表目标料堆规模的性能。
- WSL 对统一内存和锁页内存有平台限制。进入铲斗轨迹回放前，应按 10 万、50 万、100 万
  颗粒逐级测试初始化、长时间推进、GPU 显存峰值和结果稳定性。
- Chrono DEM 用于离线高保真回放和实时高度场模型标定；不与 Gazebo 多相机或 BEV 网络
  同时占用 RTX 2070。
