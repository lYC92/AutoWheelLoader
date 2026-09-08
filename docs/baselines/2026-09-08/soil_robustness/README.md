# 土料可靠性验证 — 2026-09-08

- `ctest.log`：原旋转扫掠测试和新增可靠性测试均通过。Release 构建，断言使用显式异常，在 Release 下有效。
- `kernel_stress.json`：1000 次同轨重铲，子单元数稳定为 524；1000 次不重置地形的随机挖卸中 995 次有实际转运，峰值 6496 个子单元，体积余额 7.11e-15 m³。约 10.06 s 是本机运行时间，不是仿真 RTF。
- `soil_guards.json/.txt`：Gazebo 整车、ROS 控制器和三维土料实际耦合；测试容量 0.2 m³。满斗后继续前进约 0.0684 m，地形与载荷不变。域外后轴 Y=25.781 m（地形边界 Y=24 m），斗倾角 -1.2 rad；尝试卸料保持 0.2 m³。回到域内后全部卸出，相对守恒误差 9.26e-16。

## 实现说明

凸分区合并只接受等高度、凸并集、面积保持的两片，不对不同高度求平均；每次压力循环独立核对分区面积及积分体积与网格的关系。相同路径重走还需保持已铲体积不变。
只在新增分片后尝试合并，并在裁剪前排除不相交的包围盒，避免长期重复计算。

## 可复现入口

```bash
ctest --test-dir ~/loader_sim_runtime/build/loader_soil --output-on-failure
LOADER_SOIL_GUARDS=true bash scripts/wsl/smoke_test_loader_soil_coupling.sh physics
```

这不是 1000 次整车循环验收。默认 3 m³ 容量满斗、整车反复挖取、100 次连续作业及性能统计仍需后续验收。

`ab_cycle.json/.txt`：优化后正常 A→B 循环通过，装卸 0.707106 m³，B 区比例 100%，成功返回；默认容量仍为 3 m³。
