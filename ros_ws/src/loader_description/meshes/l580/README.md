# L580 装载机外观

原始来源：[Ling-ling00/Wheel_Loader_Control](https://github.com/Ling-ling00/Wheel_Loader_Control)，
其 `meshes/` 与 `urdf/L580.xacro`。本项目之前已经接入五个经过变换的 STL，
保留在本目录根部。它们是外观参考，不能当作经过实车标定的 CAD 或动力学参数。
上游仓库未看到独立 LICENSE；本项目代码许可证不自动覆盖这些外部网格。

`visual/` 是 `tools/model/prepare_loader_meshes.py` 从现有五个 STL 生成的修复部件：

- 合并前后车身后按 x=0.68 m 几何切分并封口，让完整驾驶室留在后车架；
- 截除单轮残留的跨轴三角面，将轮胎居中到关节，外包尺寸对齐直径 1.50 m、宽 0.55 m；
- 左轮绕 x 旋转 180°，让左右轮毂朝外；轮胎和轮毂采用独立材质；
- 驾驶室面片拆为独立玻璃材质。它是有色不透明外观，不模拟真实透光；
- 动臂、铲斗保留已有网格及关节连接。

可复现生成（仓库根目录，Windows PowerShell）：

```powershell
& 'C:\Program Files\Blender Foundation\Blender 5.2\blender.exe' --background --threads 2 --python tools/model/prepare_loader_meshes.py
```

`visual/source_sha256.json` 记录转换输入哈希；原始资产未改写。
重新运行 `scripts/wsl/build_workspace.sh` 或只构建 `loader_description` 后，新目录才进入 ROS 安装路径。

URDF 默认 `mesh_visuals:=true`，可用 `mesh_visuals:=false` 回到几何体外观。
碰撞、质量、惯量、关节仍是项目原有名义模型；细节外观不能解释为更精确的碰撞或液压模型。
GPU 雷达会看到新外观，因此传感器输入确实发生变化。
雷达安装点已从后车架 z=2.25 m 移到 z=2.85 m，越过最高约 z=2.674 m 的驾驶室屋顶。
这改变了外参，旧雷达高度下的定位报告仅作为历史记录。
