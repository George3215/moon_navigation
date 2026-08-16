# Isaac Sim + OmniLRS 月面外观接入说明

本仓库已有 Isaac Sim 物理闭环入口：

- Isaac 进程：`isaac/isaac_lunar_loop.py`
- ROS2 导航栈：`ros2/mars_navigation_ros2/launch/isaac_stack.launch.py`
- 实验驱动：`ros2/mars_navigation_ros2/scripts/run_isaac_experiments.sh`

这次接入只改 Isaac 进程的 stage 外观与可选配置，不改 ROS2 topic、导航节点、heightmap 坐标系或实验指标。

## 已接入内容

默认启用 OmniLRS 外观注入，来源为 `/home/lry/OmniLRS`：

- 环境光照：默认读取 `cfg/environment/lunaryard_40m.yaml`
  - `sun_settings.intensity`
  - `sun_settings.angle`
  - `sun_settings.diffuse_multiplier`
  - `sun_settings.specular_multiplier`
  - `sun_settings.color`
  - `sun_settings.temperature`
  - `sun_settings.azimuth`
  - `sun_settings.elevation`
- 渲染参数：默认读取 `cfg/rendering/ray_tracing.yaml`
  - renderer mode
  - samples/bounces/subdivision
  - lens flares
  - motion blur
  - chromatic aberration
  - 额外固定启用 ACES tone mapping，避免 headless 渲染偏灰或过曝
- 地形材质：默认绑定 OmniLRS `assets/Textures/LunarRegolith8k.mdl`
  - 同时创建 `UsdPreviewSurface` fallback，降低 headless MDL-only 黑帧风险
  - 同一个材质 prim 会附加 PhysX/UsdPhysics 摩擦参数，避免视觉材质被物理材质覆盖
- 物理配置：默认只加载 `cfg/physics/default_physics.yaml` 用于日志和显式可选覆盖
  - 不默认采用 OmniLRS `dt=0.016666`，因为当前物理闭环的真实翻车行为依赖已校准的 240 Hz 物理步长
  - 需要时可显式加 `--use-omnilrs-physics`

## 最小运行命令

纯 Isaac 物理校准：

```bash
export ISAAC_SIM_DIR=/home/lry/isaac-sim
unset CONDA_PREFIX CONDA_DEFAULT_ENV
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -iE 'miniconda|anaconda' | paste -sd:)
$ISAAC_SIM_DIR/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py \
  --no-ros --settle 3 --duration 8 --forward 0.3 --out /tmp/isaac_omnilrs_smoke
```

完整 ROS2 + Isaac 闭环实验：

```bash
cd /home/lry/mars_navigation/ros2
bash mars_navigation_ros2/scripts/run_isaac_experiments.sh vlm
```

## 常用切换

禁用 OmniLRS 外观，回到旧的简单光照/无材质版本：

```bash
/home/lry/isaac-sim/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py --no-omnilrs-look
```

改用 PathTracing 配置：

```bash
/home/lry/isaac-sim/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py \
  --omnilrs-rendering-config /home/lry/OmniLRS/cfg/rendering/path_tracing.yaml \
  --renderer PathTracing
```

改用 OmniLRS 其他材质：

```bash
/home/lry/isaac-sim/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py \
  --omnilrs-terrain-material Basalt
```

可用别名：

- `LunarRegolith8k`
- `Basalt`
- `GravelStones`
- `Sand`
- `RockBoulderDry`
- `SeasideRock`

也可以直接传 `.mdl` 绝对路径。

显式采用 OmniLRS physics yaml：

```bash
/home/lry/isaac-sim/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py \
  --use-omnilrs-physics
```

注意：这会把 `dt` 从当前闭环默认的 1/240 改为 OmniLRS yaml 中的值，可能改变翻车、打滑和到达时间，不能和旧指标直接比较。

## 当前边界

- 几何地形仍使用 `isaac/lunar_terrain_hm.npy`，保证和 ROS2 导航栈的 `docs/picture/terrain/lunar_terrain_hm.png` 像素一致。
- OmniLRS 的程序化 crater/rock 采样器还没有替换当前 heightmap 生成器；如果替换，ROS2 map、Isaac mesh、评估区域必须同时重标定。
- OmniLRS rock USD 资产还没有实例化到闭环里；当前石头/坑壁是 heightmap 几何的一部分。直接加真实 rock USD 会改变碰撞体与路径可通行性，应单独做一轮物理和导航指标回归。
