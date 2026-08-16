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
- 大石块（`add_collidable_rocks`）：**真实碰撞体 + 逐顶点灰色噪声**，颜色参照 OmniLRS
  `GravelStones` 贴图均值（RGB≈79,78,77，玄武岩碎石），但**不加载 GravelStones.mdl**——
  该 shader 在 Isaac 5.1 编译失败（`Unable to find SdrShaderNode`）且 emissive 泄漏红色，
  改用灰色 `UsdPreviewSurface` + 逐顶点灰噪声（0.14~0.46，围绕 0.31）更稳
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

## 2026-08 视觉/材质修复（闭环月面外观回归）

针对闭环录像的五项视觉问题（岩石不真实、场景发红、引导线过粗、石头悬空、红色物体），
逐项修复并写入 `isaac/isaac_lunar_loop.py`：

1. **场景/阴影发红根因 = OmniLRS MDL emission 泄漏**。所有 OmniLRS MDL 硬编码
   `emissive_color=(1,0.1,0.1)` + `emissive_intensity=40`，但 `enable_emission: false`。
   Isaac 5.1 的 MDL runtime 忽略该开关，在**阴影面**上泄漏红色 emission（阳光面被过曝太阳
   淹没所以看似灰）。`_disable_mdl_emission`（USD inputs 覆写）不可靠——inputs 在 shader
   编译前不存在。**真正修复**：改 MDL 源文件。已应用
   `tools/fix_omnilrs_mdl_emission.sh`，把 4 个 MDL（GravelStones / rock_boulder_dry /
   LunarRegolith8k / seaside_rock_2k）的 `emissive_intensity: 40` → `0`。
   修复后 `shadow_red_frac` 从 ~100% 降到 **0.0%**，全帧 R-G ≈ +0.6~+0.9（中性）。

2. **红色石头 = GravelStones.mdl shader 编译失败**（`Unable to find SdrShaderNode`），
   损坏的 shader fallback 渲染为红色。修复：石头改用 `_make_rock_material()`（灰色
   `UsdPreviewSurface`，roughness 0.85）+ `_make_boulder_mesh()` 逐顶点灰噪声（0.14~0.46，
   均值参照 GravelStones 贴图 0.31），完全绕过 MDL。修复后石头实测 (118,115,117) 中性灰。

3. **场景偏蓝 cast = DomeLight 默认天空纹理是蓝色**。默认 `--dome-intensity=0.0`，
   仅当用户显式传入 `>0` 时才创建 DomeLight 且强制中性灰 color。

4. **曝光校准：filmIso = 100**（`/rtx/post/tonemap/filmIso`）。实测 400→terrain 过曝
   白(247)，200→238，100→阳光地形 **190~214 中性灰**。验证脚本
   `/tmp/verify_scene_colors.py` 全套通过：mean=(214,214,215)、shadow_red_frac 0.0%、
   B-(GR)=+0.0、R-G=+0.6~+0.9。

5. **路径引导线变细**：`curve.CreateWidthsAttr()` 0.10 → **0.03m**，渲染核心线 1-2px
   （带 AA 约 4-5px），不再遮挡地形。度量脚本 `/tmp/measure_path_thickness.py`。

6. **石头真实物理**：`add_collidable_rocks` 给每块石头加 `UsdPhysics.CollisionAPI` +
   `PhysxRigidBodyAPI`（mass + linearDamping），在月球重力 1.62 下自然落到地面，
   不再是悬空装饰。Mode1 高速下撞石真实翻车保持（57.4s / 10.5m，rollover 检测不变）。

以上修复验证于 mode1 重录（2026-08-16），三模式能力分化与 VLM 优势均保持。

## 当前边界

- 几何地形仍使用 `isaac/lunar_terrain_hm.npy`，保证和 ROS2 导航栈的 `docs/picture/terrain/lunar_terrain_hm.png` 像素一致。
- OmniLRS 的程序化 crater/rock 采样器还没有替换当前 heightmap 生成器；如果替换，ROS2 map、Isaac mesh、评估区域必须同时重标定。
- 大石块已实例化为独立碰撞体（`add_collidable_rocks`，程序化 icosphere 网格 + 灰色
  PreviewSurface + 真实物理），位置与 ROS2 heightmap 上的强制 boulder 对齐；坑壁仍属
  heightmap 几何。若改用 OmniLRS 真实 rock USD 资产，需重做碰撞体与路径可通行性回归。
