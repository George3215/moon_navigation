# Isaac Sim 物理闭环复现总结（README）

> 面向后续 AI / 维护者的自包含总结。读完本文即可理解本项目「是什么、做到哪一步、怎么跑、有哪些坑、产物在哪」。
> 更早的纯运动学复现说明见仓库根目录的 [`../REPRODUCTION.md`](../REPRODUCTION.md)。

## 一句话

把论文 **"VLM-Empowered Multi-Mode System for Efficient and Safe Planetary Navigation"**
（arXiv:2506.16703, IROS 2025）在 **Isaac Sim 5.1.0 + ROS 2 Humble** 上做成了**物理闭环**
——真实月面重力、真实翻滚、真实渲染相机，喂给论文原样的导航栈 + 本地 Qwen3-VL。

上游论文是 ROS 1 Noetic + Gazebo；本机是 Ubuntu 22.04 + ROS 2 Humble，因此代码移植到了 `ros2/` 目录。

## 当前状态：✅ 全部完成

- 阶段 1 月面高程图 ✅
- 阶段 2 Isaac 场景骨架 + 物理校准（真实翻滚打通）✅
- 阶段 3 ROS 桥 + 完整闭环 ✅
- 阶段 4 VLM 闭环实验 + 视频/照片录制 ✅
- 阶段 5 OmniLRS/Isaac 可视化增强 ✅
  - OmniLRS 月面材质与 sun/rendering 配置接入
  - Isaac 场景内加入有碰撞体的程序化月岩
  - 当前导航引导路径在仿真中以线段显示
  - 录制视频叠加 VLM 问答、导航模式切换和小车实际速度

**最终四模式对比实验（同一套地形 + 同一套控制参数 + 岩石膨胀检测，可视化增强版录制）**：

| 模式 | 结果 | 用时 | 路程 | 平均速度 |
|---|---|---|---|---|
| **Mode 1** 高效直行（不避障） | ❌ **真实翻车**（撞石块后仰） | 53.2 s | 10.5 m | 0.197 m/s |
| **Mode 2** 避石不避坡 | ❌ **真实翻车**（爬上坑壁侧翻） | 130.1 s | 14.7 m | 0.113 m/s |
| **Mode 3** 坡度+粗糙度保守 | ✅ **到达目标**（绕坑壁） | 468.2 s | 42.3 m | 0.090 m/s |
| **VLM** 自适应 Mode1↔Mode3 | ✅ **到达目标** | 287.7 s | 43.6 m | 0.152 m/s |

**核心结论（复现论文效率主张）**：VLM 在平地用 Mode 1（0.32 m/s 快跑约 39.9 m），
只在坑壁附近短暂切到 Mode 3（约 3.5 m），最终**比保守的 Mode 3 快 ~39%**
（287.7 s vs 468.2 s），同时像 Mode 3 一样成功存活。翻车是**纯物理**判定
（Isaac 测车体倾角 `acos(cos·roll·cos·pitch) > 60°`，不是坡度查表）。

## 架构：两个进程通过 DDS 通信

```
进程 A：Isaac Sim（isaac/isaac_lunar_loop.py）
    真实物理(月球重力1.62) + 地形mesh + Leo URDF + 有碰撞体月岩 + 渲染相机 + 单节点 Omniverse rclpy ROS 桥
         ── /pose_with_covariance (20Hz) ─────────────►  进程 B
         ── /terrain/image (rgb8, 事件驱动) ──────────►  进程 B
         ── /simulation/rollover (Bool) ──────────────►  进程 B
         ◄─ /cmd_vel (Twist) ──────────────────────────  进程 B
         ◄─ /navigation_mode (String) ─────────────────  进程 B
         ◄─ /vlm/qa (String JSON) ─────────────────────  进程 B
         ◄─ /trajectory_ctrl/global_path_updated ──────  进程 B
         ◄─ /local_planner/local_path ─────────────────  进程 B

进程 B：ROS2 导航栈（ros2 launch ... isaac_stack.launch.py，8 个节点）
    感知/规划/模式切换/评估，零改动复用论文逻辑（仅参数调优）

两者同域 ROS_DOMAIN_ID=0、同为 rmw_fastrtps_cpp。时间全走墙钟（use_sim_time=false）。
Isaac 世界帧 ≡ odom 帧；地形 mesh 平移到 (-27,-27,0)。
```

导航栈 8 节点数据流（`pose_simulator` 与 `terrain_image_publisher` 已删除，改由 Isaac 提供）：

```
image_to_map  -> /mode1/occupancy_grid  -> map_server -> global_planner -> global_path_optimizer
hazard_mapper -> /mode2|mode3/occupancy_grid -> local_planner -> path_follower
vlm_mode (或 manual_mode) -> /navigation_mode -> path_follower
```

- 运动学烟测版（含 pose_simulator + terrain_image_publisher，共 10 节点）见
  `launch/smoke_stack.launch.py` + `scripts/run_experiments.sh`。
- 物理闭环版（8 节点）见 `launch/isaac_stack.launch.py` + `scripts/run_isaac_experiments.sh`。

## 目录与关键文件

```
仓库根 /home/lry/mars_navigation/
├── isaac/
│   ├── isaac_lunar_loop.py      # 进程 A：物理+相机+ROS桥（单文件，模块函数拆分）
│   ├── expand_leo_urdf.sh       # xacro 展开 Leo URDF 成单文件
│   ├── leo.urdf                 # 展开后的 Leo（无 lidar，仅 camera_optical_frame）
│   └── lunar_terrain_hm.npy     # 世界朝向高程 H[row=y,col=x]，供 Isaac 建 mesh（喂 H.T）
├── docs/
│   ├── isaacsim_omnilrs_integration.md        # OmniLRS 材质/光照/渲染/石头/HUD 接入说明
│   └── picture/isaacsim/isaac_lunar_scene_overview.png
├── ros2/
│   ├── README.md                # 本文件
│   ├── mars_navigation_ros2/
│   │   ├── launch/isaac_stack.launch.py      # 物理闭环导航栈 launch
│   │   ├── launch/smoke_stack.launch.py      # 运动学烟测 launch
│   │   ├── scripts/run_isaac_experiments.sh  # 物理闭环实验驱动（mode1..3/vlm/all）
│   │   ├── scripts/run_experiments.sh        # 运动学实验驱动
│   │   └── mars_navigation_ros2/*.py         # 导航栈节点 + 辅助模块(astar/elevation_synthesis/ros_utils)
│   ├── tools/generate_lunar_terrain.py        # 月面地形合成
│   └── runs_isaac/<mode>/                     # 各模式实验结果（视频/照片/指标）
└── docs/picture/terrain/lunar_terrain_hm.png  # 地形灰度图（导航栈加载这份）
```

## 如何运行

### 1. 构建（一次性）

```bash
cd /home/lry/mars_navigation/ros2
# 关键：必须用系统 /usr/bin/python3（ROS Humble rclpy 是 py3.10），把 conda 从 PATH 里剔除
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniconda | grep -v -i anaconda | paste -sd:)
source /opt/ros/humble/setup.bash
# NumPy 必须钉在 1.x（~/.local 的 NumPy 2.x 会破坏 apt scipy/cv_bridge 的 1.x ABI）
/usr/bin/python3 -m pip install --user "numpy==1.24.4" openai httpx
colcon build
source install/setup.bash
```

### 2. 跑一个对比实验

```bash
cd /home/lry/mars_navigation/ros2
bash mars_navigation_ros2/scripts/run_isaac_experiments.sh mode1   # mode1|mode2|mode3|vlm|all
```

脚本会：起导航栈（进程 B，先进入等待态）→ 起 Isaac（进程 A，加载地形/Leo/settle）→
轮询 `/pose_with_covariance` 作「Isaac ready」握手 → 启动 `evaluate` 采指标 →
跑完 teardown（pkill Isaac）。结果写 `runs_isaac/<mode>/`。

### 3. 手动起两个进程（调试用）

```bash
# 进程 B：导航栈（VLM 模式）
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch mars_navigation_ros2 isaac_stack.launch.py use_vlm:=true
# 进程 A：Isaac（另开终端，父 shell 已 source ROS2）
/home/lry/isaac-sim/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py
```

## 关键设计决策与参数（为什么这么做，别改回去）

- **`max_slope_angle=60.0`**（image_to_map 基底图）：论文默认 2°。2° 时 59° 的坑壁/石块
  在基底图里就被判不可通行，全局 A* 在每个模式下都绕开坑壁，Mode1/2 永远翻不了车。
  提到 60° 后基底图全可通行 → Mode1/2 直线冲进去真实翻车，只有 Mode3 的坡度代价
  （`slope_critical_deg=30`）会把坑壁合并为障碍绕开。
- **物理翻滚需要 μ_static=2.0**：Leo URDF 轮距 0.305 m、轴距 0.4476 m、质心 ≈0.104 m
  → 俯仰侧翻角 ≈56°、侧翻角 ≈65°。若 μ=1.5，打滑角 arctan(1.5)=56.3° 恰好等于侧翻角，
  车在 57° 处打滑卡死翻不了。μ=2.0（打滑角 63.4°）让车能爬过 56° 做真实后仰。
- **模式速度重缩放 0.32/0.20/0.15 m/s**：Leo 轮关节速度上限 6.0 rad/s
  （×轮半径 0.0625 → 0.375 m/s 线速上限）。论文的 2.0/0.8/0.5 全被钳到 0.375，
  故等比缩到 0.32/0.20/0.15 仍保三模式差异。
- **纯追踪控制改 `lookahead_distance=2.0`、`heading_kp=0.4`**：运动学版的
  heading_kp=1.2 + lookahead=1.0 在真实滑移转向（skid-steer）上过校正——航向 ±11° 摆动、
  w_cmd ±0.4，滑移转向吃掉前进速度（v_cmd=0.117 实得 0.04 m/s）。加长前瞻 + 减小增益
  后 Mode3 不再爬行，424 s 到目标。节点默认参数未改，只在 launch 传参。
- **岩石检测 `rock_height_threshold=0.25` + `rock_dilation_radius_m=0.8`**：
  残差检测（`height - GaussianBlur(height,σ=2px)`）只标记每块石头的窄高斯峰（~2 格），
  远小于 ~1.3 m 的物理尺寸。阈值 0.25 干净地把 7 块石头（残差 0.33~0.55）与坑壁
  （残差 0.24）分开；0.8 m 膨胀把岩石 blob 扩到物理足迹，否则 Mode2 会擦着石头侧翼高挂卡死。
- **随机石头挪到 |y|>3 m**：避免堵住 Mode2 的绕行车道（首跑被 (-9.2,2.2) 的随机石卡死）。
- **`global_planner._nearest_free_cell()`**：车驶近膨胀后的障碍时自身格被判障碍 → A* 拒绝起点，
  连续报错冻结路径。BFS 把起点/终点吸附到最近自由格（首航点仍钉在真实位姿）。
- **`global_planner._map_cb` 里 `self.last_goal=None`**：全局规划器本来只规划一次；
  清缓存让 map_server 的 1 Hz 重发强制重规划，Mode3 才能看到后续合并进来的坑壁代价并绕行。

## 坑与注意事项（务必读，都是踩出来的）

**Isaac 侧**
- `world.step(render=True)` 推进的是 RENDER_DT（1/60），不是 PHYSICS_DT（1/240）——按渲染步长计时，否则车快 4 倍。
- 轮驱动：URDF 导入的关节 `stiffness=1e3`（位置驱动），`set_dof_velocity_targets` 无效。
  要 `UsdPhysics.DriveAPI.Get(prim,"angular")`（传 `Usd.Prim`，不是 `GetPrimPath()`）→ `GetStiffnessAttr().Set(0.0)`、`GetDampingAttr().Set(1e6)`。
- `get_dof_index`/`dof_names` 只在 `world.reset()` 之后有效。
- 相机每帧 `step(render=True)` 前都要重新摆放；默认 FOV 是长焦（~20°H），要 `set_focal_length(aperture/2)` 拉成 ~90°；默认近裁剪 ~1 m 会把近处地形裁成黑，要 `set_clipping_range(0.01,1000)`。
- `Camera.get_rgb()` 是顶朝上（row 0 = top），**不需要**上下翻转。只用 DistantLight 对比过强，加 DomeLight(~300) 均匀打光。
- `print()` 在 SimulationApp 启动后被吞——写日志文件（脚本用 `log()` 写 `isaac.log`）。
- PhysX 摩擦必须经 **绑定的 UsdShade.Material**，不能直接在 collider 上套 UsdPhysics.MaterialAPI（直接套对转向无效果）。
- 滑移转向在 1/6 g 下本来就弱：原地转 ~20% 效率、边走边转 ~6%；`max_yaw_rate=2.5` 给 ~0.5 rad/s 原地转。各向异性摩擦（纵向>横向，真实履刺）在 USD PhysX 不开放。

**ROS 桥 / 环境侧**
- `ModuleNotFoundError: rclpy._rclpy_pybind11`：Isaac 自带 py3.11 rclpy，系统是 py3.10。`isaac_lunar_loop.py` 顶部 + `run_isaac_experiments.sh` 的 Isaac 子 shell 都要清掉 ament/`/opt/ros`/conda 的 PATH/LD_LIBRARY_PATH，并把 Isaac 自带 rclpy 的路径插到最前。
- `ros2 topic echo /topic --once` 在话题尚不存在时立刻失败——显式带类型再等首条消息做握手：`ros2 topic echo /pose_with_covariance geometry_msgs/msg/PoseWithCovarianceStamped --once`。
- `pgrep -f isaac_lunar_loop.py` 会匹配到你自己的 `bash -c` 命令行——用 `ps -eo pid,cmd | grep -E "[i]saac_lunar_loop"`。

**运行脚本侧**
- 正确脚本路径是 `mars_navigation_ros2/scripts/run_isaac_experiments.sh`（不是 `scripts/...`）。
- 脚本**不会**清空 `runs_isaac/<name>/`——重跑前 `rm -rf runs_isaac/<mode>`，否则残留旧 evaluate.log/metrics 会误导。
- `TIMEOUT=540.0`：Mode3 绕坑壁约 46.5 m @ 0.15 m/s ≈ 440 s，旧 300 s 会超时。

## 结果产物（视频 / 照片 / 数据）

每个模式一套，在 `runs_isaac/<mode>/` 下：

| 文件 | 含义 |
|---|---|
| `cam.mp4` | 车头相机第一视角视频 |
| `orbit.mp4` | 环绕第三视角视频 |
| `record/cam/*.png` | 车头视角逐帧照片序列 |
| `record/orbit/*.png` | 环绕视角逐帧照片序列 |
| `metrics.json` | success/reason/time/dist/speed + 各模式占比 |
| `trajectory.csv` | (t, x, y, mode) 采样轨迹 |
| `isaac_physical.csv` | 物理侧 (t, x, y, z, tilt_deg, mode, v_cmd, w_cmd) |
| `evaluate.log` / `isaac.log` / `stack.log` | 各自日志 |

当前增强版重新录制后，`cam.mp4` 与 `orbit.mp4` 的画面会包含：

- 车体、月面、陨石坑、程序化碰撞石头、阳光阴影和 OmniLRS 月壤/岩石材质。
- HUD：当前导航模式、实际车速、`cmd_vel` 线速度/角速度、最近模式切换历史。
- VLM QA：`vlm_mode.py` 发布的 `/vlm/qa` JSON，包括问题、原始回答/解释、rock/slope 分数和选择的模式。
- 仿真内导航引导线：Mode 1 显示全局平滑路径，Mode 2/3 显示局部 A* 路径。

绝对路径示例：

```
/home/lry/mars_navigation/ros2/runs_isaac/mode1/cam.mp4
/home/lry/mars_navigation/ros2/runs_isaac/mode2/orbit.mp4
/home/lry/mars_navigation/ros2/runs_isaac/vlm/metrics.json
/home/lry/mars_navigation/ros2/runs_isaac/mode3/record/orbit/   （8020 张 png）
```

## 环境事实（便于后续排查）

- Isaac Sim 5.1.0：`/home/lry/isaac-sim`，启动器 `$ISAAC_SIM_DIR/python.sh`，RTX 4090。
- 系统 ROS：`/opt/ros/humble`（py3.10）。构建/运行全程**不要**让 conda 出现在 PATH。
- VLM：本地 OmniLRS Qwen3-VL，`http://127.0.0.1:22002/v1`（openai 兼容客户端）。
- OmniLRS 资产/配置：`/home/lry/OmniLRS`，默认使用 `cfg/environment/lunaryard_40m.yaml`、`cfg/rendering/ray_tracing.yaml`、`assets/Textures/LunarRegolith8k.mdl` 和 `assets/Textures/rock_boulder_dry.mdl`。
- 地形常量：`REAL_M=54, RES=0.2, SIZE=270, OX=OY=-27, HPG=4.820803273566/255, BASE_H=1.5`。
- 坑壁：高斯环 `H_RIM=2.2, RIM_SIGMA=0.8, R=5.0` → 面坡度 ~59°。
- 强制石头：(-9,0)、(-7.5,0)，高 1.2 m、σ=0.4 m → 面坡度 ~59°（堵 Mode1 直线）。
