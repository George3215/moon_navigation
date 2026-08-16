#!/usr/bin/env python3
"""Isaac Sim 月面导航闭环 —— 阶段2 骨架（纯物理场景 + URDF + 相机，无 ROS）。

职责（最终闭环的进程 A）：
    * 用 ``isaac/lunar_terrain_hm.npy`` 建月面地形 mesh（与 ROS 栈的 PNG 逐像素一致）
    * 月球重力 1.62 m/s²、240 Hz、TGS 求解器的物理场景
    * 导入展开后的 Leo URDF，用差分控制器驱动 4 轮
    * 挂载渲染相机，跟随车体（含俯仰/横滚）

本骨架只验证物理管线：settle 让车落到地形 -> 发前进速度 -> 记录位姿/保存相机帧。
阶段 3 才接 ROS（pose/rollover/image/cmd_vel/mode）。

运行（务必脱离 conda，避免 python.sh 的 conda 警告）：
    export ISAAC_SIM_DIR=/home/lry/isaac-sim
    unset CONDA_PREFIX CONDA_DEFAULT_ENV
    PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i miniconda | grep -v -i anaconda | paste -sd:)
    $ISAAC_SIM_DIR/python.sh /home/lry/mars_navigation/isaac/isaac_lunar_loop.py \
        --settle 3 --duration 8 --forward 0.3 --out /tmp/isaac_skel
"""

import argparse
import json
import os
import sys

# Isaac Sim 自带的 ROS2 bridge 需使用其捆绑的 humble 库（Python 3.11 编译）。系统 ROS2
# Humble 是 Python 3.10，ABI 不兼容（报 _rclpy_pybind11.cpython-311... 缺失），故必须在
# import rclpy 前：1) 把捆绑 humble/lib 加入 LD_LIBRARY_PATH；2) 从 sys.path 剔除系统
# ROS site-packages 并前置捆绑 site-packages（否则 bridge 会优先 import 到 3.10 的系统
# rclpy）；3) 指定 ROS_DISTRO / RMW。
# 参考 bridge 扩展报错提示（"Attempting to load internal rclpy for ROS Distro: humble"）。
_ISAAC_SIM_DIR = os.environ.get("ISAAC_SIM_DIR", "/home/lry/isaac-sim")
_ISAAC_ROS_LIB = _ISAAC_SIM_DIR + "/exts/isaacsim.ros2.bridge/humble/lib"
_ISAAC_ROS_SITE = _ISAAC_SIM_DIR + "/exts/isaacsim.ros2.bridge/humble/rclpy"
os.environ["ROS_DISTRO"] = "humble"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
# 解除系统 ROS2 的 ament 环境，避免 bridge 走 "system rclpy" 分支把 3.10 路径加回 sys.path。
for _k in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "OLD_PYTHONPATH", "ROS_PACKAGE_PATH"):
    os.environ.pop(_k, None)

# 剔除系统 ROS2 (3.10) 的 site-packages，前置 bridge 捆绑的 humble site-packages (3.11)。
sys.path = [p for p in sys.path if "/opt/ros" not in p]
if _ISAAC_ROS_SITE not in sys.path:
    sys.path.insert(0, _ISAAC_ROS_SITE)

# LD_LIBRARY_PATH：前置捆绑 humble/lib，剔除系统 ROS / conda 库以免 ABI 冲突。
_ld = os.environ.get("LD_LIBRARY_PATH", "")
_ld_clean = ":".join(p for p in _ld.split(":") if p and "/opt/ros" not in p and "miniconda" not in p and "anaconda" not in p)
os.environ["LD_LIBRARY_PATH"] = _ISAAC_ROS_LIB + (":" + _ld_clean if _ld_clean else "")

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=True, help="无头模式（默认）")
parser.add_argument("--settle", type=float, default=3.0, help="下落稳定时间 (s)")
parser.add_argument("--duration", type=float, default=8.0, help="前进驱动时长 (s)")
parser.add_argument("--forward", type=float, default=0.3, help="前进线速度 (m/s)，仅 --no-ros 校准模式使用")
parser.add_argument("--angular", type=float, default=0.0, help="转向角速度 (rad/s, + = 左转)，仅 --no-ros 校准模式使用")
parser.add_argument("--out", default="/tmp/isaac_skeleton", help="输出前缀（日志/帧/位姿 CSV）")
parser.add_argument("--no-ros", action="store_true", help="禁用 ROS 桥，仅跑纯物理校准（默认启用 ROS 闭环）")
parser.add_argument("--ros-domain-id", type=int, default=0, help="ROS_DOMAIN_ID（需与进程 B 一致）")
parser.add_argument("--max-time", type=float, default=900.0, help="闭环最长仿真时长 (s)，超时自动退出")
parser.add_argument("--rollover-tilt-deg", type=float, default=60.0, help="判定真实翻车的车体倾角阈值 (deg)")
parser.add_argument("--record-dir", default="", help="若非空，把 ZED 第一视角 + 第三视角帧存到该目录（供 ffmpeg 合成视频/照片）")
parser.add_argument("--record-fps", type=float, default=30.0, help="录制帧率（≤ 渲染 60Hz）")
parser.add_argument("--render-width", type=int, default=1280, help="Isaac render/swapchain width")
parser.add_argument("--render-height", type=int, default=720, help="Isaac render/swapchain height")
parser.add_argument("--renderer", default="RayTracedLighting", choices=["RayTracedLighting", "PathTracing"])
parser.add_argument("--omnilrs-root", default=os.environ.get("OMNILRS_ROOT", "/home/lry/OmniLRS"))
parser.add_argument("--omnilrs-env-config", default="", help="OmniLRS environment yaml；默认 cfg/environment/lunaryard_40m.yaml")
parser.add_argument("--omnilrs-rendering-config", default="", help="OmniLRS rendering yaml；默认 cfg/rendering/ray_tracing.yaml")
parser.add_argument("--omnilrs-physics-config", default="", help="OmniLRS physics yaml；默认 cfg/physics/default_physics.yaml")
parser.add_argument("--omnilrs-terrain-material", default="LunarRegolith8k",
                    help="OmniLRS terrain material name or .mdl path")
parser.add_argument("--no-omnilrs-look", action="store_true",
                    help="禁用 OmniLRS 材质/光照/渲染参数注入，回退到旧的简单外观")
parser.add_argument("--use-omnilrs-physics", action="store_true",
                    help="同时采用 OmniLRS physics yaml 的 dt/gravity/solver/CCD 等设置；默认保留本闭环已校准物理")
parser.add_argument("--no-rocks", action="store_true", help="禁用额外程序化碰撞石头")
parser.add_argument("--rock-count", type=int, default=7, help="额外碰撞石头数量")
parser.add_argument("--dome-intensity", type=float, default=0.0,
                    help="天穹环境光强度，0 = 无（与 OmniLRS 月面一致，仅太阳直射）；>0 时用中性灰 dome 提亮阴影")
parser.add_argument("--no-video-overlay", action="store_true", help="录制帧不叠加模式/VLM/速度 HUD")
parser.add_argument("--no-path-visualization", action="store_true", help="不在 Isaac stage 中绘制导航路径线")
args = parser.parse_args()

simulation_app = SimulationApp(
    {
        "headless": True,
        "width": args.render_width,
        "height": args.render_height,
        "renderer": args.renderer,
    }
)

import math
import time

import carb
import numpy as np
import omni
import omni.kit.commands
import trimesh
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema, Vt

from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
from isaacsim.sensors.camera import Camera

import isaaclab.terrains.height_field.utils as hf_utils
from isaaclab.terrains.utils import create_prim_from_mesh

# --------------------------------------------------------------------------- #
# 配置（与 generate_lunar_terrain.py / ROS 栈保持一致）
# --------------------------------------------------------------------------- #
REPO = "/home/lry/mars_navigation"
HEIGHT_NPY = REPO + "/isaac/lunar_terrain_hm.npy"
URDF_PATH = REPO + "/isaac/leo.urdf"

REAL_M = 54.0
RES = 0.2
OX = -27.0
OY = -27.0
START = np.array([-20.0, 0.0])          # 世界帧 ≡ odom 帧，start (-20, 0)

GRAVITY = 1.62                           # 月球重力 (m/s^2)
PHYSICS_DT = 1.0 / 240.0                 # 240 Hz（物理子步）
RENDER_DT = 1.0 / 60.0                   # 60 Hz（world.step(render=True) 每次推进的仿真时间）
WHEEL_RADIUS = 0.057
# 差分控制器的 wheel_base 是「左右轮距」：rocker ±0.14167 + 轮 ±0.08214 => 2*0.2238 ≈ 0.448。
# （0.28 是前-后轴距，误当轮距会让转向响应差 ~1.6 倍。）
WHEEL_BASE = 0.4476
WHEEL_NAMES = ["wheel_FL_joint", "wheel_FR_joint", "wheel_RL_joint", "wheel_RR_joint"]

# 相机相对 base_footprint 的安装位（由 URDF: camera_joint origin (0.0971,0,-0.0427)
# 相对 base_link，base_link 相对 base_footprint (0,0,0.19783) 推得），俯仰朝下 ~12°。
CAM_LOCAL_POS = np.array([0.0971, 0.0, 0.1551])
CAM_PITCH_DOWN = math.radians(12.0)

# 地形分段走廊边界（世界 x，与 terrain_image_publisher / generate_lunar_terrain.py 一致）：
#   x < -12 平坦 | [-12,-6) 岩石 | [-6,6) 陨石坑(挑战) | x >= 6 平坦
ZONE_ROCKY_LO = -12.0
ZONE_ROCKY_HI = -6.0
ZONE_RIDGE_HI = 6.0

LOG_PATH = args.out + ".log"
POSES_CSV = args.out + "_poses.csv"
FRAME_DIR = args.out + "_frames"
os.makedirs(FRAME_DIR, exist_ok=True)


def log(msg: str) -> None:
    """print 会被 Omniverse 吞掉，故写文件 + carb 日志双保险。"""
    line = "[isaac] " + msg
    carb.log_info(line)
    with open(LOG_PATH, "a") as f:
        f.write(line + "\n")


OMNILRS_MATERIALS = {
    "Basalt": "assets/Textures/GravelStones.mdl",
    "GravelStones": "assets/Textures/GravelStones.mdl",
    "LunarRegolith8k": "assets/Textures/LunarRegolith8k.mdl",
    "Sand": "assets/Textures/Sand.mdl",
    "RockBoulderDry": "assets/Textures/rock_boulder_dry.mdl",
    "SeasideRock": "assets/Textures/seaside_rock_2k.mdl",
}


def _load_yaml(path: str) -> dict:
    """Load a small OmniLRS YAML config without pulling Hydra into this script."""
    if not path or not os.path.exists(path):
        return {}
    try:
        import yaml

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        log(f"loaded yaml: {path}")
        return data
    except Exception as e:  # noqa: BLE001 - optional integration should degrade cleanly
        log(f"failed to load yaml {path}: {e!r}")
        return {}


def _resolve_omnilrs_path(path: str) -> str:
    if not path:
        return path
    if os.path.isabs(path):
        return path
    return os.path.join(args.omnilrs_root, path)


def _disable_mdl_emission(stage, mtl_path: str) -> None:
    """Force OmniPBR MDL emission off.

    OmniLRS MDL 文件都带 emissive_color=(1,0.1,0.1) + emissive_intensity=40 的默认值
    （enable_emission=false 理论上应关闭，但 Isaac MDL 运行时对某些材质不尊重该开关，
    导致月面石头渲染成粉色）。这里在 USD 侧把 emission 相关 inputs 全部清零/关掉。
    """
    try:
        material_prim = stage.GetPrimAtPath(mtl_path)
        if not material_prim.IsValid():
            return
        for child in material_prim.GetChildren():
            if child.IsA(UsdShade.Shader):
                emit = child.GetAttribute("inputs:enable_emission")
                if emit.IsValid():
                    emit.Set(False)
                col = child.GetAttribute("inputs:emissive_color")
                if col.IsValid():
                    col.Set(Gf.Vec3f(0.0, 0.0, 0.0))
                intensity = child.GetAttribute("inputs:emissive_intensity")
                if intensity.IsValid():
                    intensity.Set(0.0)
    except Exception as e:  # noqa: BLE001
        log(f"[material] _disable_mdl_emission failed for {mtl_path}: {e!r}")


def load_omnilrs_configs() -> dict:
    """Collect the OmniLRS configs used for appearance/rendering injection."""
    if args.no_omnilrs_look:
        log("OmniLRS look disabled (--no-omnilrs-look)")
        return {"enabled": False}

    env_cfg = args.omnilrs_env_config or os.path.join(args.omnilrs_root, "cfg/environment/lunaryard_40m.yaml")
    rendering_cfg = args.omnilrs_rendering_config or os.path.join(args.omnilrs_root, "cfg/rendering/ray_tracing.yaml")
    physics_cfg = args.omnilrs_physics_config or os.path.join(args.omnilrs_root, "cfg/physics/default_physics.yaml")
    return {
        "enabled": True,
        "environment_path": env_cfg,
        "rendering_path": rendering_cfg,
        "physics_path": physics_cfg,
        "environment": _load_yaml(env_cfg),
        "rendering": _load_yaml(rendering_cfg),
        "physics": _load_yaml(physics_cfg),
    }


def apply_omnilrs_render_settings(render_cfg: dict) -> None:
    """Apply OmniLRS renderer/post settings using the local Kit settings API."""
    if not render_cfg:
        return

    settings = carb.settings.get_settings()
    renderer = render_cfg.get("renderer", {})
    mode = renderer.get("renderer", args.renderer)
    settings.set("/rtx/rendermode", mode)
    # Keep ACES enabled; the Isaac rendering skill treats this as the most useful
    # quality control for headless captures.
    settings.set("/rtx/post/tonemap/enabled", True)
    settings.set("/rtx/post/tonemap/op", 4)
    settings.set("/rtx/post/tonemap/filmIso", 100.0)
    settings.set("/rtx/post/tonemap/whitepoint", 6500.0)
    settings.set("/rtx/post/aa/op", 3)

    rtx_map = {
        "samples_per_pixel_per_frame": "/rtx/pathtracing/spp",
        "max_bounces": "/rtx/pathtracing/maxBounces",
        "max_specular_transmission_bounces": "/rtx/pathtracing/maxSpecularAndTransmissionBounces",
        "max_volume_bounces": "/rtx/pathtracing/maxVolumeBounces",
        "subdiv_refinement_level": "/rtx/hydra/subdivision/refinementLevel",
    }
    for key, setting_path in rtx_map.items():
        if key in renderer:
            settings.set(setting_path, renderer[key])

    flares = render_cfg.get("lens_flares", {})
    if flares:
        settings.set("/rtx/post/lensFlares/enabled", bool(flares.get("enable", False)))
        settings.set("/rtx/post/lensFlares/flareScale", float(flares.get("scale", 0.5)))
        settings.set("/rtx/post/lensFlares/blades", int(flares.get("blades", 9)))
        settings.set("/rtx/post/lensFlares/apertureRotation", float(flares.get("aperture_rotation", 0.0)))
        settings.set("/rtx/post/lensFlares/sensorDiagonal", float(flares.get("sensor_diagonal", 28.0)))
        settings.set("/rtx/post/lensFlares/sensorAspectRatio", float(flares.get("sensor_aspect_ratio", 1.5)))
        settings.set("/rtx/post/lensFlares/fNumber", float(flares.get("fstop", 2.8)))
        settings.set("/rtx/post/lensFlares/focalLength", float(flares.get("focal_length", 12.0)))

    blur = render_cfg.get("motion_blur", {})
    if blur:
        settings.set("/rtx/post/motionblur/enabled", bool(blur.get("enable", False)))
        settings.set("/rtx/post/motionblur/maxBlurDiameterFraction", float(blur.get("max_blur_diameter_fraction", 0.02)))
        settings.set("/rtx/post/motionblur/exposureFraction", float(blur.get("exposure_fraction", 1.0)))
        settings.set("/rtx/post/motionblur/numSamples", int(blur.get("num_samples", 8)))

    chrom = render_cfg.get("chromatic_aberration", render_cfg.get("chromatic_aberrations", {}))
    if chrom:
        strength = chrom.get("strength", [0.0, 0.0, 0.0])
        model = chrom.get("model", ["Radial", "Radial", "Radial"])
        settings.set("/rtx/post/chromaticAberration/enabled", bool(chrom.get("enable", False)))
        settings.set("/rtx/post/chromaticAberration/strengthR", float(strength[0]))
        settings.set("/rtx/post/chromaticAberration/strengthG", float(strength[1]))
        settings.set("/rtx/post/chromaticAberration/strengthB", float(strength[2]))
        settings.set("/rtx/post/chromaticAberration/modelR", model[0])
        settings.set("/rtx/post/chromaticAberration/modelG", model[1])
        settings.set("/rtx/post/chromaticAberration/modelB", model[2])

    log(f"OmniLRS rendering applied: renderer={mode}, source={render_cfg}")


def _sun_quat_to_rotate_xyz(elevation: float, azimuth: float) -> Gf.Vec3f:
    # Same convention as OmniLRS LunaryardController: xyz Euler [0, elevation, azimuth-90].
    return Gf.Vec3f(0.0, float(elevation), float(azimuth) - 90.0)


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """标量在前 (w,x,y,z) 的四元数乘法 q1 ⊗ q2。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用标量在前四元数 q 旋转向量 v。"""
    qv = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return quat_mult(quat_mult(q, qv), qc)[1:]


# --------------------------------------------------------------------------- #
# 地形
# --------------------------------------------------------------------------- #
def build_terrain() -> None:
    """H[row=y,col=x](m) -> trimesh -> /World/Terrain，平移 (-27,-27,0)。

    convert_height_field_to_mesh 是 row->x, col->y，故喂 H.T（见方案「坐标系硬约定」）。
    """
    h = np.load(HEIGHT_NPY)
    log(f"heightmap H.shape={h.shape}, z range {h.min():.2f}..{h.max():.2f} m")
    vertices, triangles = hf_utils.convert_height_field_to_mesh(
        h.T, horizontal_scale=RES, vertical_scale=1.0
    )
    log(f"mesh vertices={vertices.shape}, triangles={triangles.shape}")
    mesh = trimesh.Trimesh(vertices=vertices, faces=triangles)
    create_prim_from_mesh("/World/Terrain", mesh, translation=(OX, OY, 0.0))
    log("terrain prim created at /World/Terrain")


def terrain_z(x: float, y: float) -> float:
    """采样地形高度（世界 x/y -> 米）。"""
    h = np.load(HEIGHT_NPY)
    col = int(round((x - OX) / RES))
    row = int(round((y - OY) / RES))
    return float(h[row, col])


# --------------------------------------------------------------------------- #
# 物理
# --------------------------------------------------------------------------- #
def setup_physics(physics_cfg: dict | None = None, physics_dt: float = PHYSICS_DT) -> None:
    """月球重力 + 240Hz + TGS，关闭 CCD/稳定化（保证真实翻车不被物理器抑制）。

    在 World 创建之后、首次 reset 之前直接写 USD 属性覆盖默认值。
    """
    cfg = (physics_cfg or {}).get("physics_scene", {})
    gravity = cfg.get("gravity", [0.0, 0.0, -GRAVITY])
    gravity_mag = abs(float(gravity[2])) if len(gravity) >= 3 else GRAVITY
    solver_type = cfg.get("solver_type", "TGS")
    enable_ccd = bool(cfg.get("enable_ccd", False))
    enable_stabilization = bool(cfg.get("enable_stabilization", False))
    use_gpu_pipeline = bool(cfg.get("use_gpu_pipeline", False))

    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(gravity_mag)
    px = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    px.CreateTimeStepsPerSecondAttr().Set(int(round(1.0 / physics_dt)))
    px.CreateSolverTypeAttr().Set(solver_type)
    px.CreateEnableCCDAttr().Set(enable_ccd)
    px.CreateEnableStabilizationAttr().Set(enable_stabilization)
    px.CreateEnableGPUDynamicsAttr().Set(use_gpu_pipeline)
    log(f"physics: gravity={gravity_mag}, dt={physics_dt}, solver={solver_type}, "
        f"ccd={enable_ccd}, stabilization={enable_stabilization}, gpu_dynamics={use_gpu_pipeline}")


def add_lighting(env_cfg: dict | None = None) -> None:
    stage = omni.usd.get_context().get_stage()
    sun_cfg = (env_cfg or {}).get("sun_settings", {})
    intensity = float(sun_cfg.get("intensity", 3000.0))
    angle = float(sun_cfg.get("angle", 0.53))
    diffuse = float(sun_cfg.get("diffuse_multiplier", 1.0))
    specular = float(sun_cfg.get("specular_multiplier", 1.0))
    color = sun_cfg.get("color", [1.0, 1.0, 1.0])
    temperature = float(sun_cfg.get("temperature", 6500.0))
    azimuth = float(sun_cfg.get("azimuth", 180.0))
    elevation = float(sun_cfg.get("elevation", 45.0))

    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Sun"))
    light.CreateIntensityAttr(intensity)
    light.CreateAngleAttr(angle)
    light.CreateDiffuseAttr(diffuse)
    light.CreateSpecularAttr(specular)
    light.CreateColorAttr(Gf.Vec3f(float(color[0]), float(color[1]), float(color[2])))
    try:
        light.CreateColorTemperatureAttr(temperature)
        light.CreateEnableColorTemperatureAttr(True)
    except Exception:
        pass
    light.AddRotateXYZOp().Set(_sun_quat_to_rotate_xyz(elevation, azimuth))
    # 天穹环境光：OmniLRS 月面仅太阳直射（真空中无大气散射），因此默认不开 dome
    # （默认 DomeLight 天空纹理是蓝色，会让场景整体偏蓝）。仅当 dome-intensity>0 时
    # 创建 dome，并强制中性灰颜色避免任何偏色。
    if args.dome_intensity > 0:
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Sky"))
        dome.CreateIntensityAttr(args.dome_intensity)
        dome.CreateColorAttr(Gf.Vec3f(0.5, 0.5, 0.5))
    log(f"lighting: OmniLRS sun intensity={intensity}, angle={angle}, azimuth={azimuth}, elevation={elevation}; dome={args.dome_intensity}")


def create_omnilrs_terrain_material(stage) -> str | None:
    """Create and bind an OmniLRS terrain material with a UsdPreview fallback."""
    material_key = args.omnilrs_terrain_material
    material_rel = OMNILRS_MATERIALS.get(material_key, material_key)
    material_path = _resolve_omnilrs_path(material_rel)
    if not os.path.exists(material_path):
        log(f"OmniLRS material missing: {material_path}; keeping default terrain appearance")
        return None

    looks_path = "/World/Looks"
    stage.DefinePrim(looks_path, "Scope")
    material_name = os.path.splitext(os.path.basename(material_path))[0]
    mtl_path = f"{looks_path}/{material_name}"
    mdl_ok = False
    try:
        omni.kit.commands.execute(
            "CreateMdlMaterialPrimCommand",
            mtl_url=material_path,
            mtl_name=material_name,
            mtl_path=mtl_path,
        )
        mdl_ok = True
        _disable_mdl_emission(stage, mtl_path)
        log(f"created OmniLRS MDL material: {mtl_path} <- {material_path}")
    except Exception as e:  # noqa: BLE001
        log(f"CreateMdlMaterialPrimCommand failed for {material_path}: {e!r}; creating PreviewSurface only")

    if not mdl_ok:
        # 仅当 MDL 加载失败时才用 UsdPreviewSurface 兜底；不要覆盖成功加载的 MDL surface output。
        material = UsdShade.Material.Define(stage, mtl_path)
        preview = UsdShade.Shader.Define(stage, f"{mtl_path}/PreviewSurface")
        preview.CreateIdAttr("UsdPreviewSurface")
        preview.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.42, 0.40, 0.36))
        preview.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
        material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), "surface")
    else:
        material = UsdShade.Material.Get(stage, mtl_path)

    terrain = stage.GetPrimAtPath("/World/Terrain")
    if terrain.IsValid():
        UsdShade.MaterialBindingAPI.Apply(terrain).Bind(material, UsdShade.Tokens.strongerThanDescendants)
        log(f"bound OmniLRS terrain material {mtl_path} to /World/Terrain")
    return mtl_path


def create_omnilrs_preview_material(stage, material_key: str, preview_color, roughness: float = 0.9) -> str:
    """Create an MDL material when available plus a PreviewSurface fallback."""
    material_rel = OMNILRS_MATERIALS.get(material_key, material_key)
    material_path = _resolve_omnilrs_path(material_rel)
    looks_path = "/World/Looks"
    stage.DefinePrim(looks_path, "Scope")
    material_name = os.path.splitext(os.path.basename(material_path))[0] if material_path else material_key
    mtl_path = f"{looks_path}/{material_name}"

    mdl_ok = False
    if material_path and os.path.exists(material_path):
        try:
            omni.kit.commands.execute(
                "CreateMdlMaterialPrimCommand",
                mtl_url=material_path,
                mtl_name=material_name,
                mtl_path=mtl_path,
            )
            mdl_ok = True
            _disable_mdl_emission(stage, mtl_path)
            log(f"created OmniLRS MDL material: {mtl_path} <- {material_path}")
        except Exception as e:  # noqa: BLE001
            log(f"CreateMdlMaterialPrimCommand failed for {material_path}: {e!r}; using PreviewSurface fallback")

    if not mdl_ok:
        # 仅当 MDL 加载失败时才用 UsdPreviewSurface 兜底；不要覆盖成功加载的 MDL surface output。
        material = UsdShade.Material.Define(stage, mtl_path)
        preview = UsdShade.Shader.Define(stage, f"{mtl_path}/PreviewSurface")
        preview.CreateIdAttr("UsdPreviewSurface")
        preview.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(float(preview_color[0]), float(preview_color[1]), float(preview_color[2]))
        )
        preview.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), "surface")
    else:
        material = UsdShade.Material.Get(stage, mtl_path)
    return mtl_path


def _make_rock_material(stage) -> UsdShade.Material:
    """Gray UsdPreviewSurface for procedural rocks (no MDL).

    The OmniLRS GravelStones.mdl shader fails to compile in Isaac 5.1
    (``Unable to find SdrShaderNode``) AND its emissive_color leaks red on
    shadowed faces, so the rocks render red. A plain gray PreviewSurface (white
    diffuse so each rock's per-vertex displayColor gray noise shows through)
    gives the same neutral-gray basalt look with zero shader-pool / emission
    risk.
    """
    mtl_path = "/World/Looks/RockGravel"
    if stage.GetPrimAtPath(mtl_path).IsValid():
        return UsdShade.Material(stage.GetPrimAtPath(mtl_path))
    material = UsdShade.Material.Define(stage, mtl_path)
    preview = UsdShade.Shader.Define(stage, f"{mtl_path}/PreviewSurface")
    preview.CreateIdAttr("UsdPreviewSurface")
    preview.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    preview.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), "surface")
    return material


def _make_boulder_mesh(seed: int, radius_xy: float, radius_z: float) -> trimesh.Trimesh:
    """Low-poly irregular boulder, base roughly at z=0.

    Rocks use the OmniLRS GravelStones *texture* color (neutral gray, mean RGB
    ≈79,78,77 → 0.31) as a per-vertex gray noise so each face reads like basalt
    gravel — WITHOUT the GravelStones.mdl shader (whose emissive leaks red AND
    whose SdrShaderNode fails to compile in Isaac 5.1, leaving the rock red).
    """
    rng = np.random.default_rng(seed)
    mesh = trimesh.creation.icosphere(subdivisions=2, radius=1.0)
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    noise = rng.normal(1.0, 0.10, size=(verts.shape[0], 1))
    verts = verts * noise
    verts[:, 0] *= radius_xy * rng.uniform(0.85, 1.20)
    verts[:, 1] *= radius_xy * rng.uniform(0.80, 1.15)
    verts[:, 2] *= radius_z * rng.uniform(0.75, 1.25)
    verts[:, 2] -= verts[:, 2].min()
    mesh.vertices = verts
    # Per-vertex gray noise (0.18..0.44, centered on GravelStones' 0.31): looks
    # like basalt gravel patches. Rendered via primvars:displayColor, then the
    # rock material's white diffuse multiplies onto it — no texture/UV needed.
    nv = len(verts)
    grey = 0.31 * (1.0 + 0.4 * rng.normal(0.0, 1.0, size=nv))
    grey = np.clip(grey, 0.14, 0.46)
    rgba = np.stack([grey, grey, grey, np.full(nv, 255.0)], axis=1).astype(np.uint8)
    mesh.visual.vertex_colors = rgba
    return mesh


def add_collidable_rocks(stage) -> list[str]:
    """Add visible static boulders with collision to the lunar scene."""
    if args.no_rocks or args.rock_count <= 0:
        log("extra collidable rocks disabled")
        return []

    # 石头用 GravelStones 的中性灰（RGB≈79,78,77，玄武岩碎石）但不加载其 MDL：
    # 该 MDL 的 shader 在 Isaac 5.1 编译失败（SdrShaderNode 找不到）且 emissive
    # 泄漏红色，直接用灰色 PreviewSurface + 逐顶点灰噪声更稳（见 _make_rock_material）。
    material = _make_rock_material(stage)

    # First two sit on the existing boulder field corridor; the rest add visible
    # obstacles around the route without changing the start/goal convention.
    specs = [
        (-9.0, 0.0, 0.55, 0.85),
        (-7.5, 0.0, 0.50, 0.75),
        (-10.2, 2.6, 0.42, 0.55),
        (-8.8, -2.8, 0.38, 0.55),
        (-2.7, 5.8, 0.52, 0.80),
        (3.8, -5.3, 0.48, 0.70),
        (8.0, 2.7, 0.36, 0.48),
        (12.5, -3.0, 0.40, 0.55),
        (15.5, 3.5, 0.36, 0.45),
    ][: max(0, args.rock_count)]

    paths = []
    stage.DefinePrim("/World/Rocks", "Xform")
    for idx, (x, y, rxy, rz) in enumerate(specs):
        mesh = _make_boulder_mesh(1000 + idx, rxy, rz)
        # 略高于地形（mesh 底部 z=0），交给 RigidBody 重力落到地表贴合，避免静态悬空。
        z = terrain_z(x, y) + 0.05
        path = f"/World/Rocks/Boulder_{idx:02d}"
        create_prim_from_mesh(path, mesh, translation=(x, y, z))
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.CollisionAPI.Apply(prim)
        try:
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr().Set("convexHull")
        except Exception as e:  # noqa: BLE001
            log(f"[rocks] MeshCollisionAPI unavailable for {path}: {e!r}")
        # 重力刚体：让石头真实落到地表（受月球重力），而非静态贴图悬空。
        UsdPhysics.RigidBodyAPI.Apply(prim)
        try:
            # 质量用 UsdPhysics.MassAPI（PhysxRigidBodyAPI 没有 CreateMassAttr）。
            # 30 kg 足够重，车撞不飞，保持原翻车物理。
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(30.0)
        except Exception as e:  # noqa: BLE001
            log(f"[rocks] MassAPI unavailable for {path}: {e!r}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)
        paths.append(path)

    log(f"added {len(paths)} collidable procedural rocks: {paths}")
    return paths


def _path_msg_to_points(msg) -> list[tuple[float, float]]:
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def update_path_curve(stage, points: list[tuple[float, float]], mode: str) -> None:
    """Visualize the currently followed path as a USD line strip."""
    if args.no_path_visualization or not points:
        return
    path = "/World/NavigationGuide/ActivePath"
    stage.DefinePrim("/World/NavigationGuide", "Xform")
    curve = UsdGeom.BasisCurves.Define(stage, path)
    pts = []
    for x, y in points[:400]:
        try:
            z = terrain_z(x, y) + 0.18
        except Exception:
            z = 0.25
        pts.append(Gf.Vec3f(float(x), float(y), float(z)))
    if len(pts) < 2:
        return
    color = Gf.Vec3f(0.1, 0.8, 1.0) if mode == "Mode 1" else Gf.Vec3f(1.0, 0.85, 0.05)
    if mode == "Mode 3":
        color = Gf.Vec3f(1.0, 0.25, 0.1)
    curve.CreateTypeAttr().Set("linear")
    curve.CreateCurveVertexCountsAttr().Set([len(pts)])
    curve.CreatePointsAttr().Set(Vt.Vec3fArray(pts))
    curve.CreateWidthsAttr().Set(Vt.FloatArray([0.03] * len(pts)))  # 细引导线，避免遮挡地形
    curve.CreateDisplayColorAttr().Set(Vt.Vec3fArray([color]))


# --------------------------------------------------------------------------- #
# Leo URDF 导入
# --------------------------------------------------------------------------- #
def import_leo() -> str:
    """导入 URDF，返回 articulation root prim path。"""
    ok, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = True      # 合并 fixed 关节 -> 只剩 4 个轮 dof
    import_config.fix_base = False               # 关键：不焊死，让车自由运动/翻车
    import_config.import_inertia_tensor = True
    import_config.make_default_prim = False
    import_config.create_physics_scene = False   # 用 setup_physics 自己的场景
    import_config.convex_decomp = False
    import_config.collision_from_visuals = False
    # execute() 返回 (status, result) 二元组，result 才是 articulation root prim path。
    status, root_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=URDF_PATH,
        import_config=import_config,
        get_articulation_root=True,
    )
    log(f"Leo imported, articulation root = {root_path}")
    return root_path


def configure_wheel_drives(wheel_names, stiffness=0.0, damping=1.0e6) -> int:
    """把 4 个轮关节的 DriveAPI 切到「速度驱动」模式。

    URDF 导入默认给关节一个 stiffness=1e3 的角驱动（等效位置驱动），此时
    ``set_dof_velocity_targets`` 会被当位置目标忽略，轮子不转。速度驱动要求
    stiffness=0 且 damping>0（见 import_carter.py 的 set_drive_parameters）。
    """
    stage = omni.usd.get_context().get_stage()
    configured = 0
    for prim in stage.Traverse():
        if prim.GetName() not in wheel_names:
            continue
        if not prim.HasAPI(UsdPhysics.DriveAPI):
            log(f"  [drive] {prim.GetName()} has NO DriveAPI -> skip")
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        drive.GetStiffnessAttr().Set(stiffness)
        drive.GetDampingAttr().Set(damping)
        if drive.GetTargetVelocityAttr() is None:
            drive.CreateTargetVelocityAttr(0.0)
        configured += 1
        log(f"  [drive] {prim.GetPrimPath()} -> velocity drive (stiffness=0, damping={damping})")
    return configured


def set_friction(terrain_path="/World/Terrain", mu_static=2.0, mu_dynamic=1.6,
                 terrain_material_path: str | None = None) -> int:
    """给地形与轮子的碰撞体绑高摩擦物理材质，保证低重力(1.62)下 skid-steer 转弯抓地。

    月面月壤内摩擦角高（μ~0.8~1.0），默认 PhysX μ=0.5 偏低，导致差速转弯时
    轮子侧向打滑、车体只横移不转（实测转弯率只有指令的 ~10%）。提高 μ 后转弯恢复。

    关键（真实翻车 vs 打滑卡死的分界）：Leo 俯仰侧翻角 ≈ atan(半轴距/质心高)
    = atan(0.1526/0.104) ≈ 56°，而车轮爬坡不打滑的条件是 tan(坡角) < μ_static。
    μ_static=1.5 时打滑角 ≈ 56.3°，与侧翻角几乎重合 → 车在 57° 坡上刚过侧翻角
    就失去抓地，只会打滑卡死、翻不过去。把 μ_static 提到 2.0（打滑角 63.4°）后，
    车能爬过 56° 侧翻角（59° 坡仍 < 63.4° 有抓地）→ 真实后空翻，而不是卡死。

    PhysX 读取摩擦的正确姿势：新建 ``UsdShade.Material``，在**材质 prim** 上 Apply
    ``UsdPhysics.MaterialAPI``（StaticFriction/DynamicFriction）+ ``PhysxSchema.PhysxMaterialAPI``，
    再用 ``UsdShade.MaterialBindingAPI.Apply(collider).Bind(material)`` 绑到碰撞体。
    直接往碰撞体 prim Apply MaterialAPI 不一定被 PhysX 采纳（实测无效果）。
    """
    stage = omni.usd.get_context().get_stage()
    mat_path = "/World/PhysicsMaterial/LunarRegolith"
    wheel_material = UsdShade.Material.Define(stage, mat_path)  # UsdShade.Material
    wheel_material_prim = wheel_material.GetPrim()  # Usd.Prim
    material_api = UsdPhysics.MaterialAPI.Apply(wheel_material_prim)
    material_api.CreateStaticFrictionAttr(mu_static)
    material_api.CreateDynamicFrictionAttr(mu_dynamic)
    material_api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(wheel_material_prim)

    terrain_material = None
    if terrain_material_path:
        terrain_material_prim = stage.GetPrimAtPath(terrain_material_path)
        if terrain_material_prim.IsValid():
            terrain_material = UsdShade.Material(terrain_material_prim)
            terrain_api = UsdPhysics.MaterialAPI.Apply(terrain_material_prim)
            terrain_api.CreateStaticFrictionAttr(mu_static)
            terrain_api.CreateDynamicFrictionAttr(mu_dynamic)
            terrain_api.CreateRestitutionAttr(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(terrain_material_prim)

    n = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        p = str(prim.GetPrimPath())
        is_wheel = any(w in p for w in ("wheel_", "Wheel"))
        is_terrain = terrain_path in p
        if not (is_wheel or is_terrain):
            continue
        material = terrain_material if is_terrain and terrain_material is not None else wheel_material
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        n += 1
    log(f"[friction] bound μ_static={mu_static}, μ_dynamic={mu_dynamic} to {n} colliders "
        f"(wheel_material={mat_path}, terrain_material={terrain_material_path or mat_path})")
    return n


# --------------------------------------------------------------------------- #
# 相机
# --------------------------------------------------------------------------- #
def make_camera() -> Camera:
    cam = Camera(
        prim_path="/World/RoverCam",
        name="rover_cam",
        resolution=(640, 480),
        frequency=30,
    )
    cam.initialize()
    # 近裁面拉近，避免近处地面被裁黑（默认近裁 ~1m 会把眼前地面裁掉）。
    cam.set_clipping_range(0.01, 1000.0)
    # 宽视场 ~90° 水平（贴近 ZED2/真实 rover 相机）：FOV_h=90° => f=aperture/(2*tan45°)=aperture/2。
    # 默认 50mm 等效焦距对应 ~20° 视场，太窄，VLM 无法识别地形。
    cam.set_focal_length(2.0955 / 2.0)
    log(f"camera initialized: clipping={cam.get_clipping_range()}, "
        f"focal_length={cam.get_focal_length():.3f} (fov_h≈90°)")
    return cam


def update_camera(cam: Camera, pos: np.ndarray, quat: np.ndarray) -> None:
    """把相机钉在车体前方：位置 = 车体原点 + 车体旋转后的安装偏移，朝向 = 车体朝向 ⊗ 俯仰。"""
    pitch = np.array(
        [math.cos(CAM_PITCH_DOWN / 2), 0.0, math.sin(CAM_PITCH_DOWN / 2), 0.0]
    )
    cam_quat = quat_mult(quat, pitch)
    cam_pos = pos + quat_rotate(quat, CAM_LOCAL_POS)
    cam.set_world_pose(position=cam_pos, orientation=cam_quat, camera_axes="world")


# 第三视角（追拍）相机：车后方 4m、上方 3m，俯视 ~28°，同时拍到车身与前方地形。
ORBIT_LOCAL_POS = np.array([-4.0, 0.0, 3.0])
ORBIT_PITCH_DOWN = math.radians(28.0)


def make_orbit_camera() -> Camera:
    cam = Camera(prim_path="/World/OrbitCam", name="orbit_cam", resolution=(1280, 720), frequency=30)
    cam.initialize()
    cam.set_clipping_range(0.05, 1000.0)
    # ~60° 水平视场（第三视角更自然，不像 ZED 那样广角）
    cam.set_focal_length(2.0955 / (2.0 * math.tan(math.radians(30.0))))
    return cam


def update_orbit_camera(cam: Camera, pos: np.ndarray, quat: np.ndarray) -> None:
    """追拍相机：钉在车体后上方，朝向 = 车体朝向 ⊗ 俯视，车体落在画面下缘、前方地形展开。"""
    pitch = np.array(
        [math.cos(ORBIT_PITCH_DOWN / 2), 0.0, math.sin(ORBIT_PITCH_DOWN / 2), 0.0]
    )
    cam_quat = quat_mult(quat, pitch)
    cam_pos = pos + quat_rotate(quat, ORBIT_LOCAL_POS)
    cam.set_world_pose(position=cam_pos, orientation=cam_quat, camera_axes="world")


def _wrap_text(text: str, max_chars: int = 64, max_lines: int = 4) -> list[str]:
    words = str(text).replace("\n", " ").split()
    lines = []
    current = ""
    for word in words:
        trial = word if not current else current + " " + word
        if len(trial) <= max_chars:
            current = trial
        else:
            if current:
                lines.append(current)
            current = word[:max_chars]
        if len(lines) >= max_lines:
            break
    if current and len(lines) < max_lines:
        lines.append(current)
    return lines


def _draw_hud(bgr, view_name: str, sim_t: float) -> None:
    if args.no_video_overlay:
        return
    import cv2

    h, w = bgr.shape[:2]
    pad = 12
    panel_w = min(w - 2 * pad, 720)
    panel_h = 172 if bridge_state.get("qa") else 112
    overlay = bgr.copy()
    cv2.rectangle(overlay, (pad, pad), (pad + panel_w, pad + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.52, bgr, 0.48, 0, bgr)

    mode = bridge_state.get("mode", "unknown")
    actual_speed = float(bridge_state.get("actual_speed", 0.0))
    v_cmd = float(bridge_state.get("v", 0.0))
    w_cmd = float(bridge_state.get("w", 0.0))
    color = (255, 230, 80)
    if mode == "Mode 2":
        color = (80, 220, 255)
    elif mode == "Mode 3":
        color = (80, 120, 255)

    y = pad + 28
    cv2.putText(bgr, f"{view_name} | t={sim_t:6.1f}s", (pad + 12, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (245, 245, 245), 2, cv2.LINE_AA)
    y += 30
    cv2.putText(bgr, f"Navigation Mode: {mode}", (pad + 12, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.78, color, 2, cv2.LINE_AA)
    y += 30
    cv2.putText(bgr, f"Speed: actual {actual_speed:.3f} m/s | cmd v={v_cmd:.3f}, w={w_cmd:.3f}",
                (pad + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 2, cv2.LINE_AA)

    qa = bridge_state.get("qa") or {}
    if qa:
        y += 30
        q = qa.get("question", "VLM question")
        a = qa.get("answer", qa.get("explanation", qa.get("answer_raw", "")))
        vlm_line = (
            f"VLM: rock={qa.get('rock_distribution', '?')} "
            f"slope={qa.get('slope', '?')} -> {qa.get('mode', mode)}"
        )
        cv2.putText(bgr, vlm_line, (pad + 12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.54, (180, 255, 180), 2, cv2.LINE_AA)
        for line in _wrap_text("Q: " + q, 78, 1) + _wrap_text("A: " + a, 78, 2):
            y += 22
            cv2.putText(bgr, line, (pad + 12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (235, 235, 235), 1, cv2.LINE_AA)

    history = bridge_state.get("mode_history", [])[-3:]
    if history:
        text = "Switches: " + " | ".join(f"{t:.1f}s {m}" for t, m in history)
        cv2.putText(bgr, text[:95], (pad + 12, h - 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (230, 230, 230), 1, cv2.LINE_AA)


def save_record_frames(cam: Camera, orbit_cam, record_dir: str, frame_idx: int, sim_t: float) -> None:
    """把 ZED 第一视角 + 第三视角 RGB 各存一帧 PNG（供 ffmpeg 合成视频/照片）。"""
    import cv2

    rgb = cam.get_rgb()
    if rgb is not None:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        _draw_hud(bgr, "Front camera", sim_t)
        cv2.imwrite(os.path.join(record_dir, "cam", f"frame_{frame_idx:06d}.png"), bgr)
    if orbit_cam is not None:
        rgb = orbit_cam.get_rgb()
        if rgb is not None:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            _draw_hud(bgr, "Orbit camera", sim_t)
            cv2.imwrite(os.path.join(record_dir, "orbit", f"frame_{frame_idx:06d}.png"), bgr)


# --------------------------------------------------------------------------- #
# ROS 桥（Omniverse rclpy，单节点）
# --------------------------------------------------------------------------- #
# 回调写入的共享状态：cmd_vel -> (v, w)，navigation_mode -> 字符串。
bridge_state = {
    "v": 0.0,
    "w": 0.0,
    "mode": "unknown",
    "qa": None,
    "actual_speed": 0.0,
    "sim_t": 0.0,
    "mode_history": [],
    "global_path": [],
    "local_path": [],
    "path_dirty": False,
}


def tilt_from_quat(quat: np.ndarray) -> float:
    """车体 +z 轴与竖直方向的夹角 (deg)，quat 标量在前 (w,x,y,z)。

    旋转矩阵 R[2,2] = 1 - 2(x^2 + y^2) = cos(tilt)，故 tilt = acos(1 - 2(x^2+y^2))，
    等价于 acos(cos(roll)·cos(pitch))（见方案「真实翻车检测」）。
    """
    x, y = float(quat[1]), float(quat[2])
    c = 1.0 - 2.0 * (x * x + y * y)
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def zone_for_x(x: float) -> str:
    if x < ZONE_ROCKY_LO:
        return "flat"
    if x < ZONE_ROCKY_HI:
        return "rocky"
    if x < ZONE_RIDGE_HI:
        return "challenging"
    return "flat"


def setup_ros_bridge():
    """启用 Omniverse ROS2 bridge，创建单节点 ``isaac_lunar_loop``。

    返回 ``(node, pose_pub, img_pub, rollover_pub)``；``--no-ros`` 或扩展不可用
    时返回 ``(None, None, None, None)``，脚本退化为纯物理校准模式。

    参考 standalone_examples/api/isaacsim.ros2.bridge/clock.py：enable_extension
    必须先于 ``import rclpy``（Omniverse 编译版，非系统版）。
    """
    if args.no_ros:
        log("ROS bridge disabled (--no-ros)")
        return None, None, None, None
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)  # 需在 rclpy.init() 前设置

        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import Path
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, String

        rclpy.init()
        node = rclpy.create_node("isaac_lunar_loop")

        pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/pose_with_covariance", 10)
        img_pub = node.create_publisher(Image, "/terrain/image", 1)
        rollover_pub = node.create_publisher(Bool, "/simulation/rollover", 1)

        def _cmd_cb(msg):
            bridge_state["v"] = float(msg.linear.x)
            bridge_state["w"] = float(msg.angular.z)

        def _mode_cb(msg):
            if msg.data != bridge_state["mode"]:
                log(f"[ros] navigation_mode -> {msg.data}")
                bridge_state["mode_history"].append((float(bridge_state.get("sim_t", 0.0)), msg.data))
                bridge_state["mode_history"] = bridge_state["mode_history"][-8:]
            bridge_state["mode"] = msg.data

        def _qa_cb(msg):
            try:
                data = json.loads(msg.data)
            except Exception:
                data = {"answer_raw": msg.data}
            if "answer" not in data:
                data["answer"] = data.get("explanation") or data.get("answer_raw", "")
            bridge_state["qa"] = data
            log(f"[ros] VLM QA -> {data.get('mode', '?')}: {data.get('answer', '')}")

        def _global_path_cb(msg):
            bridge_state["global_path"] = _path_msg_to_points(msg)
            bridge_state["path_dirty"] = True

        def _local_path_cb(msg):
            bridge_state["local_path"] = _path_msg_to_points(msg)
            bridge_state["path_dirty"] = True

        node.create_subscription(Twist, "/cmd_vel", _cmd_cb, 10)
        node.create_subscription(String, "/navigation_mode", _mode_cb, 10)
        node.create_subscription(String, "/vlm/qa", _qa_cb, 10)
        node.create_subscription(Path, "/trajectory_ctrl/global_path_updated", _global_path_cb, 1)
        node.create_subscription(Path, "/local_planner/local_path", _local_path_cb, 1)
        log("ROS bridge ready: node=isaac_lunar_loop "
            "(pubs pose/img/rollover; subs cmd_vel/mode/vlm_qa/paths)")
        return node, pose_pub, img_pub, rollover_pub
    except Exception as e:  # noqa: BLE001 - 扩展缺失 / DDS 不通
        log(f"ROS bridge setup failed: {e!r}; continuing pure-physics")
        return None, None, None, None


def publish_pose(node, pose_pub, pos: np.ndarray, quat: np.ndarray) -> None:
    if node is None or pose_pub is None:
        return
    from geometry_msgs.msg import PoseWithCovarianceStamped

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "odom"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(pos[0])
    msg.pose.pose.position.y = float(pos[1])
    msg.pose.pose.position.z = float(pos[2])
    # pxr/Isaac 四元数标量在前 (w,x,y,z)，ROS 是 (x,y,z,w)，重排。
    msg.pose.pose.orientation.x = float(quat[1])
    msg.pose.pose.orientation.y = float(quat[2])
    msg.pose.pose.orientation.z = float(quat[3])
    msg.pose.pose.orientation.w = float(quat[0])
    pose_pub.publish(msg)


def publish_image(node, img_pub, rgb) -> None:
    if node is None or img_pub is None or rgb is None:
        return
    from sensor_msgs.msg import Image

    msg = Image()
    msg.header.frame_id = "camera_frame"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height, msg.width = int(rgb.shape[0]), int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(rgb).tobytes()
    img_pub.publish(msg)


def publish_rollover(node, rollover_pub) -> None:
    if node is None or rollover_pub is None:
        return
    from std_msgs.msg import Bool

    msg = Bool()
    msg.data = True
    rollover_pub.publish(msg)


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def main() -> None:
    open(LOG_PATH, "w").close()  # 每次运行重写日志（避免 append 累积）
    log("=== Isaac lunar loop skeleton starting ===")
    omnilrs = load_omnilrs_configs()
    if omnilrs.get("enabled"):
        log("OmniLRS integration enabled: "
            f"root={args.omnilrs_root}, env={omnilrs['environment_path']}, "
            f"rendering={omnilrs['rendering_path']}, physics={omnilrs['physics_path']}, "
            f"material={args.omnilrs_terrain_material}")
        apply_omnilrs_render_settings(omnilrs.get("rendering", {}))

    physics_cfg = omnilrs.get("physics", {}) if args.use_omnilrs_physics else {}
    physics_dt = float(physics_cfg.get("physics_scene", {}).get("dt", PHYSICS_DT)) if physics_cfg else PHYSICS_DT
    my_world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=RENDER_DT)

    setup_physics(physics_cfg=physics_cfg, physics_dt=physics_dt)
    add_lighting(omnilrs.get("environment", {}) if omnilrs.get("enabled") else None)
    build_terrain()
    terrain_material_path = None
    if omnilrs.get("enabled"):
        terrain_material_path = create_omnilrs_terrain_material(omni.usd.get_context().get_stage())
    add_collidable_rocks(omni.usd.get_context().get_stage())

    root_path = import_leo()
    configure_wheel_drives(WHEEL_NAMES)
    set_friction(terrain_material_path=terrain_material_path)
    robot = my_world.scene.add(Robot(prim_path=root_path, name="leo"))

    # 定位到 start，落在月面（base_footprint 略高于地面，交给 settle 下落）。
    z_spawn = terrain_z(START[0], START[1]) + 0.3
    robot.set_world_pose(position=np.array([START[0], START[1], z_spawn]),
                         orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    log(f"robot spawned at ({START[0]}, {START[1]}, {z_spawn:.3f})")

    controller = DifferentialController(
        name="diff", wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE
    )
    cam = make_camera()

    # 录制：可选第三视角追拍相机 + 帧目录（默认关闭，不影响正常闭环）。
    record_enabled = bool(args.record_dir)
    orbit_cam = None
    if record_enabled:
        orbit_cam = make_orbit_camera()
        os.makedirs(os.path.join(args.record_dir, "cam"), exist_ok=True)
        os.makedirs(os.path.join(args.record_dir, "orbit"), exist_ok=True)
        log(f"recording enabled: dir={args.record_dir}, fps={args.record_fps}")

    my_world.reset()
    log(f"world reset; settling {args.settle}s ...")

    # 4 轮 dof 映射（reset 后 articulation 才初始化，dof_names 才可用）。
    dof_names = list(robot.dof_names)
    log(f"dof_names = {dof_names}")
    idx = {}
    for i, n in enumerate(dof_names):
        for side in ("FL", "RL", "FR", "RR"):
            if side in n:
                idx[side] = i
    num_dof = len(dof_names)
    log(f"wheel dof mapping: {idx}, num_dof={num_dof}")

    steps_settle = int(round(args.settle / RENDER_DT))
    max_steps = int(round(args.max_time / RENDER_DT))
    pose_interval = max(1, int(round(0.05 / RENDER_DT)))  # 20 Hz
    img_interval = max(1, int(round(1.0 / RENDER_DT)))    # 1 Hz 兜底心跳
    rec_interval = max(1, int(round((1.0 / args.record_fps) / RENDER_DT)))  # 录制帧率步长

    def apply_speed(v: float, w: float) -> None:
        wL, wR = controller.forward(np.array([v, w])).joint_velocities
        vel = np.zeros(num_dof)
        vel[idx["FL"]] = wL
        vel[idx["RL"]] = wL
        vel[idx["FR"]] = wR
        vel[idx["RR"]] = wR
        robot.apply_action(ArticulationAction(joint_velocities=vel))

    node, pose_pub, img_pub, rollover_pub = setup_ros_bridge()
    if node is not None:
        import rclpy  # setup_ros_bridge 内部已 import（缓存命中），此处绑定到 main 作用域

    def spin() -> None:
        if node is not None:
            rclpy.spin_once(node, timeout_sec=0.0)

    # 实时步进节奏：headless 下 world.step(render=True) 可能快于墙钟，逐帧 sleep 到 60Hz，
    # 保证 evaluate 的墙钟计时与物理速度一致（use_sim_time=false）。
    next_wall = time.monotonic() + RENDER_DT

    def pace() -> None:
        nonlocal next_wall
        now = time.monotonic()
        if next_wall > now:
            time.sleep(next_wall - now)
        next_wall += RENDER_DT
        if next_wall < time.monotonic():
            next_wall = time.monotonic()  # 落后太多：重置，不追帧

    # settle：零速度让车自然下落。ROS 模式静默 settle（不提前发 pose，避免 evaluate 把
    # 下落时间计入 traversal_time；pose 首次出现即握手 = settle 完成）。
    for i in range(steps_settle):
        spin()
        apply_speed(0.0, 0.0)
        my_world.step(render=True)
        pace()

    pos, quat = robot.get_world_pose()
    update_camera(cam, pos, quat)
    log(f"after settle: pos={np.round(pos, 3)}, tilt={tilt_from_quat(quat):.1f} deg")

    # 相机投影诊断：车头正前方 1m 的地面点投到像素，验证 pitch 方向 + buffer 朝向。
    try:
        p_ahead = np.array([pos[0] + 1.0, pos[1], 1.5])
        uv = cam.get_image_coords_from_world_points(np.array([p_ahead]))
        log(f"projection check: world {np.round(p_ahead, 2)} -> pixel {np.round(uv, 1)} "
            f"(标准 top-down: v>240 下半/地面, v<240 上半/天空)")
    except Exception as e:
        log(f"projection check failed: {e}")

    if node is None:
        # ---- 纯物理校准模式（--no-ros）：固定速度前进 + CSV + 帧 ----
        steps_drive = int(round(args.duration / RENDER_DT))
        log(f"pure-physics drive v={args.forward} m/s, w={args.angular} rad/s "
            f"for {args.duration}s ...")
        with open(POSES_CSV, "w") as csvf:
            csvf.write("t_s,x,y,z,qw,qx,qy,qz,jv0,jv1,jv2,jv3\n")
            for i in range(steps_drive):
                t = (i + 1) * RENDER_DT
                pos, quat = robot.get_world_pose()
                update_camera(cam, pos, quat)  # 相机跟随车体；须在 world.step 前更新
                apply_speed(args.forward, args.angular)
                my_world.step(render=True)

                if i % int(round(0.5 / RENDER_DT)) == 0:
                    jv = robot.get_joint_velocities()
                    csvf.write(f"{t:.3f},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                               f"{quat[0]:.4f},{quat[1]:.4f},{quat[2]:.4f},{quat[3]:.4f},"
                               f"{jv[0]:.3f},{jv[1]:.3f},{jv[2]:.3f},{jv[3]:.3f}\n")
                    csvf.flush()

                if i % int(round(1.0 / RENDER_DT)) == 0:
                    rgb = cam.get_rgb()
                    if rgb is not None:
                        # rgb 是 HxWx3 uint8，行0=图像顶部，直接转 BGR 存 PNG（无需翻转）。
                        import cv2

                        frame = int(round(t))
                        cv2.imwrite(f"{FRAME_DIR}/frame_{frame:03d}.png",
                                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        pos, quat = robot.get_world_pose()
        log(f"final pose: pos={np.round(pos, 3)}, tilt={tilt_from_quat(quat):.1f} deg")
        log(f"poses CSV -> {POSES_CSV}")
        log(f"frames   -> {FRAME_DIR}/")
        log("=== skeleton done ===")
    else:
        # ---- ROS 闭环模式：cmd_vel 驱动 + 发 pose/img/rollover，直到翻车/超时/被 teardown ----
        log(f"closed loop driving (max {args.max_time}s, rollover tilt>{args.rollover_tilt_deg}deg) ...")

        rolled_over = False
        current_zone = None
        last_img_step = -10 ** 9
        last_speed_pos = None
        last_speed_t = None
        last_visualized_path = None
        dbg_path = args.out + "_physical.csv"
        dbg = open(dbg_path, "w")
        dbg.write("t_s,x,y,z,tilt_deg,mode,v_cmd,w_cmd\n")

        for i in range(max_steps):
            sim_t = i * RENDER_DT
            bridge_state["sim_t"] = sim_t
            spin()  # 收最新 cmd_vel / navigation_mode
            pos, quat = robot.get_world_pose()
            if last_speed_pos is not None and last_speed_t is not None:
                dt = max(1e-6, sim_t - last_speed_t)
                bridge_state["actual_speed"] = float(np.linalg.norm(pos[:2] - last_speed_pos[:2]) / dt)
            last_speed_pos = np.array(pos, dtype=float)
            last_speed_t = sim_t

            tilt = tilt_from_quat(quat)
            update_camera(cam, pos, quat)
            if orbit_cam is not None:
                update_orbit_camera(orbit_cam, pos, quat)

            if not args.no_path_visualization and (bridge_state["path_dirty"] or i % 30 == 0):
                active_path = (
                    bridge_state["global_path"]
                    if bridge_state["mode"] == "Mode 1"
                    else bridge_state["local_path"]
                )
                if active_path and active_path != last_visualized_path:
                    update_path_curve(omni.usd.get_context().get_stage(), active_path, bridge_state["mode"])
                    last_visualized_path = list(active_path)
                bridge_state["path_dirty"] = False

            # 真实翻车：车体倾角超阈值（正常爬坡 ~20°，翻倒 ~90°，阈值 60° 无歧义）。
            if not rolled_over and tilt > args.rollover_tilt_deg:
                rolled_over = True
                log(f"ROLLOVER: tilt={tilt:.1f} deg > {args.rollover_tilt_deg} deg "
                    f"at ({pos[0]:.2f}, {pos[1]:.2f})")
                publish_rollover(node, rollover_pub)

            if rolled_over:
                apply_speed(0.0, 0.0)  # 冻结
            else:
                apply_speed(bridge_state["v"], bridge_state["w"])

            my_world.step(render=True)

            # 录制：按 record_fps 存 ZED + 第三视角帧（get_rgb 触发该相机的离屏渲染）。
            if record_enabled and i % rec_interval == 0:
                save_record_frames(cam, orbit_cam, args.record_dir, i // rec_interval, sim_t)

            if i % pose_interval == 0:
                publish_pose(node, pose_pub, pos, quat)

            # 地形图：跨段边界立即发一帧，否则 1Hz 兜底心跳（VLM 阻塞 ~1s，绝不连发）。
            zone = zone_for_x(pos[0])
            if zone != current_zone or (i - last_img_step) >= img_interval:
                rgb = cam.get_rgb()
                if rgb is not None:
                    publish_image(node, img_pub, rgb)
                    if zone != current_zone:
                        log(f"[img] zone={zone} x={pos[0]:.2f}")
                current_zone = zone
                last_img_step = i

            if i % int(round(1.0 / RENDER_DT)) == 0:
                dbg.write(f"{sim_t:.2f},{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},"
                          f"{tilt:.2f},{bridge_state['mode']},"
                          f"{bridge_state['v']:.3f},{bridge_state['w']:.3f}\n")
                dbg.flush()

            pace()

        dbg.close()
        pos, quat = robot.get_world_pose()
        log(f"final pose: pos={np.round(pos, 3)}, tilt={tilt_from_quat(quat):.1f} deg, "
            f"rolled_over={rolled_over}, mode={bridge_state['mode']}")
        log(f"physical log -> {dbg_path}")
        log("=== closed loop done ===")

    if node is not None:
        rclpy.shutdown()
    my_world.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
