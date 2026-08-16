r"""Isaac Sim physical closed-loop reproduction of the multi-mode navigation stack.

Same 10-node ROS 2 pipeline as ``smoke_stack.launch.py``, but the two nodes that
stand in for hardware are removed because the Isaac Sim process (process A,
``isaac/isaac_lunar_loop.py``) now supplies the real equivalents over DDS:

    pose_simulator          -> Isaac publishes ``/pose_with_covariance``
                               (real physics pose) + ``/simulation/rollover``
    terrain_image_publisher -> Isaac publishes ``/terrain/image``
                               (real rendered ZED camera RGB)

The remaining nodes are unchanged:

    image_to_map -> map_server -> global_planner -> global_path_optimizer
        \-> hazard_mapper (mode2/mode3) -> local_planner -> path_follower
    vlm_mode (or manual_mode) -> /navigation_mode -> path_follower

Because the Leo wheel joint velocity limit (6.0 rad/s) caps physical speed at
~0.375 m/s, the paper's mode speeds (2.0/0.8/0.5) are rescaled here via launch
args so the three modes still differ but all stay under the cap.

Usage:
    ros2 launch mars_navigation_ros2 isaac_stack.launch.py                 # VLM mode
    ros2 launch mars_navigation_ros2 isaac_stack.launch.py use_vlm:=false fixed_mode:=1
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = Path(__file__).resolve().parents[3]
HEIGHTMAP = REPO_ROOT / "docs/picture/terrain/lunar_terrain_hm.png"


def generate_launch_description():
    use_vlm = LaunchConfiguration("use_vlm")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    fixed_mode = LaunchConfiguration("fixed_mode")
    heightmap = LaunchConfiguration("heightmap")

    return LaunchDescription([
        DeclareLaunchArgument("use_vlm", default_value="true"),
        DeclareLaunchArgument("start_x", default_value="-20.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("goal_x", default_value="20.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("fixed_mode", default_value="0"),
        DeclareLaunchArgument("heightmap", default_value=str(HEIGHTMAP)),
        # Physical-speed rescale: the wheel joint velocity limit (6.0 rad/s)
        # caps the rover at ~0.375 m/s, so the paper's 2.0/0.8/0.5 are scaled
        # down here (ratio preserved roughly 2.7:1.7:1).
        DeclareLaunchArgument("mode1_speed", default_value="0.32"),
        DeclareLaunchArgument("mode2_speed", default_value="0.20"),
        DeclareLaunchArgument("mode3_speed", default_value="0.15"),
        # Turn-in-place is ~20% efficient under lunar gravity; keep the follower's
        # commanded yaw rate high so rotate-in-place stays brisk.
        DeclareLaunchArgument("max_yaw_rate", default_value="2.5"),

        Node(
            package="mars_navigation_ros2",
            executable="image_to_map",
            name="image_to_map",
            parameters=[{
                "image_path": heightmap,
                # Physical closed loop: raise the *base* global-accessibility slope
                # limit from the paper's 2 deg to ~60 deg.  At 2 deg the crater rim
                # (59 deg) is marked impassable in the base map, so the global A*
                # routes around it in EVERY mode and Mode 1/2 never roll over.  At
                # 60 deg the rim/boulders stay "accessible" in the base map (Mode 1
                # drives straight in and physically backflips), while Mode 3's
                # slope cost (slope_critical_deg=30) still merges the rim as an
                # obstacle to route around.
                "max_slope_angle": 60.0,
            }],
        ),
        Node(
            package="mars_navigation_ros2",
            executable="hazard_mapper",
            name="hazard_mapper",
            parameters=[{
                "image_path": heightmap,
                # Mode 2 rock detection must ignore the smooth crater rim (a 0.8 m
                # wide, 2.2 m tall Gaussian annulus -> residual ~0.24 m) but still
                # flag the forced boulders (residual ~0.53 m).  The paper's default
                # 0.1 m threshold marks 910 of 1564 rim cells as "rocks", so Mode 2
                # would route around the rim exactly like Mode 3 and never roll over.
                "rock_height_threshold": 0.25,
                # The residual detector flags only each boulder's Gaussian peak
                # (~2 cells); inflate to the ~1.3 m physical footprint so Mode 2
                # routes around the whole rock (otherwise it clips the flank and
                # high-centres).  The rim is NOT a rock (residual 0.24 < 0.25), so
                # this dilation does not re-flag the crater rim.
                "rock_dilation_radius_m": 0.8,
            }],
        ),
        Node(
            package="mars_navigation_ros2",
            executable="map_server",
            name="map_server",
        ),
        Node(
            package="mars_navigation_ros2",
            executable="global_planner",
            name="global_planner",
            parameters=[{
                "use_manual_goal": True,
                "start_x": start_x,
                "start_y": start_y,
                "goal_x": goal_x,
                "goal_y": goal_y,
            }],
        ),
        Node(
            package="mars_navigation_ros2",
            executable="global_path_optimizer",
            name="global_path_optimizer",
        ),
        Node(
            package="mars_navigation_ros2",
            executable="local_planner",
            name="local_planner",
        ),
        Node(
            package="mars_navigation_ros2",
            executable="path_follower",
            name="path_follower",
            parameters=[{
                "mode1_speed": LaunchConfiguration("mode1_speed"),
                "mode2_speed": LaunchConfiguration("mode2_speed"),
                "mode3_speed": LaunchConfiguration("mode3_speed"),
                "max_yaw_rate": LaunchConfiguration("max_yaw_rate"),
                # Physical closed-loop tuning: the kinematic port's pure-pursuit
                # heading term (heading_kp=1.2, lookahead=1.0 m) over-corrects on
                # the real skid-steer rover, making it weave and bleed forward
                # speed (skid-steer turn is ~20% efficient).  A longer lookahead
                # and a gentler heading gain keep it driving straight on flat
                # ground so Mode 3 actually reaches the goal instead of crawling.
                "lookahead_distance": 2.0,
                "heading_kp": 0.4,
            }],
        ),
        Node(
            package="mars_navigation_ros2",
            executable="vlm_mode",
            name="vlm_mode",
            condition=IfCondition(use_vlm),
        ),
        Node(
            package="mars_navigation_ros2",
            executable="manual_mode",
            name="manual_mode",
            condition=UnlessCondition(use_vlm),
            parameters=[{"fixed_mode": fixed_mode}],
        ),
    ])
