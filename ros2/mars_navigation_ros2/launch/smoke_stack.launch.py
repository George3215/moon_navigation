"""Headless closed-loop reproduction of the multi-mode navigation stack.

Launches the full ROS 2 Humble pipeline:

    image_to_map -> map_server -> global_planner -> global_path_optimizer
        \-> hazard_mapper (mode2/mode3) -> local_planner -> path_follower
    terrain_image_publisher -> vlm_mode -> /navigation_mode
    path_follower -> pose_simulator (kinematic rover)

No Gazebo or elevation_mapping_cupy is required: the rover pose is simulated
kinematically and the perception maps are derived from the terrain heightmap.

Usage:
    ros2 launch mars_navigation_ros2 smoke_stack.launch.py
    ros2 launch mars_navigation_ros2 smoke_stack.launch.py use_vlm:=false  # manual mode
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = Path(__file__).resolve().parents[3]
HEIGHTMAP = REPO_ROOT / "docs/picture/terrain/mixed_terrain_hm.png"
# Synthetic, unambiguous terrain images (the paper's real photos are read
# ambiguously by the local Qwen3-VL -- see ros2/tools/generate_terrain_images.py).
FLAT_IMG = REPO_ROOT / "docs/picture/perception/synthetic/flat.png"
ROCKY_IMG = REPO_ROOT / "docs/picture/perception/synthetic/rocky.png"
CHALLENGING_IMG = REPO_ROOT / "docs/picture/perception/synthetic/challenging.png"


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

        Node(
            package="mars_navigation_ros2",
            executable="pose_simulator",
            name="pose_simulator",
            parameters=[{
                "x": start_x,
                "y": start_y,
                "image_path": heightmap,
            }],
        ),
        Node(
            package="mars_navigation_ros2",
            executable="image_to_map",
            name="image_to_map",
            parameters=[{"image_path": heightmap}],
        ),
        Node(
            package="mars_navigation_ros2",
            executable="hazard_mapper",
            name="hazard_mapper",
            parameters=[{"image_path": heightmap}],
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
        ),
        Node(
            package="mars_navigation_ros2",
            executable="terrain_image_publisher",
            name="terrain_image_publisher",
            parameters=[{
                "flat_image": str(FLAT_IMG),
                "rocky_image": str(ROCKY_IMG),
                "challenging_image": str(CHALLENGING_IMG),
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
