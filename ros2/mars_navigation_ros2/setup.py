from setuptools import setup

package_name = "mars_navigation_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/smoke_stack.launch.py",
            "launch/isaac_stack.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lry",
    maintainer_email="lry@example.com",
    description="ROS2 Humble nodes for Mars navigation reproduction.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "image_to_map = mars_navigation_ros2.image_to_map:main",
            "map_server = mars_navigation_ros2.map_server:main",
            "global_planner = mars_navigation_ros2.global_planner:main",
            "global_path_optimizer = mars_navigation_ros2.global_path_optimizer:main",
            "local_planner = mars_navigation_ros2.local_planner:main",
            "path_follower = mars_navigation_ros2.path_follower:main",
            "manual_mode = mars_navigation_ros2.manual_mode:main",
            "vlm_mode = mars_navigation_ros2.vlm_mode:main",
            "hazard_mapper = mars_navigation_ros2.hazard_mapper:main",
            "terrain_image_publisher = mars_navigation_ros2.terrain_image_publisher:main",
            "pose_simulator = mars_navigation_ros2.pose_simulator:main",
            "evaluate = mars_navigation_ros2.evaluate:main",
        ],
    },
)
