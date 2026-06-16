from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution([
        FindPackageShare("mujoco_sim"),
        "config",
        "pointcloud_merger_d100_dynamic_self_filter.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="pointcloud_merger params with D100 dynamic self filtering enabled",
        ),
        Node(
            package="mujoco_sim",
            executable="pointcloud_merger",
            name="pointcloud_merger",
            output="screen",
            parameters=[params_file],
        ),
    ])
