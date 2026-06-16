#!/usr/bin/env python3
"""
D100 实机 SLAM Launch（Gen 1：刚性头部，三路 RoboSense 雷达）

启动顺序与依赖
--------------
  1. robot_state_publisher（URDF + 三路雷达 static TF）
  2. rslidar_sdk_node       —— 普通 SDK，驱动下巴/尾巴 Airy，输出
                                  /chin/LIDAR/POINTS    (radar_f_Link)
                                  /tail/LIDAR/POINTS    (radar_r_Link)
     rslidar_sdk_airy_lite_node
                            —— Airy Lite SDK，驱动后背雷达，输出
                                  /LIDAR/POINTS         (radar_uper_Link, 主)
                                  /rslidar_imu_data     (后背雷达内置 IMU, g)
                                可关闭让同事单独跑（start_lidar_driver:=false）。
  3. imu_converter          —— /rslidar_imu_data (g) -> /imu/data (m/s², radar_uper_Link)
  4. pointcloud_merger      —— 三路 -> /merged/LIDAR/POINTS (radar_uper_Link, 10Hz)
  5. ros2_control + ocs2    —— 控制器栈（默认 on，仅录纯 SLAM bag 时可 off）
  6. slam_tf_bridge         —— map<-odom<-base 桥接（延迟 5s）
  7. RViz                   —— 默认 on，无头采集时可 off

Lightning-LM 仍单独启动（与仿真一致）：
  ros2 run lightning run_slam_online --config src/lightning-lm/config/d100_slam_back.yaml

用法
----
  # 全栈 SLAM（含控制器和 RViz）
  ros2 launch d100_description d100_slam_real.launch.py

  # 只录三路融合 bag（同事跑两个 rslidar 驱动，自己只跑融合 + IMU 转换）
  ros2 launch d100_description d100_slam_real.launch.py \\
      start_lidar_driver:=false start_controllers:=false start_rviz:=false

  # 走机房无显示场景
  ros2 launch d100_description d100_slam_real.launch.py start_rviz:=false

  # 离线排查三雷达 side missing（会增加 /merged/LIDAR/POINTS 延迟，不建议作为在线默认）
  ros2 launch d100_description d100_slam_real.launch.py \\
      start_lidar_driver:=false start_controllers:=false start_rviz:=false \\
      use_sim_time:=true side_wait_timeout_s:=1.0 max_pending_back_frames:=12
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_description = 'd100_description'
    d100_description = get_package_share_directory(pkg_description)

    # 透传给所有"消费/产生时间戳"的节点；不传给控制器和驱动（实时控制必须 wall clock，
    # bag replay 也用不到驱动和控制器栈）。
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF
    xacro_file = os.path.join(d100_description, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ros2_control 配置
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(pkg_description), 'config', 'robot_control.yaml',
    ])
    rviz_config = os.path.join(d100_description, 'config', 'd100_slam.rviz')

    # rslidar_sdk 配置：普通 SDK 只驱动下巴/尾巴，Airy Lite SDK 只驱动后背。
    try:
        rslidar_pkg_share = get_package_share_directory('rslidar_sdk')
        rslidar_chin_tail_config = os.path.join(
            rslidar_pkg_share, 'config', 'config_d100_airy.yaml')
    except Exception:
        # 源码树兜底
        rslidar_chin_tail_config = os.path.abspath(os.path.join(
            d100_description, '..', '..', '..', '..',
            'src', 'rslidar_sdk', 'config', 'config_d100_airy.yaml'))

    try:
        rslidar_airy_lite_pkg_share = get_package_share_directory('rslidar_sdk_airy_lite')
        rslidar_back_config = os.path.join(
            rslidar_airy_lite_pkg_share, 'config', 'config_d100_back_airy_lite.yaml')
    except Exception:
        # 源码树兜底
        rslidar_back_config = os.path.abspath(os.path.join(
            d100_description, '..', '..', '..', '..',
            'src', 'airy lite_sdk', 'config', 'config_d100_back_airy_lite.yaml'))

    # ============================================================
    # 1. robot_state_publisher：URDF + static TF
    # ============================================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 100.0,
            'use_sim_time': use_sim_time,
            'ignore_timestamp': True,
        }],
    )

    # ============================================================
    # 2. LiDAR 驱动：普通 SDK 跑下巴/尾巴，Airy Lite SDK 跑后背。
    # ============================================================
    rslidar_chin_tail_driver = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        namespace='rslidar_sdk',
        output='screen',
        parameters=[{'config_path': rslidar_chin_tail_config}],
        condition=IfCondition(LaunchConfiguration('start_lidar_driver')),
    )

    rslidar_back_driver = Node(
        package='rslidar_sdk_airy_lite',
        executable='rslidar_sdk_airy_lite_node',
        namespace='rslidar_sdk_airy_lite',
        output='screen',
        parameters=[{'config_path': rslidar_back_config}],
        condition=IfCondition(LaunchConfiguration('start_lidar_driver')),
    )

    # ============================================================
    # 3. IMU 单位转换 (g -> m/s²) + frame 统一为 radar_uper_Link
    # ============================================================
    imu_converter = Node(
        package='mujoco_sim',
        executable='imu_converter',
        name='imu_converter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ============================================================
    # 4. 三路点云融合 -> /merged/LIDAR/POINTS
    # Gen 1 刚性头部：cache_static_tf=True；Gen 2 改 false。
    # ============================================================
    pointcloud_merger = Node(
        package='mujoco_sim',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('lidar_frame'),
            'back_topic': '/LIDAR/POINTS',
            'chin_topic': '/chin/LIDAR/POINTS',
            'tail_topic': '/tail/LIDAR/POINTS',
            'output_topic': '/merged/LIDAR/POINTS',
            'sync_tolerance_s': LaunchConfiguration('merge_sync_tolerance_s'),
            'side_wait_timeout_s': LaunchConfiguration('side_wait_timeout_s'),
            'max_pending_back_frames': LaunchConfiguration('max_pending_back_frames'),
            'chin_z_offset_m': LaunchConfiguration('chin_z_offset_m'),
            'tail_z_offset_m': LaunchConfiguration('tail_z_offset_m'),
            'chin_roll_offset_deg': LaunchConfiguration('chin_roll_offset_deg'),
            'chin_pitch_offset_deg': LaunchConfiguration('chin_pitch_offset_deg'),
            'chin_yaw_offset_deg': LaunchConfiguration('chin_yaw_offset_deg'),
            'tail_roll_offset_deg': LaunchConfiguration('tail_roll_offset_deg'),
            'tail_pitch_offset_deg': LaunchConfiguration('tail_pitch_offset_deg'),
            'tail_yaw_offset_deg': LaunchConfiguration('tail_yaw_offset_deg'),
            'back_max_range_m': LaunchConfiguration('back_max_range_m'),
            'chin_max_range_m': LaunchConfiguration('chin_max_range_m'),
            'tail_max_range_m': LaunchConfiguration('tail_max_range_m'),
            'chin_driver_to_link_roll_deg': LaunchConfiguration('chin_driver_to_link_roll_deg'),
            'chin_driver_to_link_pitch_deg': LaunchConfiguration('chin_driver_to_link_pitch_deg'),
            'chin_driver_to_link_yaw_deg': LaunchConfiguration('chin_driver_to_link_yaw_deg'),
            'tail_driver_to_link_roll_deg': LaunchConfiguration('tail_driver_to_link_roll_deg'),
            'tail_driver_to_link_pitch_deg': LaunchConfiguration('tail_driver_to_link_pitch_deg'),
            'tail_driver_to_link_yaw_deg': LaunchConfiguration('tail_driver_to_link_yaw_deg'),
            'dynamic_self_filter_enabled': LaunchConfiguration('dynamic_self_filter_enabled'),
            'dynamic_self_filter_padding_m': LaunchConfiguration('dynamic_self_filter_padding_m'),
            'dynamic_self_filter_shape_json': LaunchConfiguration('dynamic_self_filter_shape_json'),
            'dynamic_self_filter_require_all_shapes': False,
            'cache_static_tf': True,
            'tf_lookup_timeout_s': 2.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # ============================================================
    # 5. ros2_control + 控制器栈（条件启动）
    # ============================================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_controllers],
        remappings=[('~/robot_description', '/robot_description')],
        condition=IfCondition(LaunchConfiguration('start_controllers')),
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controllers')),
    )

    imu_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster', '-c', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    ocs2_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ocs2_quadruped_controller', '-c', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    # ============================================================
    # 6. SLAM TF 桥接（延迟 5s 等 robot_state_publisher 和 ocs2 起来）
    # ============================================================
    slam_tf_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='mujoco_sim',
                executable='slam_tf_bridge',
                name='slam_tf_bridge',
                output='screen',
                parameters=[{
                    'slam_odom_topic': 'lightning/odom',
                    'control_odom_topic': '/odom',
                    'odom_frame': 'odom',
                    'base_frame': 'base',
                    'lidar_frame': LaunchConfiguration('lidar_frame'),
                    'publish_odom_base_tf': True,
                    'use_sim_time': use_sim_time,
                }],
            ),
        ],
    )

    # ============================================================
    # 7. RViz
    # ============================================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return [
        robot_state_pub,
        rslidar_chin_tail_driver,
        rslidar_back_driver,
        imu_converter,
        pointcloud_merger,
        controller_manager,
        joint_state_spawner,
        # joint -> imu broadcaster -> ocs2 顺序与仿真一致
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[imu_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_spawner,
                on_exit=[ocs2_spawner],
            )
        ),
        slam_tf_bridge,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_lidar_driver',
            default_value='true',
            description='Launch both LiDAR SDK drivers. Set false if a colleague is running them separately.',
        ),
        DeclareLaunchArgument(
            'start_controllers',
            default_value='true',
            description='Launch ros2_control + OCS2 controller stack. Set false for SLAM-only bag recording.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Launch RViz. Set false on headless machines.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='radar_uper_Link',
            description='Reference LiDAR frame for SLAM and merger output.',
        ),
        DeclareLaunchArgument(
            'merge_sync_tolerance_s',
            default_value='0.01',
            description='Maximum timestamp delta for matching back/chin/tail LiDAR frames.',
        ),
        DeclareLaunchArgument(
            'side_wait_timeout_s',
            default_value='0.15',
            description='How long pointcloud_merger waits for delayed side LiDAR frames before fallback.',
        ),
        DeclareLaunchArgument(
            'max_pending_back_frames',
            default_value='3',
            description='Maximum queued back LiDAR frames while waiting for delayed side LiDAR frames.',
        ),
        DeclareLaunchArgument(
            'chin_z_offset_m',
            default_value='0.0',
            description='Extra Z correction applied to chin LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'tail_z_offset_m',
            default_value='0.0',
            description='Extra Z correction applied to tail LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'chin_roll_offset_deg',
            default_value='0.0',
            description='Extra roll correction for chin LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'chin_pitch_offset_deg',
            default_value='0.0',
            description='Extra pitch correction for chin LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'chin_yaw_offset_deg',
            default_value='0.0',
            description='Extra yaw correction for chin LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'tail_roll_offset_deg',
            default_value='0.0',
            description='Extra roll correction for tail LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'tail_pitch_offset_deg',
            default_value='0.0',
            description='Extra pitch correction for tail LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'tail_yaw_offset_deg',
            default_value='0.0',
            description='Extra yaw correction for tail LiDAR points after TF merge.',
        ),
        DeclareLaunchArgument(
            'back_max_range_m',
            default_value='0.0',
            description='Optional max range filter for back LiDAR in merged target frame. 0 disables it.',
        ),
        DeclareLaunchArgument(
            'chin_max_range_m',
            default_value='0.0',
            description='Optional max range filter for chin LiDAR in merged target frame. 0 disables it.',
        ),
        DeclareLaunchArgument(
            'tail_max_range_m',
            default_value='0.0',
            description='Optional max range filter for tail LiDAR in merged target frame. 0 disables it.',
        ),
        DeclareLaunchArgument(
            'chin_driver_to_link_roll_deg',
            default_value='0.0',
            description='Roll rotation from chin LiDAR driver cloud axes to URDF link axes.',
        ),
        DeclareLaunchArgument(
            'chin_driver_to_link_pitch_deg',
            default_value='0.0',
            description='Pitch rotation from chin LiDAR driver cloud axes to URDF link axes.',
        ),
        DeclareLaunchArgument(
            'chin_driver_to_link_yaw_deg',
            default_value='180.0',
            description='Yaw rotation from chin LiDAR driver cloud axes to URDF link axes.',
        ),
        DeclareLaunchArgument(
            'tail_driver_to_link_roll_deg',
            default_value='0.0',
            description='Roll rotation from tail LiDAR driver cloud axes to URDF link axes.',
        ),
        DeclareLaunchArgument(
            'tail_driver_to_link_pitch_deg',
            default_value='0.0',
            description='Pitch rotation from tail LiDAR driver cloud axes to URDF link axes.',
        ),
        DeclareLaunchArgument(
            'tail_driver_to_link_yaw_deg',
            default_value='0.0',
            description='Yaw rotation from tail LiDAR driver cloud axes to URDF link axes.',
        ),
        DeclareLaunchArgument(
            'dynamic_self_filter_enabled',
            default_value='true',
            description='Enable TF-based dynamic robot self filtering before publishing /merged/LIDAR/POINTS.',
        ),
        DeclareLaunchArgument(
            'dynamic_self_filter_padding_m',
            default_value='0.03',
            description='Padding added to each robot body box used by dynamic self filtering.',
        ),
        DeclareLaunchArgument(
            'dynamic_self_filter_shape_json',
            default_value='',
            description='Optional JSON file defining self-filter boxes. Empty uses built-in D100 boxes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Use /clock instead of wall clock. Set true when replaying a bag with '
                '`ros2 bag play --clock`. Only transmitted to robot_state_publisher, '
                'imu_converter, pointcloud_merger, slam_tf_bridge, and rviz; the control '
                'stack and LiDAR SDK nodes always use wall clock.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
