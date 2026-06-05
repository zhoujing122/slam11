硬件接口文件：src/d100_sdk （目前缺少imu读取，已有电机控制、状态读取）
  mkdir buil
  cmake ..
  make -j
 ./example 或 ./example1 

Mujoco仿真环境：src/cmujoco_sim （含有lcm数据收发）
  启动环境：python3 mujoco_quadruped_env.py
  lcm测试：
    python test/examples_motor_controller.py
    python test/examples_sensor_listener.py

ros2接口：src/hardwares/hardware_future_d100_sdk (功能：d100_sdk和mujoco lcm的选择，后期可加isaac lab等)
  为description服务
  编译：colcon build --packages-select hardware_future_d100_sdk  （包含d100_sdk的编译）

ros2_control接口：src/descriptions/GON_Future/D100
  在/ros2_ws编译：colcon build --packages-select d100_description
  启动：ros2 launch d100_description d100.launch.py
  查看imu:      ros2 topic echo /imu_sensor_broadcaster/imu --once
  查看关节数据：  ros2 topic echo /joint_states --once 
  "需要单独启动Mujoco仿真环境python mujoco_quadruped_env.py"
	

2026.02.12备份至此-接下来更新ros2_control接口，使得满足ocs2  

ros2_control接口完成，可实现ocs2控制，但目前控制尚不稳定，启动代码：
  Mujoco仿真：
    cd /ros2_ws/src/mujoco_sim/mujoco_sim
    python3 mujoco_quadruped_env.py
  遥控器：
    ros2 launch joystick_input joystick.launch.py
  ocs2控制器：
    ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=d100_description
2026.02.25备份至此-接下来更新d100_description中ocs2的超参数，使得MPC控制稳定，并且完善RViz显示  
    已完善RViz显示
2026.02.26备份至此-接下来更新d100_description中ocs2的超参数，使得MPC控制稳定
  MPC控制较稳定，还需要微调，确保MPC稳定控制
2026.02.28备份至此-接下来1、微调MPC，确保MPC稳定控制；2、加入强化控制算法

lab@System:~$ scp /home/lab/Desktop/d100_lidar_config_update_20260529_154935.tar.gz linux@192.168.124.88:/home/linux/gon_control_d
linux@192.168.124.88's password: 
d100_lidar_config_update_20260529_154935.tar. 100%  872    14.1KB/s   00:00    
lab@System:~$ ssh linux@192.168.124.88
linux@192.168.124.88's password: 

Welcome to Ubuntu 22.04.3 LTS (GNU/Linux 6.1.118 aarch64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage

This system has been minimized by removing packages and content that are
not required on a system that users do not log into.

To restore this content, you can run the 'unminimize' command.
Last login: Fri May 29 15:10:16 2026 from 192.168.124.95

linux@linux:~$ 
linux@linux:~$ cd gon_control_d/
linux@linux:~/gon_control_d$ source /opt/ros/humble/setup.bash
  source install/setup.bash
  source install_rslidar_sdk/setup.bash
  source install_airy_lite/setup.bash
  unset RMW_IMPLEMENTATION
linux@linux:~/gon_control_d$ ros2 topic list --no-daemon -t | grep -E "LIDAR|rslidar|chin|tail"
sequence size exceeds remaining buffer
sequence size exceeds remaining buffer
sequence size exceeds remaining buffer
sequence size exceeds remaining buffer
sequence size exceeds remaining buffer
sequence size exceeds remaining buffer
sequence size exceeds remaining buffer
/LIDAR/POINTS [sensor_msgs/msg/PointCloud2]
/chin/LIDAR/POINTS [sensor_msgs/msg/PointCloud2]
/chin/rslidar_imu_data [sensor_msgs/msg/Imu]
/rslidar_imu_data [sensor_msgs/msg/Imu]
/tail/LIDAR/POINTS [sensor_msgs/msg/PointCloud2]
/tail/rslidar_imu_data [sensor_msgs/msg/Imu]
linux@linux:~/gon_control_d$ ros2 topic info -v /tail/LIDAR/POINTS
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rslidar_points_destination_1
Node namespace: /rslidar_sdk
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.1e.03.d4.1c.c7.3c.00.00.00.00.00.00.32.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: pointcloud_merger
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.1e.03.da.1c.bd.d5.00.00.00.00.00.00.15.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

linux@linux:~/gon_control_d$ ros2 topic info -v /chin/LIDAR/POINTS
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rslidar_points_destination_0
Node namespace: /rslidar_sdk
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.1e.03.d4.1c.c7.3c.00.00.00.00.00.00.21.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: pointcloud_merger
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.1e.03.da.1c.bd.d5.00.00.00.00.00.00.14.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

linux@linux:~/gon_control_d$ ros2 topic info -v /LIDAR/POINTS
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rslidar_points_destination_0
Node namespace: /rslidar_sdk_airy_lite
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.1e.03.d6.1c.77.3c.00.00.00.00.00.00.21.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: pointcloud_merger
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.1e.03.da.1c.bd.d5.00.00.00.00.00.00.16.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

linux@linux:~/gon_control_d$ 

[rslidar_sdk_airy_lite_node-3] Difop Port: 7788
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3]              RoboSense Driver Parameters 
[rslidar_sdk_airy_lite_node-3] input type: ONLINE_LIDAR
[rslidar_sdk_airy_lite_node-3] lidar_type: RSAIRYLITE_ETH
[rslidar_sdk_airy_lite_node-3] frame_id: rslidar
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3]              RoboSense Input Parameters 
[rslidar_sdk_airy_lite_node-3] msop_port: 6699
[rslidar_sdk_airy_lite_node-3] difop_port: 7788
[rslidar_sdk_airy_lite_node-3] imu_port: 6688
[rslidar_sdk_airy_lite_node-3] user_layer_bytes: 0
[rslidar_sdk_airy_lite_node-3] tail_layer_bytes: 0
[rslidar_sdk_airy_lite_node-3] host_address: 192.168.1.102
[rslidar_sdk_airy_lite_node-3] group_address: 0.0.0.0
[rslidar_sdk_airy_lite_node-3] socket_recv_buf: 106496
[rslidar_sdk_airy_lite_node-3] pcap_path: 
[rslidar_sdk_airy_lite_node-3] pcap_rate: 1
[rslidar_sdk_airy_lite_node-3] pcap_repeat: 1
[rslidar_sdk_airy_lite_node-3] use_vlan: 0
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3]              RoboSense Decoder Parameters 
[rslidar_sdk_airy_lite_node-3] min_distance: 0.1
[rslidar_sdk_airy_lite_node-3] max_distance: 60
[rslidar_sdk_airy_lite_node-3] use_lidar_clock: 0
[rslidar_sdk_airy_lite_node-3] dense_points: 1
[rslidar_sdk_airy_lite_node-3] ts_first_point: 1
[rslidar_sdk_airy_lite_node-3] wait_for_difop: 1
[rslidar_sdk_airy_lite_node-3] config_from_file: 0
[rslidar_sdk_airy_lite_node-3] angle_path: 
[rslidar_sdk_airy_lite_node-3] start_angle: 0
[rslidar_sdk_airy_lite_node-3] end_angle: 360
[rslidar_sdk_airy_lite_node-3] split_frame_mode: 1
[rslidar_sdk_airy_lite_node-3] split_angle: 0
[rslidar_sdk_airy_lite_node-3] num_blks_split: 0
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3]              RoboSense Transform Parameters 
[rslidar_sdk_airy_lite_node-3] x: 0
[rslidar_sdk_airy_lite_node-3] y: 0
[rslidar_sdk_airy_lite_node-3] z: 0
[rslidar_sdk_airy_lite_node-3] roll: 0
[rslidar_sdk_airy_lite_node-3] pitch: 0
[rslidar_sdk_airy_lite_node-3] yaw: 0
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] Send PointCloud To : ROS
[rslidar_sdk_airy_lite_node-3] PointCloud Topic: /LIDAR/POINTS
[rslidar_sdk_airy_lite_node-3] ------------------------------------------------------
[rslidar_sdk_airy_lite_node-3] RoboSense-LiDAR-Driver is running.....
[robot_state_publisher-1] [INFO] [1780041921.882789826] [robot_state_publisher]: got segment FL_calf
[robot_state_publisher-1] [INFO] [1780041921.885201876] [robot_state_publisher]: got segment FL_foot
[robot_state_publisher-1] [INFO] [1780041921.886810979] [robot_state_publisher]: got segment FL_hip
[robot_state_publisher-1] [INFO] [1780041921.888287375] [robot_state_publisher]: got segment FL_thigh
[robot_state_publisher-1] [INFO] [1780041921.889692313] [robot_state_publisher]: got segment FR_calf
[robot_state_publisher-1] [INFO] [1780041921.891009462] [robot_state_publisher]: got segment FR_foot
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3310
[robot_state_publisher-1] [INFO] [1780041921.893762174] [robot_state_publisher]: got segment FR_hip
[robot_state_publisher-1] [INFO] [1780041921.894836950] [robot_state_publisher]: got segment FR_thigh
[robot_state_publisher-1] [INFO] [1780041921.895881977] [robot_state_publisher]: got segment RL_calf
[robot_state_publisher-1] [INFO] [1780041921.895919602] [robot_state_publisher]: got segment RL_foot
[robot_state_publisher-1] [INFO] [1780041921.895946143] [robot_state_publisher]: got segment RL_hip
[robot_state_publisher-1] [INFO] [1780041921.895971809] [robot_state_publisher]: got segment RL_thigh
[robot_state_publisher-1] [INFO] [1780041921.895996309] [robot_state_publisher]: got segment RR_calf
[robot_state_publisher-1] [INFO] [1780041921.896020517] [robot_state_publisher]: got segment RR_foot
[robot_state_publisher-1] [INFO] [1780041921.899447844] [robot_state_publisher]: got segment RR_hip
[robot_state_publisher-1] [INFO] [1780041921.899502969] [robot_state_publisher]: got segment RR_thigh
[robot_state_publisher-1] [INFO] [1780041921.899530385] [robot_state_publisher]: got segment base
[robot_state_publisher-1] [INFO] [1780041921.899555176] [robot_state_publisher]: got segment imu_Link
[robot_state_publisher-1] [INFO] [1780041921.899579676] [robot_state_publisher]: got segment radar_f_Link
[robot_state_publisher-1] [INFO] [1780041921.899604176] [robot_state_publisher]: got segment radar_r_Link
[robot_state_publisher-1] [INFO] [1780041921.899628384] [robot_state_publisher]: got segment radar_uper_Link
[robot_state_publisher-1] [INFO] [1780041921.899653467] [robot_state_publisher]: got segment trunk
[rslidar_sdk_node-2] Original receive buffer size: 212992 bytes
[rslidar_sdk_node-2] After setting: receive buffer size: 425984 bytes
[rslidar_sdk_node-2] Original receive buffer size: 212992 bytes
[rslidar_sdk_node-2] After setting: receive buffer size: 425984 bytes
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] Send PointCloud To : ROS
[rslidar_sdk_node-2] PointCloud Topic: /chin/LIDAR/POINTS
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] Receive Packets From : Online LiDAR
[rslidar_sdk_node-2] Msop Port: 6701
[rslidar_sdk_node-2] Difop Port: 7790
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2]              RoboSense Driver Parameters 
[rslidar_sdk_node-2] input type: ONLINE_LIDAR
[rslidar_sdk_node-2] lidar_type: RSAIRY
[rslidar_sdk_node-2] frame_id: rslidar
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2]              RoboSense Input Parameters 
[rslidar_sdk_node-2] msop_port: 6701
[rslidar_sdk_node-2] difop_port: 7790
[rslidar_sdk_node-2] imu_port: 0
[rslidar_sdk_node-2] user_layer_bytes: 0
[rslidar_sdk_node-2] tail_layer_bytes: 0
[rslidar_sdk_node-2] host_address: 192.168.3.102
[rslidar_sdk_node-2] group_address: 0.0.0.0
[rslidar_sdk_node-2] socket_recv_buf: 106496
[rslidar_sdk_node-2] pcap_path: 
[rslidar_sdk_node-2] pcap_rate: 1
[rslidar_sdk_node-2] pcap_repeat: 1
[rslidar_sdk_node-2] use_vlan: 0
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2]              RoboSense Decoder Parameters 
[rslidar_sdk_node-2] min_distance: 0.1
[rslidar_sdk_node-2] max_distance: 60
[rslidar_sdk_node-2] use_lidar_clock: 0
[rslidar_sdk_node-2] dense_points: 1
[rslidar_sdk_node-2] ts_first_point: 1
[rslidar_sdk_node-2] wait_for_difop: 1
[rslidar_sdk_node-2] config_from_file: 0
[rslidar_sdk_node-2] angle_path: 
[rslidar_sdk_node-2] start_angle: 0
[rslidar_sdk_node-2] end_angle: 360
[rslidar_sdk_node-2] split_frame_mode: 1
[rslidar_sdk_node-2] split_angle: 0
[rslidar_sdk_node-2] num_blks_split: 0
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2]              RoboSense Transform Parameters 
[rslidar_sdk_node-2] x: 0
[rslidar_sdk_node-2] y: 0
[rslidar_sdk_node-2] z: 0
[rslidar_sdk_node-2] roll: 0
[rslidar_sdk_node-2] pitch: 0
[rslidar_sdk_node-2] yaw: 0
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] Original receive buffer size: 212992 bytes
[rslidar_sdk_node-2] After setting: receive buffer size: 425984 bytes
[rslidar_sdk_node-2] Original receive buffer size: 212992 bytes
[rslidar_sdk_node-2] After setting: receive buffer size: 425984 bytes
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] Send PointCloud To : ROS
[rslidar_sdk_node-2] PointCloud Topic: /tail/LIDAR/POINTS
[rslidar_sdk_node-2] ------------------------------------------------------
[rslidar_sdk_node-2] RoboSense-LiDAR-Driver is running.....
[imu_converter-4] [INFO] [1780041923.555423623] [imu_converter]: ImuConverter: /rslidar_imu_data (g) -> /imu/data (m/s²)
[pointcloud_merger-5] [INFO] [1780041923.674561390] [pointcloud_merger]: pointcloud_merger: back=/LIDAR/POINTS chin=/chin/LIDAR/POINTS tail=/tail/LIDAR/POINTS -> /merged/LIDAR/POINTS frame=radar_uper_Link tolerance=0.010s cache_static_tf=True tf_timeout=2.0s stats_period=5.0s
[pointcloud_merger-5] [WARN] [1780041923.695736680] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041923.494416
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[robot_state_publisher-1] sequence size exceeds remaining buffer
[robot_state_publisher-1] sequence size exceeds remaining buffer
[pointcloud_merger-5] [WARN] [1780041925.702460793] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041925.594459
[robot_state_publisher-1] sequence size exceeds remaining buffer
[imu_converter-4] sequence size exceeds remaining buffer
[imu_converter-4] sequence size exceeds remaining buffer
[imu_converter-4] sequence size exceeds remaining buffer
[pointcloud_merger-5] sequence size exceeds remaining buffer
[pointcloud_merger-5] sequence size exceeds remaining buffer
[pointcloud_merger-5] sequence size exceeds remaining buffer
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[INFO] [slam_tf_bridge-6]: process started with pid [7450]
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[pointcloud_merger-5] [WARN] [1780041927.800378024] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041927.694617
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[robot_state_publisher-1] sequence size exceeds remaining buffer
[robot_state_publisher-1] sequence size exceeds remaining buffer
[robot_state_publisher-1] sequence size exceeds remaining buffer
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[pointcloud_merger-5] [INFO] [1780041928.554473603] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041928.557677517] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[slam_tf_bridge-6] [INFO] [1780041928.931974348] [slam_tf_bridge]: SlamTfBridge: lightning/odom + /odom -> TF(map->odom, odom->base)
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[rslidar_sdk_airy_lite_node-3] sequence size exceeds remaining buffer
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[robot_state_publisher-1] sequence size exceeds remaining buffer
[rslidar_sdk_node-2] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[pointcloud_merger-5] [WARN] [1780041929.804229213] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041929.694349
[pointcloud_merger-5] sequence size exceeds remaining buffer
[pointcloud_merger-5] sequence size exceeds remaining buffer
[pointcloud_merger-5] sequence size exceeds remaining buffer
[imu_converter-4] sequence size exceeds remaining buffer
[pointcloud_merger-5] sequence size exceeds remaining buffer
[imu_converter-4] sequence size exceeds remaining buffer
[pointcloud_merger-5] [WARN] [1780041931.903334351] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041931.794501
[imu_converter-4] sequence size exceeds remaining buffer
[imu_converter-4] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[slam_tf_bridge-6] sequence size exceeds remaining buffer
[pointcloud_merger-5] [INFO] [1780041933.554884350] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041933.557945059] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041934.002383619] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041933.894661
[pointcloud_merger-5] [WARN] [1780041936.100634739] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041935.994827
[pointcloud_merger-5] [WARN] [1780041938.101744206] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041937.994559
[pointcloud_merger-5] [INFO] [1780041938.554748384] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041938.557012853] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041940.202399462] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041940.094712
[pointcloud_merger-5] [WARN] [1780041942.303044930] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041942.194868
[pointcloud_merger-5] [INFO] [1780041943.555840173] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041943.558192725] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041944.402779079] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041944.294901
[pointcloud_merger-5] [WARN] [1780041946.500416591] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041946.395035
[pointcloud_merger-5] [WARN] [1780041948.502771478] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041948.394761
[pointcloud_merger-5] [INFO] [1780041948.554554758] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041948.556582689] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041950.602424627] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041950.494922
[pointcloud_merger-5] [WARN] [1780041952.702686310] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041952.595078
[pointcloud_merger-5] [INFO] [1780041953.554819663] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041953.556852553] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041954.800116363] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041954.695243
[pointcloud_merger-5] [WARN] [1780041956.802027229] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041956.695001
[pointcloud_merger-5] [INFO] [1780041958.554758615] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041958.557021627] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041958.902664359] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041958.795129
[pointcloud_merger-5] [WARN] [1780041961.002655870] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041960.895302
[pointcloud_merger-5] [WARN] [1780041963.100561533] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041962.995470
[pointcloud_merger-5] [INFO] [1780041963.554459397] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041963.556465746] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041965.102585447] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041964.995179
[pointcloud_merger-5] [WARN] [1780041967.203257772] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041967.095342
[pointcloud_merger-5] [INFO] [1780041968.554603910] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041968.556636217] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041969.302633485] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041969.195516
[pointcloud_merger-5] [WARN] [1780041971.402769685] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041971.295504
[pointcloud_merger-5] [WARN] [1780041973.503618829] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041973.395665
[pointcloud_merger-5] [INFO] [1780041973.554502947] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041973.556807085] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041975.601265050] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041975.495846
[pointcloud_merger-5] [WARN] [1780041977.603594438] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041977.495552
[pointcloud_merger-5] [INFO] [1780041978.554888286] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041978.557372964] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041979.703411157] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041979.595719
[pointcloud_merger-5] [WARN] [1780041981.803899986] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041981.696075
[pointcloud_merger-5] [INFO] [1780041983.554480976] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041983.556510659] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041983.902090628] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041983.796076
[pointcloud_merger-5] [WARN] [1780041985.902944065] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041985.795773
[pointcloud_merger-5] [WARN] [1780041988.003287701] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041987.895941
[pointcloud_merger-5] [INFO] [1780041988.554678077] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041988.556918050] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041990.104306658] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041989.996104
[pointcloud_merger-5] [WARN] [1780041992.201511067] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041992.096286
[pointcloud_merger-5] [INFO] [1780041993.554669051] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041993.556905816] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780041994.203540929] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041994.095995
[pointcloud_merger-5] [WARN] [1780041996.302908247] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041996.196167
[pointcloud_merger-5] [WARN] [1780041998.403780079] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780041998.296168
[pointcloud_merger-5] [INFO] [1780041998.554441933] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780041998.556683948] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3311
[pointcloud_merger-5] [WARN] [1780042000.504031074] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780042000.396339
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3310
[pointcloud_merger-5] [WARN] [1780042002.603183778] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780042002.496531
[pointcloud_merger-5] [INFO] [1780042003.554628751] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780042003.556880975] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3311
[pointcloud_merger-5] [WARN] [1780042004.603164663] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780042004.496223
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3310
[pointcloud_merger-5] [WARN] [1780042006.704537439] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780042006.596425
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3311
[pointcloud_merger-5] [INFO] [1780042008.554665714] [pointcloud_merger]: stats[5.0s]: pub=0 (0.0Hz) drop=50 (no_sync=50, tf=0, merge=0) pts/frame=0 (back=0 chin=0 tail=0) max_sync_dt=0.0ms
[pointcloud_merger-5] [WARN] [1780042008.557039270] [pointcloud_merger]: merger received 50 back clouds but published 0 — check topics, TF tree, and sync tolerance
[pointcloud_merger-5] [WARN] [1780042008.804328636] [pointcloud_merger]: drop back cloud: no chin/tail cloud within 0.010s of 1780042008.696565
[rslidar_sdk_airy_lite_node-3]  > Refresh Difop. 3310
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rslidar_sdk_node-2] RoboSense-LiDAR-Driver is stopping.....
[rslidar_sdk_airy_lite_node-3] RoboSense-LiDAR-Driver is stopping.....
[robot_state_publisher-1] [INFO] [1780042010.214114509] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[rslidar_sdk_airy_lite_node-3] [INFO] [1780042010.214169633] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[rslidar_sdk_airy_lite_node-3] terminate called after throwing an instance of 'std::system_error'
[rslidar_sdk_airy_lite_node-3]   what():  Invalid argument
[rslidar_sdk_node-2] [INFO] [1780042010.214112467] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[rslidar_sdk_node-2] terminate called after throwing an instance of 'std::system_error'
[rslidar_sdk_node-2]   what():  Invalid argument
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 7378]
[slam_tf_bridge-6] Traceback (most recent call last):
[slam_tf_bridge-6]   File "/home/linux/gon_control_d/install/mujoco_sim/lib/mujoco_sim/slam_tf_bridge", line 33, in <module>
[slam_tf_bridge-6]     sys.exit(load_entry_point('mujoco-sim', 'console_scripts', 'slam_tf_bridge')())
[slam_tf_bridge-6]   File "/home/linux/gon_control_d/build/mujoco_sim/mujoco_sim/slam_tf_bridge.py", line 275, in main
[slam_tf_bridge-6]     rclpy.shutdown()
[slam_tf_bridge-6]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 130, in shutdown
[slam_tf_bridge-6]     _shutdown(context=context)
[slam_tf_bridge-6]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
[slam_tf_bridge-6]     return context.shutdown()
[slam_tf_bridge-6]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 102, in shutdown
[slam_tf_bridge-6]     self.__context.shutdown()
[slam_tf_bridge-6] rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context, at ./src/rcl/init.c:241
[imu_converter-4] Traceback (most recent call last):
[imu_converter-4]   File "/home/linux/gon_control_d/build/mujoco_sim/mujoco_sim/imu_converter.py", line 61, in main
[imu_converter-4]     rclpy.spin(node)
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 229, in spin
[imu_converter-4]     executor.spin_once()
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 808, in spin_once
[imu_converter-4]     self._spin_once_impl(timeout_sec)
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 805, in _spin_once_impl
[imu_converter-4]     raise handler.exception()
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 272, in _execute_coroutine_step
[imu_converter-4]     result = coro.send(None)
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 478, in handler
[imu_converter-4]     arg = take_from_wait_list(entity)
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 400, in _take_subscription
[imu_converter-4]     msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
[imu_converter-4] RuntimeError: Unable to convert call argument to Python object (compile in debug mode for details)
[imu_converter-4] 
[imu_converter-4] During handling of the above exception, another exception occurred:
[imu_converter-4] 
[imu_converter-4] Traceback (most recent call last):
[imu_converter-4]   File "/home/linux/gon_control_d/install/mujoco_sim/lib/mujoco_sim/imu_converter", line 33, in <module>
[imu_converter-4]     sys.exit(load_entry_point('mujoco-sim', 'console_scripts', 'imu_converter')())
[imu_converter-4]   File "/home/linux/gon_control_d/build/mujoco_sim/mujoco_sim/imu_converter.py", line 66, in main
[imu_converter-4]     rclpy.shutdown()
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 130, in shutdown
[imu_converter-4]     _shutdown(context=context)
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
[imu_converter-4]     return context.shutdown()
[imu_converter-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 102, in shutdown
[imu_converter-4]     self.__context.shutdown()

