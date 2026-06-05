"""
IMU LCM-to-ROS2 桥接节点
- 订阅 LCM IMU_DATA channel（来自 MuJoCo 仿真）
- 转换为 sensor_msgs/Imu 消息
- 发布到 /imu/data（匹配 D100 ros2_control 约定）

用法：
  ros2 run mujoco_sim sim_imu_bridge
  # 或直接
  python sim_imu_bridge.py
"""

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_data_lcmt import imu_data_lcmt

try:
    import lcm
    HAS_LCM = True
except ImportError:
    HAS_LCM = False


class SimImuBridge(Node):
    """LCM IMU_DATA -> ROS2 /imu/data 桥接"""

    def __init__(self):
        super().__init__('sim_imu_bridge')

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 50)

        if not HAS_LCM:
            self.get_logger().error('lcm not installed, bridge cannot work')
            return

        # LCM 订阅
        self.lcm_node = lcm.LCM('udpm://239.255.76.67:7667?ttl=0')
        self.lcm_node.subscribe('IMU_DATA', self._lcm_callback)

        # LCM 接收线程
        self._running = True
        self._thread = threading.Thread(target=self._lcm_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('SimImuBridge: LCM(IMU_DATA) -> ROS2(/imu/data)')

    def _lcm_loop(self):
        while self._running:
            try:
                self.lcm_node.handle_timeout(100)
            except Exception:
                pass

    def _lcm_callback(self, channel, data):
        try:
            lcm_msg = imu_data_lcmt.decode(data)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode IMU_DATA: {e}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar_uper_Link'

        # LCM quat 格式: [x, y, z, w]
        msg.orientation = Quaternion(
            x=float(lcm_msg.quat[0]),
            y=float(lcm_msg.quat[1]),
            z=float(lcm_msg.quat[2]),
            w=float(lcm_msg.quat[3]),
        )

        # 角速度 (rad/s)
        msg.angular_velocity = Vector3(
            x=float(lcm_msg.omega[0]),
            y=float(lcm_msg.omega[1]),
            z=float(lcm_msg.omega[2]),
        )

        # 线加速度 (m/s^2)
        msg.linear_acceleration = Vector3(
            x=float(lcm_msg.acc[0]),
            y=float(lcm_msg.acc[1]),
            z=float(lcm_msg.acc[2]),
        )

        # 协方差设为 -1 表示未知
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
