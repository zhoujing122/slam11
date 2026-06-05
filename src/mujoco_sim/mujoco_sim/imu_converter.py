"""
IMU 单位转换节点
- 订阅 /rslidar_imu_data（Airy 内置 IMU）
- 默认按 ROS 标准 m/s² 透传加速度；如旧驱动输出 g，可设置 linear_acceleration_scale:=9.80665
- 发布到 /imu/data（Lightning-LM 订阅的标准 topic）

用法:
  ros2 run mujoco_sim imu_converter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665  # m/s²


class ImuConverter(Node):

    def __init__(self):
        super().__init__('imu_converter')
        self.declare_parameter('linear_acceleration_scale', 1.0)
        self.linear_acceleration_scale = float(
            self.get_parameter('linear_acceleration_scale').value)
        self.pub = self.create_publisher(Imu, '/imu/data', 50)
        self.sub = self.create_subscription(
            Imu, '/rslidar_imu_data', self._callback, 50)
        self._last_stamp = (0, 0)  # 去重：记录上一条时间戳
        self.get_logger().info(
            'ImuConverter: /rslidar_imu_data -> /imu/data, '
            f'linear_acceleration_scale={self.linear_acceleration_scale}')

    def _callback(self, msg: Imu):
        # 去重：相同时间戳的消息只发一次
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp
        self.pub.publish(convert_imu_message(
            msg, linear_acceleration_scale=self.linear_acceleration_scale))


def convert_imu_message(msg: Imu, linear_acceleration_scale: float = 1.0) -> Imu:
    out = Imu()
    out.header = msg.header
    # 统一到背部雷达 IMU 帧（Airy 内置 IMU 物理位于背部雷达内）
    out.header.frame_id = 'radar_uper_Link'

    # 四元数和角速度直接透传（单位已经是 rad/s）
    out.orientation = msg.orientation
    out.angular_velocity = msg.angular_velocity

    # ROS sensor_msgs/Imu 的线加速度标准单位是 m/s²。
    out.linear_acceleration.x = msg.linear_acceleration.x * linear_acceleration_scale
    out.linear_acceleration.y = msg.linear_acceleration.y * linear_acceleration_scale
    out.linear_acceleration.z = msg.linear_acceleration.z * linear_acceleration_scale

    # 协方差透传
    out.orientation_covariance = msg.orientation_covariance
    out.angular_velocity_covariance = msg.angular_velocity_covariance
    out.linear_acceleration_covariance = msg.linear_acceleration_covariance
    return out


def main(args=None):
    rclpy.init(args=args)
    node = ImuConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
