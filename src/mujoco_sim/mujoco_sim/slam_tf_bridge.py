"""
SLAM TF 桥接节点
- 订阅 Lightning-LM 的 lightning/odom (nav_msgs/Odometry)
- 订阅 OCS2 的 /odom (nav_msgs/Odometry)，并发布 odom -> base TF
- 获取 map -> lidar_frame 位姿
- 查找 TF: base -> lidar_frame (静态)
- 计算并发布 map -> odom TF

TF 树结构 (REP-105 标准):
  map -> odom (本节点发布)
    odom -> base (本节点由 OCS2 /odom 转发)
      base -> trunk -> lidar_frame (robot_state_publisher)

用法：
  ros2 run mujoco_sim slam_tf_bridge
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
import numpy as np


def quat_to_mat(q):
    """四元数 (x,y,z,w) -> 3x3 旋转矩阵

    输入会被归一化；近零四元数（degenerate）返回单位矩阵以避免除零。
    """
    x, y, z, w = q
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-10:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def mat_to_quat(R):
    """3x3 旋转矩阵 -> 四元数 (x,y,z,w)"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def tf_to_mat(tf):
    """TransformStamped -> (R, t)"""
    t = np.array([
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
    ])
    q = (
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w,
    )
    R = quat_to_mat(q)
    return R, t


class SlamTfBridge(Node):
    """将 Lightning-LM 的 odom 输出转换为 map->odom TF"""

    def __init__(self):
        super().__init__('slam_tf_bridge')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.declare_parameter('slam_odom_topic', 'lightning/odom')
        self.declare_parameter('control_odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        # 兜底 — 仅当 /lightning/odom 的 child_frame_id 为空时才使用。
        # 正常情况下源 frame 由消息自带，bridge 会跟随，无需在这里硬绑死 LiDAR/IMU。
        self.declare_parameter('lidar_frame', 'radar_uper_Link')
        self.declare_parameter('publish_odom_base_tf', True)

        self.slam_odom_topic = self.get_parameter(
            'slam_odom_topic').get_parameter_value().string_value
        self.control_odom_topic = self.get_parameter(
            'control_odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter(
            'odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.fallback_source_frame = self.get_parameter(
            'lidar_frame').get_parameter_value().string_value
        self.publish_odom_base_tf = self.get_parameter(
            'publish_odom_base_tf').get_parameter_value().bool_value

        # 订阅 Lightning-LM odom
        self.odom_sub = self.create_subscription(
            Odometry, self.slam_odom_topic, self._slam_odom_callback, 10)

        # 订阅控制器 odom，补齐 RViz 需要的 odom -> base TF。
        self.control_odom_sub = self.create_subscription(
            Odometry, self.control_odom_topic, self._control_odom_callback, 50)

        # 静态 base->source_frame 逆变换缓存。按 source frame 名做 key，这样 Lightning
        # 改 child_frame_id（例如以后从 radar_uper_Link 切到 imu_link）时自动重新查 TF。
        self._inv_static_tf = {}      # source_frame -> (R_inv, t_inv) 即 source->base
        self._T_odom_base = None       # (R, t)
        self._warned_no_static_tf = set()
        self._warned_no_odom_tf = False

        self.get_logger().info(
            f'SlamTfBridge: {self.slam_odom_topic} + '
            f'{self.control_odom_topic} -> TF(map->odom, odom->base)')

    def _lookup_inv_static_transform(self, source_frame):
        """获取 source_frame -> base 的静态变换（即 base->source_frame 的逆）

        Lightning 的 odom 把 pose 表达成 map -> child_frame_id（NavState 是 IMU body 状态，
        若以后 IMU/LiDAR 外参非零 Lightning 也可能改 child_frame_id 指向 IMU 或别的）。
        本节点完全跟随消息声明的 frame，靠 URDF 提供的静态 TF 把它带回 base，
        不再做"odom 一定是 LiDAR 位姿"的隐式假设。
        """
        if source_frame in self._inv_static_tf:
            return self._inv_static_tf[source_frame]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, rclpy.time.Time())
            R, t = tf_to_mat(tf)
            inv = (R.T, -R.T @ t)
            self._inv_static_tf[source_frame] = inv
            self.get_logger().info(
                f'cached static TF {self.base_frame}->{source_frame}: t={t}')
            return inv
        except Exception as exc:
            if source_frame not in self._warned_no_static_tf:
                self.get_logger().warn(
                    f'Waiting for static TF '
                    f'{self.base_frame}->{source_frame}: {exc}')
                self._warned_no_static_tf.add(source_frame)
            return None

    def _control_odom_callback(self, msg: Odometry):
        """收到 OCS2 /odom 后缓存并转发 odom -> base TF"""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        t = np.array([p.x, p.y, p.z])
        R = quat_to_mat((o.x, o.y, o.z, o.w))
        self._T_odom_base = (R, t)

        if not self.publish_odom_base_tf:
            return

        parent = msg.header.frame_id or self.odom_frame
        child = msg.child_frame_id or self.base_frame

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(p.x)
        tf_msg.transform.translation.y = float(p.y)
        tf_msg.transform.translation.z = float(p.z)
        tf_msg.transform.rotation.x = float(o.x)
        tf_msg.transform.rotation.y = float(o.y)
        tf_msg.transform.rotation.z = float(o.z)
        tf_msg.transform.rotation.w = float(o.w)
        self.tf_broadcaster.sendTransform(tf_msg)

    def _slam_odom_callback(self, msg: Odometry):
        """收到 Lightning-LM odom 后计算并发布 map->odom TF

        消息语义: pose 是 msg.header.frame_id (=map) 下的 msg.child_frame_id 位姿。
        我们查 source_frame -> base 的静态 TF (= base->source 的逆)，
        把 map->source 转成 map->base。
        """
        # 跟随 child_frame_id；为空时退回 launch 参数兜底。
        source_frame = msg.child_frame_id or self.fallback_source_frame
        inv_static = self._lookup_inv_static_transform(source_frame)
        if inv_static is None:
            return

        # --- map -> base ---
        # T_map_base = T_map_source * T_source_base
        # 其中 T_source_base = inv(T_base_source)，已在 inv_static 里
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t_map_source = np.array([p.x, p.y, p.z])
        R_map_source = quat_to_mat((o.x, o.y, o.z, o.w))

        R_source_base, t_source_base = inv_static
        R_map_base = R_map_source @ R_source_base
        t_map_base = R_map_source @ t_source_base + t_map_source

        # --- odom -> base (from OCS2 controller) ---
        if self._T_odom_base is not None:
            R_odom_base, t_odom_base = self._T_odom_base
        else:
            try:
                tf_odom_base = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, rclpy.time.Time())
                R_odom_base, t_odom_base = tf_to_mat(tf_odom_base)
            except Exception as exc:
                if not self._warned_no_odom_tf:
                    self.get_logger().warn(
                        f'Waiting for TF {self.odom_frame}->{self.base_frame}; '
                        f'skip map->odom until it is available: {exc}')
                    self._warned_no_odom_tf = True
                return

        # --- map -> odom = (map -> base) * inv(odom -> base) ---
        R_base_odom = R_odom_base.T
        t_base_odom = -R_odom_base.T @ t_odom_base

        R_map_odom = R_map_base @ R_base_odom
        t_map_odom = R_map_base @ t_base_odom + t_map_base

        self._publish_tf(msg.header.stamp, 'map', self.odom_frame,
                         R_map_odom, t_map_odom)

    def _publish_tf(self, stamp, parent, child, R, t):
        """发布 TF"""
        q = mat_to_quat(R)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
