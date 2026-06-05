"""Publish synthetic D100 three-LiDAR PointCloud2 inputs for merger testing."""

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


POINT_STEP = 26
POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]


class FakeLidarTripletPublisher(Node):
    def __init__(self):
        super().__init__("fake_lidar_triplet_publisher")
        self.declare_parameter("rate_hz", 10.0)

        self.back_pub = self.create_publisher(PointCloud2, "/LIDAR/POINTS", 10)
        self.chin_pub = self.create_publisher(PointCloud2, "/chin/LIDAR/POINTS", 10)
        self.tail_pub = self.create_publisher(PointCloud2, "/tail/LIDAR/POINTS", 10)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_clouds)
        self.get_logger().info(
            "fake_lidar_triplet_publisher: publishing /LIDAR/POINTS, "
            "/chin/LIDAR/POINTS, /tail/LIDAR/POINTS with static TFs"
        )

    def _publish_static_transforms(self):
        stamp = self.get_clock().now().to_msg()
        transforms = [
            _make_transform(
                stamp,
                "radar_uper_Link",
                "radar_f_Link",
	                0.34353,
	                0.000159742,
	                -0.258107,
	                -3.1416,
	                -0.69813,
	                -3.1416,
	            ),
            _make_transform(
                stamp,
                "radar_uper_Link",
                "radar_r_Link",
	                -0.7725,
	                0.000020892,
	                -0.2111381,
	                -3.1416,
	                1.3788,
	                3.1416,
	            ),
        ]
        self.tf_broadcaster.sendTransform(transforms)

    def _publish_clouds(self):
        stamp = self.get_clock().now().to_msg()
        base_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        self.chin_pub.publish(_make_cloud("radar_f_Link", stamp, base_time, 10.0))
        self.tail_pub.publish(_make_cloud("radar_r_Link", stamp, base_time, 20.0))
        self.back_pub.publish(_make_cloud("radar_uper_Link", stamp, base_time, 0.0))


def _make_cloud(frame_id, stamp, base_time, x_offset):
    points = [
        (1.0 + x_offset, 0.0, 0.0, 10.0, 0, base_time),
        (1.0 + x_offset, 0.5, 0.1, 20.0, 1, base_time + 0.001),
        (1.0 + x_offset, -0.5, -0.1, 30.0, 2, base_time + 0.002),
    ]
    data = bytearray(len(points) * POINT_STEP)
    for i, point in enumerate(points):
        offset = i * POINT_STEP
        x, y, z, intensity, ring, timestamp = point
        struct.pack_into("<fff", data, offset, x, y, z)
        struct.pack_into("<f", data, offset + 12, intensity)
        struct.pack_into("<H", data, offset + 16, ring)
        struct.pack_into("<d", data, offset + 18, timestamp)

    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = len(points)
    msg.fields = POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.row_step = len(points) * POINT_STEP
    msg.data = bytes(data)
    msg.is_dense = True
    return msg


def _make_transform(stamp, parent, child, x, y, z, roll, pitch, yaw):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    qx, qy, qz, qw = _quaternion_from_euler(roll, pitch, yaw)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def _quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarTripletPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
