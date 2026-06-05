import math
import unittest

from sensor_msgs.msg import Imu

from mujoco_sim.imu_converter import G, convert_imu_message


class ImuConverterTest(unittest.TestCase):
    def test_preserves_ros_standard_mps2_acceleration_by_default(self):
        msg = Imu()
        msg.header.frame_id = "radar"
        msg.linear_acceleration.x = 0.2
        msg.linear_acceleration.y = -0.3
        msg.linear_acceleration.z = 9.7

        out = convert_imu_message(msg)

        self.assertEqual(out.header.frame_id, "radar_uper_Link")
        self.assertTrue(math.isclose(out.linear_acceleration.x, 0.2))
        self.assertTrue(math.isclose(out.linear_acceleration.y, -0.3))
        self.assertTrue(math.isclose(out.linear_acceleration.z, 9.7))

    def test_can_scale_legacy_g_unit_acceleration(self):
        msg = Imu()
        msg.linear_acceleration.z = 1.0

        out = convert_imu_message(msg, linear_acceleration_scale=G)

        self.assertTrue(math.isclose(out.linear_acceleration.z, G))


if __name__ == "__main__":
    unittest.main()
