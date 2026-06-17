import unittest

import numpy as np

from mujoco_sim.dynamic_self_filter import (
    BoxShape,
    CompositePointFilter,
    make_dynamic_self_filter,
)
from mujoco_sim.pointcloud_merger_core import SourcePointFilter


class DynamicSelfFilterTest(unittest.TestCase):
    def test_filters_points_inside_dynamic_box(self):
        shape = BoxShape(
            name="trunk",
            frame="trunk",
            center=np.zeros(3),
            half_size=np.array([0.5, 0.5, 0.5], dtype=np.float64),
        )

        def lookup(frame):
            self.assertEqual(frame, "trunk")
            return np.eye(3), np.array([1.0, 0.0, 0.0], dtype=np.float64)

        stats = {}
        point_filter = make_dynamic_self_filter([shape], lookup, stats=stats)
        xyz = np.array(
            [
                [1.1, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )

        self.assertEqual(point_filter.keep_mask(xyz).tolist(), [False, True])
        self.assertEqual(stats["trunk"], 1)


    def test_reports_requested_active_and_missing_shapes(self):
        present = BoxShape(
            name="present",
            frame="present",
            center=np.zeros(3),
            half_size=np.array([0.5, 0.5, 0.5], dtype=np.float64),
        )
        missing = BoxShape(
            name="missing",
            frame="missing",
            center=np.zeros(3),
            half_size=np.array([0.5, 0.5, 0.5], dtype=np.float64),
        )

        def lookup(frame):
            if frame == "present":
                return np.eye(3), np.zeros(3, dtype=np.float64)
            return None

        point_filter = make_dynamic_self_filter([present, missing], lookup)

        self.assertEqual(point_filter.requested_count, 2)
        self.assertEqual(point_filter.active_count, 1)
        self.assertEqual(point_filter.missing_frames, ("missing",))

    def test_composite_filter_applies_static_and_dynamic_filters(self):
        static_filter = SourcePointFilter(min_x=0.0)
        dynamic_filter = SourcePointFilter(exclude_min_x=0.8, exclude_max_x=1.2,
                                           exclude_min_y=-0.1, exclude_max_y=0.1,
                                           exclude_min_z=-0.1, exclude_max_z=0.1)
        point_filter = CompositePointFilter([static_filter, dynamic_filter])
        xyz = np.array(
            [
                [-1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )

        self.assertEqual(point_filter.keep_mask(xyz).tolist(), [False, False, True])


if __name__ == "__main__":
    unittest.main()
