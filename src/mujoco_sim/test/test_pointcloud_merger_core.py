import math
import struct
import unittest
from types import SimpleNamespace

import mujoco_sim.pointcloud_merger_core as merger_core
from mujoco_sim.pointcloud_merger_core import (
    FLOAT32,
    FLOAT64,
    SourcePointFilter,
    UINT16,
    merge_pointclouds,
    pointcloud_timestamp_range,
    select_nearest,
    select_overlapping,
    should_wait_for_overlapping_clouds,
    should_wait_for_side_clouds,
)


POINT_FIELDS = [
    SimpleNamespace(name="x", offset=0, datatype=FLOAT32, count=1),
    SimpleNamespace(name="y", offset=4, datatype=FLOAT32, count=1),
    SimpleNamespace(name="z", offset=8, datatype=FLOAT32, count=1),
    SimpleNamespace(name="intensity", offset=12, datatype=FLOAT32, count=1),
    SimpleNamespace(name="ring", offset=16, datatype=UINT16, count=1),
    SimpleNamespace(name="timestamp", offset=18, datatype=FLOAT64, count=1),
]
POINT_STEP = 26


def make_cloud(points):
    data = bytearray(len(points) * POINT_STEP)
    for i, point in enumerate(points):
        offset = i * POINT_STEP
        struct.pack_into("<fff", data, offset, point["x"], point["y"], point["z"])
        struct.pack_into("<f", data, offset + 12, point["intensity"])
        struct.pack_into("<H", data, offset + 16, point["ring"])
        struct.pack_into("<d", data, offset + 18, point["timestamp"])

    return SimpleNamespace(
        height=1,
        width=len(points),
        fields=POINT_FIELDS,
        is_bigendian=False,
        point_step=POINT_STEP,
        row_step=len(points) * POINT_STEP,
        data=bytes(data),
        is_dense=True,
    )


def unpack_point(data, index):
    offset = index * POINT_STEP
    x, y, z = struct.unpack_from("<fff", data, offset)
    intensity = struct.unpack_from("<f", data, offset + 12)[0]
    ring = struct.unpack_from("<H", data, offset + 16)[0]
    timestamp = struct.unpack_from("<d", data, offset + 18)[0]
    return x, y, z, intensity, ring, timestamp


class PointCloudMergerCoreTest(unittest.TestCase):
    def test_merge_transforms_xyz_and_preserves_robosense_fields(self):
        back = make_cloud([
            {"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 10.0, "ring": 4, "timestamp": 100.02},
        ])
        chin = make_cloud([
            {"x": 2.0, "y": 0.0, "z": 1.0, "intensity": 20.0, "ring": 8, "timestamp": 99.98},
        ])
        tail = make_cloud([
            {"x": 0.5, "y": -1.0, "z": 0.0, "intensity": 30.0, "ring": 12, "timestamp": 100.04},
        ])

        merged = merge_pointclouds([
            (back, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0)),
            (chin, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (10.0, 0.0, 0.0)),
            (tail, ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 5.0, 0.0)),
        ])

        self.assertEqual(merged.width, 3)
        self.assertEqual(merged.row_step, POINT_STEP * 3)
        self.assertEqual(merged.point_step, POINT_STEP)
        self.assertTrue(math.isclose(merged.min_timestamp, 99.98))

        self.assertEqual(unpack_point(merged.data, 0), (12.0, 0.0, 1.0, 20.0, 8, 99.98))
        self.assertEqual(unpack_point(merged.data, 1), (1.0, 2.0, 3.0, 10.0, 4, 100.02))
        self.assertEqual(unpack_point(merged.data, 2), (1.0, 5.5, 0.0, 30.0, 12, 100.04))

    def test_merge_can_append_source_id_field(self):
        back = make_cloud([
            {"x": 0.0, "y": 0.0, "z": 0.0, "intensity": 1.0, "ring": 0, "timestamp": 100.00},
        ])
        chin = make_cloud([
            {"x": 1.0, "y": 0.0, "z": 0.0, "intensity": 2.0, "ring": 1, "timestamp": 99.99},
        ])

        merged = merge_pointclouds([
            (back, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
             (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None, None, 0.0),
            (chin, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
             (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None, None, 1.0),
        ])

        self.assertEqual(merged.width, 2)
        self.assertEqual(merged.point_step, POINT_STEP + 4)
        self.assertEqual(merged.row_step, (POINT_STEP + 4) * 2)
        self.assertEqual(merged.fields[-1].name, "source_id")
        self.assertEqual(merged.fields[-1].offset, POINT_STEP)
        self.assertEqual(merged.fields[-1].datatype, FLOAT32)
        self.assertEqual(struct.unpack_from("<f", merged.data, POINT_STEP)[0], 1.0)
        self.assertEqual(struct.unpack_from("<f", merged.data, POINT_STEP + 4 + POINT_STEP)[0], 0.0)

    def test_merge_can_clip_to_reference_timestamp_window(self):
        back = make_cloud([
            {"x": 0.0, "y": 0.0, "z": 0.0, "intensity": 1.0, "ring": 0, "timestamp": 100.00},
            {"x": 1.0, "y": 0.0, "z": 0.0, "intensity": 2.0, "ring": 0, "timestamp": 100.10},
        ])
        chin = make_cloud([
            {"x": 2.0, "y": 0.0, "z": 0.0, "intensity": 3.0, "ring": 0, "timestamp": 99.95},
            {"x": 3.0, "y": 0.0, "z": 0.0, "intensity": 4.0, "ring": 0, "timestamp": 100.05},
            {"x": 4.0, "y": 0.0, "z": 0.0, "intensity": 5.0, "ring": 0, "timestamp": 100.15},
        ])

        merged = merge_pointclouds([
            (back, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0)),
            (chin, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0)),
        ], timestamp_window=pointcloud_timestamp_range(back))

        self.assertEqual(merged.width, 3)
        self.assertEqual(merged.source_point_counts, (2, 1))
        self.assertTrue(math.isclose(merged.min_timestamp, 100.00))
        self.assertTrue(math.isclose(merged.max_timestamp, 100.10))
        self.assertEqual([unpack_point(merged.data, i)[5] for i in range(merged.width)],
                         [100.00, 100.05, 100.10])

    def test_merge_applies_extra_translation_after_frame_transform(self):
        cloud = make_cloud([
            {"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 10.0, "ring": 4, "timestamp": 100.00},
        ])

        merged = merge_pointclouds([
            (
                cloud,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                (10.0, 20.0, 30.0),
                (0.1, 0.2, 0.3),
            ),
        ])

        self.assertEqual(unpack_point(merged.data, 0),
                         (11.100000381469727, 22.200000762939453, 33.29999923706055, 10.0, 4, 100.00))

    def test_merge_applies_extra_rotation_after_frame_transform(self):
        cloud = make_cloud([
            {"x": 1.0, "y": 0.0, "z": 0.0, "intensity": 10.0, "ring": 4, "timestamp": 100.00},
        ])

        merged = merge_pointclouds([
            (
                cloud,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            ),
        ])

        self.assertEqual(unpack_point(merged.data, 0),
                         (0.0, 1.0, 0.0, 10.0, 4, 100.00))

    def test_extra_rotation_does_not_rotate_sensor_translation(self):
        cloud = make_cloud([
            {"x": 0.0, "y": 0.0, "z": 0.0, "intensity": 10.0, "ring": 4, "timestamp": 100.00},
        ])

        merged = merge_pointclouds([
            (
                cloud,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                (10.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            ),
        ])

        self.assertEqual(unpack_point(merged.data, 0),
                         (10.0, 0.0, 0.0, 10.0, 4, 100.00))

    def test_filter_is_applied_after_transform_in_target_frame(self):
        cloud = make_cloud([
            {"x": 0.0, "y": 0.0, "z": 0.0, "intensity": 10.0, "ring": 4, "timestamp": 100.00},
            {"x": -2.0, "y": 0.0, "z": 0.0, "intensity": 20.0, "ring": 5, "timestamp": 100.01},
        ])

        merged = merge_pointclouds([
            (
                cloud,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                (2.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                SourcePointFilter(min_x=1.5),
            ),
        ])

        self.assertEqual(merged.width, 1)
        self.assertEqual(merged.source_point_counts, (1,))
        self.assertEqual(unpack_point(merged.data, 0),
                         (2.0, 0.0, 0.0, 10.0, 4, 100.00))

    def test_select_overlapping_returns_all_clouds_that_intersect_window(self):
        older = SimpleNamespace(stamp=9.9, timestamp_min=9.90, timestamp_max=9.99)
        first_overlap = SimpleNamespace(stamp=10.0, timestamp_min=9.98, timestamp_max=10.08)
        second_overlap = SimpleNamespace(stamp=10.1, timestamp_min=10.08, timestamp_max=10.18)
        newer = SimpleNamespace(stamp=10.2, timestamp_min=10.21, timestamp_max=10.30)

        matches = select_overlapping(
            [older, first_overlap, second_overlap, newer],
            (10.00, 10.10),
        )

        self.assertEqual(matches, [first_overlap, second_overlap])

    def test_waits_for_overlapping_cloud_until_side_window_passes_back_window(self):
        timestamp_window = (10.00, 10.10)
        chin_cache = [SimpleNamespace(timestamp_min=9.90, timestamp_max=9.99)]
        tail_cache = [SimpleNamespace(timestamp_min=9.95, timestamp_max=10.08)]

        self.assertTrue(
            should_wait_for_overlapping_clouds(
                timestamp_window,
                ["chin"],
                chin_cache,
                tail_cache,
            )
        )

        chin_cache.append(SimpleNamespace(timestamp_min=10.11, timestamp_max=10.20))

        self.assertFalse(
            should_wait_for_overlapping_clouds(
                timestamp_window,
                ["chin"],
                chin_cache,
                tail_cache,
            )
        )

    def test_select_nearest_requires_within_tolerance(self):
        older = SimpleNamespace(stamp=10.00)
        nearest = SimpleNamespace(stamp=10.03)
        too_new = SimpleNamespace(stamp=10.20)

        self.assertIs(select_nearest([older, nearest, too_new], 10.05, 0.05), nearest)
        self.assertIsNone(select_nearest([older, nearest, too_new], 10.30, 0.05))

    def test_select_available_clouds_keeps_back_when_side_lidar_missing(self):
        back = SimpleNamespace(stamp=10.0)
        chin = None
        tail = SimpleNamespace(stamp=10.04)

        clouds, missing, sync_dt = merger_core.select_available_clouds(back, chin, tail)

        self.assertEqual(clouds, [back, tail])
        self.assertEqual(missing, ["chin"])
        self.assertTrue(math.isclose(sync_dt, 0.04))

    def test_select_available_clouds_can_require_all_three_lidars(self):
        back = SimpleNamespace(stamp=10.0)
        chin = SimpleNamespace(stamp=10.02)
        tail = None

        clouds, missing, sync_dt = merger_core.select_available_clouds(
            back, chin, tail, allow_back_only_fallback=False
        )

        self.assertEqual(clouds, [])
        self.assertEqual(missing, ["tail"])
        self.assertTrue(math.isclose(sync_dt, 0.02))

    def test_waits_for_missing_side_cloud_until_cache_passes_match_window(self):
        back_stamp = 10.0
        chin_cache = [SimpleNamespace(stamp=9.95)]
        tail_cache = [SimpleNamespace(stamp=9.98)]

        self.assertTrue(
            should_wait_for_side_clouds(
                back_stamp,
                ["chin"],
                chin_cache,
                tail_cache,
                tolerance_s=0.08,
            )
        )

        chin_cache.append(SimpleNamespace(stamp=10.10))

        self.assertFalse(
            should_wait_for_side_clouds(
                back_stamp,
                ["chin"],
                chin_cache,
                tail_cache,
                tolerance_s=0.08,
            )
        )


if __name__ == "__main__":
    unittest.main()
