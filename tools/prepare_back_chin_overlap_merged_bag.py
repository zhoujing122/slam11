#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import TopicMetadata
from sensor_msgs.msg import Imu, PointCloud2

from mujoco_sim.pointcloud_merger_core import (
    SourcePointFilter,
    merge_pointclouds,
    pointcloud_timestamp_range,
    select_overlapping,
)


IDENTITY_ROTATION = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
ZERO_TRANSLATION = (0.0, 0.0, 0.0)


@dataclass
class CachedCloud:
    cloud: PointCloud2
    stamp_ns: int
    timestamp_min: float
    timestamp_max: float
    seq: int


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def stamp_float(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def time_msg_from_float(t: float):
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    from builtin_interfaces.msg import Time

    out = Time()
    out.sec = sec
    out.nanosec = nsec
    return out


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def open_reader(uri: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def open_writer(uri: str) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return writer


def create_topics(writer: rosbag2_py.SequentialWriter, args: argparse.Namespace, copy_tf_static: bool) -> None:
    topics = [
        (args.back_topic, "sensor_msgs/msg/PointCloud2"),
        (args.chin_topic, "sensor_msgs/msg/PointCloud2"),
        (args.chin_in_back_topic, "sensor_msgs/msg/PointCloud2"),
        (args.merged_topic, "sensor_msgs/msg/PointCloud2"),
        (args.output_imu_topic, "sensor_msgs/msg/Imu"),
    ]
    if args.copy_back_imu:
        topics.append((args.back_imu_topic, "sensor_msgs/msg/Imu"))
    if copy_tf_static:
        topics.append(("/tf_static", "tf2_msgs/msg/TFMessage"))
    for name, msg_type in topics:
        writer.create_topic(TopicMetadata(name=name, type=msg_type, serialization_format="cdr"))


def cached_cloud(msg: PointCloud2, seq: int) -> CachedCloud:
    timestamp_min, timestamp_max = pointcloud_timestamp_range(msg)
    return CachedCloud(msg, stamp_ns(msg), timestamp_min, timestamp_max, seq)


def make_cloud_msg(template: PointCloud2, merged, frame_id: str = "radar_uper_Link") -> PointCloud2:
    out = PointCloud2()
    out.header = template.header
    out.header.stamp = time_msg_from_float(merged.min_timestamp)
    out.header.frame_id = frame_id
    out.height = merged.height
    out.width = merged.width
    out.fields = merged.fields
    out.is_bigendian = merged.is_bigendian
    out.point_step = merged.point_step
    out.row_step = merged.row_step
    out.data = merged.data
    out.is_dense = merged.is_dense
    return out


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prepare a back+chin merged bag using the same per-point timestamp-window semantics as pointcloud_merger."
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--back-topic", default="/LIDAR/POINTS")
    parser.add_argument("--chin-topic", default="/chin/LIDAR/POINTS")
    parser.add_argument("--chin-in-back-topic", default="/chin_in_back/LIDAR/POINTS")
    parser.add_argument("--merged-topic", default="/merged/LIDAR/POINTS")
    parser.add_argument("--back-imu-topic", default="/rslidar_imu_data")
    parser.add_argument("--output-imu-topic", default="/imu/data")
    parser.add_argument("--copy-back-imu", action="store_true")
    parser.add_argument("--copy-tf-static", action="store_true")
    parser.add_argument("--translation", nargs=3, type=float, default=(0.34353, 0.000159742, -0.258107))
    parser.add_argument("--rpy", nargs=3, type=float, default=(-3.1416, -0.69813, 3.1416), help="URDF radar_f_Link -> radar_uper_Link RPY, radians")
    parser.add_argument("--chin-driver-to-link-rpy-deg", nargs=3, type=float, default=(0.0, 0.0, 180.0), help="Same driver-frame correction used by pointcloud_merger")
    parser.add_argument(
        "--chin-self-exclude-box-back-frame",
        nargs=6,
        type=float,
        metavar=("MIN_X", "MAX_X", "MIN_Y", "MAX_Y", "MIN_Z", "MAX_Z"),
        help="Drop transformed chin points inside this radar_uper_Link box before merging; disabled unless explicitly set",
    )
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)

    tf_rotation = rotation_from_rpy(*args.rpy)
    driver_rotation = rotation_from_rpy(*(math.radians(x) for x in args.chin_driver_to_link_rpy_deg))
    chin_rotation = tf_rotation @ driver_rotation
    chin_translation = tuple(float(x) for x in args.translation)
    chin_filter = None
    if args.chin_self_exclude_box_back_frame is not None:
        min_x, max_x, min_y, max_y, min_z, max_z = args.chin_self_exclude_box_back_frame
        chin_filter = SourcePointFilter(
            exclude_min_x=min_x,
            exclude_max_x=max_x,
            exclude_min_y=min_y,
            exclude_max_y=max_y,
            exclude_min_z=min_z,
            exclude_max_z=max_z,
        )

    rclpy.init(args=None)
    reader = open_reader(args.input)
    topic_names = {topic.name for topic in reader.get_all_topics_and_types()}
    missing_topics = [topic for topic in (args.back_topic, args.chin_topic, args.back_imu_topic) if topic not in topic_names]
    if missing_topics:
        raise RuntimeError(f"input bag is missing required topics: {missing_topics}")

    writer = open_writer(str(output))
    create_topics(writer, args, copy_tf_static=args.copy_tf_static and "/tf_static" in topic_names)

    chin_cache: deque[CachedCloud] = deque()
    pending_back: deque[CachedCloud] = deque()
    next_back_seq = 0
    next_chin_seq = 0
    current_chin_max = None

    counts = {
        args.back_topic: 0,
        args.chin_topic: 0,
        args.chin_in_back_topic: 0,
        args.merged_topic: 0,
        args.output_imu_topic: 0,
    }
    if args.copy_back_imu:
        counts[args.back_imu_topic] = 0
    if args.copy_tf_static:
        counts["/tf_static"] = 0

    overlap_frames = 0
    back_only_frames = 0
    source1_points = 0
    source1_points_before_filter = 0
    source1_self_filtered_points = 0
    source0_points = 0
    overlap_chin_cloud_refs = 0
    dt_mid_ms: list[float] = []

    def emit_back(back: CachedCloud, force: bool = False) -> bool:
        nonlocal overlap_frames, back_only_frames, source1_points, source1_points_before_filter
        nonlocal source1_self_filtered_points, source0_points, overlap_chin_cloud_refs
        nonlocal current_chin_max
        window = (back.timestamp_min, back.timestamp_max)
        chin_matches = select_overlapping(chin_cache, window)
        if not chin_matches:
            if not force and (current_chin_max is None or current_chin_max < back.timestamp_max):
                return False
            back_only_frames += 1
        else:
            overlap_frames += 1
            overlap_chin_cloud_refs += len(chin_matches)
            back_mid = 0.5 * (back.timestamp_min + back.timestamp_max)
            for chin in chin_matches:
                chin_mid = 0.5 * (chin.timestamp_min + chin.timestamp_max)
                dt_mid_ms.append((chin_mid - back_mid) * 1000.0)

        transforms = [
            (back.cloud, IDENTITY_ROTATION, ZERO_TRANSLATION, ZERO_TRANSLATION, None, None, 0.0),
        ]
        for chin in chin_matches:
            transforms.append((chin.cloud, chin_rotation, chin_translation, ZERO_TRANSLATION, None, chin_filter, 1.0))

        merged = merge_pointclouds(transforms, timestamp_window=window)
        if chin_matches and chin_filter is not None:
            unfiltered_chin = merge_pointclouds(
                [(chin.cloud, chin_rotation, chin_translation, ZERO_TRANSLATION, None, None, 1.0) for chin in chin_matches],
                timestamp_window=window,
            )
            filtered_chin_count = 0
            if len(merged.source_point_counts) >= 2:
                filtered_chin_count = int(sum(merged.source_point_counts[1:]))
            unfiltered_chin_count = int(sum(unfiltered_chin.source_point_counts))
            source1_points_before_filter += unfiltered_chin_count
            source1_self_filtered_points += max(0, unfiltered_chin_count - filtered_chin_count)
        if len(merged.source_point_counts) >= 1:
            source0_points += int(merged.source_point_counts[0])
        if len(merged.source_point_counts) >= 2:
            source1_points += int(sum(merged.source_point_counts[1:]))

        merged_msg = make_cloud_msg(back.cloud, merged)
        storage_time = stamp_ns(merged_msg)
        writer.write(args.merged_topic, serialize_message(merged_msg), storage_time)
        counts[args.merged_topic] += 1

        if chin_matches:
            chin_only_merged = merge_pointclouds(
                [(chin.cloud, chin_rotation, chin_translation, ZERO_TRANSLATION, None, chin_filter, 1.0) for chin in chin_matches],
                timestamp_window=window,
            )
            if chin_only_merged.width > 0:
                chin_msg = make_cloud_msg(back.cloud, chin_only_merged)
                writer.write(args.chin_in_back_topic, serialize_message(chin_msg), stamp_ns(chin_msg))
                counts[args.chin_in_back_topic] += 1
        return True

    def flush_ready(force: bool = False) -> None:
        while pending_back:
            if not emit_back(pending_back[0], force=force):
                return
            pending_back.popleft()
            if pending_back:
                earliest = pending_back[0].timestamp_min
                while len(chin_cache) > 1 and chin_cache[0].timestamp_max < earliest:
                    chin_cache.popleft()

    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        if topic == "/tf_static" and args.copy_tf_static:
            writer.write(topic, data, storage_time)
            counts["/tf_static"] += 1
            continue
        if topic == args.back_imu_topic:
            imu = deserialize_message(data, Imu)
            imu.header.frame_id = "radar_uper_Link"
            t = stamp_ns(imu)
            writer.write(args.output_imu_topic, serialize_message(imu), t)
            counts[args.output_imu_topic] += 1
            if args.copy_back_imu:
                writer.write(args.back_imu_topic, serialize_message(imu), t)
                counts[args.back_imu_topic] += 1
            continue
        if topic == args.chin_topic:
            chin = deserialize_message(data, PointCloud2)
            writer.write(args.chin_topic, data, stamp_ns(chin))
            counts[args.chin_topic] += 1
            cached = cached_cloud(chin, next_chin_seq)
            next_chin_seq += 1
            current_chin_max = cached.timestamp_max if current_chin_max is None else max(current_chin_max, cached.timestamp_max)
            chin_cache.append(cached)
            flush_ready(force=False)
            continue
        if topic == args.back_topic:
            back = deserialize_message(data, PointCloud2)
            writer.write(args.back_topic, data, stamp_ns(back))
            counts[args.back_topic] += 1
            pending_back.append(cached_cloud(back, next_back_seq))
            next_back_seq += 1
            flush_ready(force=False)
            continue

    flush_ready(force=True)
    rclpy.shutdown()

    if counts[args.back_topic] == 0 or counts[args.chin_topic] == 0:
        raise RuntimeError(f"empty input topics: back={counts[args.back_topic]}, chin={counts[args.chin_topic]}")
    if counts[args.merged_topic] != counts[args.back_topic]:
        raise RuntimeError(f"merged/back frame mismatch: merged={counts[args.merged_topic]}, back={counts[args.back_topic]}")
    if overlap_frames == 0 or source1_points == 0:
        raise RuntimeError(f"prepared bag is not back+chin: overlap_frames={overlap_frames}, source1_points={source1_points}")

    summary = {
        "input_bag": args.input,
        "output_bag": str(output),
        "merge_semantics": "back master frame; side LiDAR selected by per-point timestamp-window overlap; late side frames waited offline until chin timestamp max covers back window; side points cropped to back window; RoboSense per-point timestamp preserved; merged points sorted by timestamp for LIO deskew",
        "topics": {
            "back_cloud": args.back_topic,
            "chin_cloud": args.chin_topic,
            "chin_in_back_cloud": args.chin_in_back_topic,
            "merged_cloud": args.merged_topic,
            "back_imu_in": args.back_imu_topic,
            "imu_out": args.output_imu_topic,
        },
        "extrinsic": {
            "source_frame": "radar_f_Link",
            "target_frame": "radar_uper_Link",
            "urdf_translation_xyz_m": [float(x) for x in args.translation],
            "urdf_rpy_rad": [float(x) for x in args.rpy],
            "driver_to_link_rpy_deg": [float(x) for x in args.chin_driver_to_link_rpy_deg],
            "effective_rotation_matrix": np.asarray(chin_rotation).tolist(),
        },
        "chin_self_filter": {
            "enabled": chin_filter is not None,
            "box_frame": "radar_uper_Link",
            "exclude_box_xyz_m": [float(x) for x in args.chin_self_exclude_box_back_frame]
            if args.chin_self_exclude_box_back_frame is not None else None,
            "source1_points_before_filter": source1_points_before_filter if chin_filter is not None else None,
            "source1_self_filtered_points": source1_self_filtered_points if chin_filter is not None else None,
            "source1_points_after_filter": source1_points if chin_filter is not None else None,
        },
        "counts": counts,
        "back_frames": counts[args.back_topic],
        "chin_frames": counts[args.chin_topic],
        "merged_frames": counts[args.merged_topic],
        "overlap_frames": overlap_frames,
        "back_only_frames": back_only_frames,
        "overlap_ratio": overlap_frames / max(1, counts[args.back_topic]),
        "overlap_chin_cloud_refs": overlap_chin_cloud_refs,
        "source0_points_in_merged": source0_points,
        "source1_points_in_merged": source1_points,
        "dt_mid_median_ms": percentile(dt_mid_ms, 50),
        "dt_mid_p95_abs_ms": percentile([abs(x) for x in dt_mid_ms], 95),
        "dt_mid_max_abs_ms": max([abs(x) for x in dt_mid_ms], default=None),
    }
    summary_path = output / "prepare_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
