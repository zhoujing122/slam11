#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import shutil
import tempfile
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import rclpy
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import TopicMetadata
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, JointState, PointCloud2
from tf2_msgs.msg import TFMessage

from mujoco_sim.dynamic_self_filter import (
    load_shapes as load_self_filter_shapes,
    make_dynamic_self_filter,
)
from mujoco_sim.pointcloud_merger_core import (
    merge_pointclouds,
    pointcloud_timestamp_range,
    select_overlapping,
)


IDENTITY_ROTATION = np.eye(3, dtype=np.float64)
ZERO_TRANSLATION = np.zeros(3, dtype=np.float64)

SOURCE_BACK = 0.0
SOURCE_CHIN = 1.0
SOURCE_TAIL = 2.0


@dataclass
class CachedCloud:
    cloud: PointCloud2
    stamp_ns: int
    timestamp_min: float
    timestamp_max: float
    seq: int



@dataclass
class TransformSample:
    stamp_ns: int
    parent: str
    child: str
    rotation: np.ndarray
    translation: np.ndarray


class DynamicTfBuffer:
    def __init__(self) -> None:
        self._static: dict[tuple[str, str], TransformSample] = {}
        self._dynamic: dict[tuple[str, str], list[TransformSample]] = defaultdict(list)

    def add(self, transform: TransformStamped, is_static: bool) -> None:
        parent = transform.header.frame_id
        child = transform.child_frame_id
        stamp = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
        sample = TransformSample(
            stamp_ns=stamp,
            parent=parent,
            child=child,
            rotation=quat_to_matrix(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
            translation=np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                dtype=np.float64,
            ),
        )
        key = (parent, child)
        if is_static:
            self._static[key] = sample
        else:
            self._dynamic[key].append(sample)

    def finalize(self) -> None:
        for samples in self._dynamic.values():
            samples.sort(key=lambda s: s.stamp_ns)

    def graph_at(self, stamp_ns: int) -> dict[str, list[tuple[str, bool, TransformSample]]]:
        graph: dict[str, list[tuple[str, bool, TransformSample]]] = defaultdict(list)
        for key, sample in self._static.items():
            parent, child = key
            graph[parent].append((child, False, sample))
            graph[child].append((parent, True, sample))
        for key, samples in self._dynamic.items():
            if not samples:
                continue
            sample = nearest_sample(samples, stamp_ns)
            parent, child = key
            graph[parent].append((child, False, sample))
            graph[child].append((parent, True, sample))
        return graph

    def lookup(self, target: str, source: str, stamp_ns: int) -> tuple[np.ndarray, np.ndarray]:
        return self.lookup_in_graph(self.graph_at(stamp_ns), target, source)

    def lookup_in_graph(self, graph: dict[str, list[tuple[str, bool, TransformSample]]], target: str, source: str) -> tuple[np.ndarray, np.ndarray]:
        if target == source:
            return IDENTITY_ROTATION, ZERO_TRANSLATION
        queue = deque([(target, IDENTITY_ROTATION, ZERO_TRANSLATION)])
        seen = {target}
        while queue:
            frame, rot_target_frame, trans_target_frame = queue.popleft()
            for nxt, inverted, sample in graph.get(frame, []):
                if nxt in seen:
                    continue
                if inverted:
                    # sample is parent -> child. We are at child and step to parent.
                    edge_rot = sample.rotation.T
                    edge_trans = -(sample.rotation.T @ sample.translation)
                else:
                    # sample is parent -> child. We are at parent and step to child.
                    edge_rot = sample.rotation
                    edge_trans = sample.translation
                rot_target_next = rot_target_frame @ edge_rot
                trans_target_next = rot_target_frame @ edge_trans + trans_target_frame
                if nxt == source:
                    return rot_target_next, trans_target_next
                seen.add(nxt)
                queue.append((nxt, rot_target_next, trans_target_next))
        raise KeyError(f"no TF path {target} <- {source}")


def nearest_sample(samples: list[TransformSample], stamp_ns: int) -> TransformSample:
    if len(samples) == 1:
        return samples[0]
    lo = 0
    hi = len(samples)
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid].stamp_ns < stamp_ns:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return samples[0]
    if lo >= len(samples):
        return samples[-1]
    before = samples[lo - 1]
    after = samples[lo]
    return before if abs(before.stamp_ns - stamp_ns) <= abs(after.stamp_ns - stamp_ns) else after


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return IDENTITY_ROTATION.copy()
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, cp * 0.0 + sp], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def stamp_ns_msg(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def stamp_float_msg(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def time_msg_from_float(t: float) -> Time:
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    out = Time()
    out.sec = sec
    out.nanosec = nsec
    return out


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


def create_topics(writer: rosbag2_py.SequentialWriter, args: argparse.Namespace, topic_names: set[str]) -> None:
    topics = [
        (args.merged_topic, "sensor_msgs/msg/PointCloud2"),
        (args.output_imu_topic, "sensor_msgs/msg/Imu"),
    ]
    if args.copy_source_clouds:
        topics.extend([
            (args.back_topic, "sensor_msgs/msg/PointCloud2"),
            (args.chin_topic, "sensor_msgs/msg/PointCloud2"),
            (args.tail_topic, "sensor_msgs/msg/PointCloud2"),
        ])
    if args.copy_original_back_imu:
        topics.append((args.back_imu_topic, "sensor_msgs/msg/Imu"))
    if args.copy_tf and "/tf" in topic_names:
        topics.append(("/tf", "tf2_msgs/msg/TFMessage"))
    if args.copy_tf and "/tf_static" in topic_names:
        topics.append(("/tf_static", "tf2_msgs/msg/TFMessage"))
    if args.copy_joint_states and "/joint_states" in topic_names:
        topics.append(("/joint_states", "sensor_msgs/msg/JointState"))
    created = set()
    for name, msg_type in topics:
        if name in created:
            continue
        writer.create_topic(TopicMetadata(name=name, type=msg_type, serialization_format="cdr"))
        created.add(name)


def cached_cloud(msg: PointCloud2, seq: int) -> CachedCloud:
    timestamp_min, timestamp_max = pointcloud_timestamp_range(msg)
    return CachedCloud(msg, stamp_ns_msg(msg), timestamp_min, timestamp_max, seq)


def make_cloud_msg(template: PointCloud2, merged, frame_id: str) -> PointCloud2:
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


def make_box_filter(shapes, tf_buffer: DynamicTfBuffer, target_frame: str, stamp_ns: int, stats: dict[str, int]):
    graph = tf_buffer.graph_at(stamp_ns)

    def lookup_shape(shape_frame: str):
        try:
            return tf_buffer.lookup_in_graph(graph, target_frame, shape_frame)
        except KeyError:
            return None

    return make_dynamic_self_filter(shapes, lookup_shape, stats=stats)



def unwrap_uncompressed_bag_if_needed(input_uri: str) -> tuple[str, tempfile.TemporaryDirectory | None]:
    path = Path(input_uri)
    metadata = path / "metadata.yaml"
    if not metadata.exists():
        return input_uri, None
    text = metadata.read_text()
    if ".db3.zstd" not in text:
        return input_uri, None
    tmp = tempfile.TemporaryDirectory(prefix="bag_uncompressed_")
    tmp_path = Path(tmp.name)
    (tmp_path / "metadata.yaml").write_text(text.replace(".db3.zstd", ".db3"))
    for db in path.glob("*.db3"):
        (tmp_path / db.name).symlink_to(db.resolve())
    return str(tmp_path), tmp


def read_tf_buffer(input_uri: str) -> tuple[DynamicTfBuffer, dict[str, str], set[str]]:
    reader = open_reader(input_uri)
    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    topic_names = set(topic_types)
    tf_buffer = DynamicTfBuffer()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ("/tf", "/tf_static"):
            continue
        msg = deserialize_message(data, get_message(topic_types[topic]))
        for transform in msg.transforms:
            tf_buffer.add(transform, is_static=(topic == "/tf_static"))
    tf_buffer.finalize()
    return tf_buffer, topic_types, topic_names


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare a three-LiDAR merged bag with dynamic robot self filtering.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--target-frame", default="radar_uper_Link")
    parser.add_argument("--back-topic", default="/LIDAR/POINTS")
    parser.add_argument("--chin-topic", default="/chin/LIDAR/POINTS")
    parser.add_argument("--tail-topic", default="/tail/LIDAR/POINTS")
    parser.add_argument("--merged-topic", default="/merged/LIDAR/POINTS")
    parser.add_argument("--back-imu-topic", default="/rslidar_imu_data")
    parser.add_argument("--output-imu-topic", default="/imu/data")
    parser.add_argument("--copy-original-back-imu", action="store_true")
    parser.add_argument("--copy-source-clouds", action="store_true")
    parser.add_argument("--copy-tf", action="store_true")
    parser.add_argument("--copy-joint-states", action="store_true")
    parser.add_argument("--wait-for-side", action="store_true", default=True)
    parser.add_argument("--no-wait-for-side", action="store_false", dest="wait_for_side")
    parser.add_argument("--max-pending-back-frames", type=int, default=12)
    parser.add_argument("--side-wait-timeout-s", type=float, default=1.0)
    parser.add_argument("--self-filter-padding", type=float, default=0.03)
    parser.add_argument("--shape-json")
    parser.add_argument("--no-self-filter", action="store_true")
    parser.add_argument("--compute-unfiltered-stats", action="store_true", help="Also run an unfiltered merge per frame for exact before/after statistics; slower")
    parser.add_argument("--max-back-frames", type=int, default=0, help="0 means all frames; useful for dry-runs")
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)

    input_uri, tmp = unwrap_uncompressed_bag_if_needed(args.input)
    rclpy.init(args=None)
    try:
        tf_buffer, topic_types, topic_names = read_tf_buffer(input_uri)
        required = [args.back_topic, args.chin_topic, args.tail_topic, args.back_imu_topic, "/tf_static"]
        missing = [topic for topic in required if topic not in topic_names]
        if missing:
            raise RuntimeError(f"input bag is missing required topics: {missing}")

        shapes = [] if args.no_self_filter else load_self_filter_shapes(args.shape_json, args.self_filter_padding)
        reader = open_reader(input_uri)
        writer = open_writer(str(output))
        create_topics(writer, args, topic_names)

        chin_cache: deque[CachedCloud] = deque()
        tail_cache: deque[CachedCloud] = deque()
        pending_back: deque[tuple[CachedCloud, int]] = deque()
        seq = {"back": 0, "chin": 0, "tail": 0}
        latest_side_max = {"chin": None, "tail": None}

        counts = defaultdict(int)
        merged_frames = 0
        full_three_frames = 0
        missing_chin_frames = 0
        missing_tail_frames = 0
        back_only_frames = 0
        source_points = defaultdict(int)
        source_points_before_filter = defaultdict(int)
        self_filtered_points = defaultdict(int)
        shape_removed = defaultdict(int)
        dt_mid_ms = {"chin": [], "tail": []}

        def source_transform(cloud: PointCloud2, source_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            stamp = stamp_ns_msg(cloud)
            rot, trans = tf_buffer.lookup(args.target_frame, cloud.header.frame_id, stamp)
            if source_name == "chin":
                driver_rot = rotation_from_rpy(0.0, 0.0, math.radians(180.0))
            else:
                driver_rot = IDENTITY_ROTATION
            return rot @ driver_rot, trans, ZERO_TRANSLATION

        def can_emit(back: CachedCloud, force: bool) -> bool:
            if force or not args.wait_for_side:
                return True
            if len(pending_back) >= max(1, args.max_pending_back_frames):
                return True
            if latest_side_max["chin"] is None or latest_side_max["tail"] is None:
                return False
            return latest_side_max["chin"] >= back.timestamp_max and latest_side_max["tail"] >= back.timestamp_max

        def emit_back(back: CachedCloud, force: bool = False) -> bool:
            nonlocal merged_frames, full_three_frames, missing_chin_frames, missing_tail_frames, back_only_frames
            if not can_emit(back, force):
                return False
            window = (back.timestamp_min, back.timestamp_max)
            chin_matches = select_overlapping(chin_cache, window)
            tail_matches = select_overlapping(tail_cache, window)
            if not chin_matches:
                missing_chin_frames += 1
            if not tail_matches:
                missing_tail_frames += 1
            if not chin_matches and not tail_matches:
                back_only_frames += 1
            if chin_matches and tail_matches:
                full_three_frames += 1
            back_mid = 0.5 * (back.timestamp_min + back.timestamp_max)
            transforms = []
            filter_stats = []
            for cloud, name, source_id in [(back.cloud, "back", SOURCE_BACK)]:
                rot, trans, extra = source_transform(cloud, name)
                stats = defaultdict(int)
                point_filter = None if args.no_self_filter else make_box_filter(shapes, tf_buffer, args.target_frame, stamp_ns_msg(cloud), stats)
                transforms.append((cloud, rot, trans, extra, None, point_filter, source_id))
                filter_stats.append((name, stats))
            for cached, name, source_id in [(m, "chin", SOURCE_CHIN) for m in chin_matches] + [(m, "tail", SOURCE_TAIL) for m in tail_matches]:
                rot, trans, extra = source_transform(cached.cloud, name)
                stats = defaultdict(int)
                point_filter = None if args.no_self_filter else make_box_filter(shapes, tf_buffer, args.target_frame, stamp_ns_msg(cached.cloud), stats)
                transforms.append((cached.cloud, rot, trans, extra, None, point_filter, source_id))
                filter_stats.append((name, stats))
                side_mid = 0.5 * (cached.timestamp_min + cached.timestamp_max)
                dt_mid_ms[name].append((side_mid - back_mid) * 1000.0)

            unfiltered = None
            if args.compute_unfiltered_stats:
                unfiltered_transforms = [
                    (item[0], item[1], item[2], item[3], item[4], None, item[6])
                    for item in transforms
                ]
                unfiltered = merge_pointclouds(unfiltered_transforms, timestamp_window=window)
            merged = merge_pointclouds(transforms, timestamp_window=window)
            removed_by_transform = []
            for name, stats in filter_stats:
                removed = 0
                for key, value in stats.items():
                    shape_removed[f"{name}:{key}"] += value
                    removed += int(value)
                removed_by_transform.append(removed)
            source_names = ["back"] + ["chin"] * len(chin_matches) + ["tail"] * len(tail_matches)
            for idx, name in enumerate(source_names):
                after_count = int(merged.source_point_counts[idx]) if idx < len(merged.source_point_counts) else 0
                if unfiltered is not None and idx < len(unfiltered.source_point_counts):
                    source_points_before_filter[name] += int(unfiltered.source_point_counts[idx])
                else:
                    removed = removed_by_transform[idx] if idx < len(removed_by_transform) else 0
                    source_points_before_filter[name] += after_count + removed
                source_points[name] += after_count
            for name in ("back", "chin", "tail"):
                self_filtered_points[name] = max(0, source_points_before_filter[name] - source_points[name])
            merged_msg = make_cloud_msg(back.cloud, merged, args.target_frame)
            writer.write(args.merged_topic, serialize_message(merged_msg), stamp_ns_msg(merged_msg))
            if args.copy_source_clouds:
                writer.write(args.back_topic, serialize_message(back.cloud), back.stamp_ns)
                for cached in chin_matches:
                    writer.write(args.chin_topic, serialize_message(cached.cloud), cached.stamp_ns)
                for cached in tail_matches:
                    writer.write(args.tail_topic, serialize_message(cached.cloud), cached.stamp_ns)
                counts[args.back_topic] += 1
                counts[args.chin_topic] += len(chin_matches)
                counts[args.tail_topic] += len(tail_matches)
            counts[args.merged_topic] += 1
            merged_frames += 1
            return True

        processed_back = 0
        while reader.has_next():
            topic, data, t = reader.read_next()
            msg_type = topic_types[topic]
            if topic == args.chin_topic:
                msg = deserialize_message(data, get_message(msg_type))
                chin_cache.append(cached_cloud(msg, seq["chin"]))
                seq["chin"] += 1
                latest_side_max["chin"] = chin_cache[-1].timestamp_max
            elif topic == args.tail_topic:
                msg = deserialize_message(data, get_message(msg_type))
                tail_cache.append(cached_cloud(msg, seq["tail"]))
                seq["tail"] += 1
                latest_side_max["tail"] = tail_cache[-1].timestamp_max
            elif topic == args.back_topic:
                msg = deserialize_message(data, get_message(msg_type))
                pending_back.append((cached_cloud(msg, seq["back"]), t))
                seq["back"] += 1
                while pending_back:
                    back, enqueue_t = pending_back[0]
                    waited = (t - enqueue_t) * 1e-9
                    force = waited >= args.side_wait_timeout_s or len(pending_back) >= max(1, args.max_pending_back_frames)
                    if not emit_back(back, force=force):
                        break
                    pending_back.popleft()
                    processed_back += 1
                    if args.max_back_frames > 0 and processed_back >= args.max_back_frames:
                        pending_back.clear()
                        raise StopIteration
            elif topic == args.back_imu_topic:
                msg = deserialize_message(data, Imu)
                writer.write(args.output_imu_topic, serialize_message(msg), t)
                counts[args.output_imu_topic] += 1
                if args.copy_original_back_imu:
                    writer.write(args.back_imu_topic, data, t)
                    counts[args.back_imu_topic] += 1
            elif topic in ("/tf", "/tf_static") and args.copy_tf:
                writer.write(topic, data, t)
                counts[topic] += 1
            elif topic == "/joint_states" and args.copy_joint_states:
                writer.write(topic, data, t)
                counts[topic] += 1
    except StopIteration:
        pass
    finally:
        if 'pending_back' in locals():
            while pending_back and (args.max_back_frames <= 0 or processed_back < args.max_back_frames):
                back, _ = pending_back.popleft()
                emit_back(back, force=True)
                processed_back += 1
        if tmp is not None:
            tmp.cleanup()
        rclpy.shutdown()

    summary = {
        "input": str(Path(args.input).resolve()),
        "output": str(output.resolve()),
        "target_frame": args.target_frame,
        "merged_frames": merged_frames,
        "full_three_frames": full_three_frames,
        "missing_chin_frames": missing_chin_frames,
        "missing_tail_frames": missing_tail_frames,
        "back_only_frames": back_only_frames,
        "written_counts": dict(counts),
        "source_points_before_filter": dict(source_points_before_filter),
        "source_points_after_filter": dict(source_points),
        "self_filtered_points": dict(self_filtered_points),
        "shape_removed_points": dict(sorted(shape_removed.items())),
        "dt_mid_ms": {
            name: {
                "count": len(values),
                "median": percentile(values, 50.0),
                "p95_abs": percentile([abs(v) for v in values], 95.0),
            }
            for name, values in dt_mid_ms.items()
        },
        "self_filter_shapes": [
            {"name": s.name, "frame": s.frame, "center": s.center.tolist(), "size": (2.0 * s.half_size).tolist()}
            for s in shapes
        ],
    }
    summary_path = output.with_suffix(output.suffix + ".summary.json") if output.suffix else Path(str(output) + ".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
