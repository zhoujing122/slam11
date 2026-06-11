#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import TopicMetadata
from sensor_msgs.msg import Imu, PointCloud2, PointField


DATATYPE_TO_NUMPY = {
    PointField.INT8: "<i1",
    PointField.UINT8: "<u1",
    PointField.INT16: "<i2",
    PointField.UINT16: "<u2",
    PointField.INT32: "<i4",
    PointField.UINT32: "<u4",
    PointField.FLOAT32: "<f4",
    PointField.FLOAT64: "<f8",
}


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def dtype_from_fields(msg: PointCloud2) -> np.dtype:
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.count != 1:
            raise ValueError(f"unsupported field count for {field.name}: {field.count}")
        if field.datatype not in DATATYPE_TO_NUMPY:
            raise ValueError(f"unsupported field datatype for {field.name}: {field.datatype}")
        names.append(field.name)
        formats.append(DATATYPE_TO_NUMPY[field.datatype])
        offsets.append(int(field.offset))
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": int(msg.point_step)})


def source_field(offset: int) -> PointField:
    field = PointField()
    field.name = "source_id"
    field.offset = int(offset)
    field.datatype = PointField.FLOAT32
    field.count = 1
    return field


def ensure_source(msg: PointCloud2, source_id: float) -> PointCloud2:
    dtype = dtype_from_fields(msg)
    points = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height).copy()
    if "source_id" in dtype.names:
        points["source_id"] = source_id
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(points.size)
        out.fields = list(msg.fields)
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.data = points.tobytes()
        out.is_dense = msg.is_dense
        return out

    names = list(dtype.names) + ["source_id"]
    formats = [dtype.fields[name][0] for name in dtype.names] + [np.dtype("<f4")]
    offsets = [dtype.fields[name][1] for name in dtype.names] + [int(msg.point_step)]
    point_step = int(msg.point_step) + 4
    out_dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})
    out_points = np.zeros(points.size, dtype=out_dtype)
    for name in dtype.names:
        out_points[name] = points[name]
    out_points["source_id"] = source_id

    out = PointCloud2()
    out.header = msg.header
    out.height = 1
    out.width = int(out_points.size)
    out.fields = list(msg.fields) + [source_field(msg.point_step)]
    out.is_bigendian = msg.is_bigendian
    out.point_step = point_step
    out.row_step = point_step * out.width
    out.data = out_points.tobytes()
    out.is_dense = msg.is_dense
    return out


def transform_chin_to_back(msg: PointCloud2, rotation: np.ndarray, translation: np.ndarray) -> PointCloud2:
    msg = ensure_source(msg, 1.0)
    dtype = dtype_from_fields(msg)
    points = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height).copy()
    xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float64, copy=False)
    xyz_out = xyz @ rotation.T + translation
    points["x"] = xyz_out[:, 0]
    points["y"] = xyz_out[:, 1]
    points["z"] = xyz_out[:, 2]
    points["source_id"] = 1.0

    out = PointCloud2()
    out.header = msg.header
    out.header.frame_id = "radar_uper_Link"
    out.height = 1
    out.width = int(points.size)
    out.fields = list(msg.fields)
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step
    out.row_step = msg.point_step * out.width
    out.data = points.tobytes()
    out.is_dense = msg.is_dense
    return out


def convert_to_dtype(msg: PointCloud2, out_dtype: np.dtype) -> np.ndarray:
    dtype = dtype_from_fields(msg)
    points = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    out = np.zeros(points.size, dtype=out_dtype)
    for name in out_dtype.names:
        if name in dtype.names:
            out[name] = points[name]
    return out


def merge_clouds(back_msg: PointCloud2, chin_msg: PointCloud2 | None) -> tuple[PointCloud2, int]:
    back = ensure_source(back_msg, 0.0)
    out_dtype = dtype_from_fields(back)
    arrays = [convert_to_dtype(back, out_dtype)]
    chin_points = 0
    if chin_msg is not None:
        chin = ensure_source(chin_msg, 1.0)
        chin_points = int(chin.width * chin.height)
        arrays.append(convert_to_dtype(chin, out_dtype))
    merged_points = np.concatenate(arrays) if len(arrays) > 1 else arrays[0]

    out = PointCloud2()
    out.header = back_msg.header
    out.header.frame_id = "radar_uper_Link"
    out.height = 1
    out.width = int(merged_points.size)
    out.fields = list(back.fields)
    out.is_bigendian = back.is_bigendian
    out.point_step = back.point_step
    out.row_step = back.point_step * out.width
    out.data = merged_points.tobytes()
    out.is_dense = back.is_dense and (chin_msg.is_dense if chin_msg is not None else True)
    return out, chin_points


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


def create_topics(writer: rosbag2_py.SequentialWriter, args: argparse.Namespace) -> None:
    for name, msg_type in (
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
        (args.back_topic, "sensor_msgs/msg/PointCloud2"),
        (args.chin_topic, "sensor_msgs/msg/PointCloud2"),
        (args.chin_in_back_topic, "sensor_msgs/msg/PointCloud2"),
        (args.merged_topic, "sensor_msgs/msg/PointCloud2"),
        (args.output_imu_topic, "sensor_msgs/msg/Imu"),
    ):
        writer.create_topic(TopicMetadata(name=name, type=msg_type, serialization_format="cdr"))


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--back-topic", default="/LIDAR/POINTS")
    parser.add_argument("--chin-topic", default="/chin/LIDAR/POINTS")
    parser.add_argument("--chin-in-back-topic", default="/chin_in_back/LIDAR/POINTS")
    parser.add_argument("--merged-topic", default="/merged/LIDAR/POINTS")
    parser.add_argument("--back-imu-topic", default="/rslidar_imu_data")
    parser.add_argument("--output-imu-topic", default="/imu/data")
    parser.add_argument("--wait-s", type=float, default=0.12)
    parser.add_argument("--max-dt-s", type=float, default=0.16)
    parser.add_argument("--translation", nargs=3, type=float, default=(0.34353, 0.000159742, -0.258107))
    parser.add_argument("--rpy", nargs=3, type=float, default=(-3.1416, -0.69813, -3.1416), help="radar_f_Link to radar_uper_Link roll pitch yaw in radians")
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)

    rotation = rotation_from_rpy(*args.rpy)
    translation = np.asarray(args.translation, dtype=np.float64)

    rclpy.init(args=None)
    reader = open_reader(args.input)
    topic_names = {topic.name for topic in reader.get_all_topics_and_types()}
    missing_topics = [topic for topic in (args.back_topic, args.chin_topic, args.back_imu_topic) if topic not in topic_names]
    if missing_topics:
        raise RuntimeError(f"input bag is missing required topics: {missing_topics}")

    writer = open_writer(args.output)
    create_topics(writer, args)

    chin_buffer: deque[tuple[int, int, PointCloud2]] = deque()
    pending_back: deque[tuple[int, PointCloud2]] = deque()
    counts = {
        "/tf_static": 0,
        args.back_topic: 0,
        args.chin_topic: 0,
        args.chin_in_back_topic: 0,
        args.merged_topic: 0,
        args.output_imu_topic: 0,
    }
    matched_chin = 0
    back_only = 0
    duplicate_chin_reuse = 0
    source1_points = 0
    dt_ms: list[float] = []
    used_chin_seq: set[int] = set()
    next_chin_seq = 0
    wait_ns = int(args.wait_s * 1e9)
    max_dt_ns = int(args.max_dt_s * 1e9)
    current_ns = 0

    def flush_ready(force: bool = False) -> None:
        nonlocal matched_chin, back_only, duplicate_chin_reuse, source1_points
        while pending_back and (force or pending_back[0][0] + wait_ns <= current_ns):
            back_ns, back_msg = pending_back.popleft()
            best_idx = None
            best_dt = None
            for idx, (chin_ns, _, _) in enumerate(chin_buffer):
                dt = abs(chin_ns - back_ns)
                if best_dt is None or dt < best_dt:
                    best_dt = dt
                    best_idx = idx
            best_msg = None
            if best_idx is not None and best_dt is not None and best_dt <= max_dt_ns:
                chin_ns, chin_seq, best_msg = chin_buffer[best_idx]
                del chin_buffer[best_idx]
                if chin_seq in used_chin_seq:
                    duplicate_chin_reuse += 1
                used_chin_seq.add(chin_seq)
                matched_chin += 1
                dt_ms.append((chin_ns - back_ns) / 1e6)
            else:
                back_only += 1
            merged, chin_points = merge_clouds(back_msg, best_msg)
            source1_points += chin_points
            writer.write(args.merged_topic, serialize_message(merged), back_ns)
            counts[args.merged_topic] += 1
            while chin_buffer and chin_buffer[0][0] < back_ns - 2 * max_dt_ns:
                chin_buffer.popleft()

    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        if topic == "/tf_static":
            writer.write(topic, data, storage_time)
            counts[topic] += 1
            continue
        if topic == args.back_imu_topic:
            imu = deserialize_message(data, Imu)
            imu.header.frame_id = "radar_uper_Link"
            t = stamp_ns(imu)
            current_ns = max(current_ns, t)
            writer.write(args.output_imu_topic, serialize_message(imu), t)
            counts[args.output_imu_topic] += 1
            flush_ready()
            continue
        if topic == args.chin_topic:
            chin_raw = deserialize_message(data, PointCloud2)
            chin_in_back = transform_chin_to_back(chin_raw, rotation, translation)
            t = stamp_ns(chin_in_back)
            current_ns = max(current_ns, t)
            writer.write(args.chin_topic, data, t)
            writer.write(args.chin_in_back_topic, serialize_message(chin_in_back), t)
            counts[args.chin_topic] += 1
            counts[args.chin_in_back_topic] += 1
            chin_buffer.append((t, next_chin_seq, chin_in_back))
            next_chin_seq += 1
            flush_ready()
            continue
        if topic == args.back_topic:
            back = deserialize_message(data, PointCloud2)
            t = stamp_ns(back)
            current_ns = max(current_ns, t)
            writer.write(args.back_topic, data, t)
            counts[args.back_topic] += 1
            pending_back.append((t, back))
            flush_ready()

    flush_ready(force=True)
    rclpy.shutdown()

    if counts[args.chin_topic] == 0:
        raise RuntimeError(f"no messages read from {args.chin_topic}")
    if matched_chin == 0 or source1_points == 0:
        raise RuntimeError(
            f"prepared bag is not a back+chin dataset: matched_chin={matched_chin}, source1_points={source1_points}"
        )
    if duplicate_chin_reuse != 0:
        raise RuntimeError(f"duplicate chin frame reuse detected: {duplicate_chin_reuse}")

    summary = {
        "input_bag": args.input,
        "output_bag": str(output),
        "input_topics": {
            "back_cloud": args.back_topic,
            "chin_cloud": args.chin_topic,
            "back_imu": args.back_imu_topic,
        },
        "output_topics": {
            "back_cloud": args.back_topic,
            "chin_cloud": args.chin_topic,
            "chin_in_back_cloud": args.chin_in_back_topic,
            "merged_cloud": args.merged_topic,
            "imu": args.output_imu_topic,
        },
        "extrinsic": {
            "source_frame": "radar_f_Link",
            "target_frame": "radar_uper_Link",
            "translation_xyz_m": [float(x) for x in translation],
            "rpy_rad": [float(x) for x in args.rpy],
        },
        "counts": counts,
        "back_frames": counts[args.back_topic],
        "chin_frames": counts[args.chin_topic],
        "chin_in_back_frames": counts[args.chin_in_back_topic],
        "merged_frames": counts[args.merged_topic],
        "imu_frames": counts[args.output_imu_topic],
        "matched_frames": matched_chin,
        "back_only_frames": back_only,
        "source1_points_in_merged": source1_points,
        "duplicate_chin_reuse": duplicate_chin_reuse,
        "dt_median_ms": percentile(dt_ms, 50),
        "dt_p95_abs_ms": percentile([abs(x) for x in dt_ms], 95),
        "dt_max_abs_ms": max([abs(x) for x in dt_ms], default=None),
        "wait_s": args.wait_s,
        "max_dt_s": args.max_dt_s,
    }
    summary_path = output / "prepare_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
