#!/usr/bin/env python3
from __future__ import annotations

import argparse
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


def dtype_from_fields(msg: PointCloud2) -> np.dtype:
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.count != 1:
            raise ValueError(f"unsupported field count for {field.name}: {field.count}")
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


def convert_to_dtype(msg: PointCloud2, out_dtype: np.dtype) -> np.ndarray:
    dtype = dtype_from_fields(msg)
    points = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    out = np.zeros(points.size, dtype=out_dtype)
    for name in out_dtype.names:
        if name in dtype.names:
            out[name] = points[name]
    return out


def merge_clouds(back_msg: PointCloud2, chin_msg: PointCloud2 | None) -> PointCloud2:
    back = ensure_source(back_msg, 0.0)
    out_dtype = dtype_from_fields(back)
    arrays = [convert_to_dtype(back, out_dtype)]
    if chin_msg is not None:
        chin = ensure_source(chin_msg, 1.0)
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


def create_topics(writer: rosbag2_py.SequentialWriter) -> None:
    for name, msg_type in (
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
        ("/LIDAR/POINTS", "sensor_msgs/msg/PointCloud2"),
        ("/merged/LIDAR/POINTS", "sensor_msgs/msg/PointCloud2"),
        ("/imu/data", "sensor_msgs/msg/Imu"),
    ):
        writer.create_topic(TopicMetadata(name=name, type=msg_type, serialization_format="cdr"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--wait-s", type=float, default=0.12)
    parser.add_argument("--max-dt-s", type=float, default=0.16)
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)

    rclpy.init(args=None)
    reader = open_reader(args.input)
    writer = open_writer(args.output)
    create_topics(writer)

    chin_buffer: deque[tuple[int, PointCloud2]] = deque()
    pending_back: deque[tuple[int, PointCloud2]] = deque()
    counts = {"/tf_static": 0, "/LIDAR/POINTS": 0, "/merged/LIDAR/POINTS": 0, "/imu/data": 0}
    matched_chin = 0
    back_only = 0
    wait_ns = int(args.wait_s * 1e9)
    max_dt_ns = int(args.max_dt_s * 1e9)
    current_ns = 0

    def flush_ready(force: bool = False) -> None:
        nonlocal matched_chin, back_only
        while pending_back and (force or pending_back[0][0] + wait_ns <= current_ns):
            back_ns, back_msg = pending_back.popleft()
            best = None
            best_dt = None
            for chin_ns, chin_msg in chin_buffer:
                dt = abs(chin_ns - back_ns)
                if best_dt is None or dt < best_dt:
                    best_dt = dt
                    best = chin_msg
            if best is not None and best_dt is not None and best_dt <= max_dt_ns:
                matched_chin += 1
            else:
                best = None
                back_only += 1
            merged = merge_clouds(back_msg, best)
            writer.write("/merged/LIDAR/POINTS", serialize_message(merged), back_ns)
            counts["/merged/LIDAR/POINTS"] += 1
            while chin_buffer and chin_buffer[0][0] < back_ns - 2 * max_dt_ns:
                chin_buffer.popleft()

    while reader.has_next():
        topic, data, storage_time = reader.read_next()
        if topic == "/tf_static":
            writer.write(topic, data, storage_time)
            counts[topic] += 1
            continue
        if topic == "/rslidar_imu_data":
            imu = deserialize_message(data, Imu)
            imu.header.frame_id = "radar_uper_Link"
            t = stamp_ns(imu)
            current_ns = max(current_ns, t)
            writer.write("/imu/data", serialize_message(imu), t)
            counts["/imu/data"] += 1
            flush_ready()
            continue
        if topic == "/chin_in_back/LIDAR/POINTS":
            chin = deserialize_message(data, PointCloud2)
            t = stamp_ns(chin)
            current_ns = max(current_ns, t)
            chin_buffer.append((t, chin))
            flush_ready()
            continue
        if topic == "/LIDAR/POINTS":
            back = deserialize_message(data, PointCloud2)
            t = stamp_ns(back)
            current_ns = max(current_ns, t)
            writer.write("/LIDAR/POINTS", data, t)
            counts["/LIDAR/POINTS"] += 1
            pending_back.append((t, back))
            flush_ready()

    flush_ready(force=True)
    rclpy.shutdown()
    for key, value in counts.items():
        print(f"{key}: {value}")
    print(f"matched_chin: {matched_chin}")
    print(f"back_only_merged: {back_only}")
    print(f"output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
