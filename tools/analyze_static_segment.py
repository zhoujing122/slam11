#!/usr/bin/env python3
"""Analyze the initial static segment of a ROS 2 bag.

This reads compressed rosbag2 bags through rosbag2_py and reports IMU
statistics plus LiDAR/camera topic timing for the first N seconds.
"""

from __future__ import annotations

import argparse
import math
import sqlite3
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def open_reader(uri: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def read_topic_types_from_db(db_path: str) -> dict[str, str]:
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    topic_types = {name: msg_type for _, name, msg_type in cur.execute("select id,name,type from topics")}
    conn.close()
    return topic_types


def iter_db_messages(db_path: str):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    topics = {topic_id: name for topic_id, name, _ in cur.execute("select id,name,type from topics")}
    for topic_id, timestamp, data in cur.execute("select topic_id,timestamp,data from messages order by timestamp"):
        yield topics[topic_id], bytes(data), timestamp
    conn.close()


def iter_bag_messages(bag_path: str):
    reader = open_reader(bag_path)
    while reader.has_next():
        yield reader.read_next()


def summarize_imu(name: str, rows: list[tuple[float, np.ndarray, np.ndarray]]) -> None:
    if not rows:
        print(f"\n{name}: no samples")
        return
    t = np.array([r[0] for r in rows], dtype=np.float64)
    acc = np.array([r[1] for r in rows], dtype=np.float64)
    gyro = np.array([r[2] for r in rows], dtype=np.float64)
    acc_norm = np.linalg.norm(acc, axis=1)
    gyro_norm = np.linalg.norm(gyro, axis=1)
    mean_acc = acc.mean(axis=0)
    std_acc = acc.std(axis=0)
    mean_gyro = gyro.mean(axis=0)
    std_gyro = gyro.std(axis=0)
    lateral = math.hypot(float(mean_acc[0]), float(mean_acc[1]))
    tilt_deg = math.degrees(math.asin(max(-1.0, min(1.0, lateral / 9.80665))))

    print(f"\n{name}:")
    print(f"  samples: {len(rows)}, header span: {t[0]:.6f} -> {t[-1]:.6f} ({t[-1] - t[0]:.3f}s)")
    print(f"  acc mean xyz:  [{mean_acc[0]: .6f}, {mean_acc[1]: .6f}, {mean_acc[2]: .6f}] m/s^2")
    print(f"  acc std xyz:   [{std_acc[0]: .6f}, {std_acc[1]: .6f}, {std_acc[2]: .6f}] m/s^2")
    print(f"  acc norm mean/std: {acc_norm.mean():.6f} / {acc_norm.std():.6f} m/s^2")
    print(f"  lateral gravity component: {lateral:.6f} m/s^2, implied tilt: {tilt_deg:.3f} deg")
    print(f"  gyro mean xyz: [{mean_gyro[0]: .8f}, {mean_gyro[1]: .8f}, {mean_gyro[2]: .8f}] rad/s")
    print(f"  gyro std xyz:  [{std_gyro[0]: .8f}, {std_gyro[1]: .8f}, {std_gyro[2]: .8f}] rad/s")
    print(f"  gyro norm mean/p95/max: {gyro_norm.mean():.8f} / {np.percentile(gyro_norm, 95):.8f} / {gyro_norm.max():.8f} rad/s")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds from first bag message")
    parser.add_argument("--window", type=float, default=2.0, help="per-window stats in seconds")
    parser.add_argument(
        "--imu-topic",
        action="append",
        default=["/rslidar_imu_data", "/chin/rslidar_imu_data"],
    )
    parser.add_argument(
        "--cloud-topic",
        action="append",
        default=["/LIDAR/POINTS", "/chin/LIDAR/POINTS"],
    )
    args = parser.parse_args()

    bag_is_db = Path(args.bag).suffix == ".db3"
    if bag_is_db:
        topic_types = read_topic_types_from_db(args.bag)
        message_iter = iter_db_messages(args.bag)
    else:
        reader = open_reader(args.bag)
        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        message_iter = iter_bag_messages(args.bag)
    print("selected topics:")
    for topic in args.imu_topic + args.cloud_topic + ["/camera/color/image_raw", "/camera/depth/image_raw"]:
        print(f"  {topic}: {topic_types.get(topic, 'MISSING')}")

    first_storage = None
    first_header = {}
    last_header = {}
    counts = defaultdict(int)
    imu_rows: dict[str, list[tuple[float, np.ndarray, np.ndarray]]] = {topic: [] for topic in args.imu_topic}
    per_second_gyro: dict[str, dict[int, list[float]]] = {topic: defaultdict(list) for topic in args.imu_topic}

    for topic, data, storage_time in message_iter:
        if first_storage is None:
            first_storage = storage_time
        rel_storage = (storage_time - first_storage) * 1e-9
        if rel_storage > args.duration:
            break

        if topic in args.imu_topic:
            msg = deserialize_message(data, Imu)
            h = stamp_sec(msg)
            acc = np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                dtype=np.float64,
            )
            gyro = np.array(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                dtype=np.float64,
            )
            imu_rows[topic].append((h, acc, gyro))
            per_second_gyro[topic][int(rel_storage)].append(float(np.linalg.norm(gyro)))
            first_header.setdefault(topic, h)
            last_header[topic] = h
            counts[topic] += 1
        elif topic in args.cloud_topic:
            msg = deserialize_message(data, PointCloud2)
            h = stamp_sec(msg)
            first_header.setdefault(topic, h)
            last_header[topic] = h
            counts[topic] += 1
        elif topic in ("/camera/color/image_raw", "/camera/depth/image_raw"):
            counts[topic] += 1

    print(f"\nfirst {args.duration:.1f}s by bag storage time:")
    for topic in args.imu_topic + args.cloud_topic + ["/camera/color/image_raw", "/camera/depth/image_raw"]:
        if topic in first_header:
            print(
                f"  {topic}: count={counts[topic]}, header {first_header[topic]:.6f} -> {last_header[topic]:.6f}"
            )
        else:
            print(f"  {topic}: count={counts[topic]}")

    for topic in args.imu_topic:
        summarize_imu(topic, imu_rows[topic])

    print("\nper-second gyro norm mean, first seconds:")
    for topic in args.imu_topic:
        print(f"  {topic}:")
        for sec in sorted(per_second_gyro[topic])[: int(args.duration)]:
            vals = np.array(per_second_gyro[topic][sec], dtype=np.float64)
            print(f"    +{sec:02d}s: mean={vals.mean():.7f}, p95={np.percentile(vals, 95):.7f}, n={len(vals)}")


if __name__ == "__main__":
    main()
