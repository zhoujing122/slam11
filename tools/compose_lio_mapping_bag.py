#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
import rosbag2_py
from rosbag2_py import TopicMetadata


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


def create_topic(writer, name: str, msg_type: str) -> None:
    writer.create_topic(TopicMetadata(name=name, type=msg_type, serialization_format="cdr"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Compose one bag with separate LIO and mapping cloud topics.")
    parser.add_argument("--lio-bag", required=True, help="Bag providing LIO /merged/LIDAR/POINTS and IMU topics")
    parser.add_argument("--mapping-bag", required=True, help="Bag providing filtered /merged/LIDAR/POINTS for mapping")
    parser.add_argument("--output", required=True)
    parser.add_argument("--lio-topic", default="/merged/LIDAR/POINTS")
    parser.add_argument("--mapping-in-topic", default="/merged/LIDAR/POINTS")
    parser.add_argument("--mapping-out-topic", default="/mapping_filtered/LIDAR/POINTS")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--copy-back-imu-topic", default="/rslidar_imu_data")
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)

    rclpy.init(args=None)
    lio_reader = open_reader(args.lio_bag)
    mapping_reader = open_reader(args.mapping_bag)
    writer = open_writer(str(output))

    create_topic(writer, args.lio_topic, "sensor_msgs/msg/PointCloud2")
    create_topic(writer, args.mapping_out_topic, "sensor_msgs/msg/PointCloud2")
    create_topic(writer, args.imu_topic, "sensor_msgs/msg/Imu")
    create_topic(writer, args.copy_back_imu_topic, "sensor_msgs/msg/Imu")

    counts = {args.lio_topic: 0, args.mapping_out_topic: 0, args.imu_topic: 0, args.copy_back_imu_topic: 0}

    while lio_reader.has_next():
        topic, data, storage_time = lio_reader.read_next()
        if topic == args.lio_topic:
            writer.write(args.lio_topic, data, storage_time)
            counts[args.lio_topic] += 1
        elif topic == args.imu_topic:
            writer.write(args.imu_topic, data, storage_time)
            counts[args.imu_topic] += 1
        elif topic == args.copy_back_imu_topic:
            writer.write(args.copy_back_imu_topic, data, storage_time)
            counts[args.copy_back_imu_topic] += 1

    while mapping_reader.has_next():
        topic, data, storage_time = mapping_reader.read_next()
        if topic == args.mapping_in_topic:
            writer.write(args.mapping_out_topic, data, storage_time)
            counts[args.mapping_out_topic] += 1

    rclpy.shutdown()

    if counts[args.lio_topic] == 0 or counts[args.mapping_out_topic] == 0 or counts[args.imu_topic] == 0:
        raise RuntimeError(f"missing required messages: {counts}")
    if counts[args.lio_topic] != counts[args.mapping_out_topic]:
        raise RuntimeError(f"lio/mapping cloud count mismatch: {counts}")

    summary = {
        "lio_bag": args.lio_bag,
        "mapping_bag": args.mapping_bag,
        "output_bag": str(output),
        "topics": {
            "lio_cloud": args.lio_topic,
            "mapping_cloud": args.mapping_out_topic,
            "imu": args.imu_topic,
            "back_imu_copy": args.copy_back_imu_topic,
        },
        "counts": counts,
        "semantics": "LIO consumes the original unfiltered merged cloud; split mapping consumes the filtered merged cloud on a separate topic so map filtering cannot change back_strict LIO sampling.",
    }
    (output / "compose_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
