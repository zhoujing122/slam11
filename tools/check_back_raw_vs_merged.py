#!/usr/bin/env python3
"""Compare raw back LiDAR points with merged-cloud source_id=0 points.

Use this on a short static bag that contains both /LIDAR/POINTS and
/merged/LIDAR/POINTS. The expected result for direct back-LIO is near-zero
nearest-neighbour distance between the raw back cloud and the back points that
pointcloud_merger writes into the merged target frame.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree


POINT_FIELD_TO_DTYPE = {
    1: "<i1",
    2: "<u1",
    3: "<i2",
    4: "<u2",
    5: "<i4",
    6: "<u4",
    7: "<f4",
    8: "<f8",
}


def resolve_bag_db(path: Path) -> Path:
    if path.is_file():
        return path
    matches = sorted(path.glob("*.db3"))
    if not matches:
        raise FileNotFoundError(f"no .db3 file found under {path}")
    if len(matches) > 1:
        raise FileExistsError(f"multiple .db3 files found under {path}: {matches}")
    return matches[0]


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def cloud_dtype(msg):
    names = []
    formats = []
    offsets = []
    for field in msg.fields:
        if field.count != 1:
            raise ValueError(f"field {field.name!r} count={field.count} unsupported")
        if field.datatype not in POINT_FIELD_TO_DTYPE:
            raise ValueError(f"field {field.name!r} datatype={field.datatype} unsupported")
        names.append(field.name)
        formats.append(POINT_FIELD_TO_DTYPE[field.datatype])
        offsets.append(int(field.offset))
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": int(msg.point_step)})


def cloud_array(msg):
    dtype = cloud_dtype(msg)
    for name in ("x", "y", "z"):
        if name not in dtype.names:
            raise ValueError(f"cloud is missing {name!r} field")
    count = int(msg.width) * int(msg.height)
    return np.frombuffer(bytes(msg.data), dtype=dtype, count=count)


def cloud_xyz(msg, source_id: float | None = None) -> np.ndarray:
    arr = cloud_array(msg)
    if source_id is not None:
        if "source_id" not in arr.dtype.names:
            raise ValueError("merged cloud is missing 'source_id' field")
        arr = arr[np.isclose(arr["source_id"].astype(np.float64), float(source_id), atol=1e-3)]
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64, copy=False)
    return xyz[np.isfinite(xyz).all(axis=1)]


def read_topics(cur) -> dict[str, tuple[int, str]]:
    return {name: (topic_id, type_name) for topic_id, name, type_name in cur.execute("select id,name,type from topics")}


def iter_clouds(cur, topics, topic_name: str):
    topic_id, type_name = topics[topic_name]
    msg_cls = get_message(type_name)
    for data, _recv_time in cur.execute(
        "select data,timestamp from messages where topic_id=? order by timestamp",
        (topic_id,),
    ):
        msg = deserialize_message(data, msg_cls)
        yield stamp_to_float(msg.header.stamp), msg


def quantile_text(values: np.ndarray) -> str:
    qs = np.quantile(values, [0.50, 0.90, 0.95, 0.99])
    return "p50={:.4f} p90={:.4f} p95={:.4f} p99={:.4f}".format(*qs)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or .db3 file")
    parser.add_argument("--raw-topic", default="/LIDAR/POINTS")
    parser.add_argument("--merged-topic", default="/merged/LIDAR/POINTS")
    parser.add_argument("--source-id", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=20)
    parser.add_argument("--stamp-tolerance-s", type=float, default=0.03)
    parser.add_argument("--sample-points", type=int, default=5000)
    parser.add_argument("--p95-threshold-m", type=float, default=0.03)
    args = parser.parse_args()

    db_path = resolve_bag_db(args.bag)
    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()
    topics = read_topics(cur)
    for topic in (args.raw_topic, args.merged_topic):
        if topic not in topics:
            raise RuntimeError(f"topic {topic!r} not found in {db_path}")

    merged = list(iter_clouds(cur, topics, args.merged_topic))
    if not merged:
        raise RuntimeError(f"topic {args.merged_topic!r} has no messages")

    all_distances = []
    centroid_deltas = []
    matched = 0
    merged_idx = 0
    rng = np.random.default_rng(0)

    for raw_stamp, raw_msg in iter_clouds(cur, topics, args.raw_topic):
        while merged_idx + 1 < len(merged) and abs(merged[merged_idx + 1][0] - raw_stamp) <= abs(merged[merged_idx][0] - raw_stamp):
            merged_idx += 1
        merged_stamp, merged_msg = merged[merged_idx]
        if abs(merged_stamp - raw_stamp) > args.stamp_tolerance_s:
            continue

        raw_xyz = cloud_xyz(raw_msg)
        merged_xyz = cloud_xyz(merged_msg, args.source_id)
        if raw_xyz.size == 0 or merged_xyz.size == 0:
            continue

        if raw_xyz.shape[0] > args.sample_points:
            raw_xyz = raw_xyz[rng.choice(raw_xyz.shape[0], args.sample_points, replace=False)]
        if merged_xyz.shape[0] > args.sample_points:
            merged_xyz = merged_xyz[rng.choice(merged_xyz.shape[0], args.sample_points, replace=False)]

        tree = cKDTree(merged_xyz)
        distances, _ = tree.query(raw_xyz, k=1, workers=-1)
        all_distances.append(distances)
        centroid_deltas.append(np.median(merged_xyz, axis=0) - np.median(raw_xyz, axis=0))
        matched += 1
        if matched >= args.max_frames:
            break

    if not all_distances:
        raise RuntimeError("no raw/merged frame pairs matched; adjust --stamp-tolerance-s or topics")

    distances = np.concatenate(all_distances)
    centroid_deltas_np = np.vstack(centroid_deltas)
    p95 = float(np.quantile(distances, 0.95))
    median_delta = np.median(centroid_deltas_np, axis=0)

    print(f"bag: {db_path}")
    print(f"matched_frames: {matched}")
    print(f"nearest_distance_m: {quantile_text(distances)} max={distances.max():.4f}")
    print("median_centroid_delta_m: [{:.4f}, {:.4f}, {:.4f}]".format(*median_delta))
    print("raw and merged source_id=0 are {}".format("CONSISTENT" if p95 <= args.p95_threshold_m else "NOT CONSISTENT"))
    return 0 if p95 <= args.p95_threshold_m else 2


if __name__ == "__main__":
    raise SystemExit(main())
