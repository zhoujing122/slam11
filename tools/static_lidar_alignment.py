#!/usr/bin/env python3
"""Evaluate static multi-LiDAR alignment from a ROS 2 sqlite bag.

The script extracts known static windows, transforms back/chin/tail LiDAR
points into the back LiDAR frame using /tf_static plus the same manual offsets
as pointcloud_merger, then reports nearest-neighbour alignment metrics.
"""

from __future__ import annotations

import argparse
from collections import deque
import math
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


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def resolve_bag_db(path: Path) -> Path:
    if path.is_file():
        return path
    matches = sorted(path.glob("*.db3"))
    if not matches:
        raise FileNotFoundError(f"no .db3 file found under {path}")
    if len(matches) > 1:
        raise FileExistsError(f"multiple .db3 files found under {path}: {matches}")
    return matches[0]


def parse_window(value: str) -> tuple[float, float]:
    start, end = value.split(":", 1)
    start_f = float(start)
    end_f = float(end)
    if end_f <= start_f:
        raise argparse.ArgumentTypeError("window end must be greater than start")
    return start_f, end_f


def fmt_quantiles(values: np.ndarray) -> str:
    values = np.asarray(values)
    if values.size == 0:
        return "n/a"
    qs = np.quantile(values, [0.05, 0.50, 0.90, 0.95])
    return "p05={:.3f} p50={:.3f} p90={:.3f} p95={:.3f}".format(*qs)


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
    return np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": int(msg.point_step),
    })


def cloud_xyz(msg) -> np.ndarray:
    dtype = cloud_dtype(msg)
    for name in ("x", "y", "z"):
        if name not in dtype.names:
            raise ValueError(f"cloud is missing {name!r} field")
    count = int(msg.width) * int(msg.height)
    arr = np.frombuffer(bytes(msg.data), dtype=dtype, count=count)
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64, copy=False)
    return xyz[np.isfinite(xyz).all(axis=1)]


def quaternion_to_matrix(rotation) -> np.ndarray:
    x = rotation.x
    y = rotation.y
    z = rotation.z
    w = rotation.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-10:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array((
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    ), dtype=np.float64)


def rpy_degrees_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array((
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    ), dtype=np.float64)


def add_edge(graph, source: str, target: str, rotation: np.ndarray, translation: np.ndarray) -> None:
    graph.setdefault(source, []).append((target, rotation, translation))


def read_topics(cur) -> dict[str, tuple[int, str]]:
    return {
        name: (topic_id, type_name)
        for topic_id, name, type_name in cur.execute("select id,name,type from topics")
    }


def read_static_tf(cur, topics) -> dict[str, list[tuple[str, np.ndarray, np.ndarray]]]:
    graph: dict[str, list[tuple[str, np.ndarray, np.ndarray]]] = {}
    if "/tf_static" not in topics:
        return graph
    topic_id, type_name = topics["/tf_static"]
    msg_cls = get_message(type_name)
    for (data,) in cur.execute(
        "select data from messages where topic_id=? order by timestamp",
        (topic_id,),
    ):
        msg = deserialize_message(data, msg_cls)
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            translation_msg = transform.transform.translation
            translation = np.array(
                (translation_msg.x, translation_msg.y, translation_msg.z),
                dtype=np.float64,
            )
            rotation = quaternion_to_matrix(transform.transform.rotation)
            add_edge(graph, child, parent, rotation, translation)
            inv_rotation = rotation.T
            add_edge(graph, parent, child, inv_rotation, -inv_rotation @ translation)
    return graph


def find_transform(graph, source: str, target: str) -> tuple[np.ndarray, np.ndarray]:
    if source == target:
        return np.eye(3), np.zeros(3)

    queue = deque([(source, np.eye(3), np.zeros(3))])
    visited = {source}
    while queue:
        frame, rotation_acc, translation_acc = queue.popleft()
        for next_frame, rotation_edge, translation_edge in graph.get(frame, []):
            if next_frame in visited:
                continue
            next_rotation = rotation_edge @ rotation_acc
            next_translation = rotation_edge @ translation_acc + translation_edge
            if next_frame == target:
                return next_rotation, next_translation
            visited.add(next_frame)
            queue.append((next_frame, next_rotation, next_translation))
    raise LookupError(f"no transform path {source} -> {target}")


def first_header_stamp(cur, topics, topic_name: str) -> float:
    topic_id, type_name = topics[topic_name]
    msg_cls = get_message(type_name)
    row = cur.execute(
        "select data from messages where topic_id=? order by timestamp limit 1",
        (topic_id,),
    ).fetchone()
    if row is None:
        raise RuntimeError(f"topic {topic_name} has no messages")
    msg = deserialize_message(row[0], msg_cls)
    return stamp_to_float(msg.header.stamp)


def extra_for_topic(args, topic_name: str) -> tuple[np.ndarray, np.ndarray]:
    if topic_name == args.chin_topic:
        return (
            rpy_degrees_to_matrix(
                args.chin_roll_offset_deg,
                args.chin_pitch_offset_deg,
                args.chin_yaw_offset_deg,
            ),
            np.array((0.0, 0.0, args.chin_z_offset_m), dtype=np.float64),
        )
    if topic_name == args.tail_topic:
        return (
            rpy_degrees_to_matrix(
                args.tail_roll_offset_deg,
                args.tail_pitch_offset_deg,
                args.tail_yaw_offset_deg,
            ),
            np.array((0.0, 0.0, args.tail_z_offset_m), dtype=np.float64),
        )
    return np.eye(3), np.zeros(3)


def driver_to_link_for_topic(args, topic_name: str) -> np.ndarray:
    if topic_name == args.chin_topic:
        return rpy_degrees_to_matrix(
            args.chin_driver_to_link_roll_deg,
            args.chin_driver_to_link_pitch_deg,
            args.chin_driver_to_link_yaw_deg,
        )
    if topic_name == args.tail_topic:
        return rpy_degrees_to_matrix(
            args.tail_driver_to_link_roll_deg,
            args.tail_driver_to_link_pitch_deg,
            args.tail_driver_to_link_yaw_deg,
        )
    return np.eye(3)


def filter_points(points: np.ndarray, args) -> np.ndarray:
    if points.size == 0:
        return points
    keep = np.ones(points.shape[0], dtype=bool)
    if args.min_range_m > 0.0:
        keep &= np.sum(points * points, axis=1) >= args.min_range_m * args.min_range_m
    if args.max_range_m > 0.0:
        keep &= np.sum(points * points, axis=1) <= args.max_range_m * args.max_range_m
    if args.min_z_m is not None:
        keep &= points[:, 2] >= args.min_z_m
    if args.max_z_m is not None:
        keep &= points[:, 2] <= args.max_z_m
    return points[keep]


def collect_transformed_cloud(
    cur,
    topics,
    graph,
    topic_name: str,
    target_frame: str,
    window_abs: tuple[float, float],
    args,
) -> tuple[np.ndarray, int]:
    topic_id, type_name = topics[topic_name]
    msg_cls = get_message(type_name)
    extra_rotation, extra_translation = extra_for_topic(args, topic_name)
    driver_to_link_rotation = driver_to_link_for_topic(args, topic_name)

    pieces = []
    used_frames = 0
    seen_frames = 0
    start, end = window_abs
    query = "select data from messages where topic_id=? order by timestamp"
    for (data,) in cur.execute(query, (topic_id,)):
        msg = deserialize_message(data, msg_cls)
        stamp = stamp_to_float(msg.header.stamp)
        if stamp < start:
            continue
        if stamp > end:
            break
        seen_frames += 1
        if (seen_frames - 1) % args.frame_step != 0:
            continue
        if used_frames >= args.max_frames:
            break

        rotation, translation = find_transform(graph, msg.header.frame_id, target_frame)
        rotation = extra_rotation @ rotation @ driver_to_link_rotation
        points = cloud_xyz(msg)
        points = filter_points(points, args)
        points = points @ rotation.T + translation + extra_translation
        pieces.append(points)
        used_frames += 1

    if not pieces:
        return np.empty((0, 3), dtype=np.float64), used_frames
    return np.concatenate(pieces, axis=0), used_frames


def voxel_downsample(points: np.ndarray, voxel_m: float, max_points: int, seed: int) -> np.ndarray:
    if points.size == 0:
        return points
    if voxel_m > 0.0:
        keys = np.floor(points / voxel_m).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(indices)]
    if max_points > 0 and points.shape[0] > max_points:
        rng = np.random.default_rng(seed)
        indices = rng.choice(points.shape[0], size=max_points, replace=False)
        points = points[np.sort(indices)]
    return points


def nn_report(source: np.ndarray, target: np.ndarray, label: str, max_eval: int, seed: int) -> np.ndarray:
    if source.size == 0 or target.size == 0:
        print(f"  {label}: n/a")
        return np.empty(0)
    eval_points = source
    if max_eval > 0 and eval_points.shape[0] > max_eval:
        rng = np.random.default_rng(seed)
        indices = rng.choice(eval_points.shape[0], size=max_eval, replace=False)
        eval_points = eval_points[indices]
    distances, _ = cKDTree(target).query(eval_points, k=1)
    print(
        "  {}: n={} {} <0.10m={:.1f}% <0.20m={:.1f}% <0.30m={:.1f}%".format(
            label,
            distances.size,
            fmt_quantiles(distances),
            100.0 * float(np.mean(distances < 0.10)),
            100.0 * float(np.mean(distances < 0.20)),
            100.0 * float(np.mean(distances < 0.30)),
        )
    )
    return distances


def z_report(points: np.ndarray, label: str) -> None:
    if points.size == 0:
        print(f"  {label}: n/a")
        return
    low = points[points[:, 2] < np.quantile(points[:, 2], 0.25), 2]
    print(f"  {label}: z_all {fmt_quantiles(points[:, 2])} low_z {fmt_quantiles(low)}")


def z_grid_search(
    side: np.ndarray,
    back: np.ndarray,
    side_name: str,
    args,
    seed: int,
) -> None:
    if args.z_search_range_m <= 0.0 or side.size == 0 or back.size == 0:
        return
    eval_points = side
    if args.nn_max_eval > 0 and eval_points.shape[0] > args.nn_max_eval:
        rng = np.random.default_rng(seed)
        eval_points = eval_points[
            rng.choice(eval_points.shape[0], size=args.nn_max_eval, replace=False)
        ]
    tree = cKDTree(back)
    values = np.arange(
        -args.z_search_range_m,
        args.z_search_range_m + 0.5 * args.z_search_step_m,
        args.z_search_step_m,
    )
    rows = []
    shifted = eval_points.copy()
    for dz in values:
        shifted[:, 0:2] = eval_points[:, 0:2]
        shifted[:, 2] = eval_points[:, 2] + dz
        distances, _ = tree.query(shifted, k=1)
        rows.append((
            float(np.median(distances)),
            -float(np.mean(distances < 0.20)),
            float(dz),
            float(np.quantile(distances, 0.90)),
            100.0 * float(np.mean(distances < 0.20)),
        ))
    rows.sort()
    best = rows[:3]
    formatted = ", ".join(
        "dz={:+.3f}m p50={:.3f} p90={:.3f} <0.20m={:.1f}%".format(
            dz, p50, p90, pct
        )
        for p50, _neg_pct, dz, p90, pct in best
    )
    print(f"  {side_name} z-search best: {formatted}")


def write_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {points.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points.shape[0]}\n"
        "DATA binary\n"
    ).encode("ascii")
    with path.open("wb") as handle:
        handle.write(header)
        handle.write(points.astype("<f4", copy=False).tobytes())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--target-frame", default="radar_uper_Link")
    parser.add_argument("--back-topic", default="/LIDAR/POINTS")
    parser.add_argument("--chin-topic", default="/chin/LIDAR/POINTS")
    parser.add_argument("--tail-topic", default="/tail/LIDAR/POINTS")
    parser.add_argument("--window", action="append", type=parse_window)
    parser.add_argument("--max-frames", type=int, default=30)
    parser.add_argument("--frame-step", type=int, default=4)
    parser.add_argument("--voxel-m", type=float, default=0.08)
    parser.add_argument("--max-points", type=int, default=120000)
    parser.add_argument("--nn-max-eval", type=int, default=50000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--min-range-m", type=float, default=0.7)
    parser.add_argument("--max-range-m", type=float, default=35.0)
    parser.add_argument("--min-z-m", type=float, default=None)
    parser.add_argument("--max-z-m", type=float, default=None)
    parser.add_argument("--chin-z-offset-m", type=float, default=0.0)
    parser.add_argument("--tail-z-offset-m", type=float, default=0.0)
    parser.add_argument("--chin-roll-offset-deg", type=float, default=0.0)
    parser.add_argument("--chin-pitch-offset-deg", type=float, default=0.0)
    parser.add_argument("--chin-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--tail-roll-offset-deg", type=float, default=0.0)
    parser.add_argument("--tail-pitch-offset-deg", type=float, default=0.0)
    parser.add_argument("--tail-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--chin-driver-to-link-roll-deg", type=float, default=0.0)
    parser.add_argument("--chin-driver-to-link-pitch-deg", type=float, default=0.0)
    parser.add_argument("--chin-driver-to-link-yaw-deg", type=float, default=180.0)
    parser.add_argument("--tail-driver-to-link-roll-deg", type=float, default=0.0)
    parser.add_argument("--tail-driver-to-link-pitch-deg", type=float, default=0.0)
    parser.add_argument("--tail-driver-to-link-yaw-deg", type=float, default=0.0)
    parser.add_argument("--z-search-range-m", type=float, default=0.0)
    parser.add_argument("--z-search-step-m", type=float, default=0.05)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()

    if args.frame_step < 1:
        parser.error("--frame-step must be >= 1")
    if args.z_search_step_m <= 0.0:
        parser.error("--z-search-step-m must be > 0")

    windows = args.window or [parse_window("0:13"), parse_window("229:241")]
    bag_db = resolve_bag_db(args.bag)
    conn = sqlite3.connect(str(bag_db))
    cur = conn.cursor()
    topics = read_topics(cur)
    for topic_name in (args.back_topic, args.chin_topic, args.tail_topic):
        if topic_name not in topics:
            raise RuntimeError(f"bag is missing required topic {topic_name}")
    graph = read_static_tf(cur, topics)
    if not graph:
        raise RuntimeError("bag does not contain usable /tf_static transforms")

    base_stamp = first_header_stamp(cur, topics, args.back_topic)
    print(f"bag={bag_db}")
    print(f"base header stamp={base_stamp:.6f}")
    print(
        "offsets: chin z={:.3f} rpy=[{:.2f},{:.2f},{:.2f}]deg driver_to_link=[{:.1f},{:.1f},{:.1f}]deg, "
        "tail z={:.3f} rpy=[{:.2f},{:.2f},{:.2f}]deg driver_to_link=[{:.1f},{:.1f},{:.1f}]deg".format(
            args.chin_z_offset_m,
            args.chin_roll_offset_deg,
            args.chin_pitch_offset_deg,
            args.chin_yaw_offset_deg,
            args.chin_driver_to_link_roll_deg,
            args.chin_driver_to_link_pitch_deg,
            args.chin_driver_to_link_yaw_deg,
            args.tail_z_offset_m,
            args.tail_roll_offset_deg,
            args.tail_pitch_offset_deg,
            args.tail_yaw_offset_deg,
            args.tail_driver_to_link_roll_deg,
            args.tail_driver_to_link_pitch_deg,
            args.tail_driver_to_link_yaw_deg,
        )
    )

    topic_items = (
        ("back", args.back_topic),
        ("chin", args.chin_topic),
        ("tail", args.tail_topic),
    )
    for window_idx, window_rel in enumerate(windows):
        window_abs = (base_stamp + window_rel[0], base_stamp + window_rel[1])
        print(
            "\nwindow {} rel={:.1f}:{:.1f}s abs={:.6f}:{:.6f}".format(
                window_idx, window_rel[0], window_rel[1], window_abs[0], window_abs[1]
            )
        )
        clouds: dict[str, np.ndarray] = {}
        for name, topic_name in topic_items:
            raw_points, frames = collect_transformed_cloud(
                cur, topics, graph, topic_name, args.target_frame, window_abs, args
            )
            points = voxel_downsample(
                raw_points, args.voxel_m, args.max_points, args.seed + window_idx
            )
            clouds[name] = points
            print(
                f"  {name}: frames={frames} raw_points={raw_points.shape[0]} "
                f"downsampled={points.shape[0]}"
            )
            z_report(points, name)
            if args.output_dir is not None:
                write_pcd(args.output_dir / f"window{window_idx}_{name}.pcd", points)

        nn_report(clouds["chin"], clouds["back"], "chin -> back NN", args.nn_max_eval, args.seed)
        nn_report(clouds["tail"], clouds["back"], "tail -> back NN", args.nn_max_eval, args.seed + 1)
        z_grid_search(clouds["chin"], clouds["back"], "chin", args, args.seed + 2)
        z_grid_search(clouds["tail"], clouds["back"], "tail", args, args.seed + 3)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
