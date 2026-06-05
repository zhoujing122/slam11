#!/usr/bin/env python3
"""Inspect ROS 2 bag timing for D100 multi-lidar SLAM diagnostics."""

from __future__ import annotations

import argparse
import bisect
import sqlite3
from pathlib import Path
from typing import Iterable

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fmt_quantiles(vals: Iterable[float]) -> str:
    vals = sorted(vals)
    if not vals:
        return "n/a"

    def q(p: float) -> float:
        idx = round((len(vals) - 1) * p)
        idx = min(len(vals) - 1, max(0, idx))
        return vals[idx]

    return (
        f"min={vals[0] * 1000:.2f}ms "
        f"p50={q(0.5) * 1000:.2f}ms "
        f"p90={q(0.9) * 1000:.2f}ms "
        f"p99={q(0.99) * 1000:.2f}ms "
        f"max={vals[-1] * 1000:.2f}ms"
    )


def nearest_offsets(base: list[float], other: list[float]) -> list[float]:
    vals: list[float] = []
    for t in base:
        idx = bisect.bisect_left(other, t)
        candidates: list[float] = []
        if idx < len(other):
            candidates.append(other[idx] - t)
        if idx > 0:
            candidates.append(other[idx - 1] - t)
        if candidates:
            vals.append(min(candidates, key=abs))
    return vals


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_db3", type=Path)
    parser.add_argument(
        "--cloud-topic",
        action="append",
        default=["/LIDAR/POINTS", "/chin/LIDAR/POINTS", "/tail/LIDAR/POINTS"],
    )
    parser.add_argument(
        "--imu-topic",
        action="append",
        default=["/rslidar_imu_data", "/imu_sensor_broadcaster/imu"],
    )
    args = parser.parse_args()

    conn = sqlite3.connect(str(args.bag_db3))
    cur = conn.cursor()
    topics = {row[1]: (row[0], row[2]) for row in cur.execute("select id,name,type from topics")}

    print("topics:")
    for name, (topic_id, typ) in sorted(topics.items()):
        count, mn, mx = cur.execute(
            "select count(*), min(timestamp), max(timestamp) from messages where topic_id=?",
            (topic_id,),
        ).fetchone()
        dur = (mx - mn) / 1e9 if mn is not None else 0.0
        print(f"  {name:32s} {typ:28s} count={count:7d} recv_dur={dur:8.3f}s")

    def sample_topic(name: str, limit: int | None = None) -> tuple[list[float], list[float]]:
        if name not in topics:
            print(f"\n{name}: missing")
            return [], []
        topic_id, typ = topics[name]
        msg_cls = get_message(typ)
        query = "select timestamp,data from messages where topic_id=? order by timestamp"
        if limit:
            query += f" limit {int(limit)}"

        out: list[tuple[float, float]] = []
        frames: dict[str, int] = {}
        fields = None
        dims = None
        for recv_ns, data in cur.execute(query, (topic_id,)):
            msg = deserialize_message(data, msg_cls)
            if not hasattr(msg, "header"):
                continue
            header_stamp = stamp_to_float(msg.header.stamp)
            out.append((recv_ns / 1e9, header_stamp))
            frames[msg.header.frame_id] = frames.get(msg.header.frame_id, 0) + 1
            if hasattr(msg, "fields") and fields is None:
                fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
                dims = (
                    msg.height,
                    msg.width,
                    msg.point_step,
                    msg.row_step,
                    len(msg.data),
                    msg.is_bigendian,
                    msg.is_dense,
                )

        recvs = [item[0] for item in out]
        hdrs = [item[1] for item in out]
        header_dts = [b - a for a, b in zip(hdrs, hdrs[1:])]

        print(f"\n{name}")
        if not out:
            print("  no messages with header")
            return hdrs, recvs
        offsets = [h - r for r, h in out]
        print(f"  frames={frames}")
        print(
            f"  recv span {recvs[0]:.6f} -> {recvs[-1]:.6f}, "
            f"header span {hdrs[0]:.6f} -> {hdrs[-1]:.6f}"
        )
        print(f"  header-recv offset: {fmt_quantiles(offsets)}")
        print(f"  header dt: {fmt_quantiles(header_dts)}")
        if fields is not None:
            print(f"  cloud dims={dims}")
            print(f"  fields={fields}")
        return hdrs, recvs

    header_series: dict[str, list[float]] = {}
    recv_series: dict[str, list[float]] = {}
    for topic in args.cloud_topic:
        header_series[topic], recv_series[topic] = sample_topic(topic)
    for topic in args.imu_topic:
        sample_topic(topic, limit=20000)

    base_topic = args.cloud_topic[0]
    print(f"\nNearest cloud sync by header stamp, relative to {base_topic}:")
    for other in args.cloud_topic[1:]:
        vals = nearest_offsets(header_series[base_topic], header_series[other])
        absvals = [abs(v) for v in vals]
        print(
            f"  {other}: signed {fmt_quantiles(vals)} | abs {fmt_quantiles(absvals)} | "
            f"within10ms={sum(v <= 0.010 for v in absvals)}/{len(absvals)} "
            f"within80ms={sum(v <= 0.080 for v in absvals)}/{len(absvals)}"
        )

    print(f"\nNearest cloud sync by bag receive time, relative to {base_topic}:")
    for other in args.cloud_topic[1:]:
        vals = nearest_offsets(recv_series[base_topic], recv_series[other])
        absvals = [abs(v) for v in vals]
        print(
            f"  {other}: signed {fmt_quantiles(vals)} | abs {fmt_quantiles(absvals)} | "
            f"within10ms={sum(v <= 0.010 for v in absvals)}/{len(absvals)} "
            f"within80ms={sum(v <= 0.080 for v in absvals)}/{len(absvals)}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
