#!/usr/bin/env python3
import argparse
import csv
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('run')
    args = parser.parse_args()
    run = Path(args.run)
    analysis = run.parent / 'analysis'
    analysis.mkdir(exist_ok=True)
    log = (run / 'run.log').read_text(errors='ignore')
    pose_re = re.compile(r"kf\s+(\d+), pose:\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)")
    poses = [(int(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))) for m in pose_re.finditer(log)]
    final = []
    seen = set()
    for row in reversed(poses):
        if row[0] in seen:
            continue
        seen.add(row[0])
        final.append(row)
    final.reverse()
    xy = np.array([[p[1], p[2]] for p in final], dtype=float)
    path_length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0
    closure = float(np.linalg.norm(xy[-1] - xy[0])) if len(xy) > 1 else 0.0
    span = xy.max(axis=0) - xy.min(axis=0) if len(xy) else np.array([0.0, 0.0])

    traj_csv = analysis / 'back_chin_no_tail_keyframe_trajectory.csv'
    with traj_csv.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['kf_id', 'x', 'y', 'z'])
        writer.writerows(final)

    traj_png = analysis / 'back_chin_no_tail_trajectory.png'
    if len(xy):
        plt.figure(figsize=(8, 7))
        plt.plot(xy[:, 0], xy[:, 1], lw=1.7)
        plt.scatter([xy[0, 0], xy[-1, 0]], [xy[0, 1], xy[-1, 1]], marker='x', s=40)
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.xlabel('x m')
        plt.ylabel('y m')
        plt.title('Back LIO + back/chin merged map, no tail')
        plt.tight_layout()
        plt.savefig(traj_png, dpi=160)
        plt.close()

    map_dir = run / 'data' / 'new_map'
    arr = np.array(Image.open(map_dir / 'map.pgm').convert('L'))
    map_png = analysis / 'back_chin_no_tail_map_preview.png'
    Image.fromarray(np.dstack([arr, arr, arr])).save(map_png)

    source_lines = log.count('[split mapping source filter]')
    fallback = log.count('fallback to back-only')
    flush = re.findall(r"split mapping flush:.*stale_map=(\d+), deskew_no_trajectory=(\d+), back_only_fallback=(\d+)", log)
    flush_text = str(flush[-1]) if flush else 'n/a'
    summary = f"""# Back LIO + back/chin merged map, no tail

Input raw bag: d100_3lidar_rgb_depth_chinimu_raw_20260609_214256
Prepared merged bag: /ros2_ws/experiments/back_chin_current_full_slam/prepared/back_chin_merged_current_headerstamp

Prepared merged counts:
- /LIDAR/POINTS: 9208
- /merged/LIDAR/POINTS: 9208
- /imu/data: 183335
- matched chin frames in merged: 7352
- back-only merged frames: 1856
- tail points: 0

SLAM config behavior:
- LIO: back_strict from /LIDAR/POINTS
- mapping/G2P5 keyframe cloud: /merged/LIDAR/POINTS
- keyframe_source_mode: all, but available sources are only back + chin
- loop closing: false for saved map; loop=true run crashed at loop candidate kf 152

No-loop saved map metrics:
- keyframes: {len(final)}
- XY path length: {path_length:.3f} m
- XY closure displacement: {closure:.3f} m
- XY span: {float(span[0]):.3f} x {float(span[1]):.3f} m
- split mapping source-filter keyframes: {source_lines}
- back-only fallback count: {fallback}
- split flush: {flush_text}
- map image: {arr.shape[1]} x {arr.shape[0]}, occupied={int((arr < 10).sum())}, free={int((arr > 245).sum())}, unknown={int(((arr >= 10) & (arr <= 245)).sum())}

Files:
- map: {map_dir}
- map preview: {map_png}
- trajectory png: {traj_png}
- trajectory csv: {traj_csv}
"""
    out = analysis / 'back_chin_no_tail_summary.md'
    out.write_text(summary)
    print(summary)


if __name__ == '__main__':
    main()
