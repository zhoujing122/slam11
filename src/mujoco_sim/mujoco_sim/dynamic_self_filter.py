from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np


IDENTITY_ROTATION = np.eye(3, dtype=np.float64)
ZERO_TRANSLATION = np.zeros(3, dtype=np.float64)


@dataclass(frozen=True)
class BoxShape:
    name: str
    frame: str
    center: np.ndarray
    half_size: np.ndarray


class CompositePointFilter:
    def __init__(self, filters: Iterable[object]) -> None:
        self._filters = [f for f in filters if f is not None and f.active()]

    def active(self) -> bool:
        return bool(self._filters)

    def keep_mask(self, xyz: np.ndarray) -> np.ndarray:
        keep = np.ones(xyz.shape[0], dtype=bool)
        for point_filter in self._filters:
            active_idx = np.flatnonzero(keep)
            if active_idx.size == 0:
                break
            local_keep = point_filter.keep_mask(xyz[active_idx])
            keep[active_idx[~local_keep]] = False
        return keep


class DynamicSelfFilter:
    def __init__(
        self,
        transforms: list[tuple[BoxShape, np.ndarray, np.ndarray]],
        stats: dict[str, int] | None = None,
        stats_prefix: str = "",
    ) -> None:
        self._transforms = transforms
        self._stats = stats
        self._stats_prefix = stats_prefix

    def active(self) -> bool:
        return bool(self._transforms)

    def keep_mask(self, xyz: np.ndarray) -> np.ndarray:
        keep = np.ones(xyz.shape[0], dtype=bool)
        for shape, rot_target_shape, trans_target_shape in self._transforms:
            active_idx = np.flatnonzero(keep)
            if active_idx.size == 0:
                break
            # lookup_transform(target, shape) gives p_target = R * p_shape + t.
            # With row-vector xyz in target frame: p_shape = (p_target - t) @ R.
            local = (xyz[active_idx] - trans_target_shape) @ rot_target_shape
            delta = np.abs(local - shape.center)
            inside = np.all(delta <= shape.half_size, axis=1)
            removed = int(np.count_nonzero(inside))
            if removed:
                if self._stats is not None:
                    key = f"{self._stats_prefix}{shape.name}" if self._stats_prefix else shape.name
                    self._stats[key] = self._stats.get(key, 0) + removed
                keep[active_idx[inside]] = False
        return keep


def default_shapes(padding: float) -> list[BoxShape]:
    def box(name: str, frame: str, center: Iterable[float], size: Iterable[float]) -> BoxShape:
        half = 0.5 * np.asarray(list(size), dtype=np.float64) + float(padding)
        return BoxShape(
            name=name,
            frame=frame,
            center=np.asarray(list(center), dtype=np.float64),
            half_size=half,
        )

    shapes = [
        # d100_description uses trunk; zyurdf exports base_link. Keep both so
        # the same default works for recorded bags from either robot description.
        box("trunk", "trunk", (0.0, 0.0, 0.0), (0.78, 0.36, 0.26)),
        box("base_link", "base_link", (0.0, 0.0, 0.0), (0.78, 0.36, 0.26)),
        box("upper_lidar_mount", "radar_uper_Link", (0.0, 0.0, -0.18), (0.42, 0.22, 0.20)),
        box("chin_mount", "radar_f_Link", (-0.10, 0.0, 0.00), (0.34, 0.22, 0.20)),
        box("tail_mount", "radar_r_Link", (0.10, 0.0, 0.00), (0.34, 0.22, 0.20)),
    ]

    def add_leg(prefix: str, suffix: str = "") -> None:
        shapes.extend(
            [
                box(f"{prefix}_hip{suffix}", f"{prefix}_hip{suffix}", (0.0, 0.0, 0.0), (0.18, 0.18, 0.16)),
                box(f"{prefix}_thigh{suffix}", f"{prefix}_thigh{suffix}", (0.0, 0.0, -0.16), (0.16, 0.16, 0.42)),
                box(f"{prefix}_calf{suffix}", f"{prefix}_calf{suffix}", (0.0, 0.0, -0.16), (0.13, 0.13, 0.42)),
                box(f"{prefix}_foot{suffix}", f"{prefix}_foot{suffix}", (0.0, 0.0, 0.0), (0.18, 0.11, 0.09)),
            ]
        )

    for leg in ("FL", "FR", "RL", "RR"):
        add_leg(leg)
    for leg in ("FL", "FR", "BL", "BR"):
        add_leg(leg, "_Link")
    return shapes


def load_shapes(path: str | None, padding: float) -> list[BoxShape]:
    if path is None or path == "":
        return default_shapes(padding)
    data = json.loads(Path(path).read_text())
    shapes: list[BoxShape] = []
    for item in data["boxes"]:
        center = np.asarray(item.get("center", [0.0, 0.0, 0.0]), dtype=np.float64)
        half = 0.5 * np.asarray(item["size"], dtype=np.float64) + float(padding)
        shapes.append(
            BoxShape(
                name=item["name"],
                frame=item["frame"],
                center=center,
                half_size=half,
            )
        )
    return shapes


def make_dynamic_self_filter(
    shapes: Iterable[BoxShape],
    lookup_target_to_source: Callable[[str], tuple[object, object] | None],
    stats: dict[str, int] | None = None,
    stats_prefix: str = "",
    require_all_shapes: bool = False,
) -> DynamicSelfFilter:
    transforms: list[tuple[BoxShape, np.ndarray, np.ndarray]] = []
    missing: list[str] = []
    for shape in shapes:
        transform = lookup_target_to_source(shape.frame)
        if transform is None:
            missing.append(shape.frame)
            continue
        rot, trans = transform
        transforms.append(
            (
                shape,
                np.asarray(rot, dtype=np.float64),
                np.asarray(trans, dtype=np.float64),
            )
        )
    if require_all_shapes and missing:
        raise KeyError(f"missing self-filter TF frames: {sorted(set(missing))}")
    return DynamicSelfFilter(transforms, stats=stats, stats_prefix=stats_prefix)
