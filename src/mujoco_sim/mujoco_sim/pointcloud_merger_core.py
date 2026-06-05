"""Core helpers for merging RoboSense XYZIRT PointCloud2 messages.

Vectorised with numpy. Each input cloud is viewed as a structured array (one
record per point), xyz is rotated/translated as a single matrix multiply, and
all clouds are concatenated and sorted by per-point timestamp in numpy. For
~50k points/cloud × 3 clouds the hot path is dominated by ``np.frombuffer`` /
``tobytes`` rather than Python overhead.
"""

from copy import copy
from dataclasses import dataclass

import numpy as np


# sensor_msgs/PointField datatype enum — exposed so tests can build fake clouds
# with the same constants as ``rosidl`` produces.
INT8 = 1
UINT8 = 2
INT16 = 3
UINT16 = 4
INT32 = 5
UINT32 = 6
FLOAT32 = 7
FLOAT64 = 8

_DATATYPE_TO_NUMPY = {
    INT8: '<i1',
    UINT8: '<u1',
    INT16: '<i2',
    UINT16: '<u2',
    INT32: '<i4',
    UINT32: '<u4',
    FLOAT32: '<f4',
    FLOAT64: '<f8',
}


@dataclass(frozen=True)
class MergedPointCloud:
    height: int
    width: int
    fields: list
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool
    min_timestamp: float
    max_timestamp: float
    source_point_counts: tuple


@dataclass(frozen=True)
class SourcePointFilter:
    """Optional point filter evaluated in the merged target frame."""

    min_x: float | None = None
    max_x: float | None = None
    min_y: float | None = None
    max_y: float | None = None
    min_z: float | None = None
    max_z: float | None = None
    min_range_m: float | None = None
    max_range_m: float | None = None
    exclude_min_x: float | None = None
    exclude_max_x: float | None = None
    exclude_min_y: float | None = None
    exclude_max_y: float | None = None
    exclude_min_z: float | None = None
    exclude_max_z: float | None = None

    def active(self):
        return any(value is not None for value in self.__dict__.values())

    def keep_mask(self, xyz):
        keep = np.ones(xyz.shape[0], dtype=bool)
        x = xyz[:, 0]
        y = xyz[:, 1]
        z = xyz[:, 2]

        if self.min_x is not None:
            keep &= x >= self.min_x
        if self.max_x is not None:
            keep &= x <= self.max_x
        if self.min_y is not None:
            keep &= y >= self.min_y
        if self.max_y is not None:
            keep &= y <= self.max_y
        if self.min_z is not None:
            keep &= z >= self.min_z
        if self.max_z is not None:
            keep &= z <= self.max_z
        if self.min_range_m is not None and self.min_range_m > 0.0:
            keep &= np.sum(xyz * xyz, axis=1) >= self.min_range_m * self.min_range_m
        if self.max_range_m is not None and self.max_range_m > 0.0:
            keep &= np.sum(xyz * xyz, axis=1) <= self.max_range_m * self.max_range_m

        box_limits = (
            self.exclude_min_x, self.exclude_max_x,
            self.exclude_min_y, self.exclude_max_y,
            self.exclude_min_z, self.exclude_max_z,
        )
        if all(value is not None for value in box_limits):
            in_box = (
                (x >= self.exclude_min_x) & (x <= self.exclude_max_x)
                & (y >= self.exclude_min_y) & (y <= self.exclude_max_y)
                & (z >= self.exclude_min_z) & (z <= self.exclude_max_z)
            )
            keep &= ~in_box

        return keep


def select_nearest(candidates, target_stamp, tolerance_s):
    best = None
    best_delta = None
    for candidate in candidates:
        delta = abs(candidate.stamp - target_stamp)
        if delta <= tolerance_s and (best_delta is None or delta < best_delta):
            best = candidate
            best_delta = delta
    return best


def select_overlapping(candidates, timestamp_window):
    """Return all cached clouds whose point timestamps overlap a time window."""
    start, end = timestamp_window
    matches = []
    for candidate in candidates:
        if candidate.timestamp_max >= start and candidate.timestamp_min <= end:
            matches.append(candidate)
    return matches


def select_available_clouds(back, chin, tail, allow_back_only_fallback=True):
    """Return clouds to merge, missing side names, and max side sync delta."""
    missing = []
    clouds = [back]
    sync_delta = 0.0

    for name, candidate in (("chin", chin), ("tail", tail)):
        if candidate is None:
            missing.append(name)
            continue
        clouds.append(candidate)
        sync_delta = max(sync_delta, abs(candidate.stamp - back.stamp))

    if missing and not allow_back_only_fallback:
        return [], missing, sync_delta
    return clouds, missing, sync_delta


def should_wait_for_side_clouds(back_stamp, missing, chin_cache, tail_cache, tolerance_s):
    """Return True if a missing side cloud could still arrive for this back scan."""
    for name in missing:
        cache = chin_cache if name == "chin" else tail_cache
        if not cache:
            return True
        newest_stamp = cache[-1].stamp
        if newest_stamp <= back_stamp + tolerance_s:
            return True
    return False


def should_wait_for_overlapping_clouds(timestamp_window, missing, chin_cache, tail_cache):
    """Return True if a missing side cloud may still arrive for this scan window."""
    _, end = timestamp_window
    for name in missing:
        cache = chin_cache if name == "chin" else tail_cache
        if not cache:
            return True
        if cache[-1].timestamp_max < end:
            return True
    return False


def pointcloud_timestamp_range(cloud):
    """Return the inclusive min/max RoboSense timestamp in ``cloud``."""
    layout = _layout(cloud)
    _validate_layout(layout)
    dtype = _layout_to_dtype(cloud)

    n = int(cloud.width) * int(cloud.height)
    if n == 0:
        return 0.0, 0.0

    buf = np.frombuffer(bytes(cloud.data), dtype=dtype, count=n)
    return float(np.min(buf['timestamp'])), float(np.max(buf['timestamp']))


def merge_pointclouds(cloud_transforms, timestamp_window=None):
    """Merge PointCloud2-like objects after transforming xyz into one frame.

    Each item is ``(cloud, rotation_matrix_3x3, translation_xyz)``,
    ``(cloud, rotation_matrix_3x3, translation_xyz, extra_translation_xyz)``,
    or ``(cloud, rotation_matrix_3x3, translation_xyz,
    extra_translation_xyz, extra_rotation_matrix_3x3)``. A sixth
    ``SourcePointFilter`` item may be supplied to drop points after xyz has
    been transformed into the target frame. A seventh ``source_id`` item may
    be supplied; when present, the merged output gains a float32 ``source_id``
    field so downstream 2D mapping can use each LiDAR's own ray origin.
    Non-xyz fields, including RoboSense per-point ``timestamp``, are copied unchanged.
    The merged points are sorted by ``timestamp`` so Lightning-LM's per-point
    deskew sees a monotonic sequence. When ``timestamp_window`` is provided,
    points outside the inclusive ``(min_timestamp, max_timestamp)`` range are
    dropped before concatenation.
    """
    if not cloud_transforms:
        raise ValueError("at least one point cloud is required")

    reference = cloud_transforms[0][0]
    layout = _layout(reference)
    _validate_layout(layout)
    dtype = _layout_to_dtype(reference)
    source_id_enabled = any(len(item) >= 7 for item in cloud_transforms)
    if source_id_enabled:
        output_fields, output_dtype, output_point_step = _layout_with_source_id(reference)
    else:
        output_fields = list(reference.fields)
        output_dtype = dtype
        output_point_step = reference.point_step

    pieces = []
    source_point_counts = []
    timestamp_min = timestamp_max = None
    if timestamp_window is not None:
        timestamp_min, timestamp_max = timestamp_window
    for item in cloud_transforms:
        if len(item) == 3:
            cloud, rotation, translation = item
            extra_translation = (0.0, 0.0, 0.0)
            extra_rotation = None
            point_filter = None
            source_id = None
        elif len(item) == 4:
            cloud, rotation, translation, extra_translation = item
            extra_rotation = None
            point_filter = None
            source_id = None
        elif len(item) == 5:
            cloud, rotation, translation, extra_translation, extra_rotation = item
            point_filter = None
            source_id = None
        elif len(item) == 6:
            cloud, rotation, translation, extra_translation, extra_rotation, point_filter = item
            source_id = None
        elif len(item) == 7:
            cloud, rotation, translation, extra_translation, extra_rotation, point_filter, source_id = item
        else:
            raise ValueError("cloud transform item must have 3, 4, 5, 6, or 7 entries")
        if _layout(cloud) != layout:
            raise ValueError("all point clouds must use the same PointCloud2 layout")

        n = int(cloud.width) * int(cloud.height)
        if n == 0:
            source_point_counts.append(0)
            continue

        # ``np.frombuffer`` produces a read-only view; copy so we can rewrite xyz.
        # ``bytes(...)`` defends against rclpy's array.array buffers and slices.
        buf = np.frombuffer(bytes(cloud.data), dtype=dtype, count=n).copy()

        if timestamp_window is not None:
            keep = (buf['timestamp'] >= timestamp_min) & (buf['timestamp'] <= timestamp_max)
            buf = buf[keep].copy()

        if buf.size == 0:
            source_point_counts.append(0)
            continue

        # Promote to float64 for the matmul to match the original implementation's
        # numerical behaviour (single-pass multiply-add in C double precision).
        # The structured-array assignment back to float32 fields casts implicitly.
        rot = np.asarray(rotation, dtype=np.float64)
        trans = np.asarray(translation, dtype=np.float64)
        extra = np.asarray(extra_translation, dtype=np.float64)
        xyz = np.column_stack(
            (buf['x'].astype(np.float64, copy=False),
             buf['y'].astype(np.float64, copy=False),
             buf['z'].astype(np.float64, copy=False))
        )
        if extra_rotation is not None:
            extra_rot = np.asarray(extra_rotation, dtype=np.float64)
            rot = extra_rot @ rot
        xyz_t = xyz @ rot.T + trans + extra

        if point_filter is not None and point_filter.active():
            keep = point_filter.keep_mask(xyz_t)
            buf = buf[keep].copy()
            xyz_t = xyz_t[keep]

        source_point_counts.append(int(buf.size))
        if buf.size == 0:
            continue

        buf['x'] = xyz_t[:, 0]
        buf['y'] = xyz_t[:, 1]
        buf['z'] = xyz_t[:, 2]

        if source_id_enabled:
            out = np.zeros(buf.size, dtype=output_dtype)
            for name in dtype.names:
                out[name] = buf[name]
            out['source_id'] = float(source_id) if source_id is not None else 0.0
            pieces.append(out)
        else:
            pieces.append(buf)

    is_dense = all(bool(item[0].is_dense) for item in cloud_transforms)

    if not pieces:
        return MergedPointCloud(
            height=1, width=0,
            fields=output_fields,
            is_bigendian=False,
            point_step=output_point_step,
            row_step=0,
            data=b'',
            is_dense=is_dense,
            min_timestamp=0.0,
            max_timestamp=0.0,
            source_point_counts=tuple(source_point_counts),
        )

    merged = np.concatenate(pieces)
    order = np.argsort(merged['timestamp'], kind='stable')
    sorted_points = merged[order]
    min_timestamp = float(sorted_points['timestamp'][0])
    max_timestamp = float(sorted_points['timestamp'][-1])

    return MergedPointCloud(
        height=1,
        width=int(sorted_points.size),
        fields=output_fields,
        is_bigendian=False,
        point_step=output_point_step,
        row_step=int(sorted_points.size) * output_point_step,
        data=sorted_points.tobytes(),
        is_dense=is_dense,
        min_timestamp=min_timestamp,
        max_timestamp=max_timestamp,
        source_point_counts=tuple(source_point_counts),
    )


def _layout(cloud):
    return {
        field.name: (field.offset, field.datatype, field.count)
        for field in cloud.fields
    }


def _validate_layout(layout):
    required = {
        "x": FLOAT32,
        "y": FLOAT32,
        "z": FLOAT32,
        "timestamp": FLOAT64,
    }
    for name, datatype in required.items():
        if name not in layout:
            raise ValueError(f"PointCloud2 field '{name}' is required")
        if layout[name][1] != datatype:
            raise ValueError(f"PointCloud2 field '{name}' has unexpected datatype")

    if layout["x"][0] + 4 != layout["y"][0] or layout["y"][0] + 4 != layout["z"][0]:
        raise ValueError("x, y, z fields must be contiguous FLOAT32 values")

    return True


def _layout_to_dtype(reference):
    """Build a numpy structured dtype matching the cloud's PointField layout."""
    names, formats, offsets = [], [], []
    for field in reference.fields:
        if field.count != 1:
            raise ValueError(
                f"PointCloud2 field '{field.name}' count={field.count} not supported"
            )
        if field.datatype not in _DATATYPE_TO_NUMPY:
            raise ValueError(
                f"PointCloud2 field '{field.name}' datatype={field.datatype} not supported"
            )
        names.append(field.name)
        formats.append(_DATATYPE_TO_NUMPY[field.datatype])
        offsets.append(int(field.offset))
    return np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': int(reference.point_step),
    })


def _layout_with_source_id(reference):
    fields = [copy(field) for field in reference.fields]
    point_step = int(reference.point_step)
    source_field = _make_field_like(fields[0], "source_id", point_step, FLOAT32, 1)
    fields.append(source_field)

    names, formats, offsets = [], [], []
    for field in fields:
        if field.count != 1:
            raise ValueError(
                f"PointCloud2 field '{field.name}' count={field.count} not supported"
            )
        if field.datatype not in _DATATYPE_TO_NUMPY:
            raise ValueError(
                f"PointCloud2 field '{field.name}' datatype={field.datatype} not supported"
            )
        names.append(field.name)
        formats.append(_DATATYPE_TO_NUMPY[field.datatype])
        offsets.append(int(field.offset))

    output_point_step = point_step + np.dtype(_DATATYPE_TO_NUMPY[FLOAT32]).itemsize
    return fields, np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': output_point_step,
    }), output_point_step


def _make_field_like(template, name, offset, datatype, count):
    field = copy(template)
    field.name = name
    field.offset = offset
    field.datatype = datatype
    field.count = count
    return field
