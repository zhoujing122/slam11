"""ROS2 node for merging three RoboSense LiDAR PointCloud2 topics.

TF lookup strategy
------------------
Controlled by parameter ``cache_static_tf`` (default ``True``).

* **D100 Gen 1 (current)**: all three LiDARs are rigidly mounted on the trunk.
  TFs ``radar_uper_Link <- radar_f_Link / radar_r_Link`` are static and never
  change at runtime. We do one ``lookup_transform`` per source frame on the
  first successful lookup (with a finite timeout to ride out the
  robot_state_publisher startup race) and cache the result. No per-frame
  TF queries after that.

* **D100 Gen 2 (future, movable head)**: the chin LiDAR rides on an actuated
  head joint, so ``radar_uper_Link <- radar_f_Link`` becomes time-varying.
  Set ``cache_static_tf: false`` to force a per-frame lookup using each
  cloud's ``header.stamp`` (not ``Time()``) so the variant matches the cloud
  timestamp. Back and tail LiDARs stay rigid; you can also switch to a
  per-frame ``rigid_frames`` whitelist if you want to cache only those.
"""

from collections import deque
from dataclasses import dataclass
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import tf2_ros

from mujoco_sim.pointcloud_merger_core import (
    SourcePointFilter,
    merge_pointclouds,
    pointcloud_timestamp_range,
    select_overlapping,
    should_wait_for_overlapping_clouds,
)


IDENTITY_ROTATION = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
ZERO_TRANSLATION = (0.0, 0.0, 0.0)


@dataclass
class CachedCloud:
    cloud: PointCloud2
    stamp: float
    timestamp_min: float
    timestamp_max: float


@dataclass
class PendingBackCloud:
    cached: CachedCloud
    enqueued_at: float


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__("pointcloud_merger")

        self.declare_parameter("target_frame", "radar_uper_Link")
        self.declare_parameter("back_topic", "/LIDAR/POINTS")
        self.declare_parameter("chin_topic", "/chin/LIDAR/POINTS")
        self.declare_parameter("tail_topic", "/tail/LIDAR/POINTS")
        self.declare_parameter("output_topic", "/merged/LIDAR/POINTS")
        # 10 Hz LiDAR -> scan period 100 ms. 10 ms tolerance keeps the merged
        # triplet temporally tight (within 1/10 of a scan). Loosen if hardware
        # latencies make this drop too many clouds.
        self.declare_parameter("sync_tolerance_s", 0.01)
        self.declare_parameter("cache_size", 20)
        # Gen 1 (rigid head): True. Gen 2 (movable head): False.
        self.declare_parameter("cache_static_tf", True)
        self.declare_parameter("tf_lookup_timeout_s", 2.0)
        self.declare_parameter("allow_back_only_fallback", True)
        self.declare_parameter("side_wait_timeout_s", 0.15)
        self.declare_parameter("max_pending_back_frames", 3)
        self.declare_parameter("chin_z_offset_m", 0.0)
        self.declare_parameter("tail_z_offset_m", 0.0)
        self.declare_parameter("chin_roll_offset_deg", 0.0)
        self.declare_parameter("chin_pitch_offset_deg", 0.0)
        self.declare_parameter("chin_yaw_offset_deg", 0.0)
        self.declare_parameter("tail_roll_offset_deg", 0.0)
        self.declare_parameter("tail_pitch_offset_deg", 0.0)
        self.declare_parameter("tail_yaw_offset_deg", 0.0)
        self.declare_parameter("chin_driver_to_link_roll_deg", 0.0)
        self.declare_parameter("chin_driver_to_link_pitch_deg", 0.0)
        self.declare_parameter("chin_driver_to_link_yaw_deg", 180.0)
        self.declare_parameter("tail_driver_to_link_roll_deg", 0.0)
        self.declare_parameter("tail_driver_to_link_pitch_deg", 0.0)
        self.declare_parameter("tail_driver_to_link_yaw_deg", 0.0)
        self.declare_parameter("back_min_range_m", 0.0)
        self.declare_parameter("chin_min_range_m", 0.0)
        self.declare_parameter("tail_min_range_m", 0.0)
        self.declare_parameter("back_max_range_m", 0.0)
        self.declare_parameter("chin_max_range_m", 0.0)
        self.declare_parameter("tail_max_range_m", 0.0)
        self.declare_parameter("back_min_z_m", -999.0)
        self.declare_parameter("chin_min_z_m", -999.0)
        self.declare_parameter("tail_min_z_m", -999.0)
        self.declare_parameter("back_max_z_m", 999.0)
        self.declare_parameter("chin_max_z_m", 999.0)
        self.declare_parameter("tail_max_z_m", 999.0)
        self.declare_parameter("back_exclude_box_enabled", False)
        self.declare_parameter("back_exclude_min_x_m", -0.6)
        self.declare_parameter("back_exclude_max_x_m", 0.6)
        self.declare_parameter("back_exclude_min_y_m", -0.5)
        self.declare_parameter("back_exclude_max_y_m", 0.5)
        self.declare_parameter("back_exclude_min_z_m", -0.8)
        self.declare_parameter("back_exclude_max_z_m", 0.4)
        # 0 disables periodic stats logging.
        self.declare_parameter("stats_period_s", 5.0)

        self.target_frame = self.get_parameter("target_frame").value
        self.sync_tolerance_s = float(self.get_parameter("sync_tolerance_s").value)
        self.cache_size = int(self.get_parameter("cache_size").value)
        self.cache_static_tf = bool(self.get_parameter("cache_static_tf").value)
        self.allow_back_only_fallback = bool(
            self.get_parameter("allow_back_only_fallback").value
        )
        self.side_wait_timeout_s = max(
            0.0,
            float(self.get_parameter("side_wait_timeout_s").value),
        )
        self.max_pending_back_frames = max(
            1,
            int(self.get_parameter("max_pending_back_frames").value),
        )
        self.chin_z_offset_m = float(self.get_parameter("chin_z_offset_m").value)
        self.tail_z_offset_m = float(self.get_parameter("tail_z_offset_m").value)
        self.chin_rotation_offset = _rpy_degrees_to_matrix(
            self.get_parameter("chin_roll_offset_deg").value,
            self.get_parameter("chin_pitch_offset_deg").value,
            self.get_parameter("chin_yaw_offset_deg").value,
        )
        self.tail_rotation_offset = _rpy_degrees_to_matrix(
            self.get_parameter("tail_roll_offset_deg").value,
            self.get_parameter("tail_pitch_offset_deg").value,
            self.get_parameter("tail_yaw_offset_deg").value,
        )
        self.chin_driver_to_link_rotation = _rpy_degrees_to_matrix(
            self.get_parameter("chin_driver_to_link_roll_deg").value,
            self.get_parameter("chin_driver_to_link_pitch_deg").value,
            self.get_parameter("chin_driver_to_link_yaw_deg").value,
        )
        self.tail_driver_to_link_rotation = _rpy_degrees_to_matrix(
            self.get_parameter("tail_driver_to_link_roll_deg").value,
            self.get_parameter("tail_driver_to_link_pitch_deg").value,
            self.get_parameter("tail_driver_to_link_yaw_deg").value,
        )
        self.source_filters = {
            "radar_uper_Link": self._point_filter_from_params("back"),
            "radar_f_Link": self._point_filter_from_params("chin"),
            "radar_r_Link": self._point_filter_from_params("tail"),
        }
        self.tf_lookup_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_s").value)
        )
        # source_frame -> (rotation_matrix_3x3, translation_xyz)
        self._tf_cache = {}

        # Rolling stats; reset on every periodic dump.
        self._stats = self._fresh_stats()

        self.chin_cache = deque(maxlen=self.cache_size)
        self.tail_cache = deque(maxlen=self.cache_size)
        self.back_pending = deque(maxlen=self.max_pending_back_frames)
        self._latest_back_timestamp_max = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("output_topic").value,
            10,
        )

        self.create_subscription(
            PointCloud2,
            self.get_parameter("chin_topic").value,
            self._chin_callback,
            20,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("tail_topic").value,
            self._tail_callback,
            20,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("back_topic").value,
            self._back_callback,
            20,
        )

        stats_period = float(self.get_parameter("stats_period_s").value)
        if stats_period > 0.0:
            self._stats_period = stats_period
            self.create_timer(stats_period, self._log_stats)

        self.get_logger().info(
            "pointcloud_merger: back=%s chin=%s tail=%s -> %s frame=%s "
            "tolerance=%.3fs cache_static_tf=%s tf_timeout=%.1fs "
            "allow_back_only_fallback=%s side_wait_timeout=%.3fs "
            "max_pending_back=%d "
            "chin_z_offset=%.3fm tail_z_offset=%.3fm "
            "chin_rpy_offset=[%.2f, %.2f, %.2f]deg "
            "tail_rpy_offset=[%.2f, %.2f, %.2f]deg "
            "chin_driver_to_link_rpy=[%.2f, %.2f, %.2f]deg "
            "tail_driver_to_link_rpy=[%.2f, %.2f, %.2f]deg "
            "min_range=[back %.2f, chin %.2f, tail %.2f]m "
            "z_window=[back %.2f..%.2f, chin %.2f..%.2f, tail %.2f..%.2f]m "
            "stats_period=%.1fs"
            % (
                self.get_parameter("back_topic").value,
                self.get_parameter("chin_topic").value,
                self.get_parameter("tail_topic").value,
                self.get_parameter("output_topic").value,
                self.target_frame,
                self.sync_tolerance_s,
                self.cache_static_tf,
                self.tf_lookup_timeout.nanoseconds / 1e9,
                self.allow_back_only_fallback,
                self.side_wait_timeout_s,
                self.max_pending_back_frames,
                self.chin_z_offset_m,
                self.tail_z_offset_m,
                float(self.get_parameter("chin_roll_offset_deg").value),
                float(self.get_parameter("chin_pitch_offset_deg").value),
                float(self.get_parameter("chin_yaw_offset_deg").value),
                float(self.get_parameter("tail_roll_offset_deg").value),
                float(self.get_parameter("tail_pitch_offset_deg").value),
                float(self.get_parameter("tail_yaw_offset_deg").value),
                float(self.get_parameter("chin_driver_to_link_roll_deg").value),
                float(self.get_parameter("chin_driver_to_link_pitch_deg").value),
                float(self.get_parameter("chin_driver_to_link_yaw_deg").value),
                float(self.get_parameter("tail_driver_to_link_roll_deg").value),
                float(self.get_parameter("tail_driver_to_link_pitch_deg").value),
                float(self.get_parameter("tail_driver_to_link_yaw_deg").value),
                self.source_filters["radar_uper_Link"].min_range_m or 0.0,
                self.source_filters["radar_f_Link"].min_range_m or 0.0,
                self.source_filters["radar_r_Link"].min_range_m or 0.0,
                self.source_filters["radar_uper_Link"].min_z or -999.0,
                self.source_filters["radar_uper_Link"].max_z or 999.0,
                self.source_filters["radar_f_Link"].min_z or -999.0,
                self.source_filters["radar_f_Link"].max_z or 999.0,
                self.source_filters["radar_r_Link"].min_z or -999.0,
                self.source_filters["radar_r_Link"].max_z or 999.0,
                stats_period,
            )
        )

    @staticmethod
    def _fresh_stats():
        return {
            "published": 0,
            "drop_no_chin_tail": 0,
            "drop_tf": 0,
            "drop_merge_error": 0,
            "drop_pending_overflow": 0,
            "fallback_missing_side": 0,
            "fallback_back_only_count": 0,
            "chin_stall_count": 0,
            "tail_stall_count": 0,
            "points_total": 0,        # sum of merged.width across published frames
            "points_back": 0,
            "points_chin": 0,
            "points_tail": 0,
            "max_sync_delta_s": 0.0,  # worst |chin-back| or |tail-back| this window
        }

    def _log_stats(self):
        s = self._stats
        attempts = (
            s["published"]
            + s["drop_no_chin_tail"]
            + s["drop_tf"]
            + s["drop_merge_error"]
            + s["drop_pending_overflow"]
        )
        rate_hz = s["published"] / self._stats_period if self._stats_period > 0 else 0.0
        if s["published"] > 0:
            avg_pts = s["points_total"] // s["published"]
            avg_back = s["points_back"] // s["published"]
            avg_chin = s["points_chin"] // s["published"]
            avg_tail = s["points_tail"] // s["published"]
        else:
            avg_pts = avg_back = avg_chin = avg_tail = 0
        self.get_logger().info(
            "stats[%.1fs]: pub=%d (%.1fHz) drop=%d "
            "(no_sync=%d, pending_overflow=%d, tf=%d, merge=%d, fallback=%d) "
            "stalls=(chin=%d tail=%d) pts/frame=%d "
            "(back=%d chin=%d tail=%d) max_sync_dt=%.1fms"
            % (
                self._stats_period,
                s["published"], rate_hz,
                (
                    s["drop_no_chin_tail"]
                    + s["drop_pending_overflow"]
                    + s["drop_tf"]
                    + s["drop_merge_error"]
                ),
                s["drop_no_chin_tail"],
                s["drop_pending_overflow"],
                s["drop_tf"],
                s["drop_merge_error"],
                s["fallback_back_only_count"],
                s["chin_stall_count"],
                s["tail_stall_count"],
                avg_pts, avg_back, avg_chin, avg_tail,
                s["max_sync_delta_s"] * 1000.0,
            )
        )
        if attempts > 0 and s["published"] == 0:
            self.get_logger().warn(
                "merger received %d back clouds but published 0 — check topics, "
                "TF tree, and sync tolerance" % attempts
            )
        self._stats = self._fresh_stats()

    def _chin_callback(self, msg):
        self.chin_cache.append(self._cached_cloud(msg))
        self._process_pending_back_clouds()

    def _tail_callback(self, msg):
        self.tail_cache.append(self._cached_cloud(msg))
        self._process_pending_back_clouds()

    def _back_callback(self, msg):
        cached = self._cached_cloud(msg)
        self._latest_back_timestamp_max = cached.timestamp_max
        self._process_pending_back_clouds()
        if len(self.back_pending) >= self.max_pending_back_frames:
            dropped = self.back_pending.popleft()
            old = dropped.cached
            self._stats["drop_pending_overflow"] += 1
            self.get_logger().warn(
                "drop pending back cloud: side lidar wait queue overflow "
                "pending=%d max=%d oldest_window=%.6f..%.6f"
                % (
                    len(self.back_pending) + 1,
                    self.max_pending_back_frames,
                    old.timestamp_min,
                    old.timestamp_max,
                ),
                throttle_duration_sec=2.0,
            )
        self.back_pending.append(PendingBackCloud(cached, time.monotonic()))
        self._process_pending_back_clouds()

    def _cached_cloud(self, msg):
        try:
            timestamp_min, timestamp_max = pointcloud_timestamp_range(msg)
        except ValueError as exc:
            stamp = _stamp_to_float(msg.header.stamp)
            self.get_logger().warn(
                "cloud timestamp range unavailable for %s, fallback to header stamp: %s"
                % (msg.header.frame_id, exc),
                throttle_duration_sec=2.0,
            )
            return CachedCloud(msg, stamp, stamp, stamp)
        return CachedCloud(
            msg,
            _stamp_to_float(msg.header.stamp),
            timestamp_min,
            timestamp_max,
        )

    def _process_pending_back_clouds(self):
        while self.back_pending:
            pending = self.back_pending[0]
            if not self._try_publish_back_cloud(pending):
                return
            self.back_pending.popleft()

    def _try_publish_back_cloud(self, pending):
        back = pending.cached
        msg = back.cloud
        target_stamp = back.stamp
        timestamp_window = (back.timestamp_min, back.timestamp_max)
        chin_matches = select_overlapping(self.chin_cache, timestamp_window)
        tail_matches = select_overlapping(self.tail_cache, timestamp_window)
        missing = []
        if not chin_matches:
            missing.append("chin")
        if not tail_matches:
            missing.append("tail")

        wait_timed_out = False
        wait_elapsed_s = 0.0
        if missing and should_wait_for_overlapping_clouds(
            timestamp_window,
            missing,
            self.chin_cache,
            self.tail_cache,
        ):
            wait_elapsed_s = self._side_wait_elapsed_s(pending)
            if wait_elapsed_s < self.side_wait_timeout_s:
                return False
            wait_timed_out = True
            for name in missing:
                if name == "chin":
                    self._stats["chin_stall_count"] += 1
                elif name == "tail":
                    self._stats["tail_stall_count"] += 1
        if missing:
            wait_note = (
                " after waiting %.0fms" % (wait_elapsed_s * 1000.0)
                if wait_timed_out
                else ""
            )
            # Throttle the per-frame warn: stats summary covers the count.
            if not self.allow_back_only_fallback:
                self._stats["drop_no_chin_tail"] += 1
                self.get_logger().warn(
                    "drop back cloud%s: no %s cloud overlapping %.6f..%.6f"
                    % (
                        wait_note,
                        "/".join(missing),
                        timestamp_window[0],
                        timestamp_window[1],
                    ),
                    throttle_duration_sec=2.0,
                )
                return True
            self._stats["fallback_missing_side"] += 1
            self._stats["fallback_back_only_count"] += 1
            self.get_logger().warn(
                "fallback to back cloud only%s: no %s cloud overlapping %.6f..%.6f"
                % (
                    wait_note,
                    "/".join(missing),
                    timestamp_window[0],
                    timestamp_window[1],
                ),
                throttle_duration_sec=2.0,
            )

        back_mid = 0.5 * (back.timestamp_min + back.timestamp_max)
        sync_dt = 0.0
        for cached in chin_matches + tail_matches:
            side_mid = 0.5 * (cached.timestamp_min + cached.timestamp_max)
            sync_dt = max(sync_dt, abs(side_mid - back_mid))
        if sync_dt > self._stats["max_sync_delta_s"]:
            self._stats["max_sync_delta_s"] = sync_dt

        labeled_clouds = [("back", back)]
        labeled_clouds.extend(("chin", cached) for cached in chin_matches)
        labeled_clouds.extend(("tail", cached) for cached in tail_matches)

        transforms = []
        source_names = []
        for source_name, cached in labeled_clouds:
            cloud = cached.cloud
            transform = self._lookup_transform(cloud.header.frame_id, cloud.header.stamp)
            if transform is None:
                self._stats["drop_tf"] += 1
                return True
            rotation = _matrix_multiply(
                transform[0],
                self._driver_to_link_rotation_for_source(cloud.header.frame_id),
            )
            source_names.append(source_name)
            transforms.append((
                cloud,
                rotation,
                transform[1],
                self._extra_translation_for_source(cloud.header.frame_id),
                self._extra_rotation_for_source(cloud.header.frame_id),
                self._point_filter_for_source(cloud.header.frame_id),
                self._source_id_for_source_name(source_name),
            ))

        try:
            # Lightning-LM treats the merged output as one RoboSense scan and
            # derives per-point relative time as point.timestamp - header.stamp.
            # Keep the reference back LiDAR scan window so asynchronous chin/tail
            # points do not stretch one frame into a longer pseudo-scan.
            merged = merge_pointclouds(
                transforms,
                timestamp_window=timestamp_window,
            )
        except ValueError as exc:
            self._stats["drop_merge_error"] += 1
            self.get_logger().error("failed to merge point clouds: %s" % exc)
            return True

        # Record per-source point counts for the published frame.
        counts = merged.source_point_counts
        for source_name, count in zip(source_names, counts):
            self._stats[f"points_{source_name}"] += count
        self._stats["points_total"] += merged.width
        self._stats["published"] += 1

        output = PointCloud2()
        output.header = Header()
        output.header.stamp = _float_to_time(merged.min_timestamp)
        output.header.frame_id = self.target_frame
        output.height = merged.height
        output.width = merged.width
        output.fields = merged.fields
        output.is_bigendian = merged.is_bigendian
        output.point_step = merged.point_step
        output.row_step = merged.row_step
        output.data = merged.data
        output.is_dense = merged.is_dense
        self.publisher.publish(output)
        return True

    def _side_wait_elapsed_s(self, pending):
        # Wall time handles a live side LiDAR stall; data time keeps rosbag
        # replay from overflowing pending frames before the wall timeout elapses.
        elapsed = max(0.0, time.monotonic() - pending.enqueued_at)
        if self._latest_back_timestamp_max is not None:
            back_elapsed = self._latest_back_timestamp_max - pending.cached.timestamp_max
            elapsed = max(elapsed, max(0.0, back_elapsed))
        return elapsed

    def _extra_translation_for_source(self, source_frame):
        if source_frame == "radar_f_Link":
            return (0.0, 0.0, self.chin_z_offset_m)
        if source_frame == "radar_r_Link":
            return (0.0, 0.0, self.tail_z_offset_m)
        return ZERO_TRANSLATION

    def _extra_rotation_for_source(self, source_frame):
        if source_frame == "radar_f_Link":
            return self.chin_rotation_offset
        if source_frame == "radar_r_Link":
            return self.tail_rotation_offset
        return IDENTITY_ROTATION

    def _driver_to_link_rotation_for_source(self, source_frame):
        if source_frame == "radar_f_Link":
            return self.chin_driver_to_link_rotation
        if source_frame == "radar_r_Link":
            return self.tail_driver_to_link_rotation
        return IDENTITY_ROTATION

    @staticmethod
    def _source_id_for_source_name(source_name):
        return {"back": 0.0, "chin": 1.0, "tail": 2.0}.get(source_name, 0.0)

    def _point_filter_for_source(self, source_frame):
        return self.source_filters.get(source_frame, SourcePointFilter())

    def _point_filter_from_params(self, source_name):
        min_range = _positive_or_none(
            self.get_parameter(f"{source_name}_min_range_m").value
        )
        max_range = _positive_or_none(
            self.get_parameter(f"{source_name}_max_range_m").value
        )
        min_z = _finite_window_bound(
            self.get_parameter(f"{source_name}_min_z_m").value,
            default=-999.0,
        )
        max_z = _finite_window_bound(
            self.get_parameter(f"{source_name}_max_z_m").value,
            default=999.0,
        )

        kwargs = {
            "min_z": min_z,
            "max_z": max_z,
            "min_range_m": min_range,
            "max_range_m": max_range,
        }

        if source_name == "back" and bool(
            self.get_parameter("back_exclude_box_enabled").value
        ):
            kwargs.update({
                "exclude_min_x": float(self.get_parameter("back_exclude_min_x_m").value),
                "exclude_max_x": float(self.get_parameter("back_exclude_max_x_m").value),
                "exclude_min_y": float(self.get_parameter("back_exclude_min_y_m").value),
                "exclude_max_y": float(self.get_parameter("back_exclude_max_y_m").value),
                "exclude_min_z": float(self.get_parameter("back_exclude_min_z_m").value),
                "exclude_max_z": float(self.get_parameter("back_exclude_max_z_m").value),
            })

        return SourcePointFilter(**kwargs)

    def _lookup_transform(self, source_frame, cloud_stamp):
        """Resolve ``source_frame -> target_frame`` rotation+translation.

        Gen 1 (cache_static_tf=True): first call per source frame waits up to
        ``tf_lookup_timeout`` and caches the result; subsequent calls are O(1).
        Gen 2 (cache_static_tf=False): per-frame lookup at ``cloud_stamp`` so
        actuated joints (e.g. movable head) resolve to the right pose.
        """
        if source_frame == self.target_frame:
            return IDENTITY_ROTATION, ZERO_TRANSLATION

        if self.cache_static_tf and source_frame in self._tf_cache:
            return self._tf_cache[source_frame]

        # Cache miss (or dynamic mode): query tf2.
        # In static mode use Time() = "latest" with a finite timeout to ride
        # out robot_state_publisher startup. In dynamic mode use the cloud's
        # own stamp so moving joints resolve at the right instant.
        query_time = Time() if self.cache_static_tf else cloud_stamp
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                query_time,
                timeout=self.tf_lookup_timeout,
            )
        except Exception as exc:
            self.get_logger().warn(
                "drop cloud: missing TF %s -> %s: %s"
                % (source_frame, self.target_frame, exc)
            )
            return None

        translation = transform.transform.translation
        rotation = _quaternion_to_matrix(transform.transform.rotation)
        result = (rotation, (translation.x, translation.y, translation.z))

        if self.cache_static_tf:
            self._tf_cache[source_frame] = result
            self.get_logger().info(
                "cached static TF %s -> %s" % (source_frame, self.target_frame)
            )
        return result


def _stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _float_to_time(value):
    sec = math.floor(value)
    nanosec = round((value - sec) * 1e9)
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    stamp = Time().to_msg()
    stamp.sec = int(sec)
    stamp.nanosec = int(nanosec)
    return stamp


def _positive_or_none(value):
    value = float(value)
    if value <= 0.0:
        return None
    return value


def _finite_window_bound(value, default):
    value = float(value)
    if math.isclose(value, default, abs_tol=1e-9):
        return None
    return value


def _quaternion_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    # Guard against near-zero norm (degenerate quaternion) to avoid div-by-zero.
    # tf2 should always send unit quaternions, but float drift is possible.
    if norm < 1e-10:
        return IDENTITY_ROTATION
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

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _rpy_degrees_to_matrix(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matrix_multiply(left, right):
    return tuple(
        tuple(
            sum(float(left[row][idx]) * float(right[idx][col]) for idx in range(3))
            for col in range(3)
        )
        for row in range(3)
    )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
