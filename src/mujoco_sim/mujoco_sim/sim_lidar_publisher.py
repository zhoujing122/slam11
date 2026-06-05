"""
MuJoCo 仿真传感器发布节点（LiDAR + IMU 统一时钟）
- 在 MuJoCo 仿真环境中模拟 3 个 LiDAR（下巴/背上/尾巴）
- 同时发布 IMU 数据（与 LiDAR 共享 MuJoCo 仿真时钟）
- 所有时间戳统一使用 data.time，避免时钟不同步

用法：替代 mujoco_quadruped_env.py + sim_imu_bridge.py 运行
  python sim_lidar_publisher.py
"""

import math
import numpy as np
import struct
import threading
import time
import os
import sys

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg

# LCM 消息（同目录）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motor_ctrl_lcmt import motor_ctrl_lcmt
from motor_states_lcmt import motor_states_lcmt
from imu_data_lcmt import imu_data_lcmt

try:
    import lcm
    HAS_LCM = True
except ImportError:
    print("Warning: lcm not installed.")
    HAS_LCM = False

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

# ============================================================
# RoboSense PointCloud2 格式
# ============================================================
POINT_STEP = 26

POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]


def resolve_model_path(*relative_parts):
    """兼容源码运行和 ros2 run 安装运行的模型路径解析。"""
    candidates = []

    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory('mujoco_sim')
            candidates.append(os.path.join(pkg_share, 'models', *relative_parts))
        except Exception:
            pass

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(script_dir, '..', 'models', *relative_parts))
    candidates.append(os.path.join(os.getcwd(), '..', 'models', *relative_parts))
    candidates.append(os.path.join(os.getcwd(), 'models', *relative_parts))

    for path in candidates:
        normalized = os.path.abspath(path)
        if os.path.exists(normalized):
            return normalized

    raise FileNotFoundError(
        'Unable to locate MuJoCo model file. Tried: ' + ', '.join(
            os.path.abspath(path) for path in candidates)
    )


def sim_time_to_ros_time(sim_time):
    """将 MuJoCo 仿真时间 (float seconds) 转为 ROS2 Time 消息"""
    sec = int(sim_time)
    nanosec = int((sim_time - sec) * 1e9)
    return TimeMsg(sec=sec, nanosec=nanosec)


class SimSensorPublisher(Node):
    """MuJoCo 仿真传感器 ROS2 发布节点（LiDAR + IMU 统一时钟）"""

    def __init__(self, model, data, lidar_freq=10.0, imu_freq=200.0):
        super().__init__('sim_lidar_publisher')
        self.model = model
        self.data = data
        self._lock = threading.Lock()

        # ======== LiDAR 发布 ========
        self.pc_pub = self.create_publisher(PointCloud2, '/LIDAR/POINTS', 10)
        self.lidar_timer = self.create_timer(1.0 / lidar_freq, self._lidar_callback)

        # ======== IMU 发布 ========
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 50)
        self.imu_timer = self.create_timer(1.0 / imu_freq, self._imu_callback)
        self._last_imu_time = -1.0

        # /imu/data 使用后背 Airy IMU (radar_uper site)，与 Lightning 点云 frame 一致
        self._imu_body_id = -1
        # 优先用 radar_uper 所在的 body，fallback 到 base_link
        for name in ["base_link", "body", "trunk", "torso"]:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                self._imu_body_id = bid
                break
        # radar_uper site ID，用于获取 IMU 朝向
        self._imu_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, 'radar_uper')
        if self._imu_site_id >= 0:
            self.get_logger().info('IMU: using radar_uper site orientation (back LiDAR IMU)')
        else:
            self.get_logger().warn('IMU: radar_uper site not found, falling back to body IMU')

        # IMU sensor indices. Prefer the back Airy site sensors so /imu/data
        # matches the radar_uper_Link frame used by Lightning-LM.
        sensor_addrs = {}
        first_gyro_adr = None
        first_accel_adr = None
        for i in range(model.nsensor):
            sname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            if not sname:
                continue
            sensor_addrs[sname] = model.sensor_adr[i]
            lname = sname.lower()
            if 'gyro' in lname and first_gyro_adr is None:
                first_gyro_adr = model.sensor_adr[i]
            if 'accel' in lname and first_accel_adr is None:
                first_accel_adr = model.sensor_adr[i]

        self._imu_gyro_adr = sensor_addrs.get('radar_uper_gyro', first_gyro_adr)
        self._imu_accel_adr = sensor_addrs.get('radar_uper_accel', first_accel_adr)

        # ======== LiDAR site IDs ========
        self.site_ids = {}
        for name in ['radar_f', 'radar_uper', 'radar_r']:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            if sid < 0:
                self.get_logger().warn(f'Site "{name}" not found')
            self.site_ids[name] = sid

        self.robot_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

        self._setup_rays(n_channels=16, h_res_deg=3.0,
                         v_fov_deg=(-25.0, 15.0), max_range=50.0)

        self._transforms_computed = False
        self._R_chin_to_back = np.eye(3)
        self._t_chin_to_back = np.zeros(3)
        self._R_tail_to_back = np.eye(3)
        self._t_tail_to_back = np.zeros(3)

        self.get_logger().info(
            f'SimSensorPublisher: LiDAR {self._n_rays} rays/lidar x3 @ {lidar_freq}Hz, '
            f'IMU @ {imu_freq}Hz, unified sim clock')

    # ================================================================
    # IMU 回调
    # ================================================================
    def _imu_callback(self):
        with self._lock:
            sim_time = self.data.time

        # 防止时间戳回跳
        if sim_time <= self._last_imu_time:
            return
        self._last_imu_time = sim_time

        msg = Imu()
        msg.header.stamp = sim_time_to_ros_time(sim_time)
        msg.header.frame_id = 'radar_uper_Link'

        with self._lock:
            # 四元数 - 使用 radar_uper site 的朝向
            if self._imu_site_id >= 0:
                xmat = self.data.site_xmat[self._imu_site_id].reshape(3, 3)
                # 旋转矩阵 -> 四元数 (scipy)
                from scipy.spatial.transform import Rotation
                q = Rotation.from_matrix(xmat).as_quat()  # [x, y, z, w]
                msg.orientation = Quaternion(
                    x=float(q[0]), y=float(q[1]),
                    z=float(q[2]), w=float(q[3]))
            elif self._imu_body_id >= 0:
                quat = self.data.xquat[self._imu_body_id]  # MuJoCo: (w, x, y, z)
                msg.orientation = Quaternion(
                    x=float(quat[1]), y=float(quat[2]),
                    z=float(quat[3]), w=float(quat[0]))
            else:
                msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # 角速度
            if self._imu_gyro_adr is not None:
                adr = self._imu_gyro_adr
                msg.angular_velocity = Vector3(
                    x=float(self.data.sensordata[adr]),
                    y=float(self.data.sensordata[adr + 1]),
                    z=float(self.data.sensordata[adr + 2]))
            else:
                msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)

            # 线加速度
            if self._imu_accel_adr is not None:
                adr = self._imu_accel_adr
                msg.linear_acceleration = Vector3(
                    x=float(self.data.sensordata[adr]),
                    y=float(self.data.sensordata[adr + 1]),
                    z=float(self.data.sensordata[adr + 2]))
            else:
                msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)

        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    # ================================================================
    # LiDAR 回调
    # ================================================================
    def _setup_rays(self, n_channels, h_res_deg, v_fov_deg, max_range):
        self._max_range = max_range
        elevations = np.linspace(
            np.radians(v_fov_deg[0]), np.radians(v_fov_deg[1]), n_channels)
        azimuths = np.arange(0, 360, h_res_deg)
        n_az = len(azimuths)
        azimuths_rad = np.radians(azimuths)

        self._n_rays = n_channels * n_az
        dirs = np.empty((self._n_rays, 3), dtype=np.float64)
        rings = np.empty(self._n_rays, dtype=np.uint16)
        az_indices = np.empty(self._n_rays, dtype=np.int32)

        idx = 0
        for ring, elev in enumerate(elevations):
            ce = np.cos(elev)
            se = np.sin(elev)
            for ai, az in enumerate(azimuths_rad):
                dirs[idx] = [ce * np.cos(az), ce * np.sin(az), se]
                rings[idx] = ring
                az_indices[idx] = ai
                idx += 1

        self._ray_dirs = dirs
        self._ray_rings = rings
        self._ray_az_indices = az_indices
        self._scan_period = 0.1
        self._n_az = n_az

    def _compute_static_transforms(self):
        back_id = self.site_ids['radar_uper']
        chin_id = self.site_ids['radar_f']
        tail_id = self.site_ids['radar_r']
        if back_id < 0 or chin_id < 0 or tail_id < 0:
            return

        back_pos = self.data.site_xpos[back_id]
        back_rot = self.data.site_xmat[back_id].reshape(3, 3)
        chin_pos = self.data.site_xpos[chin_id]
        chin_rot = self.data.site_xmat[chin_id].reshape(3, 3)
        tail_pos = self.data.site_xpos[tail_id]
        tail_rot = self.data.site_xmat[tail_id].reshape(3, 3)

        self._R_chin_to_back = back_rot.T @ chin_rot
        self._t_chin_to_back = back_rot.T @ (chin_pos - back_pos)
        self._R_tail_to_back = back_rot.T @ tail_rot
        self._t_tail_to_back = back_rot.T @ (tail_pos - back_pos)
        self._transforms_computed = True

    def _raycast_single_lidar(self, site_id):
        if site_id < 0:
            return np.empty((0, 3)), np.empty(0, dtype=np.uint16), np.empty(0)

        with self._lock:
            site_pos = self.data.site_xpos[site_id].copy()
            site_rot = self.data.site_xmat[site_id].reshape(3, 3).copy()

        dirs_world = (site_rot @ self._ray_dirs.T).T
        geomid_buf = np.array([-1], dtype=np.int32)
        points_local = []
        rings = []
        az_ids = []

        for i in range(self._n_rays):
            dist = mujoco.mj_ray(
                self.model, self.data,
                site_pos, dirs_world[i],
                None, 1, self.robot_body_id, geomid_buf)

            if 0 < dist < self._max_range:
                hit_world = site_pos + dist * dirs_world[i]
                hit_local = site_rot.T @ (hit_world - site_pos)
                points_local.append(hit_local)
                rings.append(self._ray_rings[i])
                az_ids.append(self._ray_az_indices[i])

        if not points_local:
            return np.empty((0, 3)), np.empty(0, dtype=np.uint16), np.empty(0)

        return (np.array(points_local),
                np.array(rings, dtype=np.uint16),
                np.array(az_ids))

    def _lidar_callback(self):
        if not self._transforms_computed:
            self._compute_static_transforms()

        with self._lock:
            sim_time = self.data.time

        pts_back, rings_back, az_back = self._raycast_single_lidar(
            self.site_ids['radar_uper'])
        pts_chin, rings_chin, az_chin = self._raycast_single_lidar(
            self.site_ids['radar_f'])
        pts_tail, rings_tail, az_tail = self._raycast_single_lidar(
            self.site_ids['radar_r'])

        if len(pts_chin) > 0:
            pts_chin = (self._R_chin_to_back @ pts_chin.T).T + self._t_chin_to_back
        if len(pts_tail) > 0:
            pts_tail = (self._R_tail_to_back @ pts_tail.T).T + self._t_tail_to_back

        all_pts = []
        all_rings = []
        all_az = []
        for pts, rng, az in [(pts_back, rings_back, az_back),
                              (pts_chin, rings_chin, az_chin),
                              (pts_tail, rings_tail, az_tail)]:
            if len(pts) > 0:
                all_pts.append(pts)
                all_rings.append(rng)
                all_az.append(az)

        if not all_pts:
            return

        all_pts = np.concatenate(all_pts)
        all_rings = np.concatenate(all_rings)
        all_az = np.concatenate(all_az)
        n_points = len(all_pts)

        buf = bytearray(n_points * POINT_STEP)
        for i in range(n_points):
            t_offset = all_az[i] / self._n_az * self._scan_period
            t_point = sim_time + t_offset
            offset = i * POINT_STEP
            struct.pack_into('<ffff', buf, offset,
                             all_pts[i, 0], all_pts[i, 1], all_pts[i, 2], 50.0)
            struct.pack_into('<H', buf, offset + 16, int(all_rings[i]))
            struct.pack_into('<d', buf, offset + 18, t_point)

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = sim_time_to_ros_time(sim_time)
        msg.header.frame_id = 'radar_uper_Link'
        msg.height = 1
        msg.width = n_points
        msg.fields = POINT_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * n_points
        msg.data = bytes(buf)
        msg.is_dense = True

        self.pc_pub.publish(msg)


# ============================================================
# 路径可视化 (MuJoCo viewer)
# ============================================================

class PathVisualizer:
    """在 MuJoCo viewer 中渲染规划路径和机器人轨迹"""

    def __init__(self, viewer, model, data):
        self.viewer = viewer
        self.model = model
        self.data = data
        self._planned_path = []  # [(x, y), ...]
        self._robot_trail = []   # [(x, y), ...] 机器人走过的轨迹
        self._trail_max = 2000
        self._lock = threading.Lock()

        # 机器人 body ID
        self._body_id = -1
        for name in ["base_link", "body", "trunk"]:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                self._body_id = bid
                break

    def update_planned_path(self, path_points):
        """更新规划路径"""
        with self._lock:
            self._planned_path = list(path_points)

    def record_trail(self):
        """记录机器人当前位置到轨迹"""
        if self._body_id < 0:
            return
        pos = self.data.xpos[self._body_id]
        x, y = float(pos[0]), float(pos[1])
        with self._lock:
            if (not self._robot_trail or
                    (abs(x - self._robot_trail[-1][0]) > 0.05 or
                     abs(y - self._robot_trail[-1][1]) > 0.05)):
                self._robot_trail.append((x, y))
                if len(self._robot_trail) > self._trail_max:
                    self._robot_trail.pop(0)

    def render(self):
        """将路径渲染到 MuJoCo viewer 的 user scene 中"""
        if not hasattr(self.viewer, 'user_scn'):
            return

        scn = self.viewer.user_scn
        scn.ngeom = 0  # 清除上一帧

        with self._lock:
            planned = self._planned_path[:]
            trail = self._robot_trail[:]

        # 渲染规划路径 (绿色线段)
        path_height = 0.02
        max_geoms = min(len(planned) - 1, 400)
        for i in range(max_geoms):
            if scn.ngeom >= scn.maxgeom:
                break
            p0 = planned[i]
            p1 = planned[i + 1]
            self._add_line(scn, p0, p1, path_height,
                           rgba=[0.0, 1.0, 0.2, 0.8], width=0.03)

        # 渲染航点标记 (黄色球)
        # 从 auto_navigator 中获取的航点用大球标记
        waypoint_indices = self._find_waypoint_indices(planned)
        for idx in waypoint_indices:
            if scn.ngeom >= scn.maxgeom:
                break
            if idx < len(planned):
                self._add_sphere(scn, planned[idx], 0.08,
                                 rgba=[1.0, 1.0, 0.0, 0.9])

        # 渲染机器人实际轨迹 (红色线段)
        trail_height = 0.03
        step = max(1, len(trail) // 300)  # 限制绘制数量
        for i in range(0, len(trail) - step, step):
            if scn.ngeom >= scn.maxgeom:
                break
            p0 = trail[i]
            p1 = trail[min(i + step, len(trail) - 1)]
            self._add_line(scn, p0, p1, trail_height,
                           rgba=[1.0, 0.2, 0.0, 0.7], width=0.02)

    def _find_waypoint_indices(self, path):
        """在路径中找到转折点作为航点标记"""
        if len(path) < 3:
            return [0, len(path) - 1] if path else []
        indices = [0]
        for i in range(1, len(path) - 1):
            # 检测方向变化
            dx1 = path[i][0] - path[i-1][0]
            dy1 = path[i][1] - path[i-1][1]
            dx2 = path[i+1][0] - path[i][0]
            dy2 = path[i+1][1] - path[i][1]
            # 角度变化大于30度
            cross = abs(dx1 * dy2 - dy1 * dx2)
            dot = dx1 * dx2 + dy1 * dy2
            if cross > 0.05 and (dot < 0.8 * (
                    math.hypot(dx1, dy1) * math.hypot(dx2, dy2) + 1e-6)):
                indices.append(i)
        indices.append(len(path) - 1)
        return indices

    def _add_line(self, scn, p0, p1, height, rgba, width=0.02):
        """添加一条线段 (用 capsule 模拟)"""
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, dtype=np.float32))

        # 计算 capsule from-to
        from_pt = np.array([p0[0], p0[1], height], dtype=np.float64)
        to_pt = np.array([p1[0], p1[1], height], dtype=np.float64)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                             width, from_pt, to_pt)
        g.rgba[:] = np.array([rgba[0], rgba[1], rgba[2], rgba[3]], dtype=np.float32) * 255

        scn.ngeom += 1

    def _add_sphere(self, scn, pos, radius, rgba):
        """添加一个球体标记"""
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0, 0], dtype=np.float64),
            np.array([pos[0], pos[1], 0.05], dtype=np.float64),
            np.zeros(9, dtype=np.float64),
            np.array(rgba, dtype=np.float32))

        scn.ngeom += 1


# ============================================================
# 主函数
# ============================================================

def main():
    import math
    try:
        from .mujoco_quadruped_env import QuadrupedEnv
    except ImportError:
        from mujoco_quadruped_env import QuadrupedEnv

    model_path = resolve_model_path('future_d100', 'scene_slam.xml')

    env = QuadrupedEnv(
        model_path,
        use_lcm=True,
        control_freq=1000.0,
        viewer_freq=60.0,
    )

    mujoco.mj_forward(env.model, env.data)

    rclpy.init()
    sensor_node = SimSensorPublisher(env.model, env.data,
                                      lidar_freq=10.0, imu_freq=200.0)

    # 订阅规划路径
    path_viz = None
    path_lock = threading.Lock()

    def path_callback(msg: Path):
        nonlocal path_viz
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with path_lock:
            if path_viz is not None:
                path_viz.update_planned_path(points)

    sensor_node.create_subscription(Path, '/planned_path', path_callback, 10)

    spin_thread = threading.Thread(target=rclpy.spin, args=(sensor_node,), daemon=True)
    spin_thread.start()

    env.start_viewer()

    # 创建路径可视化器（viewer 启动后）
    if env.viewer is not None:
        path_viz = PathVisualizer(env.viewer, env.model, env.data)
        print('[SLAM Sim] Path visualization enabled in MuJoCo viewer')
        print('[SLAM Sim]   Green = planned path')
        print('[SLAM Sim]   Red   = robot actual trail')
        print('[SLAM Sim]   Yellow spheres = waypoints')

    try:
        print('\n[SLAM Sim] Starting MuJoCo + LiDAR + IMU simulation...')
        print('[SLAM Sim] Close viewer window to stop\n')

        frame_count = 0
        while env.is_viewer_running():
            env.step()

            # 每 10 帧更新一次可视化 (约 6 Hz)
            frame_count += 1
            if path_viz and frame_count % 10 == 0:
                path_viz.record_trail()
                path_viz.render()

    except KeyboardInterrupt:
        print('\n[SLAM Sim] Stopping...')
    finally:
        env.close()
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
