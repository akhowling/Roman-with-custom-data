# #!/usr/bin/env python3
# import rclpy
# import numpy as np
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, Image, CameraInfo
# from sensor_msgs_py import point_cloud2
# from message_filters import Subscriber, ApproximateTimeSynchronizer

# try:
#     import cv2
#     HAS_CV2 = True
# except Exception:
#     HAS_CV2 = False


# def t2ns(t) -> int:
#     return int(t.sec) * 1_000_000_000 + int(t.nanosec)


# class LidarToDepth(Node):
#     """
#     Publishes a depth image compatible with nodes that expect 16UC1 depth (millimeters).

#     Strategy:
#       1) project LiDAR points -> per-pixel nearest depth (z-buffer)
#       2) optionally dilate / fill to increase depth coverage under masks
#       3) fill any remaining holes with a default depth (mm)

#     This avoids sparse NaN depth that causes FastSAM to output observations: [].
#     """

#     def __init__(self):
#         super().__init__("lidar_to_depth")

#         # Topics
#         self.declare_parameter("cloud_topic", "/ugv/velodyne_points")
#         self.declare_parameter("rgb_topic", "/robot/d455/color/image_raw")
#         self.declare_parameter("caminfo_topic", "/robot/d455/aligned_depth_to_color/camera_info")
#         self.declare_parameter("depth_topic", "/robot/d455/aligned_depth_to_color/image_raw")

#         # Camera intrinsics fallback (overwritten by CameraInfo when available)
#         self.declare_parameter("width", 640)
#         self.declare_parameter("height", 480)
#         self.declare_parameter("fx", 415.69219771027923)
#         self.declare_parameter("fy", 415.69219771027923)
#         self.declare_parameter("cx", 320.0)
#         self.declare_parameter("cy", 240.0)

#         # Frame/axes assumption (your old heuristic)
#         self.declare_parameter("frame_id", "camera_frame")
#         self.declare_parameter("assume_base_link_axes", True)  # your z=x_l, x=-y_l, y=-z_l

#         # Depth limits + fill
#         self.declare_parameter("min_depth_m", 0.2)
#         self.declare_parameter("max_depth_m", 30.0)
#         self.declare_parameter("default_depth_mm", 2000)  # fill holes with 2m by default
#         self.declare_parameter("fill_holes", True)

#         # Coverage amplification
#         self.declare_parameter("dilate_px", 4)     # kernel radius
#         self.declare_parameter("dilate_iters", 2)

#         # Stamping (FastSAM sync)
#         self.declare_parameter("stamp_mode", "rgb")  # rgb|cloud|max

#         # Sync settings
#         self.declare_parameter("sync_slop", 2.0)
#         self.declare_parameter("sync_queue", 50)

#         # Load params
#         self.cloud_topic = self.get_parameter("cloud_topic").value
#         self.rgb_topic = self.get_parameter("rgb_topic").value
#         self.caminfo_topic = self.get_parameter("caminfo_topic").value
#         self.depth_topic = self.get_parameter("depth_topic").value

#         self.W = int(self.get_parameter("width").value)
#         self.H = int(self.get_parameter("height").value)
#         self.fx = float(self.get_parameter("fx").value)
#         self.fy = float(self.get_parameter("fy").value)
#         self.cx = float(self.get_parameter("cx").value)
#         self.cy = float(self.get_parameter("cy").value)

#         self.frame_id = self.get_parameter("frame_id").value
#         self.assume_base_link_axes = bool(self.get_parameter("assume_base_link_axes").value)

#         self.min_z = float(self.get_parameter("min_depth_m").value)
#         self.max_z = float(self.get_parameter("max_depth_m").value)
#         self.default_mm = int(self.get_parameter("default_depth_mm").value)
#         self.fill_holes = bool(self.get_parameter("fill_holes").value)

#         self.dilate_px = int(self.get_parameter("dilate_px").value)
#         self.dilate_iters = int(self.get_parameter("dilate_iters").value)

#         self.stamp_mode = self.get_parameter("stamp_mode").value.lower()
#         self.sync_slop = float(self.get_parameter("sync_slop").value)
#         self.sync_queue = int(self.get_parameter("sync_queue").value)

#         self.pub = self.create_publisher(Image, self.depth_topic, 10)
#         self.create_subscription(CameraInfo, self.caminfo_topic, self._caminfo_cb, 10)

#         self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
#         self.cloud_sub = Subscriber(self, PointCloud2, self.cloud_topic)
#         self.sync = ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.cloud_sub],
#             queue_size=self.sync_queue,
#             slop=self.sync_slop
#         )
#         self.sync.registerCallback(self.cb)

#         self.get_logger().info(
#             f"[lidar_to_depth] out={self.depth_topic} (16UC1 mm) default={self.default_mm}mm "
#             f"fill_holes={self.fill_holes} dilate={self.dilate_px}px x{self.dilate_iters} "
#             f"(cv2={'yes' if HAS_CV2 else 'no'}) stamp_mode={self.stamp_mode}"
#         )

#     def _caminfo_cb(self, msg: CameraInfo):
#         # Update intrinsics from camera_info
#         try:
#             self.W = int(msg.width)
#             self.H = int(msg.height)
#             self.fx = float(msg.k[0])
#             self.fy = float(msg.k[4])
#             self.cx = float(msg.k[2])
#             self.cy = float(msg.k[5])
#         except Exception:
#             pass

#     def _pick_stamp(self, rgb: Image, cloud: PointCloud2):
#         if self.stamp_mode == "cloud":
#             return cloud.header.stamp
#         if self.stamp_mode == "max":
#             return rgb.header.stamp if t2ns(rgb.header.stamp) >= t2ns(cloud.header.stamp) else cloud.header.stamp
#         return rgb.header.stamp  # rgb

#     def cb(self, rgb: Image, cloud: PointCloud2):
#         H, W = self.H, self.W

#         # best_mm holds nearest depth per pixel; start as "inf"
#         INF = 65535
#         best = np.full((H, W), INF, dtype=np.uint16)

#         # Project points
#         for (x_l, y_l, z_l) in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
#             if self.assume_base_link_axes:
#                 z = float(x_l)      # forward
#                 x = float(-y_l)     # right
#                 y = float(-z_l)     # down
#             else:
#                 x = float(x_l)
#                 y = float(y_l)
#                 z = float(z_l)

#             if z <= 0.0 or z < self.min_z or z > self.max_z:
#                 continue

#             u = int((self.fx * x / z) + self.cx)
#             v = int((self.fy * y / z) + self.cy)
#             if 0 <= u < W and 0 <= v < H:
#                 mm = int(z * 1000.0)
#                 if mm < 0:
#                     continue
#                 if mm > 65534:
#                     mm = 65534
#                 # z-buffer: keep nearest
#                 if mm < best[v, u]:
#                     best[v, u] = np.uint16(mm)

#         depth = np.zeros((H, W), dtype=np.uint16)
#         valid = best < INF
#         depth[valid] = best[valid]  # elsewhere stays 0

#         # Fill / dilate for mask overlap
#         if self.fill_holes and HAS_CV2 and valid.any():
#             k = 2 * self.dilate_px + 1
#             kernel = np.ones((k, k), np.uint8)

#             # We want to propagate *minimum* depth outward.
#             # Use inverse trick so dilation (max) propagates nearer (smaller) depths.
#             inv = np.zeros_like(depth, dtype=np.uint16)
#             inv[depth > 0] = (INF - depth[depth > 0])

#             inv_d = cv2.dilate(inv, kernel, iterations=self.dilate_iters)
#             filled = np.zeros_like(depth, dtype=np.uint16)
#             filled[inv_d > 0] = (INF - inv_d[inv_d > 0])

#             # Fill holes (where depth==0) from filled
#             holes = (depth == 0)
#             depth[holes] = filled[holes]

#         # Finally: any remaining zeros -> default depth (dense, like dummy, but lidar overrides remain)
#         depth[depth == 0] = np.uint16(np.clip(self.default_mm, 1, 65534))

#         # Publish Image
#         out = Image()
#         out.header.stamp = self._pick_stamp(rgb, cloud)
#         out.header.frame_id = self.frame_id
#         out.height = H
#         out.width = W
#         out.encoding = "16UC1"
#         out.is_bigendian = 0
#         out.step = W * 2
#         out.data = depth.tobytes()

#         self.pub.publish(out)


# def main():
#     rclpy.init()
#     node = LidarToDepth()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from collections import deque

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer


def t2ns(t) -> int:
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)


def quat_to_R(qx, qy, qz, qw) -> np.ndarray:
    # (x,y,z,w) quaternion -> 3x3 rotation
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),     2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),         1 - 2*(xx + yy)],
    ], dtype=np.float32)


def nearest_pose(poses, stamp_ns: int):
    """poses: deque of (t_ns, t_vec(3,), R(3,3))"""
    if not poses:
        return None
    # linear search over small deque
    best = None
    best_dt = None
    for (tns, tvec, R) in poses:
        dt = abs(tns - stamp_ns)
        if best_dt is None or dt < best_dt:
            best_dt = dt
            best = (tns, tvec, R)
    return best


class LidarToDepth(Node):
    """
    Accumulate N LiDAR scans (optionally motion compensated using odom) and project into camera.
    Publishes 16UC1 depth in millimeters.

    Assumptions (matches your earlier mapping):
      - PointCloud points are in base_link-style coords: x forward, y left, z up.
      - Camera optical coords: x right, y down, z forward.

      Mapping base -> cam_optical:
        x_cam = -y_base
        y_cam = -z_base
        z_cam =  x_base
    """

    def __init__(self):
        super().__init__("lidar_to_depth")

        # Topics
        self.declare_parameter("cloud_topic", "/ugv/velodyne_points")
        self.declare_parameter("rgb_topic", "/robot/d455/color/image_raw")
        self.declare_parameter("caminfo_topic", "/robot/d455/aligned_depth_to_color/camera_info")
        self.declare_parameter("odom_topic", "/robot/odom")
        self.declare_parameter("depth_topic", "/robot/d455/aligned_depth_to_color/image_raw")

        # Camera intrinsics fallback (updated from CameraInfo)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 415.69219771027923)
        self.declare_parameter("fy", 415.69219771027923)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("frame_id", "camera_frame")

        # Accumulation / perf
        self.declare_parameter("max_scans", 10)          # number of scans to accumulate
        self.declare_parameter("max_age_sec", 1.0)       # drop scans older than this
        self.declare_parameter("stride", 4)              # subsample points: take every stride-th point
        self.declare_parameter("use_motion_comp", True)  # use odom to de-smear

        # Depth limits
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 30.0)

        # Sync
        self.declare_parameter("sync_slop", 0.2)
        self.declare_parameter("sync_queue", 50)

        # Hole policy: keep it simple here; you can add cv2 nearest-fill later if needed
        self.declare_parameter("default_depth_mm", 0)    # 0 = leave holes as 0 (sparser but "realer")
        # If you want guaranteed observations, set default_depth_mm=2000 (but that reintroduces "dummy-like" fill)

        # Load params
        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.caminfo_topic = self.get_parameter("caminfo_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value

        self.W = int(self.get_parameter("width").value)
        self.H = int(self.get_parameter("height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.max_scans = int(self.get_parameter("max_scans").value)
        self.max_age_ns = int(float(self.get_parameter("max_age_sec").value) * 1e9)
        self.stride = max(1, int(self.get_parameter("stride").value))
        self.use_motion_comp = bool(self.get_parameter("use_motion_comp").value)

        self.min_z = float(self.get_parameter("min_depth_m").value)
        self.max_z = float(self.get_parameter("max_depth_m").value)

        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.sync_queue = int(self.get_parameter("sync_queue").value)

        self.default_depth_mm = int(self.get_parameter("default_depth_mm").value)

        # Buffers
        self.odom_buf = deque(maxlen=4000)   # store a few seconds worth
        self.scan_buf = deque()              # (t_ns, points_base (N,3), pose_at_t (tvec,R))

        # ROS I/O
        self.pub = self.create_publisher(Image, self.depth_topic, 10)
        self.create_subscription(CameraInfo, self.caminfo_topic, self._caminfo_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 100)

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.cloud_sub = Subscriber(self, PointCloud2, self.cloud_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.cloud_sub],
            queue_size=self.sync_queue,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self._cb)

        self.get_logger().info(
            f"[lidar_to_depth] accumulating max_scans={self.max_scans}, max_age_sec={self.max_age_ns/1e9:.2f}, stride={self.stride}, motion_comp={self.use_motion_comp}\n"
            f"               publishing 16UC1 to {self.depth_topic} (default_depth_mm={self.default_depth_mm})"
        )

    def _caminfo_cb(self, msg: CameraInfo):
        try:
            self.W = int(msg.width)
            self.H = int(msg.height)
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        st = msg.header.stamp
        tns = t2ns(st)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tvec = np.array([p.x, p.y, p.z], dtype=np.float32)
        R = quat_to_R(q.x, q.y, q.z, q.w)
        self.odom_buf.append((tns, tvec, R))

    def _read_points_subsampled(self, cloud: PointCloud2) -> np.ndarray:
        pts = []
        i = 0
        for x, y, z in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            if (i % self.stride) == 0:
                pts.append((x, y, z))
            i += 1
        if not pts:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    def _cb(self, rgb: Image, cloud: PointCloud2):
        now_ns = t2ns(rgb.header.stamp)

        pts_base = self._read_points_subsampled(cloud)
        if pts_base.shape[0] == 0:
            return

        # pose at scan time (nearest)
        pose_scan = nearest_pose(self.odom_buf, t2ns(cloud.header.stamp))
        pose_now  = nearest_pose(self.odom_buf, now_ns)

        # store scan
        self.scan_buf.append((t2ns(cloud.header.stamp), pts_base, pose_scan))
        # prune by age and max_scans
        while self.scan_buf and (now_ns - self.scan_buf[0][0] > self.max_age_ns):
            self.scan_buf.popleft()
        while len(self.scan_buf) > self.max_scans:
            self.scan_buf.popleft()

        # build accumulated points in *current* base frame
        acc_pts_base_now = []

        if self.use_motion_comp and (pose_now is not None):
            _, t_now, R_now = pose_now
            R_now_T = R_now.T
            for (ts, pts, pose_s) in self.scan_buf:
                if pose_s is None:
                    # no pose, fall back: treat as already in current frame
                    acc_pts_base_now.append(pts)
                    continue
                _, t_s, R_s = pose_s
                # points in odom: p_odom = R_s * p + t_s
                p_odom = (R_s @ pts.T).T + t_s
                # to current base: p_now = R_now^T * (p_odom - t_now)
                p_now = (R_now_T @ (p_odom - t_now).T).T
                acc_pts_base_now.append(p_now)
        else:
            # no motion comp
            for (_, pts, _) in self.scan_buf:
                acc_pts_base_now.append(pts)

        P = np.concatenate(acc_pts_base_now, axis=0)
        if P.shape[0] == 0:
            return

        # base -> camera optical mapping
        x_base = P[:, 0]
        y_base = P[:, 1]
        z_base = P[:, 2]
        z_cam = x_base
        x_cam = -y_base
        y_cam = -z_base

        # depth filter
        ok = (z_cam > 0.0) & (z_cam >= self.min_z) & (z_cam <= self.max_z)
        x_cam = x_cam[ok]
        y_cam = y_cam[ok]
        z_cam = z_cam[ok]
        if z_cam.size == 0:
            return

        # project
        u = (self.fx * x_cam / z_cam + self.cx).astype(np.int32)
        v = (self.fy * y_cam / z_cam + self.cy).astype(np.int32)
        inside = (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H)
        u = u[inside]
        v = v[inside]
        z_cam = z_cam[inside]
        if z_cam.size == 0:
            return

        z_mm = np.clip((z_cam * 1000.0).astype(np.int32), 1, 65534).astype(np.uint16)

        # z-buffer via np.minimum.at
        depth_flat = np.full(self.H * self.W, 65535, dtype=np.uint16)
        idx = v * self.W + u
        np.minimum.at(depth_flat, idx, z_mm)

        depth = depth_flat.reshape(self.H, self.W)
        # convert 65535 -> 0 (missing)
        depth[depth == 65535] = 0

        if self.default_depth_mm > 0:
            depth[depth == 0] = np.uint16(min(max(self.default_depth_mm, 1), 65534))

        out = Image()
        out.header.stamp = rgb.header.stamp
        out.header.frame_id = self.frame_id
        out.height = self.H
        out.width = self.W
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = self.W * 2
        out.data = depth.tobytes()

        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarToDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
