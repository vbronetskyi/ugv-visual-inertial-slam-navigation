#!/usr/bin/env python3
import math
import re
import sys
import time as pytime
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

POSE_FILE = "/tmp/isaac_pose.txt"
SPAWN_X, SPAWN_Y = -95.0, -6.0

# fusion parameters
SLAM_POS_ALPHA = 0.7
ODOM_POS_ALPHA = 0.3
SLAM_YAW_ALPHA = 0.15
# COSTMAP_INFLATION = 0.5  # tighter = squeeze, looser = miss narrow gaps
GYRO_DEADZONE = 0.08
GYRO_LPF_ALPHA = 0.3
JUMP_THRESHOLD = 0.5
YAW_JUMP_THRESHOLD = 0.3



def wall_stamp():
    t = pytime.time()
    return TimeMsg(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class TFRelay(Node):
    def __init__(self, use_gt=False, slam_frame=False, encoder_imu=False, slam_encoder=False):
        super().__init__('tf_relay')
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.use_gt = use_gt
        self.slam_frame = slam_frame
        self.encoder_imu = encoder_imu
        self.slam_encoder = slam_encoder
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.pose_file = POSE_FILE
        self.last_x = SPAWN_X
        self.last_y = SPAWN_Y
        self.last_qz = 0.0
        self.last_qw = 1.0
        self.slam_origin = None

        # static TF: base_link -> camera_link, base_link -> imu_link
        self.static_br = StaticTransformBroadcaster(self)
        cam_tf = TransformStamped()
        cam_tf.header.stamp = wall_stamp()
        cam_tf.header.frame_id = 'base_link'
        cam_tf.child_frame_id = 'camera_link'
        cam_tf.transform.translation.x = 0.5
        cam_tf.transform.translation.z = 0.48
        cam_tf.transform.rotation.w = 1.0
        imu_tf = TransformStamped()
        imu_tf.header.stamp = wall_stamp()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.2
        imu_tf.transform.translation.z = 0.38
        imu_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform([cam_tf, imu_tf])

        # depth -> pointcloud
        self.fx = self.fy = 320.0
        self.cx, self.cy = 320.0, 240.0
        self.pc_pub = self.create_publisher(PointCloud2, '/depth_points', 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.caminfo_cb, 10)

        # IMU relay at 200Hz
        self.imu_pub = self.create_publisher(Imu, '/imu/data_wall', 50)
        self.imu_file = '/tmp/isaac_imu.txt'
        self.imu_timer = self.create_timer(1.0 / 200.0, self.imu_tick)
        self.last_imu_data = None

        # wheel odometry from cmd_vel
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_timer = self.create_timer(0.05, self.odom_integrate_tick)

        # fusion state
        self.fused_x = 0.0
        self.fused_y = 0.0
        self.fused_yaw = 0.0
        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_slam_nx = 0.0
        self.prev_slam_ny = 0.0
        self.prev_slam_nyaw = 0.0
        self.filter_initialized = False
        self.log_counter = 0

        # IMU gyro yaw
        self.imu_yaw = 0.0
        self.imu_yaw_initialized = False
        self.prev_imu_time = pytime.time()
        self.last_imu_yaw_rate = 0.0
        self.filtered_yaw_rate = 0.0

        # encoder+IMU state (Level 2 localization)
        self.enc_x = SPAWN_X
        self.enc_y = SPAWN_Y
        self.enc_yaw = 0.0
        self.prev_gt_x = None
        self.prev_gt_y = None
        self.enc_total_dist = 0.0
        self.ENCODER_NOISE = 0.005  # 0.5% distance noise
        self.HEADING_NOISE = 0.002  # rad per tick heading noise

        # Override gyro parameters for encoder+IMU mode
        if encoder_imu:
            global GYRO_DEADZONE, GYRO_LPF_ALPHA
            GYRO_DEADZONE = 0.01  # much lower - real IMU reads ~0.03 rad/s during turns
            GYRO_LPF_ALPHA = 0.15  # smoother filtering

        # SLAM+encoder fusion state (Level 3)
        self.slam_pose_file = '/tmp/slam_pose.txt'
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_yaw = 0.0
        self.slam_origin = None
        self.slam_tracking = False
        self.slam_last_update = 0.0
        self.slam_frames = 0
        self.slam_lost = 0
        self.slam_stale_threshold = 2.0  # seconds without update = stale
        self.slam_last_timestamp = -1.0
        self.using_slam = False  # current source flag for logging
        self.T_nav_slam = None   # SE(3) alignment transform: SLAM->nav
        # v10: averaging window for stable alignment (instead of single sample)
        self._align_buf = []            # [(sx,sy,sz,sqx,sqy,sqz,sqw, gt_x, gt_y, gt_yaw)]
        self._align_gt_pos0 = None      # first GT pos seen while buffering
        self.ALIGN_WINDOW_SAMPLES = 50          # ~2.5s at 20Hz
        self.ALIGN_MAX_GT_DISP_M = 0.15         # must stay within this window to trust
        self.ALIGN_MAX_YAW_STD_DEG = 0.5        # reject alignment if jittery

        self.start_time = pytime.time()
        if slam_encoder:
            mode = "SLAM+ENCODER (ORB-SLAM3 + encoder fallback)"
        elif encoder_imu:
            mode = "ENCODER+IMU (realistic sensors)"
        elif slam_frame:
            mode = "SLAM-FRAME (nav in SLAM coords)"
        elif use_gt:
            mode = "GT"
        else:
            mode = "SLAM->world"
        self.get_logger().info(f'TF relay started, mode={mode}')

    def cmd_vel_cb(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z

    def odom_integrate_tick(self):
        dt = 0.05
        self.odom_x += self.cmd_lin * dt * math.cos(self.odom_yaw)
        self.odom_y += self.cmd_lin * dt * math.sin(self.odom_yaw)
        self.odom_yaw += self.cmd_ang * dt

    def tick(self):
        try:
            with open(self.pose_file, 'r') as f:
                parts = f.readline().strip().split()
            if len(parts) < 7:
                return
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            qx, qy, qz, qw = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
        except (FileNotFoundError, ValueError, IndexError):
            return

        now = wall_stamp()
        self.last_x = x
        self.last_y = y
        self.last_qz = qz
        self.last_qw = qw

        if self.slam_encoder:
            self._tick_slam_encoder(now, x, y, z, qx, qy, qz, qw)
        elif self.encoder_imu:
            self._tick_encoder_imu(now, x, y, z, qx, qy, qz, qw)
        elif self.slam_frame:
            self._tick_slam_frame(now, x, y, z, qx, qy, qz, qw)
        elif self.use_gt:
            self._tick_gt(now, x, y, z, qx, qy, qz, qw)
        else:
            self._tick_slam_world(now, x, y, z, qx, qy, qz, qw)

    def _tick_gt(self, now, x, y, z, qx, qy, qz, qw):
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = x
        t2.transform.translation.y = y
        t2.transform.translation.z = z
        t2.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.br.sendTransform([t1, t2])
        self._publish_odom(now, x, y, z, qx, qy, qz, qw)

    def _read_slam_pose_raw(self):
        """Read raw SLAM pose from file. Returns (sx,sy,sz,sqx,sqy,sqz,sqw, ok)."""
        try:
            with open(self.slam_pose_file, 'r') as f:
                lines = f.readlines()
            if len(lines) < 1:
                return 0, 0, 0, 0, 0, 0, 1, False
            parts = lines[0].strip().split()
            if len(parts) < 8:
                return 0, 0, 0, 0, 0, 0, 1, False

            ts = float(parts[0])
            sx, sy, sz = float(parts[1]), float(parts[2]), float(parts[3])
            sqx, sqy, sqz, sqw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])

            # Stale detection
            if abs(ts - self.slam_last_timestamp) < 0.001:
                if pytime.time() - self.slam_last_update > self.slam_stale_threshold:
                    return 0, 0, 0, 0, 0, 0, 1, False
            else:
                self.slam_last_timestamp = ts
                self.slam_last_update = pytime.time()

            # Parse frames/lost from second line
            if len(lines) >= 2:
                m_f = re.search(r'frames=(\d+)', lines[1])
                m_l = re.search(r'lost=(\d+)', lines[1])
                if m_f: self.slam_frames = int(m_f.group(1))
                if m_l: self.slam_lost = int(m_l.group(1))

            return sx, sy, sz, sqx, sqy, sqz, sqw, True
        except (FileNotFoundError, ValueError, IndexError):
            return 0, 0, 0, 0, 0, 0, 1, False

    def _slam_se3_to_nav(self, sx, sy, sz, sqx, sqy, sqz, sqw):
        """Convert SLAM camera pose to nav frame using proper SE(3)->SE(2).

        On first call: compute T_nav_slam alignment transform.
        Subsequent calls: T_nav = T_nav_slam @ T_slam, then project to 2D.
        """
        # Build SLAM 4x4 transform
        R_slam = ScipyRotation.from_quat([sqx, sqy, sqz, sqw]).as_matrix()
        T_slam = np.eye(4)
        T_slam[:3, :3] = R_slam
        T_slam[:3, 3] = [sx, sy, sz]

        if self.T_nav_slam is None:
            # v10: buffer samples before aligning. Reject if robot moved more
            # than ALIGN_MAX_GT_DISP_M during the window, or yaw std > threshold.
            # SLAM camera uses OpenCV convention: x=right, y=down, z=forward
            # Nav frame uses FLU convention: x=forward, y=left, z=up
            T_FLU_from_cam = np.array([
                [0,  0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0,  0, 0, 1],
            ], dtype=float)

            gt_yaw = math.atan2(2.0 * (self.last_qw * self.last_qz),
                                 1.0 - 2.0 * self.last_qz ** 2)
            gt_x, gt_y = self.last_x, self.last_y

            # Track GT motion during the window - if robot drove too far,
            # restart the buffer (can't trust alignment on moving baseline).
            if self._align_gt_pos0 is None:
                self._align_gt_pos0 = (gt_x, gt_y)
            disp = math.hypot(gt_x - self._align_gt_pos0[0], gt_y - self._align_gt_pos0[1])
            if disp > self.ALIGN_MAX_GT_DISP_M:
                # Robot has moved too far - reset the buffer, keep trying
                self._align_buf = []
                self._align_gt_pos0 = (gt_x, gt_y)
                # Fall back to spawn alignment for this tick only
                R_nav = ScipyRotation.from_euler('z', gt_yaw).as_matrix()
                T_nav_origin = np.eye(4)
                T_nav_origin[:3, :3] = R_nav
                T_nav_origin[:3, 3] = [gt_x, gt_y, 0.0]
                T_slam_inv = np.linalg.inv(T_slam)
                T_nav_slam_tick = T_nav_origin @ T_FLU_from_cam @ T_slam_inv
                T_nav = T_nav_slam_tick @ T_slam
                return float(T_nav[0, 3]), float(T_nav[1, 3]), gt_yaw

            self._align_buf.append((sx, sy, sz, sqx, sqy, sqz, sqw, gt_x, gt_y, gt_yaw))

            if len(self._align_buf) < self.ALIGN_WINDOW_SAMPLES:
                # Still buffering - return the naive single-sample alignment
                # for this tick so TF keeps flowing.
                R_nav = ScipyRotation.from_euler('z', gt_yaw).as_matrix()
                T_nav_origin = np.eye(4)
                T_nav_origin[:3, :3] = R_nav
                T_nav_origin[:3, 3] = [gt_x, gt_y, 0.0]
                T_slam_inv = np.linalg.inv(T_slam)
                T_nav_slam_tick = T_nav_origin @ T_FLU_from_cam @ T_slam_inv
                T_nav = T_nav_slam_tick @ T_slam
                return float(T_nav[0, 3]), float(T_nav[1, 3]), gt_yaw

            # Buffer is full - average and finalise alignment
            buf = np.array(self._align_buf)
            # Average SLAM translation + quaternion (chirality-aware)
            avg_slam_t = buf[:, 0:3].mean(axis=0)
            quats = buf[:, 3:7]
            ref = quats[0]
            dots = quats @ ref
            quats_aligned = np.where(dots[:, None] < 0, -quats, quats)
            avg_q = quats_aligned.mean(axis=0)
            avg_q = avg_q / np.linalg.norm(avg_q)
            R_slam_avg = ScipyRotation.from_quat(avg_q).as_matrix()
            T_slam_avg = np.eye(4)
            T_slam_avg[:3, :3] = R_slam_avg
            T_slam_avg[:3, 3] = avg_slam_t

            avg_gt_x = float(buf[:, 7].mean())
            avg_gt_y = float(buf[:, 8].mean())
            # Yaw averaging via circular mean
            yaws = buf[:, 9]
            avg_gt_yaw = float(math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean()))
            yaw_std_deg = float(np.degrees(np.sqrt(
                np.mean((np.angle(np.exp(1j * (yaws - avg_gt_yaw))))**2))))

            if yaw_std_deg > self.ALIGN_MAX_YAW_STD_DEG:
                # Jittery window - drop oldest half and keep buffering
                self.get_logger().warn(
                    f'[ALIGN] window too jittery (yaw std {yaw_std_deg:.2f}° > '
                    f'{self.ALIGN_MAX_YAW_STD_DEG}°), extending')
                self._align_buf = self._align_buf[self.ALIGN_WINDOW_SAMPLES // 2:]
                R_nav = ScipyRotation.from_euler('z', gt_yaw).as_matrix()
                T_nav_origin = np.eye(4)
                T_nav_origin[:3, :3] = R_nav
                T_nav_origin[:3, 3] = [gt_x, gt_y, 0.0]
                T_slam_inv = np.linalg.inv(T_slam)
                T_nav_slam_tick = T_nav_origin @ T_FLU_from_cam @ T_slam_inv
                T_nav = T_nav_slam_tick @ T_slam
                return float(T_nav[0, 3]), float(T_nav[1, 3]), gt_yaw

            # Commit the averaged alignment
            R_nav = ScipyRotation.from_euler('z', avg_gt_yaw).as_matrix()
            T_nav_origin = np.eye(4)
            T_nav_origin[:3, :3] = R_nav
            T_nav_origin[:3, 3] = [avg_gt_x, avg_gt_y, 0.0]
            T_slam_avg_inv = np.linalg.inv(T_slam_avg)
            self.T_nav_slam = T_nav_origin @ T_FLU_from_cam @ T_slam_avg_inv
            # print(f"DEBUG: ran {len(ran)} waypoints")
            self.get_logger().info(
                f'[ALIGN v10] averaged over {len(buf)} samples, GT disp {disp*100:.1f}cm, '
                f'yaw std {yaw_std_deg:.3f}° - committed at spawn=({avg_gt_x:.2f},{avg_gt_y:.2f}) '
                f'yaw={math.degrees(avg_gt_yaw):.2f}°')
            # Fall through to transform-to-nav with the new alignment
            T_nav = self.T_nav_slam @ T_slam
            return float(T_nav[0, 3]), float(T_nav[1, 3]), float(np.arctan2(T_nav[1, 0], T_nav[0, 0]))

        # Transform to nav frame
        T_nav = self.T_nav_slam @ T_slam

        # Project to 2D ground plane
        nav_x = float(T_nav[0, 3])
        nav_y = float(T_nav[1, 3])
        nav_yaw = float(np.arctan2(T_nav[1, 0], T_nav[0, 0]))

        return nav_x, nav_y, nav_yaw

    def _tick_slam_encoder(self, now, x, y, z, qx, qy, qz, qw):
        """SLAM+Encoder mode with proper SE(3)->SE(2) frame conversion.

        SLAM tracking -> fuse SLAM position + SLAM yaw (70/30 with encoder)
        SLAM lost/stale -> fallback to encoder+compass
        """
        gt_yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))

        # Initialize encoder (always running as fallback)
        if self.prev_gt_x is None:
            self.prev_gt_x = x
            self.prev_gt_y = y
            self.enc_x = x
            self.enc_y = y
            self.enc_yaw = gt_yaw
            self.get_logger().info(
                f'SLAM+ENCODER init: ({x:.1f}, {y:.1f}), yaw={gt_yaw:.3f}')
            return

        # Update encoder+compass odometry (always, as fallback)
        COMPASS_NOISE = 0.05
        noisy_yaw = gt_yaw + np.random.normal(0, COMPASS_NOISE)
        dx = x - self.prev_gt_x
        dy = y - self.prev_gt_y
        displacement = math.hypot(dx, dy)
        if displacement > 0.001:
            noisy_disp = displacement * (1.0 + np.random.normal(0, self.ENCODER_NOISE))
            self.enc_total_dist += displacement
            self.enc_x += noisy_disp * math.cos(noisy_yaw)
            self.enc_y += noisy_disp * math.sin(noisy_yaw)
        self.enc_yaw = noisy_yaw
        self.prev_gt_x = x
        self.prev_gt_y = y

        # Try SLAM pose with proper SE(3)->SE(2)
        sx, sy, sz, sqx, sqy, sqz, sqw, slam_ok = self._read_slam_pose_raw()

        # Track SLAM position changes for stale detection
        if not hasattr(self, '_prev_slam_pos'):
            self._prev_slam_pos = None
            self._slam_frozen_count = 0

        if slam_ok:
            # Detect SLAM pose freeze: if SLAM position unchanged while encoder moves
            if self._prev_slam_pos is not None:
                slam_motion = math.hypot(sx - self._prev_slam_pos[0],
                                          sz - self._prev_slam_pos[1])  # camera xz plane
                if displacement > 0.1 and slam_motion < 0.01:
                    self._slam_frozen_count += 1
                else:
                    self._slam_frozen_count = 0
            self._prev_slam_pos = (sx, sz)

            # If SLAM frozen for 60+ ticks (~12s), treat as lost
            # (Nav2 can rotate-in-place long periods - don't fallback prematurely)
            if self._slam_frozen_count > 60:
                slam_ok = False

        if slam_ok:
            slam_nx, slam_ny, slam_nyaw = self._slam_se3_to_nav(
                sx, sy, sz, sqx, sqy, sqz, sqw)
            # Trust SLAM over encoder - VIO is calibrated, encoder drifts easily

        if slam_ok:
            # Trust SLAM heavily: 95% SLAM + 5% encoder (was 70/30)
            SLAM_ALPHA = 0.95
            nav_x = SLAM_ALPHA * slam_nx + (1 - SLAM_ALPHA) * self.enc_x
            nav_y = SLAM_ALPHA * slam_ny + (1 - SLAM_ALPHA) * self.enc_y
            # Use compass yaw (more reliable than SLAM SE3 projection)
            nav_yaw = self.enc_yaw
            self.using_slam = True
        else:
            # Fallback to encoder+compass
            nav_x = self.enc_x
            nav_y = self.enc_y
            nav_yaw = self.enc_yaw
            self.using_slam = False

        # Log every ~5s
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            err = math.hypot(nav_x - x, nav_y - y)
            yaw_err = abs(normalize_angle(nav_yaw - gt_yaw))
            enc_err = math.hypot(self.enc_x - x, self.enc_y - y)
            src = 'SLAM' if self.using_slam else "ENC"
            self.get_logger().info(
                f'SLAM+ENC [{src}]: nav=({nav_x:.1f},{nav_y:.1f}) '
                f'gt=({x:.1f},{y:.1f}) err={err:.2f}m '
                f'enc_err={enc_err:.2f}m yaw_err={math.degrees(yaw_err):.1f}° '
                f'dist={self.enc_total_dist:.0f}m '
                f'slam_f={self.slam_frames} lost={self.slam_lost}')

        # Publish TF
        nqz = math.sin(nav_yaw / 2)
        nqw = math.cos(nav_yaw / 2)

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = nav_x
        t2.transform.translation.y = nav_y
        t2.transform.translation.z = 0.0
        t2.transform.rotation.z = nqz
        t2.transform.rotation.w = nqw

        self.br.sendTransform([t1, t2])
        self._publish_odom(now, nav_x, nav_y, 0.0, 0.0, 0.0, nqz, nqw)

    def _tick_encoder_imu(self, now, x, y, z, qx, qy, qz, qw):
        """Encoder+IMU mode: simulate realistic Husky sensors.

        Position: GT pose diff (simulates wheel encoders) with 0.5% noise.
        Heading: GT yaw + noise (simulates compass+gyro fusion).
        On real Husky: Microstrain 3DM-GX5 provides fused heading via
        magnetometer + gyro. We simulate this as GT yaw + Gaussian noise.
        """
        gt_yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))

        if self.prev_gt_x is None:
            # first tick: initialize encoder at spawn position
            self.prev_gt_x = x
            self.prev_gt_y = y
            self.enc_x = x
            self.enc_y = y
            self.enc_yaw = gt_yaw
            # print(f">>> frame {i}/{n_frames}")
            self.get_logger().info(
                f'ENCODER+IMU init: ({x:.1f}, {y:.1f}), yaw={gt_yaw:.3f}')
            return

        # Heading: compass+gyro fusion = GT yaw + noise (~3° std)
        COMPASS_NOISE = 0.05  # ~3 degrees std
        noisy_yaw = gt_yaw + np.random.normal(0, COMPASS_NOISE)

        # Encoder: compute displacement from GT pose diff (= wheel encoder equivalent)
        dx = x - self.prev_gt_x
        dy = y - self.prev_gt_y
        displacement = math.hypot(dx, dy)

        if displacement > 0.001:
            # Add encoder noise (0.5% of distance)
            noisy_disp = displacement * (1.0 + np.random.normal(0, self.ENCODER_NOISE))
            self.enc_total_dist += displacement

            # Integrate position using noisy compass heading
            self.enc_x += noisy_disp * math.cos(noisy_yaw)
            self.enc_y += noisy_disp * math.sin(noisy_yaw)

        self.enc_yaw = noisy_yaw
        self.prev_gt_x = x
        self.prev_gt_y = y

        # Log every ~5s (100 ticks at 20Hz)
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            err = math.hypot(self.enc_x - x, self.enc_y - y)
            yaw_err = abs(normalize_angle(self.enc_yaw - gt_yaw))
            self.get_logger().info(
                f'ENC+IMU: enc=({self.enc_x:.1f},{self.enc_y:.1f}) '
                f'gt=({x:.1f},{y:.1f}) err={err:.2f}m '
                f'yaw_err={math.degrees(yaw_err):.1f}° '
                f'dist={self.enc_total_dist:.0f}m')

        # Publish TF: map->odom=identity, odom->base_link=encoder+IMU pose
        eqz = math.sin(self.enc_yaw / 2)
        eqw = math.cos(self.enc_yaw / 2)

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = self.enc_x
        t2.transform.translation.y = self.enc_y
        t2.transform.translation.z = 0.0
        t2.transform.rotation.z = eqz
        t2.transform.rotation.w = eqw

        self.br.sendTransform([t1, t2])
        self._publish_odom(now, self.enc_x, self.enc_y, 0.0, 0.0, 0.0, eqz, eqw)

    def _tick_slam_frame(self, now, x, y, z, qx, qy, qz, qw):
        """SLAM frame mode: read SLAM pose, convert to 2D nav, publish directly.

        No world frame conversion. Nav2 works in SLAM coordinate space.
        map -> odom = identity, odom -> base_link = fused SLAM pose.
        """
        try:
            with open('/tmp/slam_pose.txt', 'r') as f:
                line = f.readline().strip()
            parts = line.split()
            if len(parts) < 8:
                return
            sx = float(parts[1])
            sy = float(parts[2])
            sz = float(parts[3])
            sqx, sqy, sqz, sqw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
        except (FileNotFoundError, ValueError, IndexError):
            return

        # SLAM camera frame -> 2D nav frame
        # nav_x = slam_z (forward), nav_y = -slam_x (left)
        slam_yaw_raw = math.atan2(2 * (sqw * sqz + sqx * sqy),
                                   1 - 2 * (sqy * sqy + sqz * sqz))
        nav_x = sz
        nav_y = -sx
        nav_yaw = -slam_yaw_raw

        if not self.filter_initialized:
            self.fused_x = nav_x
            self.fused_y = nav_y
            self.fused_yaw = nav_yaw
            self.prev_slam_nx = nav_x
            self.prev_slam_ny = nav_y
            self.prev_slam_nyaw = nav_yaw
            self.prev_odom_x = self.odom_x
            self.prev_odom_y = self.odom_y
            self.imu_yaw = nav_yaw
            self.imu_yaw_initialized = True
            self.filter_initialized = True
            self.get_logger().info(
                f'SLAM-FRAME init: nav=({nav_x:.1f},{nav_y:.1f}) yaw={nav_yaw:.3f}')

        # odometry delta
        odom_dx = self.odom_x - self.prev_odom_x
        odom_dy = self.odom_y - self.prev_odom_y
        predicted_x = self.fused_x + odom_dx
        predicted_y = self.fused_y + odom_dy

        # SLAM jump detection
        slam_jump = math.hypot(nav_x - self.prev_slam_nx,
                               nav_y - self.prev_slam_ny)
        slam_yaw_jump = abs(normalize_angle(nav_yaw - self.prev_slam_nyaw))

        # position fusion
        if slam_jump < JUMP_THRESHOLD:
            self.fused_x = SLAM_POS_ALPHA * nav_x + ODOM_POS_ALPHA * predicted_x
            self.fused_y = SLAM_POS_ALPHA * nav_y + ODOM_POS_ALPHA * predicted_y
        else:
            self.fused_x = predicted_x
            self.fused_y = predicted_y
            self.get_logger().warn(f'SLAM jump {slam_jump:.1f}m rejected')

        # yaw fusion: IMU gyro + SLAM correction
        if slam_yaw_jump < YAW_JUMP_THRESHOLD:
            yaw_error = normalize_angle(nav_yaw - self.imu_yaw)
            self.imu_yaw += SLAM_YAW_ALPHA * yaw_error
        self.fused_yaw = self.imu_yaw

        # save prev
        self.prev_odom_x = self.odom_x
        self.prev_odom_y = self.odom_y
        self.prev_slam_nx = nav_x
        self.prev_slam_ny = nav_y
        self.prev_slam_nyaw = nav_yaw

        # log every ~5s
        self.log_counter += 1
        if self.log_counter % 100 == 0:
            gt_yaw = math.atan2(2 * self.last_qw * self.last_qz,
                                1 - 2 * self.last_qz ** 2)
            self.get_logger().info(
                f'SLAM-FRAME: slam=({nav_x:.1f},{nav_y:.1f}) '
                f'odom_d=({odom_dx:.2f},{odom_dy:.2f}) '
                f'gyro={self.imu_yaw:.2f} slam_yaw={nav_yaw:.2f} '
                f'fused=({self.fused_x:.1f},{self.fused_y:.1f},{self.fused_yaw:.2f}) '
                f'gt=({self.last_x:.1f},{self.last_y:.1f})')

        # publish TF: map -> odom = identity, odom -> base_link = fused pose
        fqz = math.sin(self.fused_yaw / 2)
        fqw = math.cos(self.fused_yaw / 2)

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.translation.x = self.fused_x
        t_odom_base.transform.translation.y = self.fused_y
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation.z = fqz
        t_odom_base.transform.rotation.w = fqw

        self.br.sendTransform([t_map_odom, t_odom_base])

        # publish odom
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.fused_x
        odom.pose.pose.position.y = self.fused_y
        odom.pose.pose.orientation.z = fqz
        odom.pose.pose.orientation.w = fqw
        self.odom_pub.publish(odom)

    def _tick_slam_world(self, now, x, y, z, qx, qy, qz, qw):
        """Old SLAM mode: convert SLAM pose to world frame."""
        # world -> base_link (GT)
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.x = x
        t1.transform.translation.y = y
        t1.transform.translation.z = z
        t1.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'world'
        t2.transform.rotation.w = 1.0

        t3 = self._slam_world_map_odom(now)
        self.br.sendTransform([t1, t2, t3])
        self._publish_odom(now, x, y, z, qx, qy, qz, qw)

    def _slam_world_map_odom(self, stamp):
        try:
            with open('/tmp/slam_pose.txt', 'r') as f:
                line = f.readline().strip()
            parts = line.split()
            if len(parts) < 8:
                return self._identity_tf(stamp, 'map', 'odom')
            sx, sy, sz = float(parts[1]), float(parts[2]), float(parts[3])
            sqx, sqy, sqz, sqw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            slam_yaw = math.atan2(2 * (sqw * sqz + sqx * sqy),
                                   1 - 2 * (sqy * sqy + sqz * sqz))

            if self.slam_origin is None:
                self.slam_origin = (sx, sy, sz, slam_yaw)

            s0x, s0y, s0z, s0yaw = self.slam_origin
            slam_wx = SPAWN_X + (sz - s0z)
            slam_wy = SPAWN_Y - (sx - s0x)
            slam_wyaw = -(slam_yaw - s0yaw)

            if not self.filter_initialized:
                self.fused_x = slam_wx
                self.fused_y = slam_wy
                self.fused_yaw = slam_wyaw
                self.prev_slam_nx = slam_wx
                self.prev_slam_ny = slam_wy
                self.prev_slam_nyaw = slam_wyaw
                self.prev_odom_x = self.odom_x
                self.prev_odom_y = self.odom_y
                self.imu_yaw = slam_wyaw
                self.imu_yaw_initialized = True
                self.filter_initialized = True

            odom_dx = self.odom_x - self.prev_odom_x
            odom_dy = self.odom_y - self.prev_odom_y
            predicted_x = self.fused_x + odom_dx
            predicted_y = self.fused_y + odom_dy
            slam_jump = math.hypot(slam_wx - self.prev_slam_nx, slam_wy - self.prev_slam_ny)
            slam_yaw_jump = abs(normalize_angle(slam_wyaw - self.prev_slam_nyaw))

            if slam_jump < JUMP_THRESHOLD:
                self.fused_x = SLAM_POS_ALPHA * slam_wx + ODOM_POS_ALPHA * predicted_x
                self.fused_y = SLAM_POS_ALPHA * slam_wy + ODOM_POS_ALPHA * predicted_y
            else:
                self.fused_x = predicted_x
                self.fused_y = predicted_y

            if slam_yaw_jump < YAW_JUMP_THRESHOLD:
                self.imu_yaw += SLAM_YAW_ALPHA * normalize_angle(slam_wyaw - self.imu_yaw)
            self.fused_yaw = self.imu_yaw

            self.prev_odom_x = self.odom_x
            self.prev_odom_y = self.odom_y
            self.prev_slam_nx = slam_wx
            self.prev_slam_ny = slam_wy
            self.prev_slam_nyaw = slam_wyaw

            self.log_counter += 1
            if self.log_counter % 100 == 0:
                self.get_logger().info(
                    f'SLAM-WORLD: fused=({self.fused_x:.1f},{self.fused_y:.1f}) '
                    f'gt=({self.last_x:.1f},{self.last_y:.1f})')

            gt_yaw = math.atan2(2 * self.last_qw * self.last_qz,
                                1 - 2 * self.last_qz ** 2)
            co = math.cos(-gt_yaw)
            so = math.sin(-gt_yaw)
            inv_ox = -(self.last_x * co - self.last_y * so)
            inv_oy = -(self.last_x * so + self.last_y * co)
            cs = math.cos(self.fused_yaw)
            ss = math.sin(self.fused_yaw)
            mo_x = self.fused_x + inv_ox * cs - inv_oy * ss
            mo_y = self.fused_y + inv_ox * ss + inv_oy * cs
            mo_yaw = self.fused_yaw - gt_yaw

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = mo_x
            t.transform.translation.y = mo_y
            t.transform.rotation.z = math.sin(mo_yaw / 2)
            t.transform.rotation.w = math.cos(mo_yaw / 2)
            return t
        except (FileNotFoundError, ValueError, IndexError):
            return self._identity_tf(stamp, 'map', 'odom')

    def _publish_odom(self, now, x, y, z, qx, qy, qz, qw):
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.odom_pub.publish(odom)

    def _identity_tf(self, stamp, parent, child):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.rotation.w = 1.0
        return t

    def imu_tick(self):
        try:
            with open(self.imu_file, 'r') as f:
                lines = f.readlines()
                if lines:
                    parts = lines[-1].strip().split()
                    if len(parts) >= 10:
                        self.last_imu_data = parts
                        raw_yaw_rate = float(parts[2])  # gz = yaw rate in FLU
                        now_t = pytime.time()
                        dt = min(now_t - self.prev_imu_time, 0.02)
                        self.prev_imu_time = now_t
                        self.filtered_yaw_rate = (GYRO_LPF_ALPHA * raw_yaw_rate +
                                                  (1 - GYRO_LPF_ALPHA) * self.filtered_yaw_rate)
                        effective_rate = self.filtered_yaw_rate
                        if abs(effective_rate) < GYRO_DEADZONE:
                            effective_rate = 0.0
                        self.last_imu_yaw_rate = effective_rate
                        if self.imu_yaw_initialized:
                            self.imu_yaw += effective_rate * dt
        except (FileNotFoundError, ValueError, IndexError):
            pass

        if self.last_imu_data is None:
            return
        p = self.last_imu_data
        msg = Imu()
        msg.header.stamp = wall_stamp()
        msg.header.frame_id = 'imu_link'
        msg.angular_velocity.x = float(p[0])
        msg.angular_velocity.y = float(p[1])
        msg.angular_velocity.z = float(p[2])
        msg.linear_acceleration.x = float(p[3])
        msg.linear_acceleration.y = float(p[4])
        msg.linear_acceleration.z = float(p[5])
        msg.orientation.x = float(p[6])
        msg.orientation.y = float(p[7])
        msg.orientation.z = float(p[8])
        msg.orientation.w = float(p[9])
        self.imu_pub.publish(msg)

    def depth_cb(self, msg):
        now = wall_stamp()
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        else:
            return
        step = 4
        rows = np.arange(0, msg.height, step)
        cols = np.arange(0, msg.width, step)
        v, u = np.meshgrid(rows, cols, indexing='ij')
        z = depth[v, u]
        valid = (z > 0.3) & (z < 10.0) & np.isfinite(z)
        z = z[valid]
        u_v = u[valid].astype(np.float32)
        v_v = v[valid].astype(np.float32)
        px = (u_v - self.cx) / self.fx * z
        py = (v_v - self.cy) / self.fy * z
        points = np.stack([z, -px, -py], axis=-1).astype(np.float32)
        pc = PointCloud2()
        pc.header.stamp = now
        pc.header.frame_id = 'camera_link'
        pc.height = 1
        pc.width = len(z)
        pc.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc.is_bigendian = False
        pc.point_step = 12
        pc.row_step = 12 * len(z)
        pc.data = points.tobytes()
        pc.is_dense = True
        self.pc_pub.publish(pc)

    def caminfo_cb(self, msg):
        pass


def main():
    # NOTE: this file is shared across 9 routes, keep changes backwards compatible
    use_gt = '--use-gt' in sys.argv
    slam_frame = '--slam-frame' in sys.argv
    encoder_imu = '--encoder-imu' in sys.argv
    slam_encoder = '--slam-encoder' in sys.argv
    rclpy_argv = [a for a in sys.argv if a not in ('--use-gt', '--slam-frame', '--encoder-imu', '--slam-encoder')]
    try:
        rclpy.init(args=rclpy_argv)
    except RuntimeError:
        pass
    node = TFRelay(use_gt=use_gt, slam_frame=slam_frame, encoder_imu=encoder_imu, slam_encoder=slam_encoder)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
