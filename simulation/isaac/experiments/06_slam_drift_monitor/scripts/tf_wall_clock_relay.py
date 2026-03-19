#!/usr/bin/env python3
"""
TF + Odom relay for Nav2 <- Isaac Sim.

Reads robot GT pose from /tmp/isaac_pose.txt (written by run_husky_nav2.py).
Publishes:
  - /tf: map->odom, odom->world, world->base_link (all wall clock timestamps)
  - /odom: nav_msgs/Odometry with wall clock

This completely bypasses Isaac Sim's ROS2 bridge TF/Odom publishing,
which forces sim_time timestamps and breaks Nav2 TF lookups.

GT mode: map->odom = spawn offset (-95,-6). Robot moves relative to odom.
SLAM mode: map->odom = SLAM correction (reads /tmp/slam_pose.txt).

usage:
  source /opt/ros/jazzy/setup.bash
  python3 tf_wall_clock_relay.py [--use-gt]
"""
import math
import sys
import time as pytime
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
import struct
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

POSE_FILE = "/tmp/isaac_pose.txt"
SPAWN_X, SPAWN_Y = -95.0, -6.0

# complementary filter params
JUMP_THRESHOLD = 2.0     # meters per tick - allow SLAM relocalization jumps
SLAM_YAW_ALPHA = 0.15   # SLAM corrects gyro drift (faster than before)
GYRO_DEADZONE = 0.08     # rad/s - below this = PhysX noise, ignore
GYRO_LPF_ALPHA = 0.3     # low-pass filter on gyro (0.3 = moderate smoothing)


def wall_stamp():
    t = pytime.time()
    return TimeMsg(sec=int(t), nanosec=int((t - int(t)) * 1e9))


class TFRelay(Node):
    def __init__(self, use_gt=False):
        super().__init__('tf_relay')
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.use_gt = use_gt
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.pose_file = POSE_FILE
        self.last_x = SPAWN_X
        self.last_y = SPAWN_Y
        self.last_qz = 0.0
        self.last_qw = 1.0
        self.slam_origin = None
        self.last_slam_wx = SPAWN_X
        self.last_slam_wy = SPAWN_Y
        self.last_mo_x = 0.0
        self.last_mo_y = 0.0
        self.last_mo_yaw = 0.0
        # static TF: base_link -> camera_link (D435i: 0.5m forward, 0.48m up)
        self.static_br = StaticTransformBroadcaster(self)
        cam_tf = TransformStamped()
        cam_tf.header.stamp = wall_stamp()
        cam_tf.header.frame_id = 'base_link'
        cam_tf.child_frame_id = 'camera_link'
        cam_tf.transform.translation.x = 0.5
        cam_tf.transform.translation.z = 0.48
        cam_tf.transform.rotation.w = 1.0
        # static TF: base_link -> imu_link (IMU on top plate, slightly behind camera)
        imu_tf = TransformStamped()
        imu_tf.header.stamp = wall_stamp()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.2
        imu_tf.transform.translation.z = 0.38
        imu_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform([cam_tf, imu_tf])

        # relay depth image + camera_info with SAME wall clock timestamp
        # (must be identical for depth_image_proc message_filter sync)
        # convert depth image -> pointcloud directly (bypass depth_image_proc sync issues)
        # D435i params: fx=fy=320, cx=320, cy=240, 640x480
        self.fx = self.fy = 320.0
        self.cx, self.cy = 320.0, 240.0
        self.pc_pub = self.create_publisher(PointCloud2, '/depth_points', 10)
        self.last_caminfo = None
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.caminfo_cb, 10)

        # IMU relay: read from file (written by Isaac at ~30Hz), publish at 200Hz
        self.imu_pub = self.create_publisher(Imu, '/imu/data_wall', 50)
        self.imu_file = '/tmp/isaac_imu.txt'
        self.imu_timer = self.create_timer(1.0 / 200.0, self.imu_tick)  # 200Hz
        self.last_imu_data = None

        # SLAM position + IMU gyro yaw fusion
        self.prev_slam_wx = SPAWN_X
        self.prev_slam_wy = SPAWN_Y
        self.filter_initialized = False
        self.log_counter = 0

        # IMU gyro yaw integration with LPF + deadzone
        self.imu_yaw = 0.0
        self.imu_yaw_initialized = False
        self.prev_imu_time = pytime.time()
        self.last_imu_yaw_rate = 0.0
        self.filtered_yaw_rate = 0.0

        self.start_time = pytime.time()
        mode = "GT" if use_gt else "SLAM"
        self.get_logger().info(f'TF relay started, mode={mode}, with complementary filter')

    def tick(self):
        # read pose from file
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

        # 1. world -> base_link (GT world pose from Isaac Sim)
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.x = x
        t1.transform.translation.y = y
        t1.transform.translation.z = z
        t1.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 2. odom -> world (identity: odom frame = world frame)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'world'
        t2.transform.rotation.w = 1.0

        # 3. map -> odom
        if self.use_gt:
            # GT: map = world, odom = world -> map->odom = identity
            t3 = TransformStamped()
            t3.header.stamp = now
            t3.header.frame_id = 'map'
            t3.child_frame_id = 'odom'
            t3.transform.rotation.w = 1.0
        else:
            t3 = self._slam_map_odom(now)

        self.br.sendTransform([t1, t2, t3])

        # publish odom (world pose as odom for simplicity)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.odom_pub.publish(odom)

    def cmd_vel_cb(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z

    def imu_tick(self):
        """Read IMU from file, integrate gyro yaw, publish at 200Hz."""
        try:
            with open(self.imu_file, 'r') as f:
                # read last line from ring buffer
                lines = f.readlines()
                if lines:
                    parts = lines[-1].strip().split()
                    if len(parts) >= 10:
                        self.last_imu_data = parts
                        # gz in file = yaw rate in FLU (rotation around Z/up)
                        raw_yaw_rate = float(parts[3])
                        now = pytime.time()
                        dt = min(now - self.prev_imu_time, 0.02)
                        self.prev_imu_time = now

                        # low-pass filter
                        self.filtered_yaw_rate = (GYRO_LPF_ALPHA * raw_yaw_rate +
                                                  (1 - GYRO_LPF_ALPHA) * self.filtered_yaw_rate)

                        # deadzone - ignore noise
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
        # convert depth image to pointcloud with wall clock timestamp
        now = wall_stamp()
        # decode depth (32FC1 from Isaac Sim = z-depth in meters)
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        else:
            return

        # subsample for performance (every 4th pixel)
        step = 4
        rows = np.arange(0, msg.height, step)
        cols = np.arange(0, msg.width, step)
        v, u = np.meshgrid(rows, cols, indexing='ij')
        z = depth[v, u]

        # filter valid depth
        valid = (z > 0.3) & (z < 10.0) & np.isfinite(z)
        z = z[valid]
        u_v = u[valid].astype(np.float32)
        v_v = v[valid].astype(np.float32)

        # project to 3D (camera frame: z=forward, x=right, y=down)
        x = (u_v - self.cx) / self.fx * z
        y = (v_v - self.cy) / self.fy * z

        # build PointCloud2
        points = np.stack([z, -x, -y], axis=-1).astype(np.float32)  # camera_link: x=fwd, y=left, z=up
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
        pass  # not needed - pointcloud built directly from depth

    def _slam_map_odom(self, stamp):
        """SLAM position + IMU gyro yaw fusion.

        SLAM gives good position (forward tracking accurate, lateral drift).
        IMU gyroscope gives stable short-term yaw -> reduces lateral drift.
        SLAM yaw slowly corrects gyro drift (long-term)."""
        try:
            with open('/tmp/slam_pose.txt', 'r') as f:
                line = f.readline().strip()
            parts = line.split()
            if len(parts) < 8:
                return self._identity_map_odom(stamp)

            sx, sy, sz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            slam_yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

            if self.slam_origin is None:
                self.slam_origin = (sx, sy, sz, slam_yaw)
                self.get_logger().info(
                    f'SLAM origin: ({sx:.3f},{sy:.3f},{sz:.3f}) yaw={slam_yaw:.3f}')

            # SLAM camera frame -> world
            s0x, s0y, s0z, s0yaw = self.slam_origin
            dz_s = sz - s0z  # forward
            dx_s = sx - s0x  # right
            dyaw = slam_yaw - s0yaw

            slam_wx = SPAWN_X + dz_s
            slam_wy = SPAWN_Y - dx_s
            slam_wyaw = -dyaw

            if not self.filter_initialized:
                self.prev_slam_wx = slam_wx
                self.prev_slam_wy = slam_wy
                self.imu_yaw = slam_wyaw
                self.imu_yaw_initialized = True
                self.filter_initialized = True
                self.get_logger().info(
                    f'Init: slam=({slam_wx:.1f},{slam_wy:.1f}) yaw={slam_wyaw:.3f}')

            # --- position: SLAM directly, with jump filter ---
            slam_jump = math.hypot(slam_wx - self.prev_slam_wx,
                                   slam_wy - self.prev_slam_wy)
            if slam_jump < JUMP_THRESHOLD:
                fused_x = slam_wx
                fused_y = slam_wy
            else:
                fused_x = self.prev_slam_wx
                fused_y = self.prev_slam_wy
                self.get_logger().warn(f'SLAM jump {slam_jump:.1f}m rejected')

            self.prev_slam_wx = fused_x
            self.prev_slam_wy = fused_y

            # --- yaw: IMU gyro (integrated at 200Hz in imu_tick) + slow SLAM correction ---
            yaw_error = math.atan2(math.sin(slam_wyaw - self.imu_yaw),
                                   math.cos(slam_wyaw - self.imu_yaw))
            self.imu_yaw += SLAM_YAW_ALPHA * yaw_error
            fused_yaw = self.imu_yaw

            # --- log every 5s ---
            self.log_counter += 1
            if self.log_counter % 100 == 0:
                self.get_logger().info(
                    f'slam=({fused_x:.1f},{fused_y:.1f}) '
                    f'slam_yaw={slam_wyaw:.2f} imu_yaw={self.imu_yaw:.2f} '
                    f'gyro={self.last_imu_yaw_rate:.4f} '
                    f'gt=({self.last_x:.1f},{self.last_y:.1f})')

            # --- map->odom ---
            gt_yaw = math.atan2(2 * self.last_qw * self.last_qz,
                                1 - 2 * self.last_qz ** 2)

            co = math.cos(-gt_yaw)
            so = math.sin(-gt_yaw)
            inv_ox = -(self.last_x * co - self.last_y * so)
            inv_oy = -(self.last_x * so + self.last_y * co)

            cs = math.cos(fused_yaw)
            ss = math.sin(fused_yaw)
            mo_x = fused_x + inv_ox * cs - inv_oy * ss
            mo_y = fused_y + inv_ox * ss + inv_oy * cs
            mo_yaw = fused_yaw - gt_yaw

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
            return self._identity_map_odom(stamp)

    def _identity_map_odom(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        return t


def main():
    use_gt = '--use-gt' in sys.argv
    rclpy_argv = [a for a in sys.argv if a != '--use-gt']
    try:
        rclpy.init(args=rclpy_argv)
    except RuntimeError:
        pass
    node = TFRelay(use_gt=use_gt)
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
