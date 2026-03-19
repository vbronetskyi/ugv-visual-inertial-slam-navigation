#!/usr/bin/env python3
"""
TF + Odom relay for Nav2 <- Isaac Sim.

Full sensor fusion: SLAM position + wheel odometry + IMU gyro compass.

Reads robot GT pose from /tmp/isaac_pose.txt (written by run_husky_nav2.py).
Publishes:
  - /tf: map->odom, odom->world, world->base_link (all wall clock timestamps)
  - /odom: nav_msgs/Odometry with wall clock

GT mode: map->odom = identity. Robot moves relative to odom.
SLAM mode: fuses SLAM position + cmd_vel odometry + IMU gyro yaw.

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

# fusion parameters
SLAM_POS_ALPHA = 0.7       # trust SLAM for position (SLAM base, odom smooths)
ODOM_POS_ALPHA = 0.3       # trust wheel odometry for position
SLAM_YAW_ALPHA = 0.15      # SLAM yaw correction (slow, long-term)
GYRO_YAW_ALPHA = 0.85      # IMU gyro for yaw (fast, stable short-term)
GYRO_DEADZONE = 0.08       # rad/s - ignore gyro noise
GYRO_LPF_ALPHA = 0.3       # low-pass filter on gyro
JUMP_THRESHOLD = 0.5        # m - SLAM position jump per tick, ignore
YAW_JUMP_THRESHOLD = 0.3   # rad - SLAM yaw jump per tick, ignore


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

        # static TF: base_link -> camera_link (D435i: 0.5m forward, 0.48m up)
        self.static_br = StaticTransformBroadcaster(self)
        cam_tf = TransformStamped()
        cam_tf.header.stamp = wall_stamp()
        cam_tf.header.frame_id = 'base_link'
        cam_tf.child_frame_id = 'camera_link'
        cam_tf.transform.translation.x = 0.5
        cam_tf.transform.translation.z = 0.48
        cam_tf.transform.rotation.w = 1.0
        # static TF: base_link -> imu_link
        imu_tf = TransformStamped()
        imu_tf.header.stamp = wall_stamp()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.2
        imu_tf.transform.translation.z = 0.38
        imu_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform([cam_tf, imu_tf])

        # depth -> pointcloud conversion (bypass depth_image_proc sync issues)
        self.fx = self.fy = 320.0
        self.cx, self.cy = 320.0, 240.0
        self.pc_pub = self.create_publisher(PointCloud2, '/depth_points', 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.caminfo_cb, 10)

        # IMU relay: read from file, publish at 200Hz
        self.imu_pub = self.create_publisher(Imu, '/imu/data_wall', 50)
        self.imu_file = '/tmp/isaac_imu.txt'
        self.imu_timer = self.create_timer(1.0 / 200.0, self.imu_tick)  # 200Hz
        self.last_imu_data = None

        # wheel odometry from cmd_vel
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_timer = self.create_timer(0.05, self.odom_integrate_tick)  # 20Hz

        # fusion state
        self.fused_x = SPAWN_X
        self.fused_y = SPAWN_Y
        self.fused_yaw = 0.0
        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_odom_yaw = 0.0
        self.prev_slam_wx = SPAWN_X
        self.prev_slam_wy = SPAWN_Y
        self.prev_slam_wyaw = 0.0
        self.filter_initialized = False
        self.log_counter = 0

        # IMU gyro yaw integration
        self.imu_yaw = 0.0
        self.imu_yaw_initialized = False
        self.prev_imu_time = pytime.time()
        self.last_imu_yaw_rate = 0.0
        self.filtered_yaw_rate = 0.0

        self.start_time = pytime.time()
        mode = "GT" if use_gt else "SLAM+ODOM+IMU fusion"
        self.get_logger().info(f'TF relay started, mode={mode}')

    def cmd_vel_cb(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z

    def odom_integrate_tick(self):
        """Integrate cmd_vel as dead reckoning odometry at 20Hz."""
        dt = 0.05  # 20Hz
        lin_v = self.cmd_lin
        ang_v = self.cmd_ang
        self.odom_x += lin_v * dt * math.cos(self.odom_yaw)
        self.odom_y += lin_v * dt * math.sin(self.odom_yaw)
        self.odom_yaw += ang_v * dt

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

        # 2. odom -> world (identity)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'world'
        t2.transform.rotation.w = 1.0

        # 3. map -> odom
        if self.use_gt:
            t3 = TransformStamped()
            t3.header.stamp = now
            t3.header.frame_id = 'map'
            t3.child_frame_id = 'odom'
            t3.transform.rotation.w = 1.0
        else:
            t3 = self._slam_fusion_map_odom(now)

        self.br.sendTransform([t1, t2, t3])

        # publish odom
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.odom_pub.publish(odom)

    def imu_tick(self):
        """Read IMU from file, integrate gyro yaw, publish at 200Hz."""
        try:
            with open(self.imu_file, 'r') as f:
                lines = f.readlines()
                if lines:
                    parts = lines[-1].strip().split()
                    if len(parts) >= 10:
                        self.last_imu_data = parts
                        # gz = yaw rate in FLU
                        raw_yaw_rate = float(parts[3])
                        now = pytime.time()
                        dt = min(now - self.prev_imu_time, 0.02)
                        self.prev_imu_time = now

                        # low-pass filter
                        self.filtered_yaw_rate = (GYRO_LPF_ALPHA * raw_yaw_rate +
                                                  (1 - GYRO_LPF_ALPHA) * self.filtered_yaw_rate)

                        # deadzone
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
        x = (u_v - self.cx) / self.fx * z
        y = (v_v - self.cy) / self.fy * z
        points = np.stack([z, -x, -y], axis=-1).astype(np.float32)
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

    def _slam_fusion_map_odom(self, stamp):
        """Full sensor fusion: SLAM position + wheel odometry + IMU gyro compass.

        Position: SLAM (70%) + odometry prediction (30%), with jump rejection.
        Yaw: IMU gyro compass (fast, stable) + SLAM correction (slow, long-term).
        """
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
                self.fused_x = slam_wx
                self.fused_y = slam_wy
                self.fused_yaw = slam_wyaw
                self.prev_slam_wx = slam_wx
                self.prev_slam_wy = slam_wy
                self.prev_slam_wyaw = slam_wyaw
                self.prev_odom_x = self.odom_x
                self.prev_odom_y = self.odom_y
                self.prev_odom_yaw = self.odom_yaw
                self.imu_yaw = slam_wyaw
                self.imu_yaw_initialized = True
                self.filter_initialized = True
                self.get_logger().info(
                    f'FUSION init: slam=({slam_wx:.1f},{slam_wy:.1f}) yaw={slam_wyaw:.3f}')

            # --- Odometry delta ---
            odom_dx = self.odom_x - self.prev_odom_x
            odom_dy = self.odom_y - self.prev_odom_y

            # Predict from odometry
            predicted_x = self.fused_x + odom_dx
            predicted_y = self.fused_y + odom_dy

            # --- SLAM jump detection ---
            slam_jump = math.hypot(slam_wx - self.prev_slam_wx,
                                   slam_wy - self.prev_slam_wy)
            slam_yaw_jump = abs(normalize_angle(slam_wyaw - self.prev_slam_wyaw))

            # --- Position fusion: SLAM + odometry ---
            if slam_jump < JUMP_THRESHOLD:
                self.fused_x = SLAM_POS_ALPHA * slam_wx + ODOM_POS_ALPHA * predicted_x
                self.fused_y = SLAM_POS_ALPHA * slam_wy + ODOM_POS_ALPHA * predicted_y
            else:
                # SLAM jumped - trust odometry
                self.fused_x = predicted_x
                self.fused_y = predicted_y
                self.get_logger().warn(f'SLAM jump {slam_jump:.1f}m rejected, using odom')

            # --- Yaw fusion: IMU gyro compass + SLAM correction ---
            if slam_yaw_jump < YAW_JUMP_THRESHOLD:
                # IMU as base, SLAM slowly corrects
                yaw_error = normalize_angle(slam_wyaw - self.imu_yaw)
                self.imu_yaw += SLAM_YAW_ALPHA * yaw_error
            else:
                self.get_logger().warn(
                    f'SLAM yaw jump {slam_yaw_jump:.2f}rad rejected')
            self.fused_yaw = self.imu_yaw

            # Save prev state
            self.prev_odom_x = self.odom_x
            self.prev_odom_y = self.odom_y
            self.prev_odom_yaw = self.odom_yaw
            self.prev_slam_wx = slam_wx
            self.prev_slam_wy = slam_wy
            self.prev_slam_wyaw = slam_wyaw

            # --- Log every ~5s (100 ticks at 20Hz) ---
            self.log_counter += 1
            if self.log_counter % 100 == 0:
                self.get_logger().info(
                    f'FUSION: slam=({slam_wx:.1f},{slam_wy:.1f}) '
                    f'odom_delta=({odom_dx:.2f},{odom_dy:.2f}) '
                    f'gyro_yaw={self.imu_yaw:.2f} slam_yaw={slam_wyaw:.2f} '
                    f'fused=({self.fused_x:.1f},{self.fused_y:.1f},{self.fused_yaw:.2f}) '
                    f'gt=({self.last_x:.1f},{self.last_y:.1f})')

            # --- map->odom transform ---
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
