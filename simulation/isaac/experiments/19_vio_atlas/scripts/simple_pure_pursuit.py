#!/usr/bin/env python3
"""
Simple pure pursuit publisher for mapping runs (no Nav2).

Reads robot pose from /tmp/isaac_pose.txt and publishes /cmd_vel
to follow road waypoints. Used for SLAM mapping runs.

usage:
  source /opt/ros/jazzy/setup.bash
  python3 simple_pure_pursuit.py
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# road waypoints (world frame)
ROAD = [(-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
        (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
        (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
        (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
        (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(72,-5)]

LOOKAHEAD = 2.5
MAX_LIN = 0.6
MAX_ANG = 0.6
GOAL_TOL = 1.0
POSE_FILE = "/tmp/isaac_pose.txt"


class PurePursuit(Node):
    def __init__(self):
        super().__init__('simple_pure_pursuit')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.idx = 0
        self.done = False
        # IMU initialization phase: stationary 2s, then zigzag 10s
        # excites accelerometer + gyro for proper VIO scale recovery
        self.start_time = time.time()
        self.init_done = False
        self.get_logger().info(
            f'Simple pure pursuit started, {len(ROAD)} waypoints, '
            f'init phase: 2s stationary + 10s zigzag')

    def get_pose(self):
        try:
            with open(POSE_FILE) as f:
                p = f.readline().strip().split()
            if len(p) < 7:
                return None
            x, y = float(p[0]), float(p[1])
            qz, qw = float(p[5]), float(p[6])
            yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
            return x, y, yaw
        except Exception:
            return None

    def tick(self):
        if self.done:
            return

        # imu initialization phase
        # 2s stationary (bias estimation) + 10s zigzag (scale + gravity direction)
        elapsed = time.time() - self.start_time
        if not self.init_done:
            if elapsed < 2.0:
                # Stationary
                tw = Twist()
                self.pub.publish(tw)
                return
            elif elapsed < 12.0:
                # Zigzag: alternating linear+angular for IMU excitation
                t = elapsed - 2.0  # 0..10s
                phase = t % 4.0
                tw = Twist()
                if phase < 1.0:
                    tw.linear.x = 0.5
                    tw.angular.z = 0.5
                elif phase < 2.0:
                    tw.linear.x = 0.8
                    tw.angular.z = -0.5
                elif phase < 3.0:
                    tw.linear.x = 0.3
                    tw.angular.z = 0.4
                else:
                    tw.linear.x = 0.0
                    tw.angular.z = 0.0
                self.pub.publish(tw)
                return
            else:
                self.init_done = True
                self.get_logger().info('IMU init phase complete, starting route')

        pose = self.get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # skip waypoints behind robot (X is forward axis on road route)
        while self.idx < len(ROAD) and ROAD[self.idx][0] < rx:
            self.idx += 1
        # find lookahead point ahead of robot
        while self.idx < len(ROAD):
            wx, wy = ROAD[self.idx]
            d = math.hypot(wx - rx, wy - ry)
            if d > LOOKAHEAD:
                break
            self.idx += 1

        if self.idx >= len(ROAD):
            self.get_logger().info('=== ROUTE COMPLETE ===')
            tw = Twist()
            self.pub.publish(tw)
            self.done = True
            return

        wx, wy = ROAD[self.idx]
        # angle to target in world frame
        target_yaw = math.atan2(wy - ry, wx - rx)
        # diff
        dyaw = target_yaw - ryaw
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi

        tw = Twist()
        # slow down when need to turn hard
        if abs(dyaw) > 0.5:
            tw.linear.x = MAX_LIN * 0.3
        else:
            tw.linear.x = MAX_LIN
        tw.angular.z = max(-MAX_ANG, min(MAX_ANG, 1.5 * dyaw))
        self.pub.publish(tw)

        if self.idx % 5 == 0 and abs(dyaw) < 0.1:
            self.get_logger().info(
                f'wp {self.idx}/{len(ROAD)}: pos=({rx:.0f},{ry:.0f}) target=({wx:.0f},{wy:.0f})',
                throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
