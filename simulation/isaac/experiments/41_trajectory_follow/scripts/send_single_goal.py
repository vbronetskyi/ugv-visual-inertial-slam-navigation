#!/usr/bin/env python3
"""Send single goal to Nav2: reach route endpoint."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=70.4)
    parser.add_argument("--y", type=float, default=-2.3)
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("single_goal_sender")

    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    node.get_logger().info("Waiting for Nav2...")
    client.wait_for_server(timeout_sec=30.0)
    node.get_logger().info("Nav2 ready.")

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.w = 1.0

    node.get_logger().info(f"Sending goal: ({args.x}, {args.y})")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is None:
        node.get_logger().error("Goal send failed")
        return
    gh = future.result()
    if not gh.accepted:
        node.get_logger().error("Goal REJECTED")
        return

    node.get_logger().info("Goal accepted. Navigating...")
    result_future = gh.get_result_async()

    start = time.time()
    while not result_future.done() and time.time() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=1.0)
        elapsed = time.time() - start
        if int(elapsed) % 30 == 0 and int(elapsed) > 0:
            node.get_logger().info(f"  t={elapsed:.0f}s navigating...")

    if result_future.done():
        status = result_future.result().status
        names = {1: "ACCEPTED", 2: "EXECUTING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        node.get_logger().info(f"RESULT: {names.get(status, status)}")
    else:
        node.get_logger().warn(f"TIMEOUT after {args.timeout}s")
        gh.cancel_goal_async()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
