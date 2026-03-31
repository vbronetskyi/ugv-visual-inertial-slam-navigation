#!/usr/bin/env python3
"""Exp 50 supervisor: watch robot GT position, remove obstacles at turnaround.

When robot's x coord exceeds threshold (approaching turnaround at x=70),
writes /tmp/isaac_remove_obstacles.txt. Isaac Sim picks up this signal
and calls spawn_obstacles.remove_obstacles(stage) removing all USD prims
under /World/NavObstacles.

Also clears Nav2 costmaps so replanning uses obstacle-free map.

usage:
  python3 turnaround_supervisor.py --threshold 65
"""
import argparse
import os
import subprocess
import time


def clear_nav2_costmaps():
    """Clear Nav2 global + local costmaps so removed obstacles disappear."""
    try:
        subprocess.run(
            ["ros2", "service", "call",
             "/global_costmap/clear_entirely_global_costmap",
             "nav2_msgs/srv/ClearEntireCostmap"],
            timeout=10, capture_output=True, check=False)
        subprocess.run(
            ["ros2", "service", "call",
             "/local_costmap/clear_entirely_local_costmap",
             "nav2_msgs/srv/ClearEntireCostmap"],
            timeout=10, capture_output=True, check=False)
        print("  Nav2 costmaps cleared")
    except Exception as e:
        print(f"  Costmap clear failed: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--threshold", type=float, default=65.0,
                        help="X position threshold for obstacle removal")
    parser.add_argument("--pose-file", default="/tmp/isaac_pose.txt")
    parser.add_argument("--signal-file", default="/tmp/isaac_remove_obstacles.txt")
    args = parser.parse_args()

    print(f"Supervisor: watching {args.pose_file} for x > {args.threshold}")
    triggered = False
    last_report = 0

    while True:
        try:
            with open(args.pose_file) as f:
                parts = f.readline().split()
            x = float(parts[0])
            y = float(parts[1])
        except Exception:
            time.sleep(1)
            continue

        if time.time() - last_report > 30:
            state = "TRIGGERED" if triggered else "WAITING"
            print(f"  [{state}] robot at ({x:.1f}, {y:.1f})")
            last_report = time.time()

        if not triggered and x > args.threshold:
            print(f"=== TURNAROUND DETECTED at ({x:.1f},{y:.1f}) - removing obstacles ===")
            with open(args.signal_file, "w") as f:
                f.write("remove")
            time.sleep(3)  # wait for Isaac to process
            clear_nav2_costmaps()
            triggered = True
            print(f"  Supervisor done. Robot continues return without obstacles.")

        if triggered:
            # Done - exit after a short delay to let things settle
            time.sleep(5)
            break

        time.sleep(2)


if __name__ == "__main__":
    main()
