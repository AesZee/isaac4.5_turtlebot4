#!/usr/bin/env python3
"""Scripted teleop for Phase-3 mapping — drives a fixed, bounded routine over
/cmd_vel so SLAM can build a map without a human at the keyboard.

The cmd_vel watchdog stops the robot 0.5 s after the last command, so we publish
continuously at 20 Hz. Speeds stay under the real TB4 caps (lin 0.31, ang 1.9).
The robot spawns at (0,0) facing +X (a near wall), so the routine first turns to
face the open room (-X) before driving, with 360-deg spins to sweep the lidar.

Run in an isaac-ros shell with the sim + SLAM up:
    isaac-ros ; python3 verify/scripted_teleop.py
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# (linear x m/s, angular z rad/s, seconds) — tuned to stay in the open -X region.
ROUTINE = [
    (0.0,  0.6, 5.5),    # spin ~180deg -> face the open room (-X)
    (0.0,  0.0, 0.5),
    (0.18, 0.0, 4.0),    # drive ~0.7 m into open space
    (0.0,  0.7, 9.5),    # full 360deg sweep
    (0.18, 0.0, 3.0),    # creep ~0.5 m further
    (0.0, -0.7, 9.5),    # 360deg sweep the other way
    (-0.15, 0.0, 4.0),   # back up ~0.6 m (re-observe from a new spot)
    (0.0,  0.6, 9.5),    # one more sweep
    (0.0,  0.0, 0.5),
]
RATE_HZ = 20.0


def main():
    rclpy.init()
    node = rclpy.create_node("scripted_teleop")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    dt = 1.0 / RATE_HZ
    total = sum(s[2] for s in ROUTINE)
    node.get_logger().info(f"scripted teleop: {len(ROUTINE)} segments, ~{total:.0f}s")
    for lin, ang, dur in ROUTINE:
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        end = time.monotonic() + dur
        while time.monotonic() < end and rclpy.ok():
            pub.publish(t)
            time.sleep(dt)
    pub.publish(Twist())   # explicit stop
    node.get_logger().info("scripted teleop done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
