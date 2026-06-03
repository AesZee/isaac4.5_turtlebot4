#!/usr/bin/env python3
"""Dock / undock controller for the Isaac TurtleBot4 sim.

Exposes the SAME ROS interfaces as the real Create3 base:
    /dock        action  irobot_create_msgs/action/Dock
    /undock      action  irobot_create_msgs/action/Undock
    /dock_status topic   irobot_create_msgs/msg/DockStatus

There are no IR dock sensors in the sim, so docking is *behavioral*: a simple
controller drives the robot with /cmd_vel using /odom feedback. The robot spawns
docked at the odom origin with the dock in front of it (see spawn_turtlebot4.py),
so:
  - undock = reverse off the dock by UNDOCK_DISTANCE, then turn ~180 deg to face away
             from the dock (ready to drive off for the next action),
  - dock   = drive back to the odom origin (DOCK_ODOM_POSE) and align heading.

/cmd_vel flows through the sim's watchdog -> diff drive, so this node publishes at
CTRL_HZ during a maneuver to keep the robot moving, then stops.

Run it in the Terminal 2 that already talks to the sim:
    isaac-ros
    python3 ~/isaac_tb4/scripts/dock_controller.py

Then drive it like the real robot:
    ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
    ros2 action send_goal /dock   irobot_create_msgs/action/Dock   {}
    ros2 topic echo /dock_status
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

# ── tuning ─────────────────────────────────────────────────────────────────
DOCK_ODOM_POSE = (0.0, 0.0, 0.0)   # (x, y, yaw) the robot returns to when docking
UNDOCK_DISTANCE = 0.30             # meters to reverse off the dock when undocking
UNDOCK_TURN_ANGLE = math.pi        # rad to turn after backing off (pi = face away, ready to go)
DRIVE_SPEED = 0.15                 # m/s approach speed (well under the 0.31 cap)
TURN_SPEED  = 0.8                  # rad/s
POS_TOL = 0.03                     # m  — "arrived" position tolerance
YAW_TOL = 0.10                     # rad (~6 deg) — final heading tolerance
HEADING_GATE = 0.15                # rad — turn in place until roughly facing target
HEADING_ACTIVE_DIST = 0.15         # m  — only steer toward the dock beyond this; drive
                                   #      straight when closer (avoids spin/overshoot)
DOCK_VISIBLE_RANGE = 1.0           # m  — report dock_visible / sees_dock within this
DOCK_TIMEOUT = 45.0                # s  — hard cap on the approach phase
YAW_TIMEOUT = 10.0                 # s  — hard cap on the final-heading phase
CTRL_HZ = 20.0


def _norm(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DockController(Node):
    def __init__(self):
        super().__init__("dock_controller")
        cb = ReentrantCallbackGroup()
        self.pose = None          # (x, y, yaw) in the odom frame
        self.is_docked = True     # robot spawns docked

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(DockStatus, "/dock_status", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10, callback_group=cb)
        self.create_timer(0.5, self._publish_status, callback_group=cb)

        self._dock_srv = ActionServer(
            self, Dock, "dock",
            execute_callback=self._exec_dock,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self._undock_srv = ActionServer(
            self, Undock, "undock",
            execute_callback=self._exec_undock,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self.get_logger().info(f"dock_controller ready (is_docked={self.is_docked})")

    # ── feedback / state ────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _dock_distance(self):
        if self.pose is None:
            return float("inf")
        gx, gy, _ = DOCK_ODOM_POSE
        return math.hypot(gx - self.pose[0], gy - self.pose[1])

    def _publish_status(self):
        msg = DockStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.dock_visible = self._dock_distance() < DOCK_VISIBLE_RANGE
        msg.is_docked = self.is_docked
        self.status_pub.publish(msg)

    # ── motion helpers ──────────────────────────────────────────────────────
    def _drive(self, lin=0.0, ang=0.0):
        tw = Twist()
        tw.linear.x = float(lin)
        tw.angular.z = float(ang)
        self.cmd_pub.publish(tw)

    def _stop(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)

    def _wait_for_pose(self):
        while self.pose is None and rclpy.ok():
            time.sleep(1.0 / CTRL_HZ)
        return self.pose is not None

    # ── actions ─────────────────────────────────────────────────────────────
    def _exec_undock(self, goal_handle):
        dt = 1.0 / CTRL_HZ
        if not self._wait_for_pose():
            goal_handle.abort()
            return Undock.Result(is_docked=self.is_docked)
        x0, y0, _ = self.pose
        self.get_logger().info("undocking...")
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return Undock.Result(is_docked=self.is_docked)
            x, y, _ = self.pose
            if math.hypot(x - x0, y - y0) >= UNDOCK_DISTANCE:
                break
            self._drive(lin=-DRIVE_SPEED)       # reverse, backward off the dock (dock is in front)
            time.sleep(dt)
        self._stop()

        # turn ~180 deg so the robot faces away from the dock, ready to drive off
        if UNDOCK_TURN_ANGLE != 0.0:
            _, _, yaw0 = self.pose
            goal_yaw = _norm(yaw0 + UNDOCK_TURN_ANGLE)
            self.get_logger().info("turning away from the dock...")
            t1 = time.monotonic()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    return Undock.Result(is_docked=self.is_docked)
                _, _, yaw = self.pose
                yerr = _norm(goal_yaw - yaw)
                if abs(yerr) <= YAW_TOL or time.monotonic() - t1 > YAW_TIMEOUT:
                    break
                self._drive(ang=_clamp(1.5 * yerr, -TURN_SPEED, TURN_SPEED))
                time.sleep(dt)
            self._stop()

        self.is_docked = False
        self._publish_status()
        goal_handle.succeed()
        self.get_logger().info("undocked")
        return Undock.Result(is_docked=False)

    def _exec_dock(self, goal_handle):
        dt = 1.0 / CTRL_HZ
        gx, gy, gyaw = DOCK_ODOM_POSE
        if not self._wait_for_pose():
            goal_handle.abort()
            return Dock.Result(is_docked=self.is_docked)
        self.get_logger().info("docking...")
        fb = Dock.Feedback()

        # Phase 1 — reach the dock position. Steer toward it only while far; once
        # close, drive straight (no heading chase) so it can't spin or overshoot.
        t0 = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return Dock.Result(is_docked=self.is_docked)
            x, y, yaw = self.pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            fb.sees_dock = dist < DOCK_VISIBLE_RANGE
            goal_handle.publish_feedback(fb)
            if dist <= POS_TOL or time.monotonic() - t0 > DOCK_TIMEOUT:
                break
            if dist > HEADING_ACTIVE_DIST:
                herr = _norm(math.atan2(dy, dx) - yaw)
                if abs(herr) > HEADING_GATE:
                    self._drive(ang=_clamp(2.0 * herr, -TURN_SPEED, TURN_SPEED))
                else:
                    self._drive(lin=min(DRIVE_SPEED, 0.6 * dist),
                                ang=_clamp(1.2 * herr, -TURN_SPEED, TURN_SPEED))
            else:
                self._drive(lin=min(0.08, 1.0 * dist))   # close in, straight
            time.sleep(dt)

        # Phase 2 — align to the dock's heading.
        t1 = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return Dock.Result(is_docked=self.is_docked)
            _, _, yaw = self.pose
            yerr = _norm(gyaw - yaw)
            if abs(yerr) <= YAW_TOL or time.monotonic() - t1 > YAW_TIMEOUT:
                break
            self._drive(ang=_clamp(1.5 * yerr, -TURN_SPEED, TURN_SPEED))
            time.sleep(dt)

        self._stop()
        self.is_docked = True
        self._publish_status()
        goal_handle.succeed()
        self.get_logger().info("docked")
        return Dock.Result(is_docked=True)


def main():
    rclpy.init()
    node = DockController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
