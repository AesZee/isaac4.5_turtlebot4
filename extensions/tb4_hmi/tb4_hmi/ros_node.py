"""ROS 2 bridge for the TB4 HMI panel.

Hosts an rclpy node inside the Isaac Sim process and spins it on a daemon thread,
so the omni.ui thread never blocks. The Isaac process is already on the sim's DDS
(ROS_DOMAIN_ID=0, rmw_fastrtps_cpp, localhost-only — set by run_isaacsim.sh), so
no extra env is needed; this node simply joins that graph.

Threading contract:
  * the executor thread ONLY writes into the lock-guarded ``_state`` dict (via the
    subscription callbacks) and runs action-future callbacks.
  * the UI thread ONLY calls the public command methods (drive/stop/send_dock/
    send_undock/publish_lightring) and reads ``snapshot()``.
rcl publish() / send_goal_async() are internally locked, so calling them from the
UI thread while the executor spins elsewhere is safe.

Topics / actions match the real Create 3 + dock_controller.py exactly:
  sub  /dock_status    irobot_create_msgs/msg/DockStatus
  sub  /battery_state  sensor_msgs/msg/BatteryState
  pub  /cmd_vel        geometry_msgs/msg/Twist
  pub  /cmd_lightring  irobot_create_msgs/msg/LightringLeds
  act  /dock           irobot_create_msgs/action/Dock
  act  /undock         irobot_create_msgs/action/Undock
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus, LightringLeds, LedColor
from irobot_create_msgs.action import Dock, Undock

# action_msgs/GoalStatus codes (avoid importing the whole module for three ints)
_STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}


class Tb4HmiRos:
    NODE_NAME = "tb4_hmi_panel"

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {
            "connected": False,
            "is_docked": None,        # bool | None (None = no /dock_status yet)
            "dock_visible": None,
            "battery_pct": None,      # float 0..1 | None
            "battery_status": None,   # BatteryState.POWER_SUPPLY_STATUS_* | None
            "action": "idle",         # short human string for the panel
        }

        # Join the existing rclpy context if one is already up; otherwise own it.
        self._owns_rclpy = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        self._node = Node(self.NODE_NAME)
        self._cmd_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._ring_pub = self._node.create_publisher(LightringLeds, "/cmd_lightring", 10)
        self._node.create_subscription(DockStatus, "/dock_status", self._on_dock, 10)
        self._node.create_subscription(BatteryState, "/battery_state", self._on_batt, 10)

        self._dock_cli = ActionClient(self._node, Dock, "dock")
        self._undock_cli = ActionClient(self._node, Undock, "undock")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="tb4_hmi_ros_spin", daemon=True)
        self._spin_thread.start()
        with self._lock:
            self._state["connected"] = True

    # ── subscription callbacks (executor thread) ─────────────────────────────
    def _on_dock(self, msg):
        with self._lock:
            self._state["is_docked"] = bool(msg.is_docked)
            self._state["dock_visible"] = bool(msg.dock_visible)

    def _on_batt(self, msg):
        with self._lock:
            self._state["battery_pct"] = float(msg.percentage)
            self._state["battery_status"] = int(msg.power_supply_status)

    # ── read side (UI thread) ────────────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _set_action(self, s):
        with self._lock:
            self._state["action"] = s

    # ── command side (UI thread) ─────────────────────────────────────────────
    def drive(self, lin, ang):
        tw = Twist()
        tw.linear.x = float(lin)
        tw.angular.z = float(ang)
        self._cmd_pub.publish(tw)

    def stop(self):
        self._cmd_pub.publish(Twist())

    def send_dock(self):
        self._send_action(self._dock_cli, Dock.Goal(), "dock")

    def send_undock(self):
        self._send_action(self._undock_cli, Undock.Goal(), "undock")

    def _send_action(self, client, goal, label):
        # non-blocking readiness check; dock_controller (isaac-dockd) must be up
        if not client.server_is_ready():
            client.wait_for_server(timeout_sec=0.2)
        if not client.server_is_ready():
            self._set_action(f"{label}: no server — start isaac-dockd")
            return
        self._set_action(f"{label}: sending…")
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, label))

    def _on_goal_response(self, fut, label):
        try:
            gh = fut.result()
        except Exception as e:  # noqa: BLE001
            self._set_action(f"{label}: error {e}")
            return
        if not gh.accepted:
            self._set_action(f"{label}: rejected")
            return
        self._set_action(f"{label}: running…")
        gh.get_result_async().add_done_callback(lambda f: self._on_result(f, label))

    def _on_result(self, fut, label):
        try:
            status = fut.result().status
        except Exception as e:  # noqa: BLE001
            self._set_action(f"{label}: error {e}")
            return
        self._set_action(f"{label}: {_STATUS.get(status, f'status {status}')}")

    def publish_lightring(self, rgb6, override=True):
        """rgb6: iterable of six (r, g, b) tuples, 0-255. Front segment = index 2."""
        msg = LightringLeds()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.override_system = bool(override)
        for i, (r, g, b) in enumerate(list(rgb6)[:6]):
            led = LedColor()
            led.red, led.green, led.blue = int(r), int(g), int(b)
            msg.leds[i] = led
        self._ring_pub.publish(msg)

    # ── teardown ─────────────────────────────────────────────────────────────
    def shutdown(self):
        try:
            self._executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if self._owns_rclpy and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
