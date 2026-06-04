#!/usr/bin/env python3
"""Out-of-process /cmd_vel watchdog for the Isaac TurtleBot4 sim.

Relays /cmd_vel -> /cmd_vel_watchdog (the topic the sim's OmniGraph ROS2SubscribeTwist
actually reads) and publishes a single zero Twist once no command has arrived for
CMD_VEL_TIMEOUT seconds. Holding a teleop key streams messages that keep resetting the
timer; a single tap drives briefly and stops, mirroring the real Create3 base.

WHY A SEPARATE PROCESS: the same relay used to live inside spawn_turtlebot4.py as an
in-process rclpy node. But an rclpy participant created inside the Isaac process (which
already hosts Isaac's rclcpp ROS 2 bridge participant) does NOT receive messages from
OTHER processes — teleop, dock_controller and the HMI panel all publish /cmd_vel from
separate processes, so their commands never reached the in-process node and the robot
never moved. Run from its own process, rclpy receives that external traffic normally.
The sim's OmniGraph SubTwist (rclcpp) DOES receive external traffic, which is why
feeding /cmd_vel_watchdog directly always worked — we just restore that feed here.

Run it in the Terminal that talks to the sim (started automatically by isaac-hmi/spawn,
or standalone):
    isaac-ros
    python3 ~/isaac_tb4/scripts/cmd_vel_watchdog.py
"""
import os
import time

# ── pin this process to the SIM DDS (domain isolation) ───────────────────────
# Same rationale as dock_controller.py: this machine is also configured for the real
# robot (domain 1 + discovery server) via ~/.bashrc. Force the sim's local domain so
# the watchdog always meets the sim no matter which shell starts it. Only sets THIS
# process's env; never touches the real-robot config.
os.environ.pop("ROS_DISCOVERY_SERVER", None)
os.environ.pop("ROS_SUPER_CLIENT", None)
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ISAAC_ROS_DOMAIN_ID", "0")
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Must match spawn_turtlebot4.py.
CMD_VEL_TOPIC = "/cmd_vel"
WATCHDOG_TOPIC = "/cmd_vel_watchdog"
CMD_VEL_TIMEOUT = 0.5    # seconds without a command before the wheels are zeroed
TICK_HZ = 50.0           # staleness-check rate


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("tb4_cmd_vel_watchdog")
        self._relay = self.create_publisher(Twist, WATCHDOG_TOPIC, 10)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd, 10)
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self._last_t = None
        self._stopped = True
        self.get_logger().info(
            f"cmd_vel watchdog ready ({CMD_VEL_TOPIC} -> {WATCHDOG_TOPIC}, "
            f"timeout {CMD_VEL_TIMEOUT}s)")

    def _on_cmd(self, msg):
        self._relay.publish(msg)            # forward the command immediately
        self._last_t = time.monotonic()
        self._stopped = False

    def _tick(self):
        if (not self._stopped and self._last_t is not None
                and time.monotonic() - self._last_t > CMD_VEL_TIMEOUT):
            self._relay.publish(Twist())    # one zero Twist, then go quiet
            self._stopped = True


def main():
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
