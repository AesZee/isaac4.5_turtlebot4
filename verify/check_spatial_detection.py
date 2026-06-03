#!/usr/bin/env python3
"""Phase 2 scripted acceptance gate.

Subscribes to /oakd/nn/spatial_detections (depthai_ros_msgs/SpatialDetectionArray)
and asserts the spatial-detection pipeline produced at least one LABELED detection
with a FINITE 3D pose. Saves the raw message to verify/artifacts/ for review.
Exits 0 on PASS, non-zero on FAIL (so it can gate like check_topics.sh).

Run with the sim up + the detection node running, in a shell that sources the
project ws (for the message type) and isaac-ros (for the sim DDS):
    source ~/isaac_tb4/ros2_ws/install/setup.bash ; isaac-ros
    python3 verify/check_spatial_detection.py
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetectionArray

TOPIC = "/oakd/nn/spatial_detections"
TIMEOUT_S = 30.0
OUT = os.path.join(os.path.dirname(__file__), "artifacts", "oakd_spatial_detection.txt")


def finite(v):
    return isinstance(v, float) and math.isfinite(v)


def main():
    rclpy.init()
    node = Node("check_spatial_detection")
    hit = {"msg": None}

    def on_msg(m):
        if m.detections:                       # only accept a frame that actually has a detection
            hit["msg"] = m

    node.create_subscription(SpatialDetectionArray, TOPIC, on_msg, 10)
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline and hit["msg"] is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()

    m = hit["msg"]
    if m is None:
        print(f"FAIL: no labeled detection on {TOPIC} within {TIMEOUT_S:.0f}s", file=sys.stderr)
        return 1

    d = m.detections[0]
    label = d.results[0].class_id if d.results else ""
    p = d.position
    pos_finite = finite(p.x) and finite(p.y) and finite(p.z)
    nonzero_depth = p.z > 0.0

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write(f"topic: {TOPIC}\n")
        f.write(f"frame_id: {m.header.frame_id}\n")
        f.write(f"num_detections: {len(m.detections)}\n")
        f.write(f"class_id: {label}\n")
        f.write(f"score: {d.results[0].score if d.results else None}\n")
        f.write(f"bbox: center=({d.bbox.center.position.x:.1f},{d.bbox.center.position.y:.1f}) "
                f"size=({d.bbox.size_x:.1f}x{d.bbox.size_y:.1f})\n")
        f.write(f"position_m: x={p.x:.4f} y={p.y:.4f} z={p.z:.4f}\n")

    if not label:
        print("FAIL: detection has empty class_id (not labeled)", file=sys.stderr)
        return 1
    if not (pos_finite and nonzero_depth):
        print(f"FAIL: 3D pose not finite/positive: ({p.x},{p.y},{p.z})", file=sys.stderr)
        return 1

    print(f"PASS: labeled detection '{label}' (score {d.results[0].score:.2f}) at "
          f"finite 3D pose ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m in {m.header.frame_id}. "
          f"Saved -> {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
