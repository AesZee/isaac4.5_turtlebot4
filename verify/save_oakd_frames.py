#!/usr/bin/env python3
"""Grab one frame each from the OAK-D RGB + depth topics and save them to
verify/artifacts/ for a human to eyeball later (Phase 1 acceptance).

Saves:
  oakd_rgb.png         RGB frame (rgb8 -> PNG)
  oakd_depth.png       depth frame, normalized to 8-bit for viewing
  oakd_depth.npy       depth frame, raw float32 metres (full fidelity)
  oakd_camera_info.txt the CameraInfo (frame_id + intrinsics)

Run inside an `isaac-ros` shell with the sim up:
    isaac-ros ; python3 verify/save_oakd_frames.py
Exits non-zero if a frame doesn't arrive within the timeout (so it can gate).
"""
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from PIL import Image as PILImage

OUT = os.path.join(os.path.dirname(__file__), "artifacts")
RGB_TOPIC   = "/oakd/rgb/image_raw"
DEPTH_TOPIC = "/oakd/stereo/image_raw"
INFO_TOPIC  = "/oakd/rgb/camera_info"
TIMEOUT_S   = 30.0


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    node = Node("oakd_frame_saver")
    bridge = CvBridge()
    got = {"rgb": None, "depth": None, "info": None}

    node.create_subscription(Image, RGB_TOPIC, lambda m: got.__setitem__("rgb", m), 1)
    node.create_subscription(Image, DEPTH_TOPIC, lambda m: got.__setitem__("depth", m), 1)
    node.create_subscription(CameraInfo, INFO_TOPIC, lambda m: got.__setitem__("info", m), 1)

    deadline = time.monotonic() + TIMEOUT_S
    # Wait until we have a populated camera_info (width>0) and one rgb + depth frame.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        info_ok = got["info"] is not None and got["info"].width > 0
        if got["rgb"] is not None and got["depth"] is not None and info_ok:
            break

    missing = [k for k in ("rgb", "depth", "info") if got[k] is None]
    if got["info"] is not None and got["info"].width == 0:
        missing.append("info(width=0)")
    if missing:
        print(f"FAIL: did not receive {missing} within {TIMEOUT_S:.0f}s", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1

    # RGB -> PNG
    rgb = bridge.imgmsg_to_cv2(got["rgb"], desired_encoding="rgb8")
    PILImage.fromarray(rgb).save(os.path.join(OUT, "oakd_rgb.png"))

    # Depth (32FC1 metres) -> raw .npy + normalized 8-bit PNG for viewing
    depth = bridge.imgmsg_to_cv2(got["depth"], desired_encoding="passthrough")
    depth = np.asarray(depth, dtype=np.float32)
    np.save(os.path.join(OUT, "oakd_depth.npy"), depth)
    finite = depth[np.isfinite(depth) & (depth > 0)]
    if finite.size:
        lo, hi = float(finite.min()), float(finite.max())
        norm = np.zeros_like(depth)
        m = np.isfinite(depth) & (depth > 0)
        norm[m] = (depth[m] - lo) / (hi - lo + 1e-9)
        png = (norm * 255).astype(np.uint8)
    else:
        lo = hi = 0.0
        png = np.zeros(depth.shape, dtype=np.uint8)
    PILImage.fromarray(png).save(os.path.join(OUT, "oakd_depth.png"))

    info = got["info"]
    with open(os.path.join(OUT, "oakd_camera_info.txt"), "w") as f:
        f.write(f"frame_id: {info.header.frame_id}\n")
        f.write(f"resolution: {info.width}x{info.height}\n")
        f.write(f"distortion_model: {info.distortion_model}\n")
        f.write(f"K (fx,0,cx, 0,fy,cy, 0,0,1): {list(info.k)}\n")
        f.write(f"D: {list(info.d)}\n")

    print(f"OK: saved RGB {rgb.shape}, depth {depth.shape} "
          f"(range {lo:.3f}..{hi:.3f} m, {finite.size} valid px) to {OUT}")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
