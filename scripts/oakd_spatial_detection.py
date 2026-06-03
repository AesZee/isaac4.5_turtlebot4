#!/usr/bin/env python3
"""OAK-D spatial-detection node (Phase 2).

Mirrors what the real OAK-D does on-device for spatial detections: run a 2D
detector on the RGB image, then fuse with the stereo depth to lift each 2D
detection to a 3D position in the camera optical frame. Here the "2D detector"
is a color segmentation for the known red cube (deterministic + model-free, so
it runs headless with no weights download); the depth fusion + message contract
are identical to the hardware path.

Publishes  /oakd/nn/spatial_detections  (depthai_ros_msgs/msg/SpatialDetectionArray)
— the exact topic/type the real depthai_ros_driver exposes on the TurtleBot4 —
from:
  /oakd/rgb/image_raw      (rgb8)
  /oakd/stereo/image_raw   (32FC1 depth, metres)
  /oakd/rgb/camera_info    (intrinsics)

Run in a shell that sources the project ws (for depthai_ros_msgs) + isaac-ros:
    source ~/isaac_tb4/ros2_ws/install/setup.bash
    isaac-ros
    python3 ~/isaac_tb4/scripts/oakd_spatial_detection.py
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from vision_msgs.msg import ObjectHypothesis, BoundingBox2D
from cv_bridge import CvBridge
from depthai_ros_msgs.msg import SpatialDetection, SpatialDetectionArray

RGB_TOPIC   = "/oakd/rgb/image_raw"
DEPTH_TOPIC = "/oakd/stereo/image_raw"
INFO_TOPIC  = "/oakd/rgb/camera_info"
OUT_TOPIC   = "/oakd/nn/spatial_detections"

CLASS_ID    = "red_cube"
MIN_AREA_PX = 400          # ignore tiny red specks
MIN_VALID_DEPTH_PX = 20    # need this many valid depth samples in the bbox


class SpatialDetector(Node):
    def __init__(self):
        super().__init__("oakd_spatial_detection")
        self.bridge = CvBridge()
        self.depth = None
        self.info = None
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, 1)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_info, 1)
        self.create_subscription(Image, RGB_TOPIC, self._on_rgb, 1)
        self.pub = self.create_publisher(SpatialDetectionArray, OUT_TOPIC, 10)
        self.get_logger().info(f"publishing {OUT_TOPIC} (depthai_ros_msgs/SpatialDetectionArray)")

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        if msg.width > 0:
            self.info = msg

    def _on_rgb(self, msg):
        out = SpatialDetectionArray()
        out.header = msg.header                       # stamp + optical frame from the RGB image
        if self.depth is not None and self.info is not None:
            det = self._detect(msg)
            if det is not None:
                out.detections.append(det)
        self.pub.publish(out)

    def _detect(self, rgb_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        # Red wraps the hue circle: low and high ends, both saturated + bright.
        m1 = cv2.inRange(hsv, (0, 90, 50), (12, 255, 255))
        m2 = cv2.inRange(hsv, (168, 90, 50), (180, 255, 255))
        mask = cv2.morphologyEx(m1 | m2, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < MIN_AREA_PX:
            return None
        x, y, w, h = cv2.boundingRect(c)
        u = x + w / 2.0
        v = y + h / 2.0

        # Stereo-depth fusion: median valid depth in the central part of the bbox.
        depth = np.asarray(self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough"),
                           dtype=np.float32)
        Hd, Wd = depth.shape[:2]
        sx, sy = Wd / rgb.shape[1], Hd / rgb.shape[0]   # depth may differ in size from rgb
        cx0 = int((x + 0.25 * w) * sx); cx1 = int((x + 0.75 * w) * sx)
        cy0 = int((y + 0.25 * h) * sy); cy1 = int((y + 0.75 * h) * sy)
        patch = depth[max(cy0, 0):max(cy1, 1), max(cx0, 0):max(cx1, 1)]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size < MIN_VALID_DEPTH_PX:
            return None
        Z = float(np.median(valid))

        # Back-project the bbox centre through the pinhole model -> 3D in optical frame.
        K = self.info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        det = SpatialDetection()
        hyp = ObjectHypothesis()
        hyp.class_id = CLASS_ID
        hyp.score = float(min(1.0, area / (rgb.shape[0] * rgb.shape[1] * 0.5)))
        det.results.append(hyp)
        bb = BoundingBox2D()
        bb.center.position.x = float(u)
        bb.center.position.y = float(v)
        bb.center.theta = 0.0
        bb.size_x = float(w)
        bb.size_y = float(h)
        det.bbox = bb
        det.position = Point(x=float(X), y=float(Y), z=float(Z))
        det.is_tracking = False
        det.tracking_id = ""
        return det


def main():
    rclpy.init()
    node = SpatialDetector()
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
