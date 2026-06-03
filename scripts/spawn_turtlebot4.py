"""Spawn TurtleBot4 in the mapped ground-floor scene with a full ROS 2 bridge.

Loads : ~/isaac_tb4/scenes/A-1_ground.usd  +  ~/isaac_tb4/usd/turtlebot4.usd
Wires : /clock, /cmd_vel -> diff drive, /odom, /tf, /scan (RTX lidar)
Extras: robot auto-placed at the most open free cell of the occupancy map;
        a /cmd_vel watchdog stops the robot after a command goes stale, so a
        one-shot teleop tap drives briefly then stops (like the real base).

Run   : isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py
        SPAWN_HEADLESS=1 isaac-py ...     # no GUI (for testing)
Then in another terminal:  isaac-ros ; ros2 topic list ; isaac-teleop
"""
import math
import os
import time

import numpy as np
import yaml
from PIL import Image

# ── paths / prims ──────────────────────────────────────────────────────────
HOME       = os.path.expanduser("~")
GROUND_USD = f"{HOME}/isaac_tb4/scenes/A-1_ground.usd"
ROBOT_USD  = f"{HOME}/isaac_tb4/usd/turtlebot4.usd"
MAP_YAML   = f"{HOME}/isaac_tb4/maps/A-1_map.yaml"
ROBOT_PRIM = "/World/Turtlebot4"
ART_ROOT   = "/World/Turtlebot4/base_link"   # articulation root / base rigid body

# ── options ────────────────────────────────────────────────────────────────
ENABLE_LIDAR = True
HEADLESS     = os.environ.get("SPAWN_HEADLESS", "0") == "1"
# The URDF mounts the lidar on shell_link (+0.0987 m), but that fixed-joint offset
# collapsed in the USD conversion: rplidar_link ends up only ~0.064 m above
# base_link, so the RTX lidar sits at body height and its beams hit the robot's own
# shell in every direction (a ~0.15 m ring instead of the room). Lift the sensor to
# the real TB4 lidar height so it clears the body (and the low dock). The rplidar_link
# TF frame stays put — for 2D scans on vertical walls the small z offset is harmless.
LIDAR_MOUNT_Z = 0.13   # meters above rplidar_link (-> ~0.19 m above base_link)
# At the real lidar height the beams still hit the robot's own tower standoffs and
# OAK-D camera bracket (all within ~0.27 m), which would map as phantom obstacles
# ringing the robot. Every such self-hit is closer than the room walls (>=0.3 m), so
# we drop returns below SCAN_MIN_RANGE: the RTX lidar publishes RAW_SCAN_TOPIC and a
# small filter republishes a cleaned /scan. Set SCAN_MIN_RANGE = 0 to disable.
SCAN_MIN_RANGE = 0.32          # meters; drop closer returns (robot's own structure)
RAW_SCAN_TOPIC = "/scan_raw"   # internal: RTX lidar -> scan filter -> /scan

# ── OAK-D camera (Phase 1) ───────────────────────────────────────────────────
# A single RTX camera on the OAK-D mount publishes the same topic/frame/type set
# the real depthai_ros_driver exposes on the TurtleBot4, so sim code ports 1:1:
#   /oakd/rgb/image_raw      sensor_msgs/Image       (rgb8)
#   /oakd/rgb/camera_info    sensor_msgs/CameraInfo  (intrinsics, optical frame)
#   /oakd/stereo/image_raw   sensor_msgs/Image       (32FC1 depth)
#   /oakd/points             sensor_msgs/PointCloud2 (depth point cloud)
# Like the RTX lidar, the camera only publishes WHILE RENDERING (render=True).
ENABLE_OAKD = True
OAKD_RGB_TOPIC    = "/oakd/rgb/image_raw"
OAKD_INFO_TOPIC   = "/oakd/rgb/camera_info"
OAKD_DEPTH_TOPIC  = "/oakd/stereo/image_raw"
OAKD_POINTS_TOPIC = "/oakd/points"
# depthai publishes rgb + aligned depth/points in the RGB optical frame; match it.
OAKD_FRAME = "oakd_rgb_camera_optical_frame"
# Resolution is reduced from the real 1280x720 to keep the headless RTX render
# light (so /scan stays at rate); intrinsics scale with it, parity is in the
# topic/frame/type names + message types, not the pixel count. 16:9 like the OAK-D.
OAKD_W, OAKD_H = 640, 360
# OAK-D mount on base_link: ~front-top of the tower, looking forward (+X).
OAKD_MOUNT_XYZ = (0.12, 0.0, 0.16)   # meters in base_link
# Optics -> ~69deg horizontal FOV (OAK-D RGB-ish): HFOV = 2*atan(hAperture/2/focal).
OAKD_FOCAL      = 15.24
OAKD_H_APERTURE = 20.955

# ── spawn pose ─────────────────────────────────────────────────────────────
# Where the robot appears. Set SPAWN_XY to a (x, y) tuple in map/world meters to
# pin it; leave it None to auto-pick the most open free cell of the occupancy map.
SPAWN_XY  = (0.0, 0.0)     # e.g. (-0.69, 0.32)
# Heading, radians (0 = +X, pi/2 = +Y). Env-overridable: the default spawn (0,0)
# faces a near wall on +X, so the Phase-2 detection scenario sets SPAWN_YAW=3.14159
# (face the open room on -X) where the known object sits in the camera's clear view.
SPAWN_YAW = float(os.environ.get("SPAWN_YAW", "0.0"))
SPAWN_Z   = 0.06     # height above the floor, meters

# ── dock ───────────────────────────────────────────────────────────────────
# A visual charging dock placed at the spawn pose, so the robot starts "docked".
# Behavioral dock/undock is driven by scripts/dock_controller.py over /cmd_vel
# (the dock returns the robot to the odom origin). Visual only — no collision.
# Default: robot starts docked (verified behavior). Set SPAWN_NO_DOCK=1 to omit the
# visual dock — used by the Phase-2 detection scenario so the forward camera has a
# clear view of the known object instead of staring at the dock plate it noses up to.
ADD_DOCK  = os.environ.get("SPAWN_NO_DOCK", "0") != "1"
DOCK_PRIM = "/World/Dock"

# ── known object (Phase 2: spatial detection target) ────────────────────────
# A vivid red cube in the camera's forward FOV, used as the known object the
# spatial-detection node finds (color-segment -> 2D bbox -> depth -> 3D pose).
# Placed high/far enough to clear the dock back plate (top z=0.14) that the robot
# noses up to. Visual only (no collision), so it never interferes with driving.
# Pose is in WORLD meters (robot spawns at SPAWN_XY/yaw, so +X world = forward).
ADD_KNOWN_OBJECT  = True
KNOWN_OBJ_PRIM    = "/World/KnownObject"
# In free space on -X (the open room), ~0.8 m ahead of the camera when the robot
# faces -X (SPAWN_YAW=pi). Spans z 0..0.3 so it straddles the camera optical axis
# (~0.22 m). Map free space here is clear to >=1.5 m.
KNOWN_OBJ_XYZ     = (-0.90, 0.0, 0.15)       # world meters
KNOWN_OBJ_SIZE    = (0.30, 0.30, 0.30)       # meters
KNOWN_OBJ_COLOR   = (0.90, 0.05, 0.05)       # saturated red (HSV-segmentable)
KNOWN_OBJ_LABEL   = "red_cube"

# ── initial viewport camera (GUI only) ─────────────────────────────────────
# Startup pose for the perspective viewport camera, matching the Translate/Rotate
# shown in the Property panel for /OmniverseKit_Persp. Copy new values straight
# from that panel. Set SET_INITIAL_CAMERA = False to keep Isaac's default view;
# ignored in headless.
SET_INITIAL_CAMERA = True
CAM_PRIM      = "/OmniverseKit_Persp"
CAM_TRANSLATE = (-1.71577, 1.65955, 5.12248)   # meters
CAM_ROTATE    = (20.54486, 0.0, -170.87488)    # degrees (same Euler order as the GUI)

# ── drive / robot tuning ───────────────────────────────────────────────────
# Wheel geometry (from the URDF).
WHEEL_RADIUS = 0.03575
WHEEL_BASE   = 0.233
# The URDF import left the drive wheels in POSITION drive (stiffness 625, damping
# 0), so /cmd_vel produced almost no torque and the robot barely moved. Velocity-
# driven wheels need stiffness 0 and a non-zero damping (turtlebot4.usd is already
# baked this way; set_wheel_velocity_drive re-applies it as an idempotent guard).
WHEEL_DRIVE_DAMPING = 1.0e4
# Real TurtleBot4 limits.
MAX_LINEAR_SPEED  = 0.31   # m/s
MAX_ANGULAR_SPEED = 1.9    # rad/s

# ── cmd_vel watchdog ───────────────────────────────────────────────────────
# Stop the robot if no /cmd_vel arrives within this many seconds, mirroring the
# real TurtleBot4/Create3 base which times out a velocity command. Raise it for
# longer coasting per teleop tap. The sim's Twist subscriber reads WATCHDOG_TOPIC
# (fed by the watchdog) instead of /cmd_vel directly.
CMD_VEL_TIMEOUT = 0.5
WATCHDOG_TOPIC  = "/cmd_vel_watchdog"


# ── occupancy-map helpers (no Isaac imports — safe before boot) ─────────────
def free_spawn_xy():
    """Return the (x, y) world coords of the most open free cell in the map."""
    meta = yaml.safe_load(open(MAP_YAML))
    res = float(meta["resolution"]); ox, oy, _ = meta["origin"]
    img = np.array(Image.open(os.path.join(os.path.dirname(MAP_YAML), meta["image"])).convert("L")).astype(np.float32)
    H = img.shape[0]
    free = img > 191  # clearly free
    try:
        from scipy.ndimage import distance_transform_edt
        d = distance_transform_edt(free)
        j, i = np.unravel_index(int(np.argmax(d)), d.shape)
    except Exception:
        ys, xs = np.where(free); k = len(xs) // 2; j, i = ys[k], xs[k]
    x = ox + (i + 0.5) * res
    y = oy + (H - j - 0.5) * res
    return float(x), float(y)


# ── boot Isaac Sim (must happen before any omni/pxr import) ─────────────────
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": HEADLESS})

from isaacsim.core.utils.extensions import enable_extension
for ext in ("isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots", "isaacsim.sensors.rtx"):
    enable_extension(ext)
sim_app.update()

import omni.usd
import omni.kit.commands
import omni.timeline
import omni.graph.core as og
import omni.replicator.core as rep
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage


# ── scene-build helpers (need the omni/pxr imports above) ───────────────────
def place_robot(stage, sx, sy):
    """Place the robot at (sx, sy)/SPAWN_YAW, just above the floor."""
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PRIM))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(sx, sy, SPAWN_Z))
    xform.AddRotateZOp().Set(math.degrees(SPAWN_YAW))   # RotateZ takes degrees
    print(f"[spawn] robot start = ({sx:.2f}, {sy:.2f}, yaw={SPAWN_YAW:.2f} rad)")


def add_dock(stage, sx, sy):
    """Add a simple visual charging dock at the spawn pose (robot starts docked).

    Built in the robot's frame: base plate + back plate sit just in front of the
    robot (+X), which it faces — so undocking drives the robot backward off it.
    Visual only (no collision) so it never interferes with driving.
    """
    dock = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, DOCK_PRIM))
    dock.AddTranslateOp().Set(Gf.Vec3d(sx, sy, 0.0))
    dock.AddRotateZOp().Set(math.degrees(SPAWN_YAW))

    def _plate(name, center, scale):
        cube = UsdGeom.Cube.Define(stage, f"{DOCK_PRIM}/{name}")
        cube.GetSizeAttr().Set(1.0)                       # unit cube -> scale = meters
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.10, 0.12)])
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))

    _plate("base", (0.12, 0.0, 0.006), (0.13, 0.18, 0.012))    # floor plate (in front)
    _plate("back", (0.18, 0.0, 0.070), (0.02, 0.18, 0.140))    # upright plate the robot noses up to
    print(f"[spawn] dock added at ({sx:.2f}, {sy:.2f})")


def set_wheel_velocity_drive(stage):
    """Force the drive wheels into velocity drive (stiffness 0, damping > 0)."""
    for wheel in ("left_wheel_joint", "right_wheel_joint"):
        joint = stage.GetPrimAtPath(f"{ROBOT_PRIM}/joints/{wheel}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular") or UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(WHEEL_DRIVE_DAMPING)
        drive.CreateTargetVelocityAttr().Set(0.0)
    print(f"[spawn] wheel velocity drive set (damping={WHEEL_DRIVE_DAMPING})")


def add_known_object(stage):
    """Add a vivid red cube (visual only) as the Phase-2 detection target."""
    cx, cy, cz = KNOWN_OBJ_XYZ
    cube = UsdGeom.Cube.Define(stage, KNOWN_OBJ_PRIM)
    cube.GetSizeAttr().Set(1.0)                       # unit cube -> scale = meters
    cube.CreateDisplayColorAttr([Gf.Vec3f(*KNOWN_OBJ_COLOR)])
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xf.AddScaleOp().Set(Gf.Vec3f(*KNOWN_OBJ_SIZE))
    # Semantic label for parity with a real labeled detector (optional consumer).
    try:
        from pxr import Semantics
        prim = cube.GetPrim()
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr().Set("class")
        sem.CreateSemanticDataAttr().Set(KNOWN_OBJ_LABEL)
    except Exception as e:
        print(f"[spawn] (known object semantic label skipped: {e})")
    print(f"[spawn] known object '{KNOWN_OBJ_LABEL}' at {KNOWN_OBJ_XYZ}")


def add_dome_light(stage):
    """Add a simple dome light so the scene is visible."""
    UsdGeom.Xformable(stage.DefinePrim("/World/DomeLight", "DomeLight"))
    stage.GetPrimAtPath("/World/DomeLight").GetAttribute("inputs:intensity").Set(1000.0)


def set_viewport_camera(stage):
    """Set the perspective viewport camera to the pose copied from the GUI.

    The persp camera's transform lives in the session layer, so writing CAM_PRIM's
    xform ops directly is silently shadowed. Instead we compose the copied
    translate/rotateXYZ on a scratch in-memory prim (USD handles the Euler order
    exactly), read off an eye + look-at target, and push that through Isaac's
    set_camera_view, which updates the viewport via its own API. Faithful for the
    upright orbit views the GUI produces (middle/Y rotate component 0 = no roll).
    """
    from isaacsim.core.utils.viewports import set_camera_view

    tmp = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(tmp, UsdGeom.GetStageUpAxis(stage))
    xf = UsdGeom.Xformable(UsdGeom.Camera.Define(tmp, "/Cam"))
    xf.AddTranslateOp().Set(Gf.Vec3d(*CAM_TRANSLATE))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*CAM_ROTATE))

    m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    eye = m.ExtractTranslation()
    forward = m.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0)).GetNormalized()   # camera looks down -Z
    target = eye + forward

    set_camera_view(eye=list(eye), target=list(target), camera_prim_path=CAM_PRIM)
    print(f"[spawn] viewport camera set -> t={CAM_TRANSLATE}, r={CAM_ROTATE}")


def add_rtx_lidar():
    """Create the RTX lidar on rplidar_link; return its render-product path."""
    _, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/Lidar",
        parent=f"{ROBOT_PRIM}/rplidar_link",
        config="RPLIDAR_S2E",
        translation=Gf.Vec3d(0.0, 0.0, LIDAR_MOUNT_Z),   # lift above the robot body
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    rp = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="TB4_LidarRP")
    print(f"[spawn] lidar render product = {rp.path}")
    return rp.path


def add_oakd_camera(stage):
    """Create the OAK-D RTX camera on base_link; return its render-product path.

    USD cameras look down local -Z with +Y up; the explicit transform below points
    the camera along the robot's +X (forward) with +Z up, mounted at OAKD_MOUNT_XYZ.
    The matrix rows are the camera's local x/y/z axes expressed in base_link
    (det = +1, a proper rotation): local-x -> -Y, local-y -> +Z, local-z -> -X.
    """
    cam_path = f"{ART_ROOT}/oakd_rgb_camera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    cam.CreateFocalLengthAttr().Set(OAKD_FOCAL)
    cam.CreateHorizontalApertureAttr().Set(OAKD_H_APERTURE)
    cam.CreateVerticalApertureAttr().Set(OAKD_H_APERTURE * OAKD_H / OAKD_W)
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.05, 20.0))

    tx, ty, tz = OAKD_MOUNT_XYZ
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(
        0.0, -1.0, 0.0, 0.0,
        0.0,  0.0, 1.0, 0.0,
       -1.0,  0.0, 0.0, 0.0,
         tx,   ty,  tz, 1.0,
    ))

    rp = rep.create.render_product(cam_path, [OAKD_W, OAKD_H], name="TB4_OakdRP")
    print(f"[spawn] OAK-D camera render product = {rp.path} ({OAKD_W}x{OAKD_H})")
    return rp.path


def build_action_graph(stage, render_product_path, oakd_rp_path=None):
    """Build the ROS 2 bridge OmniGraph (clock, cmd_vel->drive, odom, tf, scan, oakd)."""
    keys = og.Controller.Keys
    graph_path = "/World/ActionGraph"

    nodes = [
        ("OnTick",   "omni.graph.action.OnPlaybackTick"),
        ("Context",  "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLin", "omni.graph.nodes.BreakVector3"),
        ("BreakAng", "omni.graph.nodes.BreakVector3"),
        ("DiffCtrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("ArtCtrl",  "isaacsim.core.nodes.IsaacArticulationController"),
        ("Odom",     "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom",  "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubRawTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("PubTf",    "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    if ENABLE_LIDAR:
        nodes.append(("PubScan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
    oakd = ENABLE_OAKD and oakd_rp_path is not None
    if oakd:
        nodes += [
            ("OakRgb",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("OakDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("OakPcl",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("OakInfo",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]

    values = [
        ("PubClock.inputs:topicName", "/clock"),
        ("SubTwist.inputs:topicName", WATCHDOG_TOPIC),   # fed by the cmd_vel watchdog
        ("DiffCtrl.inputs:wheelRadius", WHEEL_RADIUS),
        ("DiffCtrl.inputs:wheelDistance", WHEEL_BASE),
        ("DiffCtrl.inputs:maxLinearSpeed", MAX_LINEAR_SPEED),
        ("DiffCtrl.inputs:maxAngularSpeed", MAX_ANGULAR_SPEED),
        ("ArtCtrl.inputs:robotPath", ART_ROOT),
        ("ArtCtrl.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
        ("PubOdom.inputs:topicName", "/odom"),
        ("PubOdom.inputs:odomFrameId", "odom"),
        ("PubOdom.inputs:chassisFrameId", "base_link"),
        ("PubRawTf.inputs:topicName", "/tf"),
        ("PubRawTf.inputs:parentFrameId", "odom"),
        ("PubRawTf.inputs:childFrameId", "base_link"),
        ("PubTf.inputs:topicName", "/tf"),
    ]
    if ENABLE_LIDAR:
        values += [
            ("PubScan.inputs:topicName", RAW_SCAN_TOPIC),   # cleaned -> /scan by ScanFilter
            ("PubScan.inputs:frameId", "rplidar_link"),
            ("PubScan.inputs:type", "laser_scan"),
            ("PubScan.inputs:renderProductPath", render_product_path),
        ]
    if oakd:
        values += [
            ("OakRgb.inputs:topicName", OAKD_RGB_TOPIC),
            ("OakRgb.inputs:frameId", OAKD_FRAME),
            ("OakRgb.inputs:type", "rgb"),
            ("OakRgb.inputs:renderProductPath", oakd_rp_path),
            ("OakDepth.inputs:topicName", OAKD_DEPTH_TOPIC),
            ("OakDepth.inputs:frameId", OAKD_FRAME),
            ("OakDepth.inputs:type", "depth"),
            ("OakDepth.inputs:renderProductPath", oakd_rp_path),
            ("OakPcl.inputs:topicName", OAKD_POINTS_TOPIC),
            ("OakPcl.inputs:frameId", OAKD_FRAME),
            ("OakPcl.inputs:type", "depth_pcl"),
            ("OakPcl.inputs:renderProductPath", oakd_rp_path),
            ("OakInfo.inputs:topicName", OAKD_INFO_TOPIC),
            ("OakInfo.inputs:frameId", OAKD_FRAME),
            ("OakInfo.inputs:renderProductPath", oakd_rp_path),
        ]

    connect = [
        ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
        ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
        ("OnTick.outputs:tick", "DiffCtrl.inputs:execIn"),
        ("OnTick.outputs:tick", "ArtCtrl.inputs:execIn"),
        ("OnTick.outputs:tick", "Odom.inputs:execIn"),
        ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
        ("OnTick.outputs:tick", "PubRawTf.inputs:execIn"),
        ("OnTick.outputs:tick", "PubTf.inputs:execIn"),
        ("Context.outputs:context", "PubClock.inputs:context"),
        ("Context.outputs:context", "SubTwist.inputs:context"),
        ("Context.outputs:context", "PubOdom.inputs:context"),
        ("Context.outputs:context", "PubRawTf.inputs:context"),
        ("Context.outputs:context", "PubTf.inputs:context"),
        ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubRawTf.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubTf.inputs:timeStamp"),
        ("SubTwist.outputs:linearVelocity", "BreakLin.inputs:tuple"),
        ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
        ("BreakLin.outputs:x", "DiffCtrl.inputs:linearVelocity"),
        ("BreakAng.outputs:z", "DiffCtrl.inputs:angularVelocity"),
        ("DiffCtrl.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
        ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
        ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ("Odom.outputs:position", "PubOdom.inputs:position"),
        ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
        ("Odom.outputs:position", "PubRawTf.inputs:translation"),
        ("Odom.outputs:orientation", "PubRawTf.inputs:rotation"),
    ]
    if ENABLE_LIDAR:
        connect += [
            ("OnTick.outputs:tick", "PubScan.inputs:execIn"),
            ("Context.outputs:context", "PubScan.inputs:context"),
        ]
    if oakd:
        for n in ("OakRgb", "OakDepth", "OakPcl", "OakInfo"):
            connect += [
                ("OnTick.outputs:tick", f"{n}.inputs:execIn"),
                ("Context.outputs:context", f"{n}.inputs:context"),
            ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.SET_VALUES: values, keys.CONNECT: connect},
    )

    # relationship (target) inputs can't go through SET_VALUES — set them via USD
    def set_targets(node, rel, targets):
        stage.GetPrimAtPath(f"{graph_path}/{node}").GetRelationship(rel).SetTargets(targets)

    set_targets("Odom", "inputs:chassisPrim", [ART_ROOT])
    set_targets("PubTf", "inputs:parentPrim", [ART_ROOT])
    # Publish the robot's link tree under base_link (so base_link->rplidar_link etc.
    # exist on /tf). ART_ROOT is the articulation root -> its whole tree is published;
    # rplidar_link is added explicitly in case the fixed sensor joint was merged.
    # Without this, /scan (frame rplidar_link) has no transform and tf2 drops it,
    # breaking SLAM, the Nav2 costmaps, and the RViz laser display.
    set_targets("PubTf", "inputs:targetPrims", [ART_ROOT, f"{ROBOT_PRIM}/rplidar_link"])
    print("[spawn] action graph built")


class CmdVelWatchdog:
    """Relay /cmd_vel -> WATCHDOG_TOPIC, then publish one zero Twist once no
    command has arrived for CMD_VEL_TIMEOUT seconds. Holding a teleop key streams
    messages that keep resetting the timer; a single tap drives briefly and stops.
    """

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._Twist = Twist
        self.node = rclpy.create_node("tb4_cmd_vel_watchdog")
        self._relay = self.node.create_publisher(Twist, WATCHDOG_TOPIC, 10)
        self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self._last_t = None
        self._stopped = True

    def _on_cmd(self, msg):
        self._relay.publish(msg)            # forward the command immediately
        self._last_t = time.monotonic()
        self._stopped = False

    def spin(self):
        """Pump ROS callbacks once and stop the wheels if the command is stale."""
        self._rclpy.spin_once(self.node, timeout_sec=0.0)
        if (not self._stopped and self._last_t is not None
                and time.monotonic() - self._last_t > CMD_VEL_TIMEOUT):
            self._relay.publish(self._Twist())
            self._stopped = True

    def close(self):
        self.node.destroy_node()


class ScanFilter:
    """Drop lidar returns closer than SCAN_MIN_RANGE by republishing RAW_SCAN_TOPIC
    (the RTX lidar's raw scan) to /scan with those returns set to +inf. Removes the
    robot's own tower/camera/shell self-hits so they don't map as phantom obstacles;
    the header (stamp/frame) is preserved so TF still lines up.
    """

    def __init__(self):
        import rclpy
        from sensor_msgs.msg import LaserScan
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node("tb4_scan_filter")
        self._pub = self.node.create_publisher(LaserScan, "/scan", 10)
        self.node.create_subscription(LaserScan, RAW_SCAN_TOPIC, self._on_scan, 10)

    def _on_scan(self, msg):
        rmin = SCAN_MIN_RANGE
        msg.ranges = [r if r >= rmin else float("inf") for r in msg.ranges]
        msg.range_min = max(msg.range_min, rmin)
        self._pub.publish(msg)

    def spin(self):
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        self.node.destroy_node()


# ── build the scene ─────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

add_reference_to_stage(GROUND_USD, "/World/Map")
add_reference_to_stage(ROBOT_USD, ROBOT_PRIM)

spawn_x, spawn_y = SPAWN_XY if SPAWN_XY is not None else free_spawn_xy()
place_robot(stage, spawn_x, spawn_y)
set_wheel_velocity_drive(stage)
if ADD_DOCK:
    add_dock(stage, spawn_x, spawn_y)
if ADD_KNOWN_OBJECT:
    add_known_object(stage)
add_dome_light(stage)
render_product_path = add_rtx_lidar() if ENABLE_LIDAR else None
oakd_rp_path = add_oakd_camera(stage) if ENABLE_OAKD else None
build_action_graph(stage, render_product_path, oakd_rp_path)

# ── run ─────────────────────────────────────────────────────────────────────
world.reset()

if os.environ.get("OAKD_DEBUG") == "1" and ENABLE_OAKD:
    _cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    _m = _cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{ART_ROOT}/oakd_rgb_camera"))
    _t = _m.ExtractTranslation()
    _fwd = _m.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    _up = _m.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
    _bl = _cache.GetLocalToWorldTransform(stage.GetPrimAtPath(ART_ROOT)).ExtractTranslation()
    print(f"[oakd-debug] base_link world t={tuple(round(v,3) for v in _bl)}")
    print(f"[oakd-debug] cam world t={tuple(round(v,3) for v in _t)} "
          f"forward={tuple(round(v,3) for v in _fwd)} up={tuple(round(v,3) for v in _up)}")

omni.timeline.get_timeline_interface().play()

# Small in-process ROS nodes (share one rclpy context; single shutdown at the end).
ros_nodes = []
try:
    watchdog = CmdVelWatchdog()
    ros_nodes.append(watchdog)
    print(f"[spawn] cmd_vel watchdog active (timeout {CMD_VEL_TIMEOUT}s)")
except Exception as e:
    watchdog = None
    print(f"[spawn] WARNING: cmd_vel watchdog disabled ({e}); robot will not auto-stop")
if ENABLE_LIDAR and SCAN_MIN_RANGE > 0:
    try:
        ros_nodes.append(ScanFilter())
        print(f"[spawn] scan filter active ({RAW_SCAN_TOPIC} -> /scan, min {SCAN_MIN_RANGE}m)")
    except Exception as e:
        print(f"[spawn] WARNING: scan filter disabled ({e}); /scan_raw carries self-hits")

# Set the camera after the first rendered frame so it wins over any auto-framing.
cam_pending = SET_INITIAL_CAMERA and not HEADLESS

print("[spawn] simulating... (Ctrl-C to stop)")
while sim_app.is_running():
    for n in ros_nodes:
        n.spin()
    world.step(render=True)   # render every step: required for the RTX lidar /scan
    if cam_pending:
        set_viewport_camera(stage)
        cam_pending = False

for n in ros_nodes:
    n.close()
try:
    import rclpy
    if rclpy.ok():
        rclpy.shutdown()
except Exception:
    pass
sim_app.close()
