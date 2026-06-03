# GOAL — OAK-D sensing + ignition-style HMI extension for isaac_tb4

You are working in the **isaac_tb4** repo (Isaac Sim 4.5 + TurtleBot4, ROS 2 Humble).
Your job is to bring the simulated TB4 to **real-robot parity** so code I write here ports
to the physical TurtleBot4 unchanged, and to build an **Isaac Sim GUI extension** that mimics
the TB4's HMI (status light ring + dock/undock controls).

## Before you write any code — read and understand the project
1. Read `README.md` and `COMMANDS.md` in full. Also read `~/Downloads/isaacsim_turtlebot4_setup.md`.
2. Read `scripts/spawn_turtlebot4.py` and `scripts/dock_controller.py` end to end. Note how
   `PubTf`, `ScanFilter`, the cmd_vel watchdog, and the dock actions are wired.
3. Summarize back to me, in 10 lines or less: the current topic/frame contract, what is
   VERIFIED WORKING, and exactly what you plan to touch. **Do not start Phase 1 until I confirm.**

## Hard constraints (do not violate)
- **Do not break the verified-working path.** `isaac-py scripts/spawn_turtlebot4.py` must still
  spawn the robot, publish `/scan /odom /tf /clock /cmd_vel`, respond to `isaac-teleop`, and
  dock/undock. Re-run it after every phase and confirm before moving on.
- **Match the REAL TB4 interface contract exactly** — topic names, message types, and TF frame
  names must equal what `depthai_ros_driver` + `irobot_create_msgs` publish on hardware. The whole
  point is portability. When unsure of a real name/type, look it up; don't invent one.
- The sim runs **isolated on ROS_DOMAIN_ID=0, no discovery server**. All ROS interaction goes
  through the `isaac-ros` shell. Never touch the real-robot config (domain 1 / discovery server).
- **RTX sensors only output while rendering** — the sim steps with `render=True`. Any new camera
  has the same requirement as the lidar; account for it.
- **Bring the sim up HEADLESS when you launch it yourself.** Any time *you* need the sim running to
  verify something, start it with no GUI window (headless `SimulationApp`). I'll run the windowed
  GUI myself when I want to watch it. **Headless must NOT mean no-render** — keep RTX rendering /
  render products active even when headless, or `/scan` and the new `/oakd/*` topics go silent
  (see the rule above). Add a `HEADLESS` toggle (env var or CLI flag) to `spawn_turtlebot4.py`
  defaulting to windowed, and an `isaac-py-headless`-style entry point so verification runs need no
  display. The Phase 4 HMI extension is inherently GUI — don't try to verify it headless; document
  that I enable it in the Isaac Sim window.
- **Back up before editing binaries.** Before modifying `usd/turtlebot4.usd`, copy it (the repo
  already uses the `.bak` convention). Keep edits to the USD scripted/repeatable where possible.
- Work in **small, verifiable increments**. After each phase deliver: the command to run, the
  expected output, and how to confirm it didn't regress earlier phases. Don't fabricate results —
  if you can't verify something in this environment, say so and tell me the exact command to run.
- Update `README.md`, `COMMANDS.md`, and add `isaac-*` aliases for every new entry point, in the
  same style as the existing ones.

---

## Phase 1 — OAK-D camera (RGB + depth) with real-TB4 topic parity
Add an OAK-D-class sensor to the robot so it behaves like the real OAK-D-Pro/Lite.

**Mount:** front of the robot, looking forward, at the real OAK-D height (~0.15–0.18 m). Add the
camera frames to the TF tree published by `PubTf` (don't fork TF into a second publisher).

**Publish (match `depthai_ros_driver` namespace `oakd`):**
- `/oakd/rgb/image_raw` (`sensor_msgs/Image`) + `/oakd/rgb/camera_info` (`sensor_msgs/CameraInfo`)
- `/oakd/stereo/image_raw` (depth, `sensor_msgs/Image`, 16UC1 mm or 32FC1 m — match the driver)
  + matching `camera_info`
- `/oakd/points` (`sensor_msgs/PointCloud2`) — registered depth cloud usable by a Nav2 voxel/obstacle layer
- Optical frames named like the real driver: `oakd_rgb_camera_optical_frame`,
  `oakd_link`, etc. Verify exact names against the real driver and use those.

**Acceptance:**
- With the sim running: `ros2 topic list` shows the `/oakd/*` topics; `ros2 topic hz` shows a
  sane rate; `ros2 topic echo --once /oakd/rgb/camera_info` has correct frame + intrinsics.
- In RViz, the RGB image renders and the point cloud lines up with the room/lidar scan.
- `/scan`, `/odom`, `/tf`, dock/undock all still work.

## Phase 2 — Object detection + spatial (depth) sensing
Make the OAK-D usable for perception the way the real spatial NN is.

- Run a detection node on `/oakd/rgb/image_raw` (e.g. an ultralytics YOLO model in Isaac's Python,
  or Isaac's built-in detection). Keep it a **separate, optional node/launch** — never a hard
  dependency of the spawn script.
- Publish `vision_msgs/Detection2DArray` on `/oakd/nn/detections`, and fuse with depth to publish
  **spatial 3D detections** `vision_msgs/Detection3DArray` on `/oakd/nn/spatial_detections`
  (this mirrors the real OAK-D on-device spatial NN). Use the real driver's topic names if they
  differ from these — check and match.
- Add an `isaac-oakd-detect` alias/launch and document model choice + how to swap models.

**Acceptance:** with the sim running and a known object in front of the robot,
`ros2 topic echo /oakd/nn/spatial_detections` returns a labeled detection with a plausible 3D
position in the camera/base frame.

## Phase 3 — Lidar / SLAM regression guard + clean demo loop
Lidar→SLAM is already VERIFIED. Don't change its behavior; make it turnkey and protect it.

- Confirm `/scan` is unchanged after Phases 1–2 (same frame `rplidar_link`, same filter at
  `SCAN_MIN_RANGE`, same rate).
- Provide a documented one-command-per-shell loop: spawn → `isaac-slam` → drive (`isaac-teleop`)
  → save the map → relaunch with `isaac-nav` + localization on the saved map. Add aliases if any
  step lacks one.

**Acceptance:** I can build a map by driving, save it, then localize + navigate on it via an RViz
"Nav2 Goal", all on sim time. Document the exact commands.

## Phase 4 — Isaac Sim HMI extension ("ignition-style" panel)
Build a proper Isaac Sim **extension** (omni.ext `IExt`, `extension.toml`, dockable `omni.ui`
window) that mimics the TB4's HMI and front panel. Live in `extensions/tb4_hmi/` (or the repo's
preferred layout) and document how to enable it in Isaac Sim.

**Light ring (Create 3 parity):** a 6-segment RGB ring widget reflecting robot state, driven by
real signals:
- subscribe to `/dock_status` (`irobot_create_msgs/msg/DockStatus`) and battery state
  (`sensor_msgs/msg/BatteryState` on the real TB4) to choose colors/animation
- mirror Create 3 semantics: e.g. docked/charging = pulsing, undocked/idle = solid, low battery /
  error = distinct state. State the mapping you implement.
- optional: also publish `/cmd_lightring` (`irobot_create_msgs/msg/LightringLeds`) and drive a
  matching emissive ring prim on the robot in-sim, so the on-robot ring and the panel agree.

**Controls:**
- **Dock** and **Undock** buttons that send goals to the existing `/dock` and `/undock` actions
  (reuse `scripts/dock_controller.py` — start its server if not running, or document that
  `isaac-dockd` must be up). Show goal/feedback/result state in the panel.
- Battery % readout; a few teleop nudge buttons (forward/turn) publishing `/cmd_vel`; an E-STOP
  that zeroes `/cmd_vel`.

**ROS-from-extension constraint:** the Isaac GUI process must talk ROS 2 on **domain 0** with the
sim's DDS settings (same as `isaac-ros`). Make sure `rclpy` spins without blocking the UI thread
(use a timer/executor pattern) and document any env the extension needs.

**Acceptance:**
- Extension loads in Isaac Sim and docks as a panel.
- Clicking **Undock** reverses + turns the robot; **Dock** returns it — same as `isaac-undock` /
  `isaac-dock`.
- The ring changes when `/dock_status` flips and when battery drops.
- Nothing about the panel breaks the spawn/scan/teleop path.

---

## Deliverables checklist (end state)
- [ ] `/oakd/*` RGB + depth + points topics, correct types/frames, in TF, render-gated.
- [ ] Optional detection node → `/oakd/nn/spatial_detections` (3D), modular launch.
- [ ] Lidar/SLAM still verified; documented map→save→localize→nav loop.
- [ ] `extensions/tb4_hmi/` Isaac Sim extension: 6-seg light ring + dock/undock + battery + e-stop.
- [ ] `README.md` + `COMMANDS.md` updated; new `isaac-*` aliases added in existing style.
- [ ] A short `VERIFY.md` (or README section) with the exact per-phase commands and expected output.

Start with the "read and understand the project" step and your 10-line summary + plan. Wait for my
go-ahead before Phase 1.
