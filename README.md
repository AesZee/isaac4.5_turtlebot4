# isaac_tb4 — TurtleBot4 in Isaac Sim project

Working folder for Isaac Sim 4.5 + TurtleBot4 (ROS 2 Humble).
Full setup notes: ~/Downloads/isaacsim_turtlebot4_setup.md
Terminal command cheat sheet: [COMMANDS.md](COMMANDS.md)

## Layout
- scripts/  Python scripts (run with `isaac-py <file>`)
- usd/      robot USD (turtlebot4.usd) + URDF source
- scenes/   ground-floor scene USD built from the SLAM map (A-1_ground.usd)
- maps/     ROS occupancy maps (A-1_map.pgm/.yaml)
- layouts/  saved GUI layouts (.json)

## Assets (already built)
- usd/turtlebot4.usd      — robot, converted from turtlebot4_description URDF
- scenes/A-1_ground.usd   — floor + collidable walls from maps/A-1_map (aligned to map frame)

Rebuild the map scene for another map:
    scripts/run_generate_map.sh maps/<map>.yaml scenes/<out>.usd 0.6   # args: yaml out wall_height

## ▶ Run the simulation  (VERIFIED WORKING)

Use TWO terminals.

### Terminal 1 — launch the sim (this IS isaac + the scene; do NOT also run `isaac`)
    isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py
- First load ~1-2 min. Window opens, robot appears in the mapped room, auto-plays.
- Robot auto-spawns at the most open free cell of the map (~ -0.69, 0.32).
- Ctrl-C here stops the sim. Keep it running while you use Terminal 2.

### Terminal 2 — talk to the sim
    isaac-ros            # switch this shell to the SIM's DDS (domain 0, no discovery server, resets daemon)
    ros2 topic list      # -> /clock /cmd_vel /odom /tf /scan
    isaac-teleop         # drive via /cmd_vel (i/k/j/l keys)

## Published / subscribed topics
| topic              | dir | rate   | notes |
|--------------------|-----|--------|-------|
| /clock             | pub | ~80 Hz | sim time (for use_sim_time:=true) |
| /cmd_vel           | sub |        | Twist -> differential drive (drive this; teleop/Nav2 publish here) |
| /cmd_vel_watchdog  | -   |        | internal relay: watchdog -> diff drive (don't publish to this) |
| /odom              | pub | ~74 Hz | odom->base_link |
| /tf                | pub |        | odom->base_link + the robot link tree (incl. base_link->rplidar_link) |
| /scan_raw          | -   |        | internal: raw RTX lidar -> scan filter (don't consume this) |
| /scan              | pub |        | filtered lidar (rplidar_link), 360 deg, returns >=0.32 m kept |
| /oakd/rgb/image_raw   | pub | render-gated | OAK-D RGB (rgb8), frame oakd_rgb_camera_optical_frame |
| /oakd/rgb/camera_info | pub | render-gated | OAK-D RGB intrinsics (matches /oakd/rgb/image_raw) |
| /oakd/stereo/image_raw| pub | render-gated | OAK-D depth (32FC1, metres), aligned to the RGB optical frame |
| /oakd/points          | pub | render-gated | OAK-D depth point cloud (sensor_msgs/PointCloud2) |

## OAK-D camera (Phase 1 — matches depthai_ros_driver on the real TB4)
A single RTX camera on the OAK-D mount (base_link, looking +X) publishes the same
topic/frame/type set the real `depthai_ros_driver` exposes, so perception code ports
1:1 to hardware. Like the RTX lidar, it only publishes **while rendering** (the spawn
script steps with `render=True`; headless is fine, but don't disable rendering).
- Topics/frame as in the table above; all in `oakd_rgb_camera_optical_frame`.
- Resolution is **640x360** (reduced from the real 1280x720 to keep the headless RTX
  render light so `/scan` stays at rate); intrinsics scale with it. Tune `OAKD_W/H`,
  `OAKD_MOUNT_XYZ`, and the optics (`OAKD_FOCAL`/`OAKD_H_APERTURE`) at the top of
  `scripts/spawn_turtlebot4.py`. Toggle the whole camera with `ENABLE_OAKD`.
- Save sample frames for review (RGB + depth PNG + raw .npy + camera_info) — run in an
  `isaac-ros` shell with the sim up:

      python3 ~/isaac_tb4/verify/save_oakd_frames.py     # -> verify/artifacts/

- The robot starts nosed up to the dock, so a fresh frame mostly sees the dock plate;
  undock (or place an object in front) for a clear forward view.

## Spatial detection (Phase 2 — matches depthai_ros_driver on the real TB4)
`scripts/oakd_spatial_detection.py` publishes `/oakd/nn/spatial_detections`
(`depthai_ros_msgs/msg/SpatialDetectionArray` — the exact hardware topic/type) by
running a 2D detector on the RGB image and fusing it with the stereo depth to lift
each detection to a 3D position in `oakd_rgb_camera_optical_frame` — the same path
the OAK-D runs on-device. The sim's "2D detector" is a color segmentation for a
known red cube (model-free, so it runs headless with no weights download); the
depth-fusion + message contract are identical to hardware.
- The type lives in a vendored **subset** package `ros2_ws/src/depthai_ros_msgs`
  (authentic upstream field layout from luxonis/depthai-ros humble), built into a
  project-local colcon ws: `source ros2_ws/install/setup.bash`.
- The detection scenario needs a clear camera view. The committed default spawn
  `(0,0)`+yaw0 faces a near wall on +X, so reproduce with the env toggles:

      # terminal 1 — sim facing the open room, dock omitted, cube spawned, headless
      SPAWN_KNOWN_OBJECT=1 SPAWN_NO_DOCK=1 SPAWN_YAW=3.14159 SPAWN_HEADLESS=1 isaac-py scripts/spawn_turtlebot4.py
      # terminal 2 — detector
      source ~/isaac_tb4/ros2_ws/install/setup.bash ; isaac-ros
      python3 ~/isaac_tb4/scripts/oakd_spatial_detection.py
      # terminal 3 — gate (asserts labeled detection + finite 3D pose)
      source ~/isaac_tb4/ros2_ws/install/setup.bash ; isaac-ros
      python3 ~/isaac_tb4/verify/check_spatial_detection.py

- Tune the target (`KNOWN_OBJ_*`) and `ENABLE_OAKD`/`ADD_KNOWN_OBJECT` at the top
  of `scripts/spawn_turtlebot4.py`. `OAKD_DEBUG=1` prints the camera's world pose.

## Realistic driving (matches the real TurtleBot4)
- **Wheels are velocity-driven.** The URDF import left the wheels in *position* drive
  (stiffness 625, damping 0) so /cmd_vel barely moved the robot. They're now velocity
  drive (stiffness 0, damping 1e4) — baked into usd/turtlebot4.usd and re-applied at
  spawn as a guard. Backup at usd/turtlebot4.usd.bak.
- **Top speed is the real cap:** maxLinearSpeed 0.31 m/s, maxAngularSpeed 1.9 rad/s.
- **cmd_vel watchdog:** like the real Create3 base, the robot stops if no /cmd_vel
  arrives within CMD_VEL_TIMEOUT (0.5 s). So a single teleop key *tap* drives briefly
  then stops; *holding* a key streams commands and keeps it moving. Tune CMD_VEL_TIMEOUT
  at the top of scripts/spawn_turtlebot4.py for longer coasting per tap.

## Lidar / /scan (so it matches the room)
Two things had to be fixed for the laser to be usable by SLAM/Nav2/RViz — both live
at the top of scripts/spawn_turtlebot4.py:
- **TF tree** — `PubTf` publishes the robot's link tree (`base_link->rplidar_link`
  etc.). Without it, `/scan` (frame rplidar_link) had no transform and tf2 dropped
  every scan, so SLAM/costmaps/RViz got nothing.
- **Lidar height** — the URDF mounts the lidar on shell_link, but that offset
  collapsed in the USD conversion, leaving it at body height where it only saw the
  robot's own shell (a ~0.15 m ring). `LIDAR_MOUNT_Z` (0.13 m) lifts it to the real
  ~0.19 m so it sees the room.
- **Self-hit filter** — at that height the beams still hit the tower legs / OAK-D
  bracket (all < ~0.27 m). The RTX lidar publishes `/scan_raw`; a small `ScanFilter`
  drops returns below `SCAN_MIN_RANGE` (0.32 m) and republishes `/scan`, so the
  robot's own structure isn't mapped as phantom obstacles. Those occluded wedges read
  as "no return" and fill in as the robot turns. Raise `LIDAR_MOUNT_Z` toward ~0.20 m
  for fuller 360° coverage (lidar a bit above scale); set `SCAN_MIN_RANGE = 0` to
  disable the filter.

## Dock / undock (like the real TB4)
The sim exposes the **same interfaces as the real Create3 base** — there are no IR
dock sensors in sim, so docking is *behavioral*: a controller drives the robot via
/cmd_vel using /odom feedback. The robot spawns at the odom origin nosed up to a
visual dock in front of it. The full cycle:
- **undock** — reverse straight off the dock (~0.3 m), then turn ~180° so the robot
  faces away from the dock, ready to drive off for the next action.
- **dock** — drive back to the odom origin (turning around as needed) and align onto
  the dock. Steers toward the dock while far, drives straight when close (no spin),
  and always terminates (hard timeouts).

| interface     | type                                   | notes |
|---------------|----------------------------------------|-------|
| /undock       | action irobot_create_msgs/action/Undock | reverse off the dock (~0.3 m), then turn ~180° to face away |
| /dock         | action irobot_create_msgs/action/Dock   | drive back to the odom origin + align onto the dock |
| /dock_status  | topic  irobot_create_msgs/msg/DockStatus | is_docked / dock_visible (~2 Hz) |

Run the controller in the Terminal 2 that talks to the sim, then send goals:

    isaac-ros
    isaac-dockd       # action server (= python3 scripts/dock_controller.py) — leave running
    # in another isaac-ros shell:
    isaac-undock      # = ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
    isaac-dock        # = ros2 action send_goal /dock   irobot_create_msgs/action/Dock   {}
    ros2 topic echo /dock_status

- The dock is a visual prim only (no collision) placed at the spawn pose; toggle with
  `ADD_DOCK` in spawn_turtlebot4.py.
- Tuning at the top of dock_controller.py: `UNDOCK_DISTANCE`, `UNDOCK_TURN_ANGLE`
  (set 0 to skip the turn), speeds, tolerances, and the approach/heading timeouts.
- Docking is an odom-frame approximation (no IR/AMCL); it relies on the near-perfect
  sim odometry, which holds because the robot returns to where odom started.

## Spawn pose & viewport camera (top of scripts/spawn_turtlebot4.py)
- **Robot spawn pose** — `SPAWN_XY`, `SPAWN_YAW` (radians), `SPAWN_Z`. Set `SPAWN_XY`
  to an `(x, y)` in map/world meters to pin the robot, or `None` to auto-pick the most
  open free cell of the occupancy map.
- **Startup viewport camera** — `SET_INITIAL_CAMERA`, `CAM_TRANSLATE`, `CAM_ROTATE`.
  To capture a view: orbit to it in the GUI, then copy the **Translate** and **Rotate**
  off the `/OmniverseKit_Persp` Property panel into `CAM_TRANSLATE` / `CAM_ROTATE`.
  Set `SET_INITIAL_CAMERA = False` for Isaac's default framing; ignored in headless.
  Note: that camera lives in the session layer, so it's applied via `set_camera_view`
  (eye + look-at) after the first frame — faithful for upright orbit views (the middle
  Rotate component is 0 = no roll).

## Gotchas (already handled)
- This machine is configured for the REAL robot (ROS_DOMAIN_ID=1 + discovery server).
  The sim runs ISOLATED on domain 0 with no discovery server. `isaac-ros` matches it.
- The `ros2` daemon caches the old graph when you switch domains -> `isaac-ros` now runs
  `ros2 daemon stop` so a fresh daemon picks up the sim. (If a terminal ever shows only
  /rosout + /parameter_events while the sim is up, run `ros2 daemon stop` and retry.)
- The RTX lidar only outputs /scan while RENDERING — the script steps with render=True.
- Don't launch `isaac` (empty GUI) AND the spawn script together = two Isaac instances.

## Nav2 / SLAM (sim time)
Both are verified working with the sim (frames/topics match: base_link, odom, scan,
use_sim_time). `isaac-nav` launches only the **navigation** stack — it has no
map->odom on its own, so pair it with SLAM (mapping) or localization (saved map):

    # each in its own  isaac-ros  shell, with the sim running:
    isaac-slam           # build a map + provide map->odom   (use_sim_time:=true)
    isaac-nav            # navigation stack                  (use_sim_time:=true)
    isaac-rviz           # set a "Nav2 Goal" and watch it plan/drive

- Run `isaac-nav` **alone** and the costmaps just wait ("Invalid frame map/odom") —
  that's expected; it needs SLAM or localization.launch.py (+ a saved map) for
  map->odom.
