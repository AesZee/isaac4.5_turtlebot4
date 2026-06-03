# Terminal commands — isaac_tb4 cheat sheet

Quick reference for driving the TurtleBot4 Isaac Sim. Aliases are defined in `~/.bashrc`
(run `source ~/.bashrc` after editing it). See `README.md` for details.

## Terminal 1 — launch the sim
```bash
isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py     # launch sim + scene + robot (auto-plays)
SPAWN_HEADLESS=1 isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py   # no GUI (testing)
```
- Leave it running. `Ctrl-C` stops the sim.
- Do NOT also run `isaac` (that opens a second, empty Isaac instance).

## Terminal 2 — talk to the sim
Run `isaac-ros` **first** in every shell that talks to the sim (switches to the sim's
DDS: domain 0, no discovery server, resets the ros2 daemon).
```bash
isaac-ros            # point this shell at the sim
ros2 topic list      # -> /clock /cmd_vel /odom /tf /scan /dock_status ...
ros2 topic echo /odom
```

### OAK-D camera (Phase 1)
```bash
ros2 topic echo --once /oakd/rgb/camera_info        # frame + intrinsics
ros2 topic hz /oakd/points                          # depth point cloud rate
python3 ~/isaac_tb4/verify/save_oakd_frames.py      # save RGB+depth frames -> verify/artifacts/
```
Topics: `/oakd/rgb/image_raw` `/oakd/rgb/camera_info` `/oakd/stereo/image_raw` `/oakd/points`
(frame `oakd_rgb_camera_optical_frame`). Render-gated like `/scan`.

### Spatial detection (Phase 2)
```bash
# sim facing the open room with the known object in view (dock omitted):
SPAWN_KNOWN_OBJECT=1 SPAWN_NO_DOCK=1 SPAWN_YAW=3.14159 SPAWN_HEADLESS=1 isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py
# detector + gate (each needs the project ws for depthai_ros_msgs):
source ~/isaac_tb4/ros2_ws/install/setup.bash ; isaac-ros
python3 ~/isaac_tb4/scripts/oakd_spatial_detection.py     # publishes /oakd/nn/spatial_detections
python3 ~/isaac_tb4/verify/check_spatial_detection.py     # PASS = labeled detection + finite 3D pose
```
Type: `depthai_ros_msgs/msg/SpatialDetectionArray` (vendored in `ros2_ws/src`, built once with
`cd ~/isaac_tb4/ros2_ws && colcon build`).

### Drive (teleop)
```bash
isaac-teleop         # keyboard teleop on /cmd_vel  (i=fwd, ,=back, j/l=turn, k=stop, q/z=speed)
```
A single key tap drives briefly then stops (cmd_vel watchdog); hold to keep moving.

### Dock / undock
```bash
isaac-dockd          # dock/undock action server — leave running in its own shell
# in another isaac-ros shell:
isaac-undock         # reverse off the dock, then turn ~180° to face away
isaac-dock           # drive back onto the dock and align
ros2 topic echo /dock_status      # is_docked / dock_visible
```

### Navigation / SLAM / RViz (sim time)
Run each in its own `isaac-ros` shell. `isaac-nav` has no map->odom on its own, so
pair it with SLAM (or localization + a saved map):
```bash
isaac-slam           # build map + provide map->odom  (use_sim_time:=true)
isaac-nav            # navigation stack               (use_sim_time:=true)  — run WITH slam
isaac-rviz           # RViz: set a "Nav2 Goal"        (use_sim_time:=true)
```

## Alias reference (`~/.bashrc`)
| alias | expands to |
|-------|------------|
| `isaac`        | `$HOME/run_isaacsim.sh`                    (launch Isaac GUI) |
| `isaac-py`     | `$HOME/run_isaacsim.sh python`             (run a script in the Isaac env) |
| `isaac-ros`    | switch shell to the sim's DDS + reset daemon (function) |
| `isaac-teleop` | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| `isaac-dockd`  | `python3 $HOME/isaac_tb4/scripts/dock_controller.py` |
| `isaac-dock`   | `ros2 action send_goal /dock irobot_create_msgs/action/Dock {}` |
| `isaac-undock` | `ros2 action send_goal /undock irobot_create_msgs/action/Undock {}` |
| `isaac-nav`    | `ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true` |
| `isaac-slam`   | `ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true` |
| `isaac-rviz`   | `ros2 launch turtlebot4_viz view_robot.launch.py use_sim_time:=true` |

## Typical session
```bash
# Terminal 1
isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py

# Terminal 2
isaac-ros
isaac-dockd

# Terminal 3
isaac-ros
isaac-undock        # leave the dock
isaac-teleop        # drive around
isaac-dock          # return to the dock
```
