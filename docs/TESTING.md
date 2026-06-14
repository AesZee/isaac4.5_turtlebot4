# TESTING — try Phases 1–3 yourself (windowed GUI)

Quick guide to run and verify the OAK-D work in the **windowed** sim (not headless).
The spawn script is windowed by default — you only get headless if you set
`SPAWN_HEADLESS=1`, so just leave it unset.

## Rules
- **One Isaac instance at a time.** Don't launch a second `isaac-py …` (or `isaac`)
  while a sim is up. `Ctrl-C` in the sim terminal stops it.
- Run **`isaac-ros` first** in every terminal that talks to the sim (not the sim
  terminal itself).
- Phase 2 needs the project ws for the message type:
  `source ~/isaac_tb4/ros2_ws/install/setup.bash`.

## Spawn options (env vars, all optional)
| var | default | effect |
|-----|---------|--------|
| `SPAWN_HEADLESS` | unset | `1` = no GUI window (testing only) |
| `SPAWN_NO_DOCK` | unset | `1` = omit the visual dock (clear forward view) |
| `SPAWN_YAW` | `0.0` | spawn heading, radians (`3.14159` = face the open room) |
| `SPAWN_KNOWN_OBJECT` | unset | `1` = spawn the red detection cube |
| `SPAWN_HMI` | unset | `1` = load the HMI panel extension (Phase 4) |

With none set: windowed, docked, yaw 0, no cube, OAK-D on (the verified default).

---

## Phase 1 — OAK-D RGB + depth + points
```bash
# Terminal 1 — windowed sim
isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py

# Terminal 2
isaac-ros
ros2 topic list | grep oakd                       # 4 /oakd/* topics
./verify/check_topics.sh /oakd/rgb/image_raw /oakd/rgb/camera_info /oakd/stereo/image_raw /oakd/points
ros2 topic echo --once /oakd/rgb/camera_info      # frame + non-empty intrinsics
python3 ~/isaac_tb4/verify/save_oakd_frames.py    # -> verify/artifacts/oakd_rgb.png + depth
ros2 run rqt_image_view rqt_image_view /oakd/rgb/image_raw   # watch live
```
**PASS** = gate prints `PASS`, camera_info frame `oakd_rgb_camera_optical_frame`
with non-zero `k`, and the saved RGB/depth look right.

## Phase 2 — spatial detection
```bash
# Terminal 1 — windowed, faces open room, dock off, cube spawned
SPAWN_KNOWN_OBJECT=1 SPAWN_NO_DOCK=1 SPAWN_YAW=3.14159 isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py

# Terminal 2 — detector
source ~/isaac_tb4/ros2_ws/install/setup.bash
isaac-ros
python3 ~/isaac_tb4/scripts/oakd_spatial_detection.py

# Terminal 3 — gate
source ~/isaac_tb4/ros2_ws/install/setup.bash
isaac-ros
python3 ~/isaac_tb4/verify/check_spatial_detection.py   # PASS = labeled cube + finite 3D pose
ros2 topic echo /oakd/nn/spatial_detections             # watch live
```
**PASS** = gate prints a labeled `red_cube` with a finite 3D position
(roughly `z ≈ 0.64 m`).

## Phase 3 — lidar / SLAM mapping
```bash
# Terminal 1 — windowed, default config (docked, no cube)
isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py

# Terminal 2
isaac-ros ; isaac-slam            # slam_toolbox (sim time)
# Terminal 3
isaac-ros ; isaac-rviz            # watch the map build (your visual sign-off)
# Terminal 4
isaac-ros
python3 ~/isaac_tb4/verify/scripted_teleop.py   # auto-drives a sweep (or use isaac-teleop)
ros2 run nav2_map_server map_saver_cli -f maps/my_test_map --ros-args -p use_sim_time:=true
./verify/check_map.sh maps/my_test_map          # PASS = base contract + /scan intact + valid map
```
**PASS** = base `/scan /odom /tf /clock /cmd_vel` contract still alive, `/scan`
frame `rplidar_link` filtered at `SCAN_MIN_RANGE`, and a non-empty map saved.

## Phase 4 — HMI panel (GUI only)
This one is inherently GUI — there's no headless gate to run; verify it by eye.
```bash
# Terminal 1 — windowed sim WITH the HMI panel loaded
isaac-hmi                         # = SPAWN_HMI=1 isaac-py scripts/spawn_turtlebot4.py
# Terminal 2 — action server + battery (needed for Dock/Undock + Battery readout)
isaac-ros ; isaac-dockd
```
**`isaac-dockd` must reach the panel on the sim's domain 0.** It now self-pins to the sim
DDS (domain 0, localhost, no discovery server) on startup, so it connects even if you forget
`isaac-ros` first — but running `isaac-ros` first is still the convention. The panel proves
the link visually: the **light ring turns green** (docked) within a second or two of
`isaac-dockd` coming up. If it stays **dim blue ("connecting…")** and Battery/Dock read `—`,
the panel and dockd aren't on the same domain (or dockd isn't running).

If the panel itself isn't visible, dock it from *Window ▸ TurtleBot4 HMI* (or enable manually
via *Window ▸ Extensions* ▸ add `~/isaac_tb4/extensions` to the search paths ▸ toggle it on).
If Dock/Undock stay `idle` *and* Battery shows `—` even with the ring not green, the in-panel
ROS bridge failed to start — look in the `isaac-hmi` launch terminal for
`[tb4_hmi] ROS bridge failed to start: …` and report that line.

**Check by eye:**
- Panel loads and docks; **light ring** shows green while docked (rotating "comet" = charging),
  flips to **white** after Undock.
- **Battery** climbs while docked, falls after Undock; **Dock** line reads docked/undocked.
- **Undock** reverses + turns the robot (same as `isaac-undock`); **Dock** returns it; the
  **Action** line tracks sending → running → succeeded.
- **Teleop** taps nudge the robot; **E-STOP** latches it stopped and the ring goes red pulse.
- Cross-check the ring topic: `isaac-ros ; ros2 topic echo /cmd_lightring --once`.
- **Regression:** `/scan /odom /tf /clock /cmd_vel` still publish (`./verify/check_topics.sh`).

---

See `README.md` / `COMMANDS.md` for the full reference, and `STATUS.md` for the
phase log.
