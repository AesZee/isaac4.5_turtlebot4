# STATUS — Phases 1–4 complete (merged to master)

Resume protocol: read this top-to-bottom, then continue from **Next action**. Update the phase
table + Next action + Log after every completed or failed step. Keep entries short (context hygiene).

## Branch
- master (Phases 1–4 all merged; last commit 54e7368, pushed to origin/master)
- feature branches feat/phase4-hmi and feat/oakd-phases-1-3 pruned (local + origin)
- last green commit: 54e7368 (Phase 4 HMI verified working)

## Environment notes (for resume)
- Sim: launched headless+render via `SPAWN_HEADLESS=1 ~/run_isaacsim.sh python scripts/spawn_turtlebot4.py`,
  backgrounded, log at /tmp/sim.log. Single instance. Python print() is block-buffered to the log,
  so detect readiness by ROS topics, not log grep.
- ROS from a non-interactive Bash shell INHERITS the real-robot DDS (domain 1 + discovery server +
  super-client) via ~/.bashrc -> /etc/turtlebot4_discovery/setup.bash. The gate now overrides all of
  it. To probe ad-hoc, use `bash -ic 'isaac-ros >/dev/null; ros2 ...'`.
- guard.sh PreToolUse hook DENIES any Bash command whose text contains the discovery-server token,
  ROS_DOMAIN_ID!=0, rm -rf, force-push, hard reset. Keep that token out of commit messages / commands;
  run the gate via `./verify/check_topics.sh` (it sets DDS internally, so its text stays clean).

## Phases
| phase | state | last gate | notes |
|-------|-------|-----------|-------|
| baseline (verified contract) | GREEN | PASS 06-05 | gate fixed (DDS env + pipefail/grep-q false-neg); commit c99ad24 |
| 1 — OAK-D RGB + depth + points | GREEN | PASS 06-05 | 4 /oakd topics @rate; camera_info frame+K ok; frames saved; commit 380ead2 |
| 2 — detection + spatial 3D | GREEN | PASS 06-05 | red_cube @ finite 3D (0.006,0.019,0.641)m; vendored depthai_ros_msgs; commit b9ccad1 |
| 3 — lidar/SLAM regression guard | GREEN | PASS 06-05 | base+/scan intact; map maps/A-1_phase3_map.* (60/10/30); commit 9410f41 |
| 4 — HMI extension | GREEN | GUI 06-05 | human GUI sign-off: panel/ring/battery/dock/teleop/e-stop all confirmed in Isaac Sim window |

## Next action
ALL PHASES 1–4 GREEN and MERGED TO MASTER. Phase 4 (HMI) signed off by the human in the Isaac Sim
GUI: panel docks, ring tracks dock/charge state, battery % updates, Dock/Undock + teleop nudges
drive, E-STOP halts. Fast-forwarded into master (54e7368), pushed to origin, and the merged feature
branches pruned. Nothing pending — repo is on a single clean master.

## Suggested aliases for the human (add to ~/.bashrc; not added unattended)
- isaac-oakd-frames : python3 ~/isaac_tb4/verify/save_oakd_frames.py
- isaac-detect      : source ~/isaac_tb4/ros2_ws/install/setup.bash; python3 ~/isaac_tb4/scripts/oakd_spatial_detection.py
- isaac-teleop-auto : python3 ~/isaac_tb4/verify/scripted_teleop.py

## Phase-2 reproduction (for the human)
Detection scenario needs a clear camera view; the committed default spawn (0,0)+yaw0 faces a near
wall on +X. Reproduce with: terminal 1 `SPAWN_KNOWN_OBJECT=1 SPAWN_NO_DOCK=1 SPAWN_YAW=3.14159 SPAWN_HEADLESS=1
isaac-py scripts/spawn_turtlebot4.py`; terminal 2 `source ros2_ws/install/setup.bash; isaac-ros; python3
scripts/oakd_spatial_detection.py`; terminal 3 same source+isaac-ros then
`python3 verify/check_spatial_detection.py`.

## Stop conditions (leave for human)
A gate fails twice · baseline goes red · a step needs the GUI · a change might be destructive ·
proceeding would require touching the real-robot config or a second Isaac instance.

## Log
- init: initialized.
- baseline: git init + branch feat/oakd-phases-1-3 (no prior repo). Reconciled gate DDS env with
  isaac-ros; fixed pipefail/grep-q false negative that failed healthy /scan. Sim up headless+render.
  `./verify/check_topics.sh` PASS (/scan ~84Hz, /odom ~81Hz). Commit c99ad24.
- phase1 GREEN: added OAK-D RTX camera in spawn_turtlebot4.py (rgb/depth/points/camera_info, 640x360,
  frame oakd_rgb_camera_optical_frame). Gate w/ 4 /oakd topics PASS; base contract still green.
  camera_info K=fx/fy 465.45, cx 320, cy 180. verify/save_oakd_frames.py saved rgb+depth to artifacts/.
  Commits 380ead2 (feat) + docs. Sim relaunched once (PYTHONUNBUFFERED) — single instance maintained.
  Gotcha: never `pkill -f spawn_turtlebot4.py` from a shell whose own cmdline contains that string (it
  kills itself) — kill by PID excluding $$/$PPID. Sim relaunch can segfault in viewport-init if done
  too soon after a kill; wait ~6s + ensure GPU clear (nvidia-smi) then relaunch.
- phase2 GREEN: vendored depthai_ros_msgs (authentic SpatialDetection[Array] from upstream humble)
  built into ros2_ws; scripts/oakd_spatial_detection.py (color-seg + depth->3D) publishes
  /oakd/nn/spatial_detections. Root cause of empty camera view: spawn (0,0) faces a wall at x=0.24
  (map +X edge); fixed via SPAWN_YAW=pi + cube on -X + SPAWN_NO_DOCK. Gate PASS (red_cube, finite 3D).
  Base contract still green. Commit b9ccad1. Then refactored cube behind SPAWN_KNOWN_OBJECT (default
  off) so it stays out of the verified scene/map; re-verified Phase 2 PASS (commit 6cc0316).
- phase3 GREEN: restarted sim in DEFAULT config (docked, yaw0, no cube). check_topics.sh PASS; /scan
  frame rplidar_link, 360deg, range_min 0.32 (filter intact). isaac-slam + verify/scripted_teleop.py
  (bounded /cmd_vel sweep) built a map; map_saver -> maps/A-1_phase3_map.pgm/.yaml (60% free/10% occ/
  30% unknown, valid). verify/check_map.sh PASS. PNG render in artifacts/. Commit 9410f41.
- ALL PHASES 1-3 GREEN. Phase 4 (HMI) deliberately NOT attempted unattended (deferred/GUI).
- phase4 BUILT (attended, human drives GUI): new Isaac extension extensions/tb4_hmi/ (omni.ext IExt +
  omni.ui dockable panel): 6-seg light ring, battery %, Dock/Undock buttons, teleop nudges, E-STOP.
  Hosts its own rclpy node on the Isaac process (domain 0, already set by run_isaacsim.sh) spun on a
  daemon thread; omni.kit update-event callback marshals state to omni.ui on the main thread. Verified
  feasibility first: rclpy + irobot_create_msgs (Dock/Undock + LightringLeds) import under isaac-py.
  Battery: folded a synthetic /battery_state (sensor_msgs/BatteryState; charges docked / drains
  undocked) into dock_controller.py. Panel publishes /cmd_lightring (irobot_create_msgs/LightringLeds).
  Loading: SPAWN_HMI=1 gate in spawn_turtlebot4.py (default OFF, verified path untouched) adds
  extensions/ to the ext search path + enable_extension("tb4_hmi"); alias isaac-hmi added to ~/.bashrc.
  Docs: README + COMMANDS + TESTING (Phase 4) + extensions/tb4_hmi/docs/README.md. Offline-validated
  (py_compile all; isaac-py import check; leds[6] assignment). GUI sign-off pending (human).
- phase4 GREEN: human verified isaac-hmi in the Isaac Sim GUI — panel docks, light ring tracks
  dock/charge state, battery % updates, Dock/Undock + teleop nudges drive the robot, E-STOP halts.
  README + STATUS updated to verified-working; committed on feat/phase4-hmi. ALL PHASES 1-4 GREEN.
- merge: fast-forwarded feat/phase4-hmi into master (5845a10..54e7368, linear history per repo
  convention), pushed master to origin. Pruned merged branches feat/phase4-hmi (local-only) and
  feat/oakd-phases-1-3 (local + origin). Repo is now a single clean master in sync with origin.
- re-verify 2026-06-05: re-ran all gates on a fresh headless+render sim (one instance at a time;
  torn down by PID, GPU freed). Phase 3 check_map.sh PASS (/scan rplidar_link, 360deg, range_min
  0.32; map 60/10/30). Phase 1 check_topics.sh + save_oakd_frames PASS (4 /oakd topics @rate;
  camera_info oakd_rgb_camera_optical_frame, K fx/fy 465.45 cx320 cy180; RGB+depth 640x360). Phase 2
  (cube config) check_spatial_detection PASS (red_cube score 0.41 @ (0.006,0.019,0.641)m). Phase 4
  GUI-only: human re-confirmed isaac-hmi works (not headless-verifiable). Regenerated verify
  artifacts reverted; tree clean. README given a status table + spawn-pose fix (commit 5aeccfe).
