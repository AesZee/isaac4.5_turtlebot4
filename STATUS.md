# STATUS — unattended Phases 1–3

Resume protocol: read this top-to-bottom, then continue from **Next action**. Update the phase
table + Next action + Log after every completed or failed step. Keep entries short (context hygiene).

## Branch
- work branch: feat/oakd-phases-1-3
- last green commit: c99ad24 (baseline gate green)

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
| baseline (verified contract) | GREEN | PASS | gate fixed (DDS env + pipefail/grep-q false-neg); commit c99ad24 |
| 1 — OAK-D RGB + depth + points | GREEN | PASS | 4 /oakd topics @rate; camera_info frame+K ok; frames saved; commit 380ead2 |
| 2 — detection + spatial 3D | GREEN | PASS | red_cube @ finite 3D (0.006,0.019,0.641)m; vendored depthai_ros_msgs; commit b9ccad1 |
| 3 — lidar/SLAM regression guard | in progress | — | base contract still green; build+save a map |
| 4 — HMI extension | **DEFERRED (manual/GUI)** | — | do NOT attempt unattended |

## Next action
Phase 3: regression-guard lidar/SLAM. Restart the sim in DEFAULT config (docked, yaw 0 — the
verified-working spawn) and re-run `./verify/check_topics.sh` (base contract must stay green:
/scan frame rplidar_link, filtered at SCAN_MIN_RANGE). Then build a map via scripted teleop
(publish /cmd_vel to drive) with `isaac-slam`, save it to maps/ with map_saver, and confirm the
saved .pgm/.yaml exist + are non-empty. Do NOT do RViz/visual nav sign-off (human's).

## Phase-2 reproduction (for the human)
Detection scenario needs a clear camera view; the committed default spawn (0,0)+yaw0 faces a near
wall on +X. Reproduce with: terminal 1 `SPAWN_NO_DOCK=1 SPAWN_YAW=3.14159 SPAWN_HEADLESS=1 isaac-py
scripts/spawn_turtlebot4.py`; terminal 2 `source ros2_ws/install/setup.bash; isaac-ros; python3
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
  Base contract still green. Commit b9ccad1. Detector left running (bg). Starting Phase 3.
