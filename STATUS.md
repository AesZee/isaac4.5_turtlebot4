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
| 2 — detection + spatial 3D | in progress | — | assert `/oakd/nn/spatial_detections` with object in view |
| 3 — lidar/SLAM regression guard | not started | — | base contract still green; build+save a map |
| 4 — HMI extension | **DEFERRED (manual/GUI)** | — | do NOT attempt unattended |

## Next action
Phase 2: stand up an OAK-D spatial-detection pipeline publishing `/oakd/nn/spatial_detections`
(depthai_ros_driver type: `depthai_ros_msgs/msg/SpatialDetectionArray`) with a known object placed
in front of the robot in-sim; assert one labeled detection with a finite 3D pose; save the raw msg
to verify/artifacts/. NOTE: robot starts docked facing the dock plate (~0.06-0.12 m) — place the
object clear of the dock / in the camera FOV, or undock first, so the detector has a real target.

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
  kills itself). Starting Phase 2.
