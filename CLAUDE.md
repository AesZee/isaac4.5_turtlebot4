# CLAUDE.md — isaac_tb4 working rules

Isaac Sim 4.5 + TurtleBot4, ROS 2 Humble. This file is the standing context for the repo.
The current task lives in `CLAUDE_CODE_GOAL.md`. Background: `README.md`, `COMMANDS.md`,
and `~/Downloads/isaacsim_turtlebot4_setup.md`. **Read those before acting; don't restate
their content back unless asked.**

## Golden rule
There is a VERIFIED-WORKING path. Do not regress it. After any change, re-run the spawn and
confirm `/scan /odom /tf /clock /cmd_vel` publish, `isaac-teleop` drives, and dock/undock work.
If you can't verify in this environment, say so and give me the exact command to run — never
claim a result you didn't produce.

## How to run (don't reinvent this)
- The sim IS Isaac + the scene: `isaac-py scripts/spawn_turtlebot4.py`. **Never** launch `isaac`
  (empty GUI) and the spawn script at the same time — that's two Isaac instances.
- Talk to the sim from a second shell: run `isaac-ros` first (switches to the sim's DDS and
  resets the daemon), then `ros2 ...`, `isaac-teleop`, `isaac-dockd`, etc.
- Scripts run under Isaac's Python via `isaac-py`, not system python.

## Hard invariants — never violate
- **Domain isolation.** The sim runs on `ROS_DOMAIN_ID=0` with NO discovery server. All ROS
  interaction goes through `isaac-ros`. This machine is also configured for the REAL robot
  (domain 1 + discovery server) — never touch that config.
- **Render gating.** RTX sensors (lidar, and any camera you add) only publish WHILE RENDERING.
  The sim steps with `render=True`. **Headless ≠ no-render**: if you run headless, keep RTX
  rendering on or `/scan` and `/oakd/*` go silent.
- **Headless when you launch it.** If *you* need the sim up to verify something, run it headless
  (no GUI window). I run the windowed GUI myself. GUI-only things (e.g. the HMI extension) are
  mine to verify — don't try to confirm them headless.
- **Daemon cache.** Switching domains leaves a stale `ros2` graph; `isaac-ros` runs
  `ros2 daemon stop`. If a shell shows only `/rosout` + `/parameter_events` with the sim up,
  the daemon is stale — stop it and retry.
- **Back up binaries.** Copy `usd/turtlebot4.usd` to `.bak` before editing it (repo convention).
  Prefer scripted, repeatable USD edits over manual GUI changes.
- **Real-TB4 parity.** New topics/frames/types must match what `depthai_ros_driver` +
  `irobot_create_msgs` use on hardware, so sim code ports to the real robot. Look names up;
  don't invent them.
- **One TF publisher.** The robot link tree is published by `PubTf` in `spawn_turtlebot4.py`.
  Add new frames there — don't start a competing TF publisher.

## Topic / frame contract (current)
- pub: `/clock` (~80 Hz), `/odom` (~74 Hz, odom→base_link), `/tf`, `/scan` (filtered, frame
  `rplidar_link`, 360°, returns ≥ `SCAN_MIN_RANGE` 0.32 m kept).
- sub: `/cmd_vel` (Twist → diff drive; this is the one to drive).
- internal, do NOT publish/consume: `/cmd_vel_watchdog`, `/scan_raw`.
- actions/topics: `/undock`, `/dock` (`irobot_create_msgs/action/*`), `/dock_status`
  (`irobot_create_msgs/msg/DockStatus`, ~2 Hz).

## Robot behavior facts
- Wheels are **velocity-driven** (stiffness 0, damping 1e4). Don't revert to position drive.
- Speed caps are the real ones: maxLinearSpeed 0.31 m/s, maxAngularSpeed 1.9 rad/s.
- **cmd_vel watchdog**: robot stops if no `/cmd_vel` within `CMD_VEL_TIMEOUT` (0.5 s) — so a key
  *tap* moves briefly, *holding* keeps it moving. Expected, not a bug.
- Docking is an odom-frame behavioral approximation (no IR/AMCL); it relies on near-perfect sim
  odometry returning to the odom origin.

## Conventions
- Every new entry point gets an `isaac-*` alias in the same style as the existing ones, and is
  documented in `README.md` + `COMMANDS.md`.
- Small, verifiable increments. For each change: the command to run, the expected output, and
  the regression check. Gate between phases — don't sprint through multiple phases unattended.
- Ask before large refactors or anything that rewrites the verified-working spawn flow.
