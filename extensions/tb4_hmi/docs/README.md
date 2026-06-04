# tb4_hmi — TurtleBot4 HMI panel

An Isaac Sim extension that mimics the Create 3 HMI for the simulated TurtleBot4:
a 6-segment light ring, battery readout, dock/undock buttons, teleop nudges, and an
E-STOP. It hosts its own `rclpy` node inside the Isaac process and talks to the sim
over the same DDS the spawn uses (`ROS_DOMAIN_ID=0`, `rmw_fastrtps_cpp`), so it needs
no extra environment — just launch Isaac the normal way (`isaac` / `isaac-py`).

## Enable it

**Auto (recommended):** launch the sim with the HMI flag —
`SPAWN_HMI=1 isaac-py scripts/spawn_turtlebot4.py` (alias `isaac-hmi`). The spawn adds
this folder to the extension search path and enables `tb4_hmi`. Default is OFF, so the
verified spawn path is untouched.

**Manual:** *Window ▸ Extensions* ▸ gear/⚙ ▸ add `…/isaac_tb4/extensions` to the search
paths ▸ search "TurtleBot4 HMI" ▸ toggle ON. Dock the panel anywhere.

## Use it

- **Dock / Undock** send goals to the `/dock` and `/undock` actions — start the server
  first: `isaac-ros` then `isaac-dockd` (`scripts/dock_controller.py`). The Action line
  shows sending → running → succeeded/aborted. Without the server it says "no server".
- **Battery** reads `/battery_state` (published by `dock_controller.py`: charges while
  docked, drains while undocked).
- **Light ring** reflects state (see mapping below) and is mirrored on `/cmd_lightring`
  (`irobot_create_msgs/msg/LightringLeds`) for the real on-robot ring.
- **Teleop** buttons publish `/cmd_vel`; a tap is a brief nudge (the 0.5 s cmd_vel
  watchdog stops the robot after each tap — expected). **E-STOP** latches `/cmd_vel` to
  zero until released.

## Light-ring mapping (Create 3 parity)

| state | color | animation |
|-------|-------|-----------|
| E-STOP engaged | red | fast pulse |
| connecting / no data | dim blue | solid |
| low battery (<15%), not charging | amber | pulse |
| docked + charging | green | rotating comet |
| docked + full | green | solid |
| undocked / idle | white | solid |
