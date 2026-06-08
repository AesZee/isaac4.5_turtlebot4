# Bashrc Settings

Recommended `~/.bashrc` additions for this workspace. These aliases keep the Isaac
Sim TurtleBot4 workflow short and, more importantly, keep sim ROS traffic isolated
from the real TurtleBot4 configuration.

## Core Aliases

Add this block to `~/.bashrc`:

```bash
# isaac_tb4 workspace helpers
alias isaac="$HOME/run_isaacsim.sh"
alias isaac-py="$HOME/run_isaacsim.sh python"

isaac-ros() {
  unset ROS_DISCOVERY_SERVER
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 daemon stop >/dev/null 2>&1 || true
}

alias isaac-teleop="ros2 run teleop_twist_keyboard teleop_twist_keyboard"
alias isaac-dockd="python3 $HOME/isaac_tb4/scripts/dock_controller.py"
alias isaac-dock="ros2 action send_goal /dock irobot_create_msgs/action/Dock '{}'"
alias isaac-undock="ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'"
alias isaac-nav="ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true"
alias isaac-slam="ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true"
alias isaac-rviz="ros2 launch turtlebot4_viz view_robot.launch.py use_sim_time:=true"
alias isaac-hmi="SPAWN_HMI=1 $HOME/run_isaacsim.sh python $HOME/isaac_tb4/scripts/spawn_turtlebot4.py"
```

Reload the shell after editing:

```bash
source ~/.bashrc
```

## Optional Convenience Aliases

These are useful for the current verification scripts, but they are not required:

```bash
alias isaac-oakd-frames="python3 $HOME/isaac_tb4/verify/save_oakd_frames.py"
alias isaac-detect="source $HOME/isaac_tb4/ros2_ws/install/setup.bash; python3 $HOME/isaac_tb4/scripts/oakd_spatial_detection.py"
alias isaac-teleop-auto="python3 $HOME/isaac_tb4/verify/scripted_teleop.py"
```

## Workspace Setup

The project-local ROS 2 workspace contains the vendored `depthai_ros_msgs` package.
Source it only in terminals that need those message types:

```bash
source ~/isaac_tb4/ros2_ws/install/setup.bash
```

Spatial detection needs this before running `oakd_spatial_detection.py` or
`verify/check_spatial_detection.py`.

## Usage Pattern

Use two or more terminals:

```bash
# Terminal 1: launch the sim
isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py

# Terminal 2: talk to the sim
isaac-ros
ros2 topic list
isaac-teleop
```

Run `isaac-ros` first in every terminal that talks to the simulator. It switches the
shell to the simulator DDS settings and stops the ROS daemon so stale discovery data
does not leak between domains.

## Important DDS Rules

This machine may also be configured for a real TurtleBot4 using a different
`ROS_DOMAIN_ID` and a discovery server. Do not replace that real-robot setup in
`.bashrc`. The simulator should use:

```bash
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DISCOVERY_SERVER unset
FASTRTPS_DEFAULT_PROFILES_FILE unset
```

If the sim is running but `ros2 topic list` only shows `/rosout` and
`/parameter_events`, run:

```bash
isaac-ros
ros2 topic list
```

## Alias Reference

| alias | purpose |
|-------|---------|
| `isaac` | Launch Isaac Sim GUI through `~/run_isaacsim.sh` |
| `isaac-py` | Run a Python script inside the Isaac Sim environment |
| `isaac-ros` | Switch the current shell to sim DDS settings |
| `isaac-teleop` | Keyboard teleop on `/cmd_vel` |
| `isaac-dockd` | Start the dock/undock action server |
| `isaac-dock` | Send a `/dock` action goal |
| `isaac-undock` | Send an `/undock` action goal |
| `isaac-nav` | Launch Nav2 with `use_sim_time:=true` |
| `isaac-slam` | Launch SLAM Toolbox with `use_sim_time:=true` |
| `isaac-rviz` | Launch TurtleBot4 RViz with `use_sim_time:=true` |
| `isaac-hmi` | Launch the sim with the HMI extension enabled |
