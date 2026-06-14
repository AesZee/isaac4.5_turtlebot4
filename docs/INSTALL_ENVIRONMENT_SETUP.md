# Isaac Sim 4.5 + TurtleBot4 Environment Setup

This guide records the setup used for this `isaac_tb4` workspace. It assumes you
followed pages 1-6 of `2. Isaac Sim 공정환경 구축.pdf`, installed Isaac Sim 4.5
from NVIDIA's official Isaac Sim distribution, then continued with pages 12-18
of the same PDF for swap memory setup.

The target environment is Ubuntu 22.04, ROS 2 Humble, Isaac Sim 4.5, and
TurtleBot4 simulation assets in `~/isaac_tb4`.

## 1. Base Ubuntu Packages

Install the basic build tools from the PDF:

```bash
sudo apt update
sudo apt install -y git git-lfs build-essential gcc-11 g++-11
git lfs install
```

Set GCC/G++ 11 as the default compiler for Isaac Sim compatibility:

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110
gcc --version
g++ --version
```

Expected version on Ubuntu 22.04 is GCC/G++ 11.4.x.

## 2. ROS 2 Humble

Install ROS 2 Humble before running the simulator bridge.

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash
```

If ROS 2 was not previously installed, also make sure the standard ROS 2 apt
repository setup has been completed first.

## 3. Project ROS Dependencies

This repository uses ROS packages for TurtleBot4, Create3 docking actions,
teleop, SLAM, map saving, image conversion, and vision message types.

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-pil \
  python3-yaml \
  ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-viz \
  ros-humble-irobot-create-msgs \
  ros-humble-teleop-twist-keyboard \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs
```

These packages are needed by:

- `scripts/dock_controller.py`: `/dock`, `/undock`, `/dock_status`,
  `/battery_state`
- `scripts/oakd_spatial_detection.py`: OpenCV, cv_bridge, vision messages
- `verify/save_oakd_frames.py`: cv_bridge and PIL image export
- `COMMANDS.md` aliases: teleop, SLAM, Nav2 map saver, RViz/navigation launchers

## 4. Isaac Sim 4.5 Install

Install Isaac Sim 4.5 through NVIDIA's official install method. On this machine,
Isaac Sim 4.5 is installed into a conda environment named `isaacsim`.

The local install script used here was:

```bash
source /home/aes/miniconda3/etc/profile.d/conda.sh
conda create -n isaacsim python=3.10 -y
conda activate isaacsim
python -m pip install --upgrade pip
python -m pip install 'isaacsim[all,extscache]==4.5.0.0' --extra-index-url https://pypi.nvidia.com
```

Accept the NVIDIA Omniverse EULA when launching Isaac Sim:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
```

First launch can take several minutes because shaders and extension caches are
compiled.

## 5. Isaac Sim Launcher Wrapper

Create `~/run_isaacsim.sh` so Isaac Sim always starts with the correct ROS 2
bridge environment and isolated DDS settings:

```bash
#!/usr/bin/env bash
set -e

unset PYTHONPATH

source /home/aes/miniconda3/etc/profile.d/conda.sh
conda activate isaacsim

export OMNI_KIT_ACCEPT_EULA=YES

source /opt/ros/humble/setup.bash
[ -f /home/aes/turtlebot4_ws/install/setup.bash ] && source /home/aes/turtlebot4_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=${ISAAC_ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=1

if [ "$1" == "python" ]; then
    shift
    exec python "$@"
else
    exec isaacsim "$@"
fi
```

Make it executable:

```bash
chmod +x ~/run_isaacsim.sh
```

This repo uses:

```bash
~/run_isaacsim.sh
~/run_isaacsim.sh python ~/isaac_tb4/scripts/spawn_turtlebot4.py
```

## 6. Swap Memory Setup

Pages 12-18 of the PDF configure 8 GB swap. This is recommended on machines with
limited RAM because Isaac Sim can be memory-heavy.

Check current memory and swap:

```bash
free -h
swapon --show
```

Create and enable an 8 GB swap file:

```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
free -h
```

If `fallocate` fails because of disk constraints, use:

```bash
sudo dd if=/dev/zero of=/swapfile bs=1M count=8192
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

Persist it after reboot:

```bash
grep swapfile /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

Set low swappiness so RAM is preferred:

```bash
grep -q '^vm.swappiness=' /etc/sysctl.conf \
  && sudo sed -i 's/^vm.swappiness=.*/vm.swappiness=10/' /etc/sysctl.conf \
  || echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## 7. Clone / Restore This Workspace

Place the project at:

```bash
~/isaac_tb4
```

If cloning from git:

```bash
cd ~
git clone <repo-url> isaac_tb4
cd ~/isaac_tb4
git lfs pull
```

This repository expects the following committed assets to exist:

- `usd/turtlebot4.usd`
- `usd/turtlebot4.urdf`
- `scenes/A-1_ground.usd`
- `maps/A-1_map.yaml`
- `maps/A-1_map.pgm`
- `scripts/spawn_turtlebot4.py`

## 8. Build Local ROS Message Workspace

The project vendors a small subset of `depthai_ros_msgs` under
`ros2_ws/src/depthai_ros_msgs`. It is needed for:

- `/oakd/nn/spatial_detections`
- `scripts/oakd_spatial_detection.py`
- `verify/check_spatial_detection.py`

Build it once:

```bash
cd ~/isaac_tb4/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

Source it only in terminals that need the OAK-D spatial detection message type:

```bash
source ~/isaac_tb4/ros2_ws/install/setup.bash
```

## 9. Bash Aliases

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

Reload:

```bash
source ~/.bashrc
```

Important: run `isaac-ros` first in every terminal that talks to the simulator.
This switches that shell to the sim's ROS domain 0 and clears stale ROS daemon
discovery state.

## 10. Run the Simulation

Use two terminals.

Terminal 1:

```bash
isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py
```

Terminal 2:

```bash
isaac-ros
ros2 topic list
isaac-teleop
```

Expected core topics include:

- `/clock`
- `/cmd_vel`
- `/odom`
- `/tf`
- `/scan`
- `/oakd/rgb/image_raw`
- `/oakd/rgb/camera_info`
- `/oakd/stereo/image_raw`
- `/oakd/points`

Do not run `isaac` separately while `spawn_turtlebot4.py` is running. The spawn
script already opens Isaac Sim and loads the scene.

## 11. Optional Feature Checks

OAK-D frame capture:

```bash
isaac-ros
python3 ~/isaac_tb4/verify/save_oakd_frames.py
```

Spatial detection:

```bash
# Terminal 1
SPAWN_KNOWN_OBJECT=1 SPAWN_NO_DOCK=1 SPAWN_YAW=3.14159 isaac-py ~/isaac_tb4/scripts/spawn_turtlebot4.py

# Terminal 2
source ~/isaac_tb4/ros2_ws/install/setup.bash
isaac-ros
python3 ~/isaac_tb4/scripts/oakd_spatial_detection.py

# Terminal 3
source ~/isaac_tb4/ros2_ws/install/setup.bash
isaac-ros
python3 ~/isaac_tb4/verify/check_spatial_detection.py
```

Dock / undock:

```bash
isaac-ros
isaac-dockd
```

In another terminal:

```bash
isaac-ros
isaac-undock
isaac-dock
ros2 topic echo /dock_status
```

SLAM / mapping:

```bash
isaac-ros
isaac-slam
```

In another terminal:

```bash
isaac-ros
python3 ~/isaac_tb4/verify/scripted_teleop.py
ros2 run nav2_map_server map_saver_cli -f maps/my_test_map --ros-args -p use_sim_time:=true
./verify/check_map.sh maps/my_test_map
```

HMI panel:

```bash
isaac-hmi
```

In another terminal:

```bash
isaac-ros
isaac-dockd
```

If the panel does not appear, enable it from Isaac Sim:

```text
Window -> Extensions -> add ~/isaac_tb4/extensions to search paths -> enable TurtleBot4 HMI
```

## 12. Common Problems

Only `/rosout` and `/parameter_events` appear:

```bash
isaac-ros
ros2 topic list
```

This resets the ROS daemon and points the shell at sim DDS domain 0.

Isaac Sim starts but ROS bridge topics are missing:

- Confirm `~/run_isaacsim.sh` sources `/opt/ros/humble/setup.bash` before running
  `isaacsim`.
- Confirm `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- Confirm the sim is not using the real robot discovery server.

Spatial detection import fails:

```bash
cd ~/isaac_tb4/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Teleop, docking, Nav2, or SLAM command is missing:

```bash
sudo apt install -y \
  ros-humble-teleop-twist-keyboard \
  ros-humble-irobot-create-msgs \
  ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-viz \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server
```

Isaac Sim is very slow or crashes during first load:

- Make sure swap is enabled with `free -h`.
- Close other GPU-heavy applications.
- First launch is slower because shaders and extension caches are generated.

