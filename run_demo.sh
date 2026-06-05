#!/usr/bin/env bash
# One-shot launcher for the full simulation demo.
#
#   ./run_demo.sh
#
# It (1) rebuilds pf_ros2 so you never run stale code, then opens 3 terminals:
#   1. Gazebo + bridge + filter nodes   (ros2 launch pf_ros2 sim.launch.py)
#   2. RViz2 with the preconfigured view (Fixed Frame=odom, MarkerArray /pf/viz)
#   3. hold-to-drive keyboard teleop     (simulation/teleop_key.py)
#
# Requirements: ROS 2 Humble + ros_gz (see docs/SETUP.md). Uses gnome-terminal.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"

# conda shadows ROS libraries — make sure it's off in this shell
conda deactivate 2>/dev/null || true

if [ ! -f "$ROS_SETUP" ]; then
  echo "ERROR: ROS 2 Humble not found at $ROS_SETUP — install it first (docs/SETUP.md)."
  exit 1
fi

# 1) always rebuild so everyone runs the latest pulled code
echo ">>> Building pf_ros2 (so you never run stale code)..."
source "$ROS_SETUP"
( cd "$HERE" && colcon build --packages-select pf_ros2 )

# helper: open a titled gnome-terminal running CMD, then keep the shell open.
# 'bash -c' (non-login) avoids auto-activating conda; we source ROS ourselves.
open_term() {  # open_term "title" "command"
  gnome-terminal --title="$1" -- bash -c "$2; echo; echo '[exited — press Enter to close]'; read" &
}

SRC="source '$ROS_SETUP' && source '$HERE/install/setup.bash'"

# 1. sim + bridge + filter
open_term "PF: sim + filter" "$SRC && ros2 launch pf_ros2 sim.launch.py"

# give Gazebo and the nodes time to come up before RViz / teleop connect
sleep 6

# 2. RViz2 with the saved config
open_term "PF: RViz2" "$SRC && rviz2 -d '$HERE/rviz/pf.rviz'"

# 3. hold-to-drive teleop (shebang forces /usr/bin/python3 for the gz bindings)
open_term "PF: teleop (arrows/WASD)" "'$HERE/simulation/teleop_key.py'"

echo ">>> 3 terminals opened. Drive with the teleop window focused."
