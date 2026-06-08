#!/bin/bash
# Start the whole physical-robot demo INSIDE the dts gui-tools container:
# PF node + camera view + RViz (preconfigured: Fixed Frame=map, MarkerArray /pf/viz).
#
#   bash /tmp/run_robot.sh [robot] [tag_size_m]
#   e.g.  bash /tmp/run_robot.sh rabbit 0.065
#
# Drive separately from a HOST terminal:  dts duckiebot keyboard_control <robot>
set -e
ROBOT="${1:-rabbit}"
TAG="${2:-0.065}"

source /code/catkin_ws/devel/setup.bash

echo ">>> particle filter node (robot=$ROBOT, tag_size=$TAG) ..."
python3 /tmp/duckiebot_pf_ros1.py _robot:="$ROBOT" _tag_size:="$TAG" &
sleep 3

echo ">>> camera view (/$ROBOT/camera_node/image/compressed) ..."
rosrun rqt_image_view rqt_image_view /$ROBOT/camera_node/image/compressed &

echo ">>> RViz (Fixed Frame=map, MarkerArray /pf/viz) ..."
rosrun rviz rviz -d /tmp/pf_robot.rviz &

echo ">>> all started. Ctrl+C here stops everything."
wait
