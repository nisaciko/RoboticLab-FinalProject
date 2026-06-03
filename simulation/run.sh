#!/usr/bin/env bash
# Launch the Gazebo Harmonic simulation with the model path set so that
# model://aruco_marker (and any other models in simulation/models) resolve.
#
# Usage (from anywhere):
#   ./simulation/run.sh                 # open the room world
#   ./simulation/run.sh -s -r           # headless server, etc. (extra args pass through)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$HERE/models:$GZ_SIM_RESOURCE_PATH"
exec gz sim "$HERE/worlds/room.sdf" "$@"
