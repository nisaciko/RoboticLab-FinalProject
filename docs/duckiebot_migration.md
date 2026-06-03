# Sim → Real Duckiebot Migration (Yol B)

The filter core (`pf_core/`) and the ROS 2 nodes are **identical** in sim and on
the robot. Migration only connects the robot's ROS1 topics into ROS 2 and points
the nodes at them.

## Architecture

```
Duckiebot (ROS1 Noetic / daffy)  ──ros1_bridge──►  ROS 2 Humble (laptop)  ──►  pf_ros2 + pf_core
```

Bridge guide (we used Hamza Hoca's Repo fro bridge):
https://github.com/awwad-hamza/Duckiebot-Ros2-Humble-Bridge-Setup

## Steps

1. **Start the bridge** (do NOT run `roscore` manually — the ROS1 master lives on
   the Duckiebot):
   ```bash
   docker pull ros:foxy-ros1-bridge
   docker run --rm -it --network host --ipc host \
     -e ROS_MASTER_URI=http://<duckiebot>.local:11311 \
     ros:foxy-ros1-bridge \
     ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   ```

2. **Camera** — `/<robot>/camera_node/image/compressed` is `sensor_msgs/CompressedImage`
   (a standard type) → bridges automatically. ✅

3. **Odometry** — Duckiebot wheel encoders are `duckietown_msgs/WheelEncoderStamped`
   (custom) and will NOT auto-bridge. Easiest fix: a small ROS1 node in your
   Duckietown `template-ros` project that subscribes to the encoders and
   publishes `nav_msgs/Odometry` (standard) → bridges cleanly. (~30 lines.)

4. **Tag map** — measure the real room and update `pf_core/tag_map.py` (and the
   AprilTag print size) to the real coordinates.

5. **AprilTag detection** — reuse dt-core's AprilTag detector on the robot, or
   keep detection in `apriltag_obs_node` on the ROS 2 side from the bridged image.

6. **Run** the same filter pointed at the bridged topics:
   ```bash
   ros2 launch pf_ros2 duckiebot.launch.py
   ```

## Gotcha checklist
- [ ] Bridge sees the topics? (`ros2 topic list` shows `/<robot>/...`)
- [ ] Odometry republisher running (custom → `nav_msgs/Odometry`)?
- [ ] Tag map updated to real room measurements?
- [ ] AprilTag physical size matches what the detector expects?
