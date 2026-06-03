# Simulation (Gazebo Harmonic)  [Owner: Nisa]

Closed rectangular room with **8 AprilTags (same ID), 2 per wall, placed
asymmetrically**, plus a mobile robot with a forward-facing camera, driven by
teleop. The filter consumes the robot's odometry.

```
worlds/   room.sdf            closed room + 8 tag instances (asymmetric)
models/   robot + camera, AprilTag model(s)
launch/   gz sim + ros_gz_bridge (camera, odom, cmd_vel <-> ROS 2)
```

## Build checklist (Gün 1–2)
- [ ] Closed rectangular room (dimensions chosen by us) in `worlds/room.sdf`
- [ ] 8 AprilTags, all same ID, same physical size, asymmetric (2 per wall)
- [ ] Tag world positions match `pf_core/tag_map.py`
- [ ] Mobile robot + forward-facing camera
- [ ] Teleop works; `cmd_vel` drives the robot
- [ ] Odometry topic published and verified
- [ ] `ros_gz_bridge` exposes camera + odometry to ROS 2 Humble

> Bridged via `ros_gz` (`ros-humble-ros-gzharmonic`). Topic names set here feed
> `pf_ros2/launch/sim.launch.py`.
