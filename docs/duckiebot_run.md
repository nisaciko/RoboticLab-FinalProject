# Running the particle filter on the real Duckiebot — from scratch

ROS1-native approach (no ros1_bridge). One rospy node runs in the dts gui-tools
container on the laptop, reads the robot's topics directly, runs the same
`pf_core` filter, and draws to RViz1. Camera calibration is read automatically.

> **Set the robot name ONCE** and every command below adapts (container name,
> topics, node param all use `$ROBOT`). Export it in EVERY terminal you open:
> ```bash
> export ROBOT=rabbit        # ← your robot's hostname (e.g. rabbit, chicken)
> ```

## 0. One-time (already done, won't need again)
- `docker context use default`   ← system Docker, not Docker Desktop
- Lab tags are AprilTag **tag36h11** (different IDs is fine — the filter ignores IDs)

## 1. Power on the robot, wait for it to boot. Confirm it's reachable:
```bash
ping -c1 $ROBOT.local
```

## 2. Start the gui-tools container (laptop)
```bash
dts start_gui_tools $ROBOT
```
Inside the container, verify the ROS1 connection:
```bash
source /code/catkin_ws/devel/setup.bash
rostopic list | head            # should show /$ROBOT/... topics
```

## 3. Copy the filter into the container (HOST terminal — normal laptop shell)
```bash
export ROBOT=rabbit          # set again in this terminal
cd ~/RoboticLab-FinalProject
docker cp pf_core                         dts_gui_tools_$ROBOT:/tmp/pf_core
docker cp duckiebot/duckiebot_pf_ros1.py  dts_gui_tools_$ROBOT:/tmp/
```
(Redo this whenever the container is recreated — /tmp resets.)

## 4. Run the filter (CONTAINER terminal)
```bash
source /code/catkin_ws/devel/setup.bash
python3 /tmp/duckiebot_pf_ros1.py _robot:=$ROBOT _tag_size:=0.065 &
rosrun rviz rviz
```
- `_tag_size` = printed tag36h11 side length in metres (lab tags = 0.065).
- In RViz: **Fixed Frame = map**, then **Add → By topic → /$ROBOT/pf/viz (MarkerArray)**.

## 5. Drive with the joystick.
Watch RViz: particle cloud (green) collapses onto the magenta "estimate" sphere
as the robot moves and the camera sees tags.

---

## If it doesn't converge — the 2 numbers to check
1. **Detections** — the node logs every 30 frames: `[frame N] tags this frame=K`.
   K must be > 0 (camera actually sees tags). If always 0 → point the camera at a tag.
2. **Motion** — while driving: `rostopic hz /$ROBOT/kinematics_node/velocity`
   (or `/$ROBOT/car_cmd_switch_node/cmd`). Must be publishing.

## Quick reference
| What | Where |
|---|---|
| ROS1 node | `duckiebot/duckiebot_pf_ros1.py` (robot via `_robot:=` param) |
| filter core (copied to /tmp/pf_core) | `pf_core/` |
| real tag map | `pf_core/tag_map.py` → `TagMap.lab_room()` (1.24 x 0.80 m) |
| viz topic | `/$ROBOT/pf/viz` (MarkerArray, Fixed Frame `map`) |
