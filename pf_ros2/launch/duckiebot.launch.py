"""Launch the SAME particle filter against the real Duckiebot (via ros1_bridge).

Run this AFTER the ros1_bridge container is up (see docs/duckiebot_migration.md),
so the Duckiebot's ROS1 topics already appear in ROS 2. The filter core and the
nodes are byte-for-byte identical to sim.launch.py — only the INPUTS differ:
compressed camera, the robot's real camera calibration, real odometry, and the
real-room tag map.

TODO(lab) — fill these in from the actual robot:
  * ROBOT                : the robot's hostname
  * camera fx/fy/cx/cy   : /data/config/calibrations/camera_intrinsic/<robot>.yaml
  * tag_size             : the physical side length of the printed tag36h11 (m)
  * ODOM topic           : `ros2 topic list` after the bridge is up
  * pf_core/tag_map.py   : measure the real room + tag positions
"""
from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT = "rabbit"                                 # the robot's hostname
CAM   = f"/{ROBOT}/camera_node/image/compressed"
ODOM  = f"/{ROBOT}/odometry"                      # from odom_republisher.py (nav_msgs/Odometry)


def generate_launch_description():
    return LaunchDescription([

        # static TF so RViz knows the 'odom' frame (markers are published in it)
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
             output="screen"),

        # camera -> (range, bearing): compressed image + the robot's real intrinsics
        Node(package="pf_ros2", executable="apriltag_obs_node", name="apriltag_obs",
             output="screen",
             parameters=[{
                 "image_topic":    CAM,
                 "use_compressed": True,
                 "tag_size":       0.065,    # TODO(lab): real printed tag36h11 side (m)
                 "fx": 320.0, "fy": 320.0,   # TODO(lab): from camera calibration
                 "cx": 320.0, "cy": 240.0,   # TODO(lab): from camera calibration
             }]),

        # filter — remap /odom to the robot's odometry topic
        Node(package="pf_ros2", executable="pf_node", name="pf",
             output="screen",
             remappings=[("/odom", ODOM)]),

        # visualization (no Gazebo ground-truth /model/robot/pose on the real robot)
        Node(package="pf_ros2", executable="viz_node", name="viz",
             output="screen",
             remappings=[("/odom", ODOM)]),
    ])
