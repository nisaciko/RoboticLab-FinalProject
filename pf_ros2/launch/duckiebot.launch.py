"""Launch the SAME filter against the real Duckiebot (via ros1_bridge).  [Nisa]

Only the input topic names differ from sim.launch.py — the filter core and nodes
are identical. Assumes the ros1_bridge container is already running (see
docs/duckiebot_migration.md) so the Duckiebot's ROS1 topics appear in ROS 2.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot = "duckiebot"  # TODO(Nisa): set robot hostname
    return LaunchDescription([
        Node(package="pf_ros2", executable="apriltag_obs_node", name="apriltag_obs",
             parameters=[{"image_topic": f"/{robot}/camera_node/image/compressed"}]),
        Node(package="pf_ros2", executable="pf_node", name="pf",
             # odom comes from the small ROS1 republisher bridged as nav_msgs/Odometry
             parameters=[{"odom_topic": f"/{robot}/odometry"}]),
        Node(package="pf_ros2", executable="viz_node", name="viz"),
    ])
