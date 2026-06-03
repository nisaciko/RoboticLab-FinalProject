"""Launch the filter against the Gazebo Harmonic simulation.  [Nisa]

Brings up (TODO): gazebo world + ros_gz_bridge, apriltag_obs_node, pf_node,
viz_node, and teleop. Input topics here are the SIM topic names.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(Nisa): include gazebo launch + ros_gz_bridge; set sim topic names.
    return LaunchDescription([
        Node(package="pf_ros2", executable="apriltag_obs_node", name="apriltag_obs"),
        Node(package="pf_ros2", executable="pf_node", name="pf",
             parameters=[{"odom_topic": "/odom", "image_topic": "/camera/image"}]),
        Node(package="pf_ros2", executable="viz_node", name="viz"),
    ])
