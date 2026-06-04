"""Launch the full simulation stack:
  1. Gazebo Harmonic with room.sdf
  2. ros_gz_bridge  (/odom, /camera, /cmd_vel)
  3. apriltag_obs_node  (camera → observations)
  4. pf_node            (odometry + observations → pose estimate)
  5. viz_node           (MarkerArray for RViz2)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def _find_repo_root():
    """Walk up from this file to the dir holding simulation/worlds/room.sdf.
    Works whether the launch runs from source or from install/ (copy or symlink),
    since install/ lives inside the repo root."""
    d = os.path.dirname(os.path.realpath(__file__))
    while d != '/':
        if os.path.exists(os.path.join(d, 'simulation', 'worlds', 'room.sdf')):
            return d
        d = os.path.dirname(d)
    raise RuntimeError('repo root (simulation/worlds/room.sdf) not found')


_REPO = _find_repo_root()
_WORLD = os.path.join(_REPO, 'simulation', 'worlds', 'room.sdf')
_MODELS = os.path.join(_REPO, 'simulation', 'models')


def generate_launch_description():
    return LaunchDescription([

        # Make pf_core importable inside the ROS nodes
        SetEnvironmentVariable('PYTHONPATH',
            _REPO + ':' + os.environ.get('PYTHONPATH', '')),

        # Gazebo Harmonic — '-r' starts the sim RUNNING (not paused)
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', _MODELS),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', _WORLD],
            output='screen'),

        # Static TF so RViz knows the 'odom' frame (markers are published in it)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
             output='screen'),

        # ROS ↔ Gazebo bridge
        # format:  topic@ros_type[gz_type   (gz→ros)
        #          topic@ros_type]gz_type   (ros→gz)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            output='screen'),

        # Filter nodes
        Node(package='pf_ros2', executable='apriltag_obs_node', name='apriltag_obs',
             output='screen'),
        Node(package='pf_ros2', executable='pf_node', name='pf',
             output='screen'),
        Node(package='pf_ros2', executable='viz_node', name='viz',
             output='screen'),
    ])
