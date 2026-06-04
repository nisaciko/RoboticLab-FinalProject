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

_REPO = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..'))

_WORLD = os.path.join(_REPO, 'simulation', 'worlds', 'room.sdf')
_MODELS = os.path.join(_REPO, 'simulation', 'models')


def generate_launch_description():
    return LaunchDescription([

        # Make pf_core importable inside the ROS nodes
        SetEnvironmentVariable('PYTHONPATH',
            _REPO + ':' + os.environ.get('PYTHONPATH', '')),

        # Gazebo Harmonic
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', _MODELS),
        ExecuteProcess(
            cmd=['gz', 'sim', _WORLD],
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
