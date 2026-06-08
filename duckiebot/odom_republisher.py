#!/usr/bin/env python
"""ROS1 republisher: Duckiebot custom msgs -> standard nav_msgs/Odometry.

The Duckiebot's pose/velocity are duckietown_msgs (Pose2DStamped, Twist2DStamped),
which ros1_bridge does NOT bridge. This node subscribes to them and republishes a
standard nav_msgs/Odometry on /<robot>/odometry, which DOES bridge to ROS 2, where
pf_node consumes it (duckiebot.launch.py remaps /odom -> /<robot>/odometry).

Run INSIDE the dts gui-tools container (it has the Duckiebot's ROS1 + duckietown_msgs):

    # from the host: copy this file into the running gui-tools container
    docker cp duckiebot/odom_republisher.py <gui_tools_container>:/tmp/
    # then, inside the container:
    source /opt/ros/noetic/setup.bash
    python /tmp/odom_republisher.py _robot:=rabbit

Do NOT run roscore — the ROS1 master is on the Duckiebot.
"""
import rospy
from nav_msgs.msg import Odometry
from duckietown_msgs.msg import Pose2DStamped, Twist2DStamped
from tf.transformations import quaternion_from_euler


class OdomRepublisher:
    def __init__(self):
        self.robot = rospy.get_param("~robot", "rabbit")
        self._pose = None      # last Pose2DStamped (x, y, theta)
        self._twist = None     # last Twist2DStamped (v, omega)

        self.pub = rospy.Publisher("/%s/odometry" % self.robot, Odometry, queue_size=10)
        rospy.Subscriber("/%s/velocity_to_pose_node/pose" % self.robot,
                         Pose2DStamped, self._pose_cb, queue_size=10)
        rospy.Subscriber("/%s/kinematics_node/velocity" % self.robot,
                         Twist2DStamped, self._twist_cb, queue_size=10)
        rospy.loginfo("odom_republisher: /%s/{velocity_to_pose,kinematics} -> "
                      "/%s/odometry (nav_msgs/Odometry)", self.robot, self.robot)

    def _twist_cb(self, msg):
        self._twist = msg

    def _pose_cb(self, msg):
        self._pose = msg
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, msg.theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        if self._twist is not None:
            odom.twist.twist.linear.x = self._twist.v
            odom.twist.twist.angular.z = self._twist.omega
        self.pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("pf_odom_republisher")
    OdomRepublisher()
    rospy.spin()
