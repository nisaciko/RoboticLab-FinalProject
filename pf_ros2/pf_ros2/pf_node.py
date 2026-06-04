"""ROS 2 adapter that drives pf_core.ParticleFilter from live Gazebo topics."""
import sys
import os

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

# Make pf_core importable when run inside a colcon workspace
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

from pf_core.particle_filter import ParticleFilter
from pf_core.tag_map import TagMap


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))


class PfNode(Node):
    def __init__(self):
        super().__init__("pf_node")

        tag_map = TagMap.default_room()
        x_range, y_range = TagMap.ROOM_BOUNDS

        self._pf = ParticleFilter(
            tag_map,
            num_particles=500,
            motion_noise=np.array([0.05, 0.05]),
            meas_noise=np.array([0.3, 0.3]),
            seed=42,
        )
        self._pf.initialize_uniform(x_range=x_range, y_range=y_range)

        self._last_time: float | None = None
        self._pending_obs: list | None = None

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Float32MultiArray, "/observations", self._obs_cb, 10)

        self._pose_pub    = self.create_publisher(PoseStamped,      "/pf/pose",      10)
        self._cloud_pub   = self.create_publisher(PoseArray,         "/pf/particles", 10)
        self._weights_pub = self.create_publisher(Float32MultiArray, "/pf/weights",   10)

        self.get_logger().info("pf_node ready — %d particles" % self._pf.num_particles)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _obs_cb(self, msg: Float32MultiArray):
        d = msg.data
        self._pending_obs = [(d[i], d[i + 1]) for i in range(0, len(d), 2)]

    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        v     = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        if self._last_time is None:
            self._last_time = now
            return

        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        obs = self._pending_obs
        self._pending_obs = None

        est = self._pf.step(u=np.array([v, omega]), dt=dt, observations=obs)

        self._publish_pose(est, msg.header.stamp)
        self._publish_cloud(msg.header.stamp)

    # ── publishers ────────────────────────────────────────────────────────────

    def _publish_pose(self, est, stamp):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(est[0])
        msg.pose.position.y = float(est[1])
        qx, qy, qz, qw = _yaw_to_quat(float(est[2]))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pose_pub.publish(msg)

    def _publish_cloud(self, stamp):
        pa = PoseArray()
        pa.header.stamp    = stamp
        pa.header.frame_id = "odom"
        for p in self._pf.particles:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            qx, qy, qz, qw = _yaw_to_quat(float(p[2]))
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pa.poses.append(pose)
        self._cloud_pub.publish(pa)

        wt = Float32MultiArray()
        wt.data = [float(w) for w in self._pf.weights]
        self._weights_pub.publish(wt)


def main(args=None):
    rclpy.init(args=args)
    node = PfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
