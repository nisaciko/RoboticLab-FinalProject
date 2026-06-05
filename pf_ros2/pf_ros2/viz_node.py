"""Visualization node — publishes a MarkerArray to RViz2 showing:
  - room outline and tag positions
  - particle cloud (red=low weight → green=high weight)
  - odometry-only trajectory (blue)
  - particle-filter pose-estimate trajectory (green)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

_TAG_XY = [
    [-2.94, -1.2], [-2.94,  0.8],
    [ 2.94, -0.5], [ 2.94,  1.4],
    [-2.00, -1.94], [1.00, -1.94],
    [-1.00,  1.94], [1.80,  1.94],
]
_ROOM_CORNERS = [(-3, -2), (3, -2), (3, 2), (-3, 2), (-3, -2)]
_FRAME = "odom"
_MAX_PATH = 2000


def _pt(x, y, z=0.0) -> Point:
    p = Point(); p.x = float(x); p.y = float(y); p.z = float(z)
    return p


def _col(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA(); c.r = float(r); c.g = float(g); c.b = float(b); c.a = float(a)
    return c


class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")

        self._weights: list[float] = []
        self._particles: list = []
        self._odom_path: list[tuple] = []
        self._pf_path:   list[tuple] = []
        self._true_xy = None          # ground-truth robot pose from Gazebo

        self.create_subscription(PoseArray,         "/pf/particles", self._cloud_cb,   10)
        self.create_subscription(Float32MultiArray, "/pf/weights",   self._weights_cb, 10)
        self.create_subscription(PoseStamped,       "/pf/pose",      self._pose_cb,    10)
        self.create_subscription(Odometry,          "/odom",         self._odom_cb,    10)
        self.create_subscription(PoseStamped, "/model/robot/pose",   self._true_cb,    10)

        self._pub = self.create_publisher(MarkerArray, "/pf/viz", 10)

        # static markers (room + tags) at 1 Hz
        self.create_timer(1.0,  self._pub_static)
        # dynamic markers (cloud + paths) at 10 Hz
        self.create_timer(0.1,  self._pub_dynamic)

        self.get_logger().info(
            "viz_node ready — open RViz2, set Fixed Frame='odom', add MarkerArray on /pf/viz")

    # ── subscribers ──────────────────────────────────────────────────────────

    def _weights_cb(self, msg: Float32MultiArray):
        self._weights = list(msg.data)

    def _cloud_cb(self, msg: PoseArray):
        self._particles = msg.poses

    def _pose_cb(self, msg: PoseStamped):
        xy = (msg.pose.position.x, msg.pose.position.y)
        self._pf_path.append(xy)
        if len(self._pf_path) > _MAX_PATH:
            self._pf_path.pop(0)

    def _odom_cb(self, msg: Odometry):
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom_path.append(xy)
        if len(self._odom_path) > _MAX_PATH:
            self._odom_path.pop(0)

    def _true_cb(self, msg: PoseStamped):
        self._true_xy = (msg.pose.position.x, msg.pose.position.y)

    # ── publishers ────────────────────────────────────────────────────────────

    def _pub_static(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # room outline
        m = Marker()
        m.header.stamp = stamp; m.header.frame_id = _FRAME
        m.ns = "room"; m.id = 0; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.05; m.color = _col(0.8, 0.8, 0.8)
        m.points = [_pt(x, y) for x, y in _ROOM_CORNERS]
        ma.markers.append(m)

        # tag cubes
        for i, (tx, ty) in enumerate(_TAG_XY):
            t = Marker()
            t.header.stamp = stamp; t.header.frame_id = _FRAME
            t.ns = "tags"; t.id = i; t.type = Marker.CUBE; t.action = Marker.ADD
            t.pose.position.x = float(tx)
            t.pose.position.y = float(ty)
            t.pose.position.z = 0.5
            t.pose.orientation.w = 1.0
            t.scale.x = 0.05; t.scale.y = 0.2; t.scale.z = 0.2
            t.color = _col(1.0, 0.5, 0.0)
            ma.markers.append(t)

        self._pub.publish(ma)

    def _pub_dynamic(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # particle cloud — colour: red (low weight) → green (high weight)
        if self._particles:
            w = self._weights if len(self._weights) == len(self._particles) \
                else [1.0 / len(self._particles)] * len(self._particles)
            w_max = max(w) if max(w) > 0 else 1.0

            m = Marker()
            m.header.stamp = stamp; m.header.frame_id = _FRAME
            m.ns = "particles"; m.id = 0
            m.type = Marker.POINTS; m.action = Marker.ADD
            m.scale.x = 0.04; m.scale.y = 0.04
            for pose, wi in zip(self._particles, w):
                t = wi / w_max
                m.points.append(_pt(pose.position.x, pose.position.y))
                m.colors.append(_col(1.0 - t, t, 0.0, 0.7))
            ma.markers.append(m)

        # odometry trajectory (blue)
        if len(self._odom_path) > 1:
            m = Marker()
            m.header.stamp = stamp; m.header.frame_id = _FRAME
            m.ns = "odom_path"; m.id = 0
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.03; m.color = _col(0.2, 0.4, 1.0)
            m.points = [_pt(x, y) for x, y in self._odom_path]
            ma.markers.append(m)

        # PF estimate trajectory (green)
        if len(self._pf_path) > 1:
            m = Marker()
            m.header.stamp = stamp; m.header.frame_id = _FRAME
            m.ns = "pf_path"; m.id = 0
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.03; m.color = _col(0.0, 1.0, 0.3)
            m.points = [_pt(x, y) for x, y in self._pf_path]
            ma.markers.append(m)

        # GROUND TRUTH: the robot's true pose (big cyan sphere). The filter is
        # correct only if the green high-weight cluster sits ON this sphere.
        if self._true_xy is not None:
            m = Marker()
            m.header.stamp = stamp; m.header.frame_id = _FRAME
            m.ns = "true_pose"; m.id = 0
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(self._true_xy[0])
            m.pose.position.y = float(self._true_xy[1])
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color = _col(0.0, 0.9, 1.0, 0.9)   # cyan
            ma.markers.append(m)

        if ma.markers:
            self._pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
