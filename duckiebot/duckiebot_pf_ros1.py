#!/usr/bin/env python3
"""All-in-one ROS1 particle filter for the real Duckiebot (no ros1_bridge).

Reuses pf_core (pure NumPy) — the SAME filter as the sim. Subscribes directly to
the Duckiebot's ROS1 topics, runs the filter, and publishes a MarkerArray for
RViz1. Camera intrinsics are read automatically from camera_info.

Run inside the dts gui-tools container (it has ROS1 + duckietown_msgs + cv2 4.2):

  # ---- from a HOST terminal (normal laptop shell, NOT the container) ----
  cd ~/RoboticLab-FinalProject
  docker cp pf_core                         dts_gui_tools_rabbit:/tmp/pf_core
  docker cp duckiebot/duckiebot_pf_ros1.py  dts_gui_tools_rabbit:/tmp/

  # ---- inside the CONTAINER terminal ----
  python3 /tmp/duckiebot_pf_ros1.py _robot:=rabbit _tag_size:=0.065
  # another container shell, for visualization:
  rosrun rviz rviz       # Fixed Frame = map, Add -> MarkerArray on /rabbit/pf/viz

IMPORTANT: update pf_core/tag_map.py to the REAL room (measured tag positions)
before trusting the result — the filter localizes against that map.
"""
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
import tf2_ros

sys.path.insert(0, "/tmp")            # where pf_core was docker cp'd
from pf_core.particle_filter import ParticleFilter
from pf_core.tag_map import TagMap

_DICT   = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)  # cv2 4.2 old API
_PARAMS = aruco.DetectorParameters_create()
_FRAME  = "map"


def _pt(x, y, z=0.0):
    p = Point(); p.x = float(x); p.y = float(y); p.z = float(z); return p


def _col(r, g, b, a=1.0):
    c = ColorRGBA(); c.r = float(r); c.g = float(g); c.b = float(b); c.a = float(a); return c


class DuckiebotPF(object):
    def __init__(self):
        self.robot    = rospy.get_param("~robot", "rabbit")
        self.tag_size = float(rospy.get_param("~tag_size", 0.065))

        tag_map = TagMap.lab_room()               # real measured lab room (1.24 x 0.80 m)
        self.tags_xy = tag_map.tags_xy
        (xlo, xhi), (ylo, yhi) = TagMap.LAB_BOUNDS
        self.room = [(xlo, ylo), (xhi, ylo), (xhi, yhi), (xlo, yhi), (xlo, ylo)]
        # Noise tuned for the SMALL real room (~0.9 m): tag distances are 0.3–0.9 m,
        # so the sim's 0.3 m range noise was far too loose to localise. Tighter
        # noise = the filter trusts detections more = the cloud actually collapses.
        self.pf = ParticleFilter(tag_map, num_particles=5000,
                                 motion_noise=np.array([0.03, 0.05]),
                                 meas_noise=np.array([0.06, 0.10]), seed=42)
        self.pf.initialize_uniform(x_range=(xlo, xhi), y_range=(ylo, yhi))

        ts = self.tag_size
        self.objp = np.array([[-ts/2, ts/2, 0], [ts/2, ts/2, 0],
                              [ts/2, -ts/2, 0], [-ts/2, -ts/2, 0]], dtype=np.float32)
        self.K = None
        self.dist = np.zeros((4, 1))

        self.latest_obs = None
        self.latest_v = 0.0
        self.latest_omega = 0.0
        self.last_t = None
        self.odom_path = []
        self.latest_odom = None      # (x, y, theta) from velocity_to_pose
        self.anchor = None           # (ox0,oy0,oth0, ex0,ey0,eth0) set once the cloud converges
        self.pf_path = []
        self.frame_count = 0
        self.detect_count = 0
        self.est = None

        self._static_map_tf()
        self.pub = rospy.Publisher("/pf/viz", MarkerArray, queue_size=1)  # frame-agnostic topic
        rospy.Subscriber("/%s/camera_node/camera_info" % self.robot, CameraInfo, self._cinfo_cb, queue_size=1)
        rospy.Subscriber("/%s/camera_node/image/compressed" % self.robot, CompressedImage, self._img_cb, queue_size=1)
        rospy.Subscriber("/%s/kinematics_node/velocity" % self.robot, Twist2DStamped, self._vel_cb, queue_size=30)
        rospy.Subscriber("/%s/velocity_to_pose_node/pose" % self.robot, Pose2DStamped, self._pose_cb, queue_size=30)
        rospy.Timer(rospy.Duration(0.1), self._filter_step)      # filter runs at 10 Hz regardless of motion
        rospy.Timer(rospy.Duration(0.2), self._publish_markers)
        rospy.loginfo("duckiebot_pf ready (robot=%s tag_size=%.3f) — waiting for camera_info + topics",
                      self.robot, self.tag_size)

    def _static_map_tf(self):
        self._br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = _FRAME
        t.child_frame_id = "pf_origin"
        t.transform.rotation.w = 1.0
        self._br.sendTransform(t)

    # --- sensors ---
    def _cinfo_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            if len(msg.D) >= 4:
                self.dist = np.array(msg.D, dtype=np.float64).reshape(-1, 1)
            rospy.loginfo("camera_info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                          self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])

    def _img_cb(self, msg):
        if self.K is None:
            return
        gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        self.frame_count += 1
        corners, ids, _ = aruco.detectMarkers(gray, _DICT, parameters=_PARAMS)
        obs = []
        if ids is not None and len(corners):
            self.detect_count += len(ids)
            _r, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.tag_size, self.K, self.dist)
            for tv in tvecs:
                tx, _ty, tz = tv[0]
                if tz <= 0:
                    continue
                r = float(np.hypot(tx, tz))
                b = float(np.arctan2(-tx, tz))               # +bearing = tag to the LEFT
                obs.append((r, float(np.arctan2(np.sin(b), np.cos(b)))))
        self.latest_obs = obs
        if self.frame_count % 30 == 0:
            rospy.loginfo("[frame %d] tags this frame=%d  total=%d",
                          self.frame_count, len(obs), self.detect_count)

    def _vel_cb(self, msg):
        # just store the latest velocity; the filter steps on its own timer
        self.latest_v = msg.v
        self.latest_omega = msg.omega

    def _filter_step(self, _evt):
        now = rospy.Time.now().to_sec()
        if self.last_t is None:
            self.last_t = now
            return
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0.0 or dt > 1.0:
            return
        obs = self.latest_obs
        self.latest_obs = None
        self.est = self.pf.step(u=np.array([self.latest_v, self.latest_omega]),
                                dt=dt, observations=obs)
        self.pf_path.append((float(self.est[0]), float(self.est[1])))
        if len(self.pf_path) > 2000:
            self.pf_path.pop(0)

        # Once the cloud has converged, freeze an anchor that maps the odom frame
        # onto the map at this instant — so the displayed odom path starts on the
        # filter estimate and its later drift is visible.
        if self.anchor is None and self.latest_odom is not None:
            if float(np.std(self.pf.particles[:, :2])) < 0.12:
                ox, oy, oth = self.latest_odom
                self.anchor = (ox, oy, oth,
                               float(self.est[0]), float(self.est[1]), float(self.est[2]))
                rospy.loginfo("odom path anchored to filter estimate (cloud converged)")

    def _pose_cb(self, msg):
        self.latest_odom = (msg.x, msg.y, msg.theta)
        self.odom_path.append((msg.x, msg.y))
        if len(self.odom_path) > 2000:
            self.odom_path.pop(0)

    # --- visualization (MarkerArray for RViz1) ---
    def _publish_markers(self, _evt):
        ma = MarkerArray()
        now = rospy.Time.now()

        def base(ns, mid, mtype, sx):
            m = Marker()
            m.header.stamp = now; m.header.frame_id = _FRAME
            m.ns = ns; m.id = mid; m.type = mtype; m.action = Marker.ADD
            m.scale.x = sx; m.pose.orientation.w = 1.0
            return m

        room = base("room", 0, Marker.LINE_STRIP, 0.01)
        room.color = _col(0.8, 0.8, 0.8)
        room.points = [_pt(x, y) for x, y in self.room]
        ma.markers.append(room)

        for i, (tx, ty) in enumerate(self.tags_xy):
            t = base("tags", i, Marker.CUBE, 0.05)
            t.scale.y = t.scale.z = 0.08
            t.pose.position.x = float(tx); t.pose.position.y = float(ty); t.pose.position.z = 0.1
            t.color = _col(1.0, 0.5, 0.0)
            ma.markers.append(t)

        pc = base("particles", 0, Marker.POINTS, 0.012); pc.scale.y = 0.012  # small dots for the 1.24x0.80 m room
        w = self.pf.weights; wmax = float(w.max()) if w.max() > 0 else 1.0
        for p, wi in zip(self.pf.particles, w):
            tnorm = wi / wmax
            pc.points.append(_pt(p[0], p[1]))
            pc.colors.append(_col(1.0 - tnorm, tnorm, 0.0, 0.8))
        ma.markers.append(pc)

        # odometry-only path, rigidly aligned to the filter at the convergence
        # moment (anchor) so both start together and the odom DRIFT is visible.
        if self.anchor is not None and len(self.odom_path) > 1:
            ox0, oy0, oth0, ex0, ey0, eth0 = self.anchor
            dth = eth0 - oth0
            c, s = np.cos(dth), np.sin(dth)
            op = base("odom_path", 0, Marker.LINE_STRIP, 0.006)
            op.color = _col(0.2, 0.4, 1.0)
            for ox, oy in self.odom_path:
                dx, dy = ox - ox0, oy - oy0
                op.points.append(_pt(ex0 + c * dx - s * dy, ey0 + s * dx + c * dy))
            ma.markers.append(op)

        if len(self.pf_path) > 1:
            pp = base("pf_path", 0, Marker.LINE_STRIP, 0.006)
            pp.color = _col(0.0, 1.0, 0.3)
            pp.points = [_pt(x, y) for x, y in self.pf_path]
            ma.markers.append(pp)

        # estimated robot pose (weighted mean) — a big magenta sphere = "the robot"
        if self.est is not None:
            e = base("estimate", 0, Marker.SPHERE, 0.1)
            e.scale.y = e.scale.z = 0.1
            e.pose.position.x = float(self.est[0]); e.pose.position.y = float(self.est[1])
            e.pose.position.z = 0.12
            e.color = _col(1.0, 0.0, 1.0, 0.95)
            ma.markers.append(e)

        self.pub.publish(ma)


if __name__ == "__main__":
    rospy.init_node("duckiebot_pf")
    DuckiebotPF()
    rospy.spin()
