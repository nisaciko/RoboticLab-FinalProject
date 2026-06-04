"""Detect ArUco markers in the camera image and publish (range, bearing) observations."""
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
except ImportError:
    _BRIDGE = None

# Camera intrinsics derived from room.sdf:
#   horizontal_fov=2.0 rad, image 640x480
_FX = 320.0 / np.tan(1.0)   # ~205.4 px
_FY = _FX
_CAM_MATRIX = np.array([[_FX, 0, 320.0],
                         [0, _FY, 240.0],
                         [0,  0,   1.0]], dtype=np.float64)
_DIST = np.zeros((4, 1))
_TAG_SIZE = 0.2   # metres — matches model.sdf box size

_DICT   = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
_PARAMS = aruco.DetectorParameters()
_DETECTOR = aruco.ArucoDetector(_DICT, _PARAMS)

# Marker corner object points (centred, z=0), in detectMarkers() order:
# top-left, top-right, bottom-right, bottom-left.
_OBJP = np.array([[-_TAG_SIZE / 2,  _TAG_SIZE / 2, 0],
                  [ _TAG_SIZE / 2,  _TAG_SIZE / 2, 0],
                  [ _TAG_SIZE / 2, -_TAG_SIZE / 2, 0],
                  [-_TAG_SIZE / 2, -_TAG_SIZE / 2, 0]], dtype=np.float32)


def _marker_tvecs(corners):
    """Translation of each marker in the camera frame.

    Replaces cv2.aruco.estimatePoseSingleMarkers (removed in OpenCV 4.7+) with
    solvePnP + IPPE_SQUARE, the method OpenCV recommends for square markers.
    """
    tvecs = []
    for c in corners:
        ok, _rvec, tvec = cv2.solvePnP(_OBJP, c[0], _CAM_MATRIX, _DIST,
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            tvecs.append(tvec.reshape(3))
    return tvecs


class AprilTagObsNode(Node):
    def __init__(self):
        super().__init__("apriltag_obs_node")
        self.create_subscription(Image, "/camera", self._image_cb, 10)
        self._pub = self.create_publisher(Float32MultiArray, "/observations", 10)
        self.get_logger().info("apriltag_obs_node ready")

    def _image_cb(self, msg: Image):
        if _BRIDGE:
            frame = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        else:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            frame = raw.reshape(msg.height, msg.width, -1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = _DETECTOR.detectMarkers(gray)

        flat = []
        if ids is not None and len(corners):
            for tvec in _marker_tvecs(corners):
                tx, _ty, tz = tvec       # camera frame: z forward, x right
                r = float(np.hypot(tx, tz))
                b = float(np.arctan2(-tx, tz))          # left positive
                b = float(np.arctan2(np.sin(b), np.cos(b)))
                flat += [r, b]

        out = Float32MultiArray()
        out.data = flat
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagObsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
