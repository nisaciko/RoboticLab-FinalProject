"""Detect AprilTag (36h11) markers in the camera image and publish (range, bearing).

Serves BOTH the Gazebo sim (raw sensor_msgs/Image) and the real Duckiebot
(sensor_msgs/CompressedImage). The image topic, compression, camera intrinsics
and tag size are ROS parameters — defaults match the sim; the Duckiebot launch
overrides them. The filter core only consumes (range, bearing), so nothing else
changes between sim and robot.
"""
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

_DICT     = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)  # the lab's physical tags
_PARAMS   = aruco.DetectorParameters()
_DETECTOR = aruco.ArucoDetector(_DICT, _PARAMS)

_DEBUG_SAVE_FRAME = 10   # dump one grayscale frame to /tmp to verify what the camera sees


class AprilTagObsNode(Node):
    def __init__(self):
        super().__init__("apriltag_obs_node")

        # --- parameters (defaults match the Gazebo sim) ---
        # sim intrinsics: horizontal_fov=2.0 rad, 640x480 -> fx = 320 / tan(1.0)
        fx_default = 320.0 / np.tan(1.0)
        self.declare_parameter("image_topic", "/camera")
        self.declare_parameter("use_compressed", False)   # True for the Duckiebot camera
        self.declare_parameter("tag_size", 0.4)           # metres (printed / sim tag side)
        self.declare_parameter("fx", fx_default)
        self.declare_parameter("fy", fx_default)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        topic      = self.get_parameter("image_topic").value
        compressed = bool(self.get_parameter("use_compressed").value)
        ts         = float(self.get_parameter("tag_size").value)
        fx = float(self.get_parameter("fx").value); fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value); cy = float(self.get_parameter("cy").value)

        self._cam_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float64)
        self._dist = np.zeros((4, 1))
        # corner object points: top-left, top-right, bottom-right, bottom-left
        self._objp = np.array([[-ts / 2,  ts / 2, 0], [ ts / 2,  ts / 2, 0],
                               [ ts / 2, -ts / 2, 0], [-ts / 2, -ts / 2, 0]], dtype=np.float32)

        self._frame_count = 0
        self._detect_count = 0
        self._debug_saved = False

        if compressed:
            self.create_subscription(CompressedImage, topic, self._compressed_cb, 10)
        else:
            self.create_subscription(Image, topic, self._image_cb, 10)
        self._pub = self.create_publisher(Float32MultiArray, "/observations", 10)
        self.get_logger().info(
            f"apriltag_obs_node ready — topic={topic} compressed={compressed} "
            f"tag_size={ts} fx={fx:.1f} cx={cx} cy={cy}")

    # --- image entry points ---
    def _compressed_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            self.get_logger().error("failed to decode CompressedImage")
            return
        self._process(gray, encoding="compressed")

    def _image_cb(self, msg: Image):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            n_ch = max(1, len(raw) // (msg.height * msg.width))
            if n_ch > 1:
                frame = raw.reshape(msg.height, msg.width, n_ch)
                code = cv2.COLOR_RGB2GRAY if msg.encoding in ('rgb8', 'RGB8') else cv2.COLOR_BGR2GRAY
                gray = cv2.cvtColor(frame, code)
            else:
                gray = raw.reshape(msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f"image decode error: {e}")
            return
        self._process(gray, encoding=msg.encoding)

    # --- detection + observation publishing ---
    def _marker_tvecs(self, corners):
        tvecs = []
        for c in corners:
            ok, _rvec, tvec = cv2.solvePnP(self._objp, c[0], self._cam_matrix, self._dist,
                                           flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                tvecs.append(tvec.reshape(3))
        return tvecs

    def _process(self, gray, encoding=""):
        self._frame_count += 1

        # dump one frame so we can visually verify what the camera sees
        if self._frame_count == _DEBUG_SAVE_FRAME and not self._debug_saved:
            self._debug_saved = True
            try:
                cv2.imwrite("/tmp/pf_camera_frame.png", gray)
                self.get_logger().warn("DEBUG: saved /tmp/pf_camera_frame.png "
                                       "(open with: xdg-open /tmp/pf_camera_frame.png)")
            except Exception as e:
                self.get_logger().error(f"frame save error: {e}")

        corners, ids, _ = _DETECTOR.detectMarkers(gray)

        flat = []
        if ids is not None and len(corners):
            self._detect_count += len(ids)
            for tvec in self._marker_tvecs(corners):
                tx, _ty, tz = tvec
                if tz <= 0:
                    continue
                r = float(np.hypot(tx, tz))
                # bearing positive = tag to the LEFT of the robot heading.
                # solvePnP camera X = image-right = robot -Y, so tx < 0 when the
                # tag is to the LEFT → negate to get a positive bearing.
                b = float(np.arctan2(-tx, tz))
                b = float(np.arctan2(np.sin(b), np.cos(b)))
                flat += [r, b]

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"[frame {self._frame_count}] enc={encoding} "
                f"detections_this_frame={len(flat) // 2} total={self._detect_count}")

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
