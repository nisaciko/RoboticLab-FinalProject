"""Detect ArUco markers in the camera image and publish (range, bearing) observations."""
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# cv_bridge segfaults on NumPy 2.x — use the raw-bytes path unconditionally.
_BRIDGE = None

# Camera intrinsics from room.sdf: horizontal_fov=2.0 rad, 640x480
_FX = 320.0 / np.tan(1.0)   # ~205.4 px
_FY = _FX
_CAM_MATRIX = np.array([[_FX, 0, 320.0],
                         [0, _FY, 240.0],
                         [0,  0,   1.0]], dtype=np.float64)
_DIST = np.zeros((4, 1))

# Physical size of the ArUco marker square (metres) — matches model.sdf box Y/Z.
_TAG_SIZE = 0.4

_DICT     = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
_PARAMS   = aruco.DetectorParameters()
_DETECTOR = aruco.ArucoDetector(_DICT, _PARAMS)

# Corner object points: top-left, top-right, bottom-right, bottom-left (ArUco order).
_OBJP = np.array([[-_TAG_SIZE / 2,  _TAG_SIZE / 2, 0],
                  [ _TAG_SIZE / 2,  _TAG_SIZE / 2, 0],
                  [ _TAG_SIZE / 2, -_TAG_SIZE / 2, 0],
                  [-_TAG_SIZE / 2, -_TAG_SIZE / 2, 0]], dtype=np.float32)


def _marker_tvecs(corners):
    tvecs = []
    for c in corners:
        ok, _rvec, tvec = cv2.solvePnP(_OBJP, c[0], _CAM_MATRIX, _DIST,
                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            tvecs.append(tvec.reshape(3))
    return tvecs


_DEBUG_SAVE_FRAME = 10   # save a raw camera frame at this frame number for visual inspection

class AprilTagObsNode(Node):
    def __init__(self):
        super().__init__("apriltag_obs_node")
        self._frame_count = 0
        self._detect_count = 0
        self._debug_saved = False

        self.create_subscription(Image, "/camera", self._image_cb, 10)
        self._pub = self.create_publisher(Float32MultiArray, "/observations", 10)
        self.get_logger().info("apriltag_obs_node ready — waiting for /camera frames")

    def _image_cb(self, msg: Image):
        self._frame_count += 1

        # --- decode image ---
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            n_ch = len(raw) // (msg.height * msg.width)
            if n_ch == 0:
                n_ch = 1
            frame = raw.reshape(msg.height, msg.width, n_ch) if n_ch > 1 \
                    else raw.reshape(msg.height, msg.width)
            # encoding is rgb8 from Gazebo; for grayscale conversion channel order
            # doesn't matter for B&W markers, but handle explicitly to be safe
            if frame.ndim == 3:
                if msg.encoding in ('rgb8', 'RGB8'):
                    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                else:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame
        except Exception as e:
            self.get_logger().error(f"image decode error: {e}")
            return

        # --- save one frame to /tmp so we can visually verify what the camera sees ---
        if self._frame_count == _DEBUG_SAVE_FRAME and not self._debug_saved:
            self._debug_saved = True
            try:
                save_path = "/tmp/pf_camera_frame.png"
                if frame.ndim == 3 and msg.encoding in ('rgb8', 'RGB8'):
                    cv2.imwrite(save_path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                else:
                    cv2.imwrite(save_path, frame)
                self.get_logger().warn(
                    f"DEBUG: saved frame {self._frame_count} → {save_path}  "
                    f"(open with: eog {save_path}  or: xdg-open {save_path})")
            except Exception as e:
                self.get_logger().error(f"frame save error: {e}")

        # --- detect ---
        corners, ids, _ = _DETECTOR.detectMarkers(gray)

        flat = []
        if ids is not None and len(corners):
            self._detect_count += len(ids)
            for tvec in _marker_tvecs(corners):
                tx, _ty, tz = tvec
                if tz <= 0:
                    continue
                r = float(np.hypot(tx, tz))
                # Bearing: positive = tag to the LEFT of the robot heading.
                # Gazebo camera facing +X: image-left = robot +Y (left).
                # solvePnP camera frame X = image-right = robot -Y, so tx < 0
                # when the tag is to the LEFT → negate to get positive bearing.
                b = float(np.arctan2(-tx, tz))
                b = float(np.arctan2(np.sin(b), np.cos(b)))
                flat += [r, b]

        # --- log every 30 frames ---
        if self._frame_count % 30 == 0:
            n_det = len(flat) // 2
            self.get_logger().info(
                f"[frame {self._frame_count}] encoding={msg.encoding} "
                f"size={msg.width}x{msg.height} "
                f"detections_this_frame={n_det} "
                f"total_detections_so_far={self._detect_count}"
            )

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
