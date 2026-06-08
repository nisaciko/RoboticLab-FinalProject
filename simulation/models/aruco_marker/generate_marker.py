"""Generate the ArUco marker texture used by the 8 wall tags.  [Nisa]

All 8 tags in the room share the SAME ID (the project requires identical tags),
so we generate one marker image and reuse it on every tag instance.

    python3 simulation/models/aruco_marker/generate_marker.py

Produces materials/textures/aruco_0.png next to this script. Re-run to change the
dictionary / id / resolution. Keep DICT and ID in sync with the detector in
pf_ros2 (aruco_obs_node) and the report.
"""
import os
import cv2
import numpy as np

DICT = cv2.aruco.DICT_APRILTAG_36h11   # AprilTag 36h11 — matches the lab's physical tags
MARKER_ID = 0                  # same ID on all 8 tags
SIDE_PX = 600                  # marker resolution (excl. border)
BORDER_PX = 100                # white quiet zone (REQUIRED for detection)

here = os.path.dirname(os.path.abspath(__file__))
out_dir = os.path.join(here, "materials", "textures")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, f"aruco_{MARKER_ID}.png")

dictionary = cv2.aruco.getPredefinedDictionary(DICT)
marker = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, SIDE_PX)

# pad with a white quiet zone so the marker is detectable
canvas = np.full((SIDE_PX + 2 * BORDER_PX, SIDE_PX + 2 * BORDER_PX), 255, np.uint8)
canvas[BORDER_PX:BORDER_PX + SIDE_PX, BORDER_PX:BORDER_PX + SIDE_PX] = marker

cv2.imwrite(out_path, canvas)
print(f"wrote {out_path}  ({canvas.shape[1]}x{canvas.shape[0]} px, "
      f"DICT_APRILTAG_36h11 id={MARKER_ID})")
