"""World-frame map of the 8 AR tags.  [Owner: Nisa]

The filter KNOWS where every tag is in the world frame; what it does NOT know is
which physical tag a given camera detection corresponds to (all tags share one
ID). That ambiguity is handled in sensor_model.py.

Tags must be placed ASYMMETRICALLY (2 per wall) so the layout is locally unique,
which is what lets the filter converge. Keep these coordinates in sync with the
Gazebo world in simulation/worlds/ (sim) and with the measured real room (robot).
"""
from __future__ import annotations

import numpy as np


class TagMap:
    """Container for the known world-frame tag positions.

    Each tag is (x, y) in metres in the world frame. (Orientation can be added
    later if the sensor model needs tag yaw; start with position-only.)
    """

    def __init__(self, tags_xy: np.ndarray):
        tags_xy = np.asarray(tags_xy, dtype=float)
        assert tags_xy.ndim == 2 and tags_xy.shape[1] == 2, "tags_xy must be (N, 2)"
        self.tags_xy = tags_xy

    @property
    def num_tags(self) -> int:
        return self.tags_xy.shape[0]

    @classmethod
    def default_room(cls) -> "TagMap":
        """Placeholder 8-tag asymmetric layout for a room.

        TODO(Nisa): replace with the real coordinates that match the Gazebo
        world once the room is built. These are only so tests/visualisation have
        something to draw. Room assumed roughly [0, W] x [0, H].
        """
        # NOTE: intentionally asymmetric (no two walls mirror each other).
        return cls(np.array([
            [0.0, 0.8], [0.0, 2.6],     # left wall  (x=0)
            [4.0, 1.2], [4.0, 3.1],     # right wall (x=W)
            [1.0, 0.0], [2.9, 0.0],     # bottom wall (y=0)
            [1.5, 4.0], [3.3, 4.0],     # top wall   (y=H)
        ]))
