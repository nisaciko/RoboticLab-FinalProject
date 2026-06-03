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

    # Room is centred at the origin: x in [-3, 3], y in [-2, 2] (interior 6 x 4 m).
    ROOM_BOUNDS = ((-3.0, 3.0), (-2.0, 2.0))  # (x_range, y_range)

    @classmethod
    def default_room(cls) -> "TagMap":
        """The 8-tag asymmetric layout — MUST match simulation/worlds/room.sdf.

        2 tags per wall, intentionally asymmetric (no wall mirrors another), so
        the configuration is locally unique and the filter can converge.
        """
        return cls(np.array([
            [-2.94, -1.2], [-2.94, 0.8],   # left wall  (x = -3)
            [ 2.94, -0.5], [ 2.94, 1.4],   # right wall (x = +3)
            [-2.00, -1.94], [1.00, -1.94], # front wall (y = -2)
            [-1.00,  1.94], [1.80,  1.94], # back wall  (y = +2)
        ]))
