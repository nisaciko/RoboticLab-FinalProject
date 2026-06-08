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

    # Room is centred at the origin: x in [-2.5, 2.5], y in [-2, 2] (interior 5 x 4 m).
    ROOM_BOUNDS = ((-2.5, 2.5), (-2.0, 2.0))  # (x_range, y_range)

    @classmethod
    def default_room(cls) -> "TagMap":
        """The 8-tag asymmetric layout — MUST match simulation/worlds/room.sdf.

        2 tags per wall, intentionally asymmetric. Minimum 180°-phantom distance
        between any tag and its rotational counterpart is 0.7 m, preventing the
        filter from false-converging to a 180°-rotated position.
        No corner has two tags within 1 m — avoids corner-trap false convergence.
        """
        return cls(np.array([
            [-2.44, -1.2], [-2.44,  0.8],   # left wall  (x = -2.5)
            [ 2.44,  0.3], [ 2.44, -1.5],   # right wall (x = +2.5)
            [-1.60, -1.94], [0.80, -1.94],  # front wall (y = -2)
            [ 0.20,  1.94], [-1.50,  1.94], # back wall  (y = +2)
        ]))

    # --- REAL lab room (measured 2026-06-08): 1.24 x 0.80 m, origin at centre ---
    LAB_BOUNDS = ((-0.62, 0.62), (-0.40, 0.40))  # (x_range, y_range) metres

    @classmethod
    def lab_room(cls) -> "TagMap":
        """The 8 PHYSICAL tag positions in the real lab room (used on the
        Duckiebot). tag36h11 with DIFFERENT IDs — the filter ignores IDs, only
        positions matter. Rectangle 1.24 m (long) x 0.80 m (short), origin at the
        centre: x along the long edges, y along the short edges. Tags are ~0.16 m
        high (irrelevant to the 2D filter). Measured 2026-06-08."""
        return cls(np.array([
            [-0.32,  0.40], [0.08,  0.40],    # long edge "top"    (y = +0.40)
            [-0.16, -0.40], [0.24, -0.40],    # long edge "bottom" (y = -0.40)
            [-0.62, -0.15], [-0.62, 0.15],    # short edge "left"  (x = -0.62)
            [ 0.62, -0.14], [ 0.62, 0.26],    # short edge "right" (x = +0.62)
        ]))
