"""Multi-hypothesis sensor (measurement) model.  [Owner: Yahya]

Because all 8 AR tags share the same ID, a single detection does not tell us
which tag was seen. The likelihood of an observation given a particle pose must
therefore sum over ALL tags as possible sources:

        p(z | x_particle) = Σ_{i=1..8} p(z | x_particle, tag_i)

DO NOT shortcut this to "nearest tag only" — the assignment explicitly forbids it
and it breaks convergence. This is the likelihood step of Bayesian filtering.
"""
from __future__ import annotations

import numpy as np


def observation_likelihood(particles: np.ndarray, observations, tag_map,
                           sigma) -> np.ndarray:
    """Likelihood p(z | x) for each particle, summed over all tag hypotheses.

    Args:
        particles:    (N, 3) array of [x, y, theta].
        observations: the detections from one camera frame. Suggested form: a
                      list/array of (range, bearing) measurements, one per
                      detected tag in the image.
        tag_map:      TagMap with the N_tags known world positions.
        sigma:        measurement noise (std devs for range / bearing).

    Returns:
        (N,) array of likelihoods (un-normalised), one per particle. For multiple
        observations in a frame, combine per-observation likelihoods (assume
        conditional independence → product, or sum of log-likelihoods).

    sigma layout: [sigma_r, sigma_b]
      sigma_r — std dev for range   (metres)
      sigma_b — std dev for bearing (radians)
    """
    N = len(particles)
    if not observations:
        return np.ones(N)

    if sigma is None:
        sigma = np.array([0.3, 0.3])
    sigma_r, sigma_b = sigma[0], sigma[1]

    # Half field-of-view of the camera. The real Duckiebot uses a WIDE fisheye
    # (~160°), so a centre robot sees wall tags at large bearings (±70-90°). The
    # old narrow value (1.0 rad ~57°, from the sim camera) excluded those tags,
    # which made the filter favour edge/corner particles (tags appear "ahead"
    # there) and biased the estimate to the walls. Use the front hemisphere.
    _HALF_FOV = np.pi / 2   # ~90°

    tags = tag_map.tags_xy          # (T, 2)
    px = particles[:, 0]            # (N,)
    py = particles[:, 1]
    pth = particles[:, 2]

    # predicted range/bearing from every particle to every tag — shape (N, T)
    dx = tags[:, 0] - px[:, np.newaxis]   # (N, T)
    dy = tags[:, 1] - py[:, np.newaxis]
    pred_r = np.hypot(dx, dy)
    pred_b = np.arctan2(dy, dx) - pth[:, np.newaxis]
    pred_b = np.arctan2(np.sin(pred_b), np.cos(pred_b))   # wrap to (-pi, pi]

    # FOV mask: True where a tag is within the camera's field of view (N, T)
    in_fov = np.abs(pred_b) <= _HALF_FOV

    total = np.ones(N)
    for r_obs, b_obs in observations:
        dr = r_obs - pred_r                                # (N, T)
        db = b_obs - pred_b
        db = np.arctan2(np.sin(db), np.cos(db))           # wrap bearing diff

        L = (np.exp(-0.5 * (dr / sigma_r) ** 2) *
             np.exp(-0.5 * (db / sigma_b) ** 2) *
             in_fov)                                       # (N, T)

        # Floor prevents complete underflow when no in-FOV tag matches well.
        L_sum = np.maximum(L.sum(axis=1), 1e-300)         # (N,)
        total *= L_sum

    return total
