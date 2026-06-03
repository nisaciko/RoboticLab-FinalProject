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

    TODO(Yahya):
      1. For each particle, predict the (range, bearing) to every tag in tag_map.
      2. For each real observation, compute a Gaussian likelihood against each
         tag's predicted measurement and SUM over the tags (multi-hypothesis).
      3. Combine across observations in the frame.
    """
    raise NotImplementedError("Yahya: multi-hypothesis sensor model (sum over all tags)")
