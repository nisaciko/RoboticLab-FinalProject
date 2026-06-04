"""Resampling.  [Owner: Umay]

Resamples particles in proportion to their weights so that high-likelihood
hypotheses survive and unlikely ones die out. Low-variance (systematic)
resampling is recommended — it draws with a single random offset, which reduces
sampling noise compared to naive multinomial resampling.
"""
from __future__ import annotations

import numpy as np


def low_variance_resample(particles: np.ndarray, weights: np.ndarray,
                          rng: np.random.Generator) -> np.ndarray:
    """Low-variance (systematic) resampling.

    Args:
        particles: (N, 3) array.
        weights:   (N,) normalised weights (sum to 1).
        rng:       NumPy random Generator.

    Returns:
        (N, 3) array of resampled particles (weights become uniform afterwards).

    """
    N = len(particles)
    r = rng.uniform(0.0, 1.0 / N)
    cumsum = np.cumsum(weights)
    indices = np.empty(N, dtype=int)
    i = 0
    for m in range(N):
        u = r + m / N
        while u > cumsum[i]:
            i += 1
        indices[m] = i
    return particles[indices]


def effective_sample_size(weights: np.ndarray) -> float:
    """N_eff = 1 / Σ w_i^2 — useful to decide *when* to resample."""
    w = np.asarray(weights, dtype=float)
    return 1.0 / np.sum(w ** 2)
