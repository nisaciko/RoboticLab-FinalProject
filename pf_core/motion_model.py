"""Motion (prediction) model.  [Owner: Yahya]

Moves each particle according to the odometry control input, adding Gaussian
noise so the particle cloud spreads to reflect motion uncertainty. This is the
prediction step of Bayesian filtering: it propagates the prior through p(x_t | x_{t-1}, u_t).
"""
from __future__ import annotations

import numpy as np


def sample_motion(particles: np.ndarray, u: np.ndarray, dt: float,
                  noise: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Apply the motion model to every particle.

    Args:
        particles: (N, 3) array of [x, y, theta].
        u:         control input from odometry. Suggested form: [v, omega]
                   (linear m/s, angular rad/s) OR a relative pose delta
                   [dx, dy, dtheta] — pick one convention and document it.
        dt:        timestep in seconds.
        noise:     motion noise parameters (e.g. std devs). Document the layout.
        rng:       NumPy random Generator (pass one in for reproducible tests).

    Returns:
        (N, 3) array of moved particles. theta should be wrapped to (-pi, pi].

    noise layout: [sigma_v, sigma_omega]
      sigma_v     — std dev added to linear velocity  (m/s)
      sigma_omega — std dev added to angular velocity (rad/s)
    """
    if noise is None:
        noise = np.array([0.05, 0.05])
    sigma_v, sigma_omega = noise[0], noise[1]
    N = len(particles)

    v_noisy = u[0] + rng.normal(0.0, sigma_v, N)
    omega_noisy = u[1] + rng.normal(0.0, sigma_omega, N)

    out = particles.copy()
    theta = particles[:, 2]
    out[:, 0] += v_noisy * np.cos(theta) * dt
    out[:, 1] += v_noisy * np.sin(theta) * dt
    out[:, 2] += omega_noisy * dt
    out[:, 2] = np.arctan2(np.sin(out[:, 2]), np.cos(out[:, 2]))
    return out
