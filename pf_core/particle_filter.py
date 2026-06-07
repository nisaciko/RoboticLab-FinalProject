"""End-to-end particle filter.  [Owner: Yahya, init by Umay]

This is the framework-neutral filter the ROS adapter (and the offline tests)
drive. Bayesian-filtering correspondence:
    - particles + weights  = belief (posterior) over pose
    - predict()            = prior propagation  p(x_t | x_{t-1}, u)
    - update()             = likelihood weighting  p(z | x), then normalise
    - resample()           = keep the posterior representation healthy

Keep this file free of any ROS import.
"""
from __future__ import annotations

import numpy as np

from . import motion_model, resampling, sensor_model


class ParticleFilter:
    def __init__(self, tag_map, num_particles: int = 1000,
                 motion_noise=None, meas_noise=None, seed: int | None = None):
        self.tag_map = tag_map
        self.num_particles = num_particles
        self.motion_noise = motion_noise
        self.meas_noise = meas_noise
        self.rng = np.random.default_rng(seed)

        self.particles = np.zeros((num_particles, 3))      # [x, y, theta]
        self.weights = np.full(num_particles, 1.0 / num_particles)
        self._obs_update_count = 0   # resampling suppressed until this reaches _RESAMPLE_WARMUP
        self._RESAMPLE_WARMUP = 150  # ~5 s at 30 Hz camera — let weights build before collapsing

    # --- initialisation -----------------------------------------------------
    def initialize_uniform(self, x_range, y_range):
        """Scatter particles uniformly over the room with random heading.

        The robot starts at an UNKNOWN location, so the initial belief is a
        uniform prior over the free space.

        """
        self._x_range = x_range
        self._y_range = y_range
        self.particles[:, 0] = self.rng.uniform(x_range[0], x_range[1], self.num_particles)
        self.particles[:, 1] = self.rng.uniform(y_range[0], y_range[1], self.num_particles)
        self.particles[:, 2] = self.rng.uniform(-np.pi, np.pi, self.num_particles)
        self.weights[:] = 1.0 / self.num_particles

    # --- Bayesian filter steps ---------------------------------------------
    def predict(self, u, dt: float):
        self.particles = motion_model.sample_motion(
            self.particles, u, dt, self.motion_noise, self.rng)
        # Particles cannot pass through walls — clamp to room interior.
        if hasattr(self, '_x_range'):
            self.particles[:, 0] = np.clip(
                self.particles[:, 0], self._x_range[0], self._x_range[1])
            self.particles[:, 1] = np.clip(
                self.particles[:, 1], self._y_range[0], self._y_range[1])

    def update(self, observations):
        """Reweight by the multi-hypothesis likelihood, then normalise."""
        likelihood = sensor_model.observation_likelihood(
            self.particles, observations, self.tag_map, self.meas_noise)
        self.weights = self.weights * likelihood
        total = self.weights.sum()
        if total > 0:
            self.weights /= total
        else:
            # all-zero likelihood (filter lost) — reset to uniform
            self.weights[:] = 1.0 / self.num_particles

    def resample(self):
        self.particles = resampling.low_variance_resample(
            self.particles, self.weights, self.rng)

        # Roughen: jitter so resampled duplicates diverge and the cloud
        # can still escape a wrong hypothesis.
        self.particles = resampling.roughen(
            self.particles, xy_sigma=0.10, theta_sigma=0.05, rng=self.rng)

        self.weights[:] = 1.0 / self.num_particles

    def step(self, u, dt, observations, resample_threshold: float | None = None):
        """One full filter cycle: predict → update → (maybe) resample."""
        self.predict(u, dt)
        if observations is not None:
            self.update(observations)
            self._obs_update_count += 1
            if self._obs_update_count >= self._RESAMPLE_WARMUP:
                n_eff = resampling.effective_sample_size(self.weights)
                thresh = resample_threshold or (self.num_particles / 2.0)
                if n_eff < thresh:
                    self.resample()
        return self.estimate()

    # --- output -------------------------------------------------------------
    def estimate(self):
        """Weighted-mean pose estimate (x, y, theta).

        theta is averaged via the unit-circle mean to handle wraparound.
        """
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        s = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        c = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        theta = np.arctan2(s, c)
        return np.array([x, y, theta])
