"""Offline particle-filter tests — run WITHOUT ROS or Gazebo.

    python3 -m pytest tests/ -v

These drive pf_core with synthetic motion + synthetic tag observations so the
filter can be developed and debugged in pure Python. They will fail with
NotImplementedError until Yahya/3 implement the core — that is the intended
TDD target.
"""
from __future__ import annotations

import numpy as np
from pf_core.particle_filter import ParticleFilter
from pf_core.tag_map import TagMap


def synthetic_observation(true_pose, tag_map, rng, sigma_r=0.05, sigma_b=0.03,
                          fov=np.radians(120.0), max_range=4.0):
    """Generate (range, bearing) observations from the true pose to each tag.

    Models a FORWARD-FACING camera: a tag is observed only if it lies within the
    horizontal field of view (|bearing| <= fov/2, forward = 0) and within
    max_range. This is realistic — the robot usually sees only 1-2 of the 8 tags
    at a time, which is exactly why the filter needs the asymmetric layout and
    several steps of motion to converge.

    Returns a list of (range, bearing) — possibly EMPTY when no tag is in view.
    (pf_core.update must tolerate an empty observation list: no new evidence,
    so it should leave the weights unchanged.)

    fov / max_range default roughly to a Duckiebot-ish wide lens; tune to match
    the real camera later.
    """
    x, y, th = true_pose
    obs = []
    for tx, ty in tag_map.tags_xy:
        dx, dy = tx - x, ty - y
        true_r = np.hypot(dx, dy)
        true_b = np.arctan2(np.sin(np.arctan2(dy, dx) - th),
                            np.cos(np.arctan2(dy, dx) - th))  # wrapped to (-pi, pi]
        if true_r > max_range or abs(true_b) > fov / 2.0:
            continue  # outside the camera's view -> not detected
        r = true_r + rng.normal(0, sigma_r)
        b = true_b + rng.normal(0, sigma_b)
        obs.append((r, np.arctan2(np.sin(b), np.cos(b))))
    return obs


def test_filter_converges_on_synthetic_run():
    rng = np.random.default_rng(0)
    tag_map = TagMap.default_room()
    pf = ParticleFilter(tag_map, num_particles=2000, seed=0)
    pf.initialize_uniform(x_range=(0, 4), y_range=(0, 4))

    true_pose = np.array([1.0, 1.0, 0.0])
    v, omega, dt = 0.3, 0.2, 0.1

    print(f"\nStep 0   | true=({true_pose[0]:.2f}, {true_pose[1]:.2f}) "
          f"| est=(-.-- , -.--) | err= -.-- m | particles spread across room")

    log_steps = {20, 50, 100, 150, 199}

    for step in range(200):
        true_pose = true_pose + np.array(
            [v * np.cos(true_pose[2]) * dt, v * np.sin(true_pose[2]) * dt, omega * dt])
        true_pose[2] = np.arctan2(np.sin(true_pose[2]), np.cos(true_pose[2]))

        obs = synthetic_observation(true_pose, tag_map, rng)
        est = pf.step(u=np.array([v, omega]), dt=dt, observations=obs)

        err = np.hypot(est[0] - true_pose[0], est[1] - true_pose[1])
        print(f"Step {step+1:<4d} | true=({true_pose[0]:.2f}, {true_pose[1]:.2f}) "
                f"| est=({est[0]:.2f}, {est[1]:.2f}) "
                f"| err={err:.2f} m | tags seen this step: {len(obs)}")

    err = np.hypot(est[0] - true_pose[0], est[1] - true_pose[1])
    print(f"\nFinal error: {err:.3f} m  ({'PASS' if err < 0.5 else 'FAIL'} threshold=0.5m)")
    assert err < 0.5, f"filter did not converge: position error {err:.2f} m"


def test_estimate_is_weighted_mean():
    """estimate() is pure glue and should work before the models exist."""
    pf = ParticleFilter(TagMap.default_room(), num_particles=3, seed=0)
    pf.particles = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
    pf.weights = np.array([0.5, 0.25, 0.25])
    est = pf.estimate()
    assert np.isclose(est[0], 1.5) and np.isclose(est[1], 0.0)
