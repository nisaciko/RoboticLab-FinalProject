#!/usr/bin/python3
"""Offline particle-filter visual demo (no ROS / no Gazebo needed).

Drives a synthetic robot on a loop inside the room, feeds the SAME pf_core filter
synthetic (range, bearing) observations of the wall tags, and animates:

    - the room + the 8 AR tags
    - the particle cloud (coloured by weight)
    - the TRUE pose (green) vs the filter estimate (red)
    - the odometry-only trajectory (grey, drifts) vs the filter trajectory (red)

It also saves snapshots (initial spread / partial convergence / final) to
report/figures/ for the report.

Run (use the SYSTEM python so numpy/matplotlib are available):
    ./visualization/offline_demo.py
    ./visualization/offline_demo.py --no-show     # only save figures (headless)
"""
import os
import sys
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pf_core.particle_filter import ParticleFilter
from pf_core.tag_map import TagMap


def synthetic_observation(true_pose, tag_map, rng, sigma_r=0.05, sigma_b=0.03,
                          fov=np.radians(120.0), max_range=4.0):
    """(range, bearing) to every tag within the forward-facing camera's view."""
    x, y, th = true_pose
    obs = []
    for tx, ty in tag_map.tags_xy:
        dx, dy = tx - x, ty - y
        r = np.hypot(dx, dy)
        b = np.arctan2(np.sin(np.arctan2(dy, dx) - th), np.cos(np.arctan2(dy, dx) - th))
        if r > max_range or abs(b) > fov / 2.0:
            continue
        obs.append((r + rng.normal(0, sigma_r),
                    np.arctan2(np.sin(b + rng.normal(0, sigma_b)),
                               np.cos(b + rng.normal(0, sigma_b)))))
    return obs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-show", action="store_true", help="save figures only, no live window")
    # 5000 particles needed for reliable GLOBAL localization (uniform over the
    # whole room); fewer can lock onto a wrong symmetric cluster early.
    ap.add_argument("--particles", type=int, default=5000)
    ap.add_argument("--steps", type=int, default=400)
    args = ap.parse_args()
    if args.no_show:
        matplotlib.use("Agg")

    rng = np.random.default_rng(0)
    tag_map = TagMap.default_room()
    (xlo, xhi), (ylo, yhi) = TagMap.ROOM_BOUNDS

    pf = ParticleFilter(tag_map, num_particles=args.particles, seed=0)
    pf.initialize_uniform(x_range=(xlo, xhi), y_range=(ylo, yhi))

    # synthetic ground truth: a wide loop near the walls so the robot sees many
    # different tags over time (this is what breaks the symmetry and lets the
    # cloud collapse to the true pose). radius = v / omega.
    true_pose = np.array([2.0, -1.5, -np.pi / 2])
    odom_pose = true_pose.copy()          # dead-reckoning (drifts), "odometry only"
    v, omega, dt = 0.5, 0.25, 0.1         # radius 2.0 m loop

    true_traj, odom_traj, est_traj = [], [], []
    fig_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "report", "figures")
    os.makedirs(fig_dir, exist_ok=True)
    snaps = {5: "initial_spread", args.steps // 3: "partial_convergence",
             args.steps - 1: "final_converged"}

    plt.ion() if not args.no_show else None
    fig, ax = plt.subplots(figsize=(7, 6))

    def wrap(a):
        return np.arctan2(np.sin(a), np.cos(a))

    for step in range(args.steps):
        # advance ground truth (exact) and odometry (noisy dead-reckoning)
        true_pose = true_pose + np.array([v*np.cos(true_pose[2])*dt,
                                          v*np.sin(true_pose[2])*dt, omega*dt])
        true_pose[2] = wrap(true_pose[2])
        odom_pose = odom_pose + np.array([(v+rng.normal(0, 0.02))*np.cos(odom_pose[2])*dt,
                                          (v+rng.normal(0, 0.02))*np.sin(odom_pose[2])*dt,
                                          (omega+rng.normal(0, 0.03))*dt])
        odom_pose[2] = wrap(odom_pose[2])

        obs = synthetic_observation(true_pose, tag_map, rng)
        est = pf.step(u=np.array([v, omega]), dt=dt, observations=obs)

        true_traj.append(true_pose[:2].copy())
        odom_traj.append(odom_pose[:2].copy())
        est_traj.append(est[:2].copy())

        if args.no_show and step not in snaps:
            continue

        # ---- draw ----
        ax.clear()
        ax.add_patch(plt.Rectangle((xlo, ylo), xhi-xlo, yhi-ylo, fill=False, lw=2, ec="0.4"))
        ax.scatter(tag_map.tags_xy[:, 0], tag_map.tags_xy[:, 1], marker="s", s=90,
                   c="black", label="AR tags", zorder=5)
        sc = ax.scatter(pf.particles[:, 0], pf.particles[:, 1], c=pf.weights, cmap="viridis",
                        s=6, alpha=0.5, label="particles")
        T = np.array(true_traj); O = np.array(odom_traj); E = np.array(est_traj)
        ax.plot(O[:, 0], O[:, 1], "--", color="orange", lw=1.5, label="odometry only")
        ax.plot(E[:, 0], E[:, 1], "-", color="red", lw=1.5, label="filter estimate")
        ax.plot(T[:, 0], T[:, 1], "-", color="green", lw=1.0, alpha=0.6, label="true path")
        ax.scatter(*true_pose[:2], c="green", s=80, marker="*", zorder=6)
        ax.scatter(*est[:2], c="red", s=60, marker="x", zorder=6)
        err = np.hypot(est[0]-true_pose[0], est[1]-true_pose[1])
        ax.set_title(f"step {step}  |  error {err:.2f} m  |  tags seen {len(obs)}")
        ax.set_xlim(xlo-0.5, xhi+0.5); ax.set_ylim(ylo-0.5, yhi+0.5)
        ax.set_aspect("equal"); ax.legend(loc="upper right", fontsize=8)

        if step in snaps:
            path = os.path.join(fig_dir, f"{snaps[step]}.png")
            fig.savefig(path, dpi=120, bbox_inches="tight")
            print(f"saved {path}")
        if not args.no_show:
            plt.pause(0.001)

    err = np.hypot(est_traj[-1][0]-true_traj[-1][0], est_traj[-1][1]-true_traj[-1][1])
    print(f"final error: {err:.3f} m  (figures in report/figures/)")
    if not args.no_show:
        plt.ioff(); plt.show()


if __name__ == "__main__":
    main()
