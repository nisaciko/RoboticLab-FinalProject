# Particle Filter Localization with AR Tags

AIN451 Final Project — a particle filter that localizes a mobile robot inside a
closed rectangular room using a forward-facing camera and **8 AprilTags that all
share the same ID**. Because the tags are indistinguishable, the sensor model is
multi-hypothesis: every observation is explained by *all* tags, and the
asymmetric tag layout lets the filter converge as the robot drives around.

Runs in **two environments that share the exact same filter core**:

| Environment | Stack | Status |
|---|---|---|
| Simulation (required) | ROS 2 Humble + Gazebo Harmonic | primary |
| Real Duckiebot (required) | Duckietown ROS1 (daffy) ↔ ROS 2 Humble via `ros1_bridge` | see [docs/duckiebot_migration.md](docs/duckiebot_migration.md) |

## Architecture — why this is easy to migrate

The localization math lives in **`pf_core/`, which has zero ROS imports** (pure
NumPy). ROS is only a thin adapter that feeds it inputs and publishes results:

```
  motion input (odometry)  ─┐
                            ├─►  pf_core.ParticleFilter  ─►  pose estimate + particle cloud
  camera ► AprilTag ► (range,bearing) per detection ─┘
```

To move from sim to the Duckiebot you **do not touch `pf_core/`** — you only swap
the thin ROS adapter's input topics and update the tag map to the real room.
You can also develop and test the whole filter **offline with synthetic data**,
no Gazebo or ROS required (see `tests/`).

## Repository layout

```
pf_core/          # PURE NumPy filter — no ROS. Shared by sim AND robot.   [Yahya + 3]
  motion_model.py     predict(): move particles by odometry + Gaussian noise   [K2]
  sensor_model.py     p(z|x) = Σ_{i=1..8} p(z|x, tag_i)  (multi-hypothesis)    [K2]
  resampling.py       low-variance resampling                                  [K3]
  particle_filter.py  predict / update / resample / estimate orchestration     [K2]
  tag_map.py          world-frame positions of the 8 tags                      [K1]
tests/            # offline synthetic-data tests (run without ROS/Gazebo)
pf_ros2/          # thin ROS 2 Humble adapter (ament_python)                   [K1]
  nodes/  pf_node · apriltag_obs_node · viz_node
  launch/ sim.launch.py · duckiebot.launch.py
simulation/       # Gazebo Harmonic world, models, launch                      [Nisa]
  worlds/ models/ launch/
visualization/    # particle cloud + trajectories rendering                    [Umay]
docs/             # migration guide, setup notes
report/           # report + presentation source                              [K1 + K3]
```

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble — *not yet installed on the dev machine; see Setup.*
- Gazebo Harmonic (`gz sim`) — already installed
- Python 3.10+, NumPy

## Setup

```bash
# pf_core has no ROS dependency — you can work on the filter immediately:
python3 -m pip install numpy pytest
python3 -m pytest tests/ -v        # will fail until pf_core is implemented (TDD)
```

Full from-scratch setup for a new machine (Python, ROS 2 Humble, Gazebo
Harmonic, bridge, build) is in **[docs/SETUP.md](docs/SETUP.md)** — share this
with teammates.

## Branch strategy

- `main` — protected, always working. Merge only via reviewed PR.
- `dev` — integration branch; features merge here first.
- `feature/k1-*`, `feature/k2-*`, `feature/k3-*` — one branch per person/task.

Flow: `feature/*` → PR into `dev` → (tested) `dev` → PR into `main`.

## Team

| | Owner | Responsibility |
|---|---|---|
| Nisa | sim + repo + report | this repo, Gazebo world, tag map, report/slides |
| Yahya | filter core | motion + sensor model, weighting, resampling, end-to-end |
| Umay | support + viz | particle init, resampling, visualization, report |
