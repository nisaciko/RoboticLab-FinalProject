# Particle Filter Localization with AR Tags

**AIN451 — Robotics Final Project.** A particle filter that localizes a mobile
robot inside a room using only a forward‑facing camera and AR tags on the walls.

The tags are **indistinguishable to the robot** (in simulation all 8 share one
ID; in the lab they are different physical tags but the filter ignores the ID),
so a single detection does not tell the filter *which* tag was seen. The sensor
model is therefore **multi‑hypothesis**: every observation is explained by *all*
tags at once,

```
p(z | x) = Σ_{i=1..8} p(z | x, tag_i)
```

and the **asymmetric** tag layout is what lets the cloud collapse onto the true
pose as the robot drives around. The same filter runs in **Gazebo simulation**
and on a **real Duckiebot**.

---

## Result

The particle cloud starts spread over the whole room (the robot's location is
unknown) and converges to the true pose once the robot moves and observes a few
tags — both in Gazebo and on the real Duckiebot. The filter estimate stays
accurate while raw **odometry drifts away**, which is exactly what the filter is
there to fix.

Screenshots of the particle cloud at the **initial spread → partial convergence
→ final converged** stages, together with the simulation and real‑robot demo
videos, are included in the **report** (`report/`).

---

## Architecture — one filter, two robots

The localization math lives in **`pf_core/`, which has zero ROS imports** (pure
NumPy). ROS is only a thin adapter that feeds it inputs and publishes results:

```
  motion input (odometry / wheel velocity) ─┐
                                            ├─►  pf_core.ParticleFilter ─► pose estimate + particle cloud
  camera ► AR‑tag detect ► (range, bearing) ─┘
```

Because the core is framework‑agnostic, the **same filter** is reused unchanged:

| Environment | Stack | Marker | Code |
|---|---|---|---|
| **Simulation** (required) | ROS 2 Humble + Gazebo Harmonic | 8× ArUco (`DICT_4X4_50`, same ID) | `main` branch, `pf_ros2/` |
| **Real Duckiebot** (required) | ROS 1 (Duckietown *daffy*), run from the laptop | AprilTag `tag36h11` (lab tags) | [`duckie-bot`](../../tree/duckie-bot) branch, `duckiebot/` |

Going from sim to the real robot **does not touch `pf_core/`** — only the thin
ROS wrapper and the tag map (measured for the real room) change.

---

## Run the simulation (this branch)

Setup (ROS 2 Humble + Gazebo Harmonic): see **[docs/SETUP.md](docs/SETUP.md)**.

```bash
colcon build && source install/setup.bash
ros2 launch pf_ros2 sim.launch.py        # Gazebo + ros_gz bridge + filter + RViz
```
Drive the robot with the keyboard; watch the particle cloud collapse in RViz.

You can also see the filter **without ROS/Gazebo** (pure Python):

```bash
python3 -m pytest tests/ -v                 # offline convergence test (passes)
python3 visualization/offline_demo.py       # animated matplotlib demo (no ROS/Gazebo)
```

## Run on the real Duckiebot

The real‑robot implementation is on the **[`duckie-bot`](../../tree/duckie-bot)**
branch — a single ROS 1 node that runs in the Duckietown `dts` GUI‑tools
container, reads the robot's topics directly, runs the same `pf_core`, and draws
to RViz. Full step‑by‑step runbook: **`docs/duckiebot_run.md`** on that branch.

---

## Repository layout

```
pf_core/            PURE NumPy particle filter — no ROS, shared by sim AND robot
  motion_model.py     predict(): move particles by velocity + Gaussian noise
  sensor_model.py     multi-hypothesis likelihood  p(z|x) = Σ p(z|x, tag_i)
  resampling.py       low-variance resampling + roughening
  particle_filter.py  predict / update / resample / estimate
  tag_map.py          known world-frame tag positions (sim room + real lab room)
tests/              offline synthetic-data tests (run without ROS/Gazebo)
visualization/      offline matplotlib demo (+ report figures)
pf_ros2/            ROS 2 Humble adapter for the Gazebo simulation
  nodes: apriltag_obs_node · pf_node · viz_node   |   launch: sim.launch.py
simulation/         Gazebo Harmonic world, robot + camera model, AR-tag model
duckiebot/          (duckie-bot branch) ROS 1 node for the real Duckiebot
docs/               setup + Duckiebot run/migration guides
report/             report + presentation + figures
```

## How the filter works (Bayesian view)

- **Belief** = the particle set + weights (an approximation of the posterior).
- **Predict** propagates the prior through the motion model `p(xₜ | xₜ₋₁, u)`
  (wheel velocity + Gaussian noise).
- **Update** weights each particle by the likelihood `p(z | x)` — the
  multi‑hypothesis sum over all tags — then normalises (the posterior).
- **Resample** (low‑variance) keeps the representation healthy; roughening adds
  small jitter so the cloud can still escape a wrong hypothesis.

## Team

Hayrunnisa Çiko 2220356043
Zeynep Umay İyim 2220765025
Mohamed Yahya Mansouri 2220765064

Course: AIN451, Dr. Özgür Erkent — R.A. Mehmet Muratoğlu.
