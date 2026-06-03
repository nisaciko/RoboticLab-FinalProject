# Full System Setup Guide

This guide takes a **fresh Ubuntu 22.04 machine** to a working development setup
for the Particle Filter Localization project. Follow it top to bottom.

There are two tracks:

- **Track A — Filter only (no ROS/Gazebo needed).** If you only work on the
  filter core (`pf_core/`) or visualization logic, you just need Python + NumPy.
  This is the fastest way to start. → do **Part 1** and you're done.
- **Track B — Full simulation.** If you run the Gazebo simulation, you also need
  ROS 2 Humble + Gazebo Harmonic. → do **Part 1 → Part 5**.

> Target OS: **Ubuntu 22.04 LTS** (Jammy). ROS 2 Humble and Gazebo Harmonic are
> chosen specifically for 22.04. If you are on a different Ubuntu, tell the team
> before installing — versions are coupled.

---

## Part 0 — Base tools

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget build-essential lsb-release gnupg software-properties-common
```

Clone the repo (pick a folder you like):

```bash
git clone https://github.com/nisaciko/RoboticLab-FinalProject.git
cd RoboticLab-FinalProject
```

---

## Part 1 — Python + the filter core (Track A stops here)

The filter (`pf_core/`) is **pure NumPy, zero ROS**. You can develop and test it
with nothing but Python.

```bash
sudo apt install -y python3-pip
python3 -m pip install --upgrade numpy pytest
```

Verify — run the offline tests from the repo root:

```bash
python3 -m pytest tests/ -v
```

Expected right now (before the core is implemented):

```
tests/test_offline.py::test_filter_converges_on_synthetic_run XFAIL
tests/test_offline.py::test_estimate_is_weighted_mean        PASSED
```

`XFAIL` = "expected to fail" because the motion/sensor/resampling code is not
written yet. As Yahya/Umay implement `pf_core/`, that test should flip to
`PASSED`. **You do not need ROS or Gazebo for any of this.**

---

## Part 2 — ROS 2 Humble (Track B)

Official reference (source of truth if anything below drifts):
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 2.1 Set locale (UTF-8)

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2.2 Add the ROS 2 apt repository

```bash
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.3 Install ROS 2 Humble desktop + dev tools

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 2.4 Source ROS (add to your shell so every terminal has it)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.5 Verify

```bash
printenv ROS_DISTRO          # -> humble
ros2 run demo_nodes_cpp talker   # Ctrl+C to stop; should print "Publishing: ..."
```

---

## Part 3 — Gazebo Harmonic (Track B)

> If `gz sim --version` already prints `8.x`, Harmonic is installed — skip to Part 4.

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic
```

Verify:

```bash
gz sim --version     # -> Gazebo Sim, version 8.x
gz sim shapes.sdf    # opens the Gazebo GUI with demo shapes; close it to continue
```

---

## Part 4 — ROS 2 ↔ Gazebo bridge + project tools (Track B)

Humble's *default* Gazebo is Fortress, so install the **Harmonic-specific**
bridge explicitly, plus AprilTag detection and teleop:

```bash
sudo apt install -y \
  ros-humble-ros-gzharmonic \
  ros-humble-apriltag-ros \
  ros-humble-teleop-twist-keyboard
```

> If `ros-humble-ros-gzharmonic` is not found, build `ros_gz` from source with
> `export GZ_VERSION=harmonic` — see https://github.com/gazebosim/ros_gz.

Initialise rosdep (resolves package dependencies when building):

```bash
sudo rosdep init     # ok if it says "already initialized"
rosdep update
```

---

## Part 5 — Build and run the workspace (Track B)

The filter (`pf_core/`) must be importable by the ROS nodes. For now the simplest
way is to put the repo root on `PYTHONPATH` (proper packaging is a TODO):

```bash
# from the repo root:
echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)" >> ~/.bashrc
source ~/.bashrc
```

Build the ROS 2 package and source the overlay:

```bash
# from the repo root
colcon build
source install/setup.bash
```

Run the simulation stack (once the Gazebo world + launch files are filled in):

```bash
ros2 launch pf_ros2 sim.launch.py
```

Useful checks while it runs (new terminal):

```bash
ros2 topic list                 # see camera / odom / cmd_vel topics
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive the robot
```

---

## Part 6 — Real Duckiebot (separate flow)

The robot side uses Duckietown's ROS1 stack reached through `ros1_bridge`; it does
**not** require installing ROS1 on your machine. See
[duckiebot_migration.md](duckiebot_migration.md) for the bridge steps.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ros2: command not found` | You didn't source ROS. `source /opt/ros/humble/setup.bash` (and check it's in `~/.bashrc`). |
| `ModuleNotFoundError: pf_core` | Repo root not on `PYTHONPATH` (Part 5), or you're not running from the repo root. |
| `apt` can't find `ros-humble-*` | The ROS repo/key step (2.2) didn't take. Re-run it, then `sudo apt update`. |
| `gz sim` opens black/crashes | GPU/driver issue. Try `export LIBGL_ALWAYS_SOFTWARE=1` (slow but works), or update graphics drivers. |
| `ros-humble-ros-gzharmonic` missing | Use the source build note in Part 4. |
| Gazebo and ROS versions mismatch | This project is Humble + Harmonic on Ubuntu 22.04 only. Don't mix Fortress/Classic. |

---

## Quick reference: who needs what

| Person | Role | Needs |
|---|---|---|
| Nisa | sim + repo + report | Full Track B (Parts 0–5) |
| Yahya | filter core | Track A (Part 1). Track B only to test in sim. |
| Umay | viz + support | Track A (Part 1); Track B to see RViz/live viz. |
