# Visualization  [Owner: Umay]

Rendering of the live run (see `pf_ros2/pf_ros2/viz_node.py` for the ROS node).
Standalone plotting helpers (e.g. matplotlib for report figures) can live here so
they are reusable both online (RViz/live) and offline (from recorded data).

Must show in one view: room + tags, particle cloud (coloured by weight),
odometry-only trajectory, and filter pose-estimate trajectory.
