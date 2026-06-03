from setuptools import find_packages, setup
import os
from glob import glob

package_name = "pf_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    # NOTE: pf_core (the pure-NumPy filter) must be importable at runtime.
    # Simplest for dev: `pip install -e .` from the repo root, or add the repo
    # root to PYTHONPATH. TODO(Nisa): finalise pf_core packaging.
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Nisa",
    maintainer_email="claudekullanirim@gmail.com",
    description="Thin ROS 2 adapter around pf_core.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pf_node = pf_ros2.pf_node:main",
            "apriltag_obs_node = pf_ros2.apriltag_obs_node:main",
            "viz_node = pf_ros2.viz_node:main",
        ],
    },
)
