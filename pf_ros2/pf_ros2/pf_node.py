"""ROS 2 node that drives pf_core.ParticleFilter.  [Owner: Nisa]

This is the THIN adapter. It contains no filter math — it only:
  - subscribes to the motion input (odometry / nav_msgs/Odometry)
  - subscribes to tag observations (from apriltag_obs_node)
  - calls pf.predict() / pf.update() / pf.resample()
  - publishes the pose estimate and the particle cloud (for viz_node)

Migration sim -> Duckiebot = only the subscribed topic names change (set via
parameters / launch remap). The filter core does not change.
"""
import rclpy
from rclpy.node import Node

# from pf_core.particle_filter import ParticleFilter
# from pf_core.tag_map import TagMap


class PfNode(Node):
    def __init__(self):
        super().__init__("pf_node")
        # TODO(Nisa):
        #   - declare params: num_particles, motion/meas noise, topic names,
        #     room bounds, tag map source
        #   - build TagMap + ParticleFilter, initialize_uniform(room bounds)
        #   - create subscriptions (odom, observations) + publishers
        #     (pose estimate, particle cloud as visualization_msgs/MarkerArray
        #      or geometry_msgs/PoseArray)
        self.get_logger().info("pf_node skeleton — wire up pf_core here")


def main(args=None):
    rclpy.init(args=args)
    node = PfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
