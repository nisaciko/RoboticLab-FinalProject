"""Real-time visualization node.  [Owner: Umay]

Must show, in ONE view (RViz2 or matplotlib):
  - the room and the AR tag positions
  - the particle cloud (all particles, coloured by weight)
  - the odometry-only trajectory (robot's path if it trusted odometry alone)
  - the particle-filter pose-estimate trajectory (weighted mean over time)

Showing odometry vs filter estimate side by side makes the filter's benefit
visible (required by the assignment).
"""
import rclpy
from rclpy.node import Node


class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")
        # TODO(Umay): subscribe to particle cloud + pose estimate + raw
        # odometry; render tags/room/cloud/trajectories. RViz2 MarkerArray is
        # the easiest path; matplotlib also fine for screenshots.
        self.get_logger().info("viz_node skeleton")


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
