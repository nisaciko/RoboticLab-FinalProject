"""Turn camera AprilTag detections into (range, bearing) observations. [Nisa]

Subscribes to AprilTag detections (from apriltag_ros, fed by the camera image)
and republishes a compact per-frame observation list that pf_node consumes.
Crucially it must NOT resolve tag identity — all tags share one ID, so it just
reports each detection's relative geometry and lets the multi-hypothesis sensor
model sum over all tags.

Same node works for sim and Duckiebot: only the input image/detection topic
changes (Duckiebot camera is sensor_msgs/CompressedImage, which bridges cleanly).
"""
import rclpy
from rclpy.node import Node


class AprilTagObsNode(Node):
    def __init__(self):
        super().__init__("apriltag_obs_node")
        # TODO(Nisa):
        #   - subscribe to apriltag detections (or raw image + detect here)
        #   - for each detection compute (range, bearing) in the robot frame
        #   - publish the per-frame observation list for pf_node
        self.get_logger().info("apriltag_obs_node skeleton")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagObsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
