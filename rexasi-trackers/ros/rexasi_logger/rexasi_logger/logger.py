import os
import rclpy
from rclpy.node import Node

from rexasi_tracker_msgs.msg import Tracks


class Logger(Node):
    """
    Code: https://github.com/tryolabs/norfair
    Documentation: https://tryolabs.github.io/norfair/2.2/reference/
    """

    def __init__(self):
        super().__init__(
            "LOGGER", automatically_declare_parameters_from_overrides=True
        )

        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.output_folder = self.get_parameter("output_folder").get_parameter_value().string_value
        self.tracks_topic = self.get_parameter("tracks_topic").get_parameter_value().string_value

        output_path = os.path.join(self.output_folder, "tracker_output.csv")
        self.output_file = open(output_path, "w")
        self.output_file.write("id;x;y;timestamp\n")
        self.get_logger().info(f"Output file {output_path} opened")

        # SUBSCRIPTIONS
        self.subscriber = self.create_subscription(
            Tracks, self.tracks_topic, self.msg_event, 10
        )
        self.get_logger().info(f"Subscribed to {self.tracks_topic}")

    def msg_event(self, track_data):
        for idx in range(len(track_data.centers)):
            self.output_file.write(
               f"{track_data.identities[idx]};{track_data.centers[idx].position.x};{track_data.centers[idx].position.y};{track_data.header.stamp.sec}{track_data.header.stamp.nanosec:09d}\n"
            )
        self.output_file.flush()

def main(args=None):
    rclpy.init(args=args)
    logger = Logger()
    rclpy.spin(logger)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    logger.destroy_node()
    rclpy.shutdown()

