#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class OdometeryPublisher(Node):

    def __init__(self):
        super().__init__("odometery_publisher")
        self.get_logger().info("Hello world from the Python node odometery_publisher")


def main(args=None):
    rclpy.init(args=args)

    odometery_publisher = OdometeryPublisher()

    try:
        rclpy.spin(odometery_publisher)
    except KeyboardInterrupt:
        pass

    odometery_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
