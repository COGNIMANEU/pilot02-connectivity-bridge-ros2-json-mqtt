#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from demo_messages.msg import DemoCommand  # Import the custom message

class DemoCommandSubscriber(Node):
    def __init__(self):
        super().__init__('demo_command_subscriber')
        self.get_logger().info('DemoCommandSubscriber started!')

        # Subscribe to the /democommandmessage topic
        self.subscription = self.create_subscription(
            DemoCommand,
            '/democommandmessage',
            self.callback,
            10
        )

        # Timer to shut down the node after 30 seconds
        self.timer = self.create_timer(30.0, self.shutdown_node)

    def callback(self, msg):
        # Print each field of the DemoCommand message
        self.get_logger().info(f'Received DemoCommand: Command Name: {msg.command_name}  Target Value: {msg.target_value}  Execute Now: {"Yes" if msg.execute_now else "No"}')

    def shutdown_node(self):
        self.get_logger().info('DemoCommandSubscriber shutting down after 30 seconds.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DemoCommandSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
