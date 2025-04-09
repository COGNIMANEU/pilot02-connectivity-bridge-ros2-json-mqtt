#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from demo_messages.msg import DemoState  # Import custom message
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import time

class DemoStatePublisher(Node):
    def __init__(self):
        super().__init__('demo_state_publisher')
        self.get_logger().info('DemoStatePublisher started!')

        # Create publisher for the /demomsg topic
        self.publisher = self.create_publisher(DemoState, '/demostatemessage', 10)

        self.counter = 0
        self.max_messages = 10  # Publish 10 times (once per second)
        self.timer = self.create_timer(1.0, self.publish_message)  # 1-second interval

    def publish_message(self):
        if self.counter < self.max_messages:
            msg = DemoState()

            # Fill in simple fields
            msg.is_active = True
            msg.error_code = 0
            msg.battery_level = 87.5
            msg.status_message = f'Status update {self.counter + 1}'

            # Fill in complex fields
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.position = Point()
            msg.position.x = float(self.counter)
            msg.position.y = float(self.counter) * 0.5
            msg.position.z = 1.0

            # Fill in an array (list of integers)
            msg.sensor_readings = [10 + self.counter, 20 + self.counter, 30 + self.counter]

            self.get_logger().info(f'Publishing DemoState message: {msg}')

            # Publish the message
            self.publisher.publish(msg)
            self.get_logger().info(f'Published DemoState message #{self.counter + 1}')
            self.counter += 1
        else:
            self.get_logger().info('Finished publishing DemoState messages.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DemoStatePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
