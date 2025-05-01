#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BasicMover(Node):
    def __init__(self):
        super().__init__('basic_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)
        self.state = 0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def move(self):
        msg = Twist()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = current_time - self.start_time

        # Etapas de movimento
        if self.state == 0:  # Andar para frente 3s
            if elapsed < 3:
                msg.linear.x = 0.2
            else:
                self.state = 1
                self.start_time = current_time

        elif self.state == 1:  # Parar 1s
            if elapsed < 1:
                msg.linear.x = 0.0
            else:
                self.state = 2
                self.start_time = current_time

        elif self.state == 2:  # Girar 2s
            if elapsed < 2:
                msg.angular.z = 0.5
            else:
                self.state = 0
                self.start_time = current_time

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = BasicMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
