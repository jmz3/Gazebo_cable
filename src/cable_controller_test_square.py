#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import Wrench, Point
import numpy as np
import time


class CableController(Node):
    def __init__(self):
        super().__init__('cable_controller')
        
        # Create a publisher to publish the target position
        self.pub = self.create_publisher(Point, 'cable_target_position', 10)
        
        # Wait for the publisher to be ready
        time.sleep(1.0)
        
    def move_end_sphere(self):
        '''
        Move the end sphere to the target position
        '''
        target_position = Point()
        offset = 0.4

        # Go to the first target position
        target_position.x = 0.2
        target_position.y = 0.6
        target_position.z = 2.0
        self.pub.publish(target_position)
        self.get_logger().info(f'Publishing target: x={target_position.x}, y={target_position.y}, z={target_position.z}')

        # Wait for the end sphere to reach the target position
        time.sleep(2)

        # Go to the second target position
        target_position.x = 0.2
        target_position.y = 0.3
        target_position.z = 2.0
        self.pub.publish(target_position)
        self.get_logger().info(f'Publishing target: x={target_position.x}, y={target_position.y}, z={target_position.z}')

        # Wait for the end sphere to reach the target position
        time.sleep(2)

        # Go to the third target position
        target_position.x = -0.2
        target_position.y = 0.3
        target_position.z = 2.0
        self.pub.publish(target_position)
        self.get_logger().info(f'Publishing target: x={target_position.x}, y={target_position.y}, z={target_position.z}')

        # Wait for the end sphere to reach the target position
        time.sleep(2)

        # Go to the fourth target position
        target_position.x = -0.2
        target_position.y = 0.6
        target_position.z = 2.0
        self.pub.publish(target_position)
        self.get_logger().info(f'Publishing target: x={target_position.x}, y={target_position.y}, z={target_position.z}')

        # Wait for the end sphere to reach the target position
        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    
    cable_controller = CableController()
    
    try:
        while rclpy.ok():
            cable_controller.move_end_sphere()
            rclpy.spin_once(cable_controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cable_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
