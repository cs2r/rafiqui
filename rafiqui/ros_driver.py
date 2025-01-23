#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Empty
from ament_index_python.packages import get_package_share_directory
import os
from .robot import Robot

class JointSubscriber(Node):
    def __init__(self, robot):
        super().__init__('Joint_subscriber')
        self.robot = robot
        self.motors = robot.motors

        for motor_name in self.motors.keys():
            self.create_subscription(Int32, f'/{motor_name}/cmd', self.create_callback(motor_name), 1)

        self.create_subscription(Empty, "release_all", self.release_cb, 1)

    def create_callback(self, motor_name):
        def callback(msg):
            motor = self.motors[motor_name]
            motor.set_position(msg.data)
            self.get_logger().info(f'Received command for {motor_name}: {msg.data}')

        return callback

    def release_cb(self):
        print("releasing-----------------------------------")
        self.robot.release_all()

def main(args=None):
    rclpy.init(args=args)
    config_path = os.path.join(get_package_share_directory('rafiqui'), 'config.json')
    my_robot = Robot(config_path)
    ctrl_node = JointSubscriber(my_robot)
    print("ros driver launched")

    try:
        rclpy.spin(ctrl_node)
    except KeyboardInterrupt:
        pass

    my_robot.release_all()
    ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
