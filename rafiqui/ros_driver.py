import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from robot import Robot

class JointSubscriber(Node):
    def __init__(self, motors):
        super().__init__('Joint_subscriber')
        self.motors = motors

        for motor_name in self.motors.keys():
            self.create_subscription(Int32, f'/{motor_name}/cmd', self.create_callback(motor_name), 1)

    def create_callback(self, motor_name):
        def callback(msg):
            motor = self.motors[motor_name]
            motor.set_position(msg.data)
            self.get_logger().info(f'Received command for {motor_name}: {msg.data}')

        return callback

def main(args=None):
    rclpy.init(args=args)
    my_robot = Robot("config.json")
    ctrl_node = JointSubscriber(my_robot.motors)
    print("ros driver launched")

    try:
        rclpy.spin(ctrl_node)
    except KeyboardInterrupt:
        pass

    ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
