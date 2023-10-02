import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Text_To_Cmd_Vel(Node):

    def __init__(self, turtle_speed, turtle_angular_speed):
        super().__init__('text_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.speed = turtle_speed
        self.angle = turtle_angular_speed
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.command_callback)

    def command_reaction(self, command):
        message = Twist()
        if command == "turn_left":
            message.angular.z = self.angle
        elif command == "turn_right":
            message.angular.z = -self.angle
        elif command == "move_forward":
            message.linear.x = self.speed
        elif command == "move_backward":
            message.linear.x = -self.speed
        else:
            self.get_logger().info(f"Can't execute {command}.\nCommands list: turn_left, turn_right, move_forward, move_backward.")
        return message

    def command_callback(self):
        command = input('Print your command: ')
        msg = self.command_reaction(command)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    process = Text_To_Cmd_Vel(2.0, np.pi / 2)

    rclpy.spin(process)

    process.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()