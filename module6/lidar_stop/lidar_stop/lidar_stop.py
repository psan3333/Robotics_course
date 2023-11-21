import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.subscriber_
        self.__padding = 2

    def listener_callback(self, msg: LaserScan):
        msg_to_send = Twist()
        msg_to_send.linear.x = msg.ranges[math.ceil((msg.angle_max - msg.angle_min) / 2 / msg.angle_increment)] - self.__padding
        self.publisher_.publish(msg_to_send)


def main(args=None):
    rclpy.init(args=args)

    process = LidarPublisher()

    rclpy.spin(process)

    process.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()