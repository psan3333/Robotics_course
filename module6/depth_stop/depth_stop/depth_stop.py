import rclpy
import struct
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class DepthPublisher(Node):

    def __init__(self):
        super().__init__('depth_publisher')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            Image,
            '/diff_drive/depth/depth_image',
            self.listener_callback,
            10
        )
        self.subscriber_
        self.__padding = 2

    def listener_callback(self, msg: Image):
        msg_to_send = Twist()
        center = 4 * (msg.height // 2 * msg.width + msg.width // 2)
        distance = struct.unpack('f', msg.data[center:center + 4])[0]
        msg_to_send.linear.x = distance - self.__padding
        if distance - self.__padding < 0:
            msg_to_send.linear.x = 0.0
        self.publisher_.publish(msg_to_send)


def main(args=None):
    rclpy.init(args=args)

    process = DepthPublisher()

    rclpy.spin(process)

    process.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()