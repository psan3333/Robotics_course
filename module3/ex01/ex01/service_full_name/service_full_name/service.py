import rclpy

from srv_type.srv import SummFullName
from std_msgs.msg import String
from rclpy.node import Node


class ServiceName(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'summ_full_name', self.service_callback)

    def service_callback(self, request, response):
        response.full_name = f'{request.last_name} {request.name} {request.surname}'
        self.get_logger().info(f'Message received: {response.full_name}')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = ServiceName()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()