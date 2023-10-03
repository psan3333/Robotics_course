import sys
import rclpy

from srv_type.srv import SummFullName
from std_msgs.msg import String
from rclpy.node import Node


class ClientName(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(SummFullName, 'summ_full_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SummFullName.Request()

    def send_request(self, last_name, name, surname):
        self.req.last_name = last_name
        self.req.name = name
        self.req.surname = surname
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(
            f'Passed: {last_name}, {name}, {surname}'
        )
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = ClientName()
    response = minimal_client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()