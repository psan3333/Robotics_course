import sys
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from message_turtle_commands.action import MessageTurtleCommands


class ActionTurtleClient(Node):

    def __init__(self, commands):
        super().__init__('action_turtle_client')
        self.command_parameters = {
            'forward': [2, 0],
            'backward': [2, 0],
            'turn_left': [0, 90],
            'turn_right': [0, 90]
        }
        self.commands = commands
        if commands == []:
            self.get_logger().info("No commands written")
            raise Exception()
        else:
            for command in self.commands:
                if command not in self.command_parameters.keys():
                    self.get_logger().info(f"Command {command} is not available.\nAvailable commands: {self.command_parameters.keys()}")
                    raise Exception()
        self.command_number = 0
        self._action_client = ActionClient(self, MessageTurtleCommands, 'MessageTurtleCommands')

    def send_goal(self):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = self.commands[self.command_number]
        goal_msg.s = self.command_parameters[goal_msg.command][0]
        goal_msg.angle = self.command_parameters[goal_msg.command][1]

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('server is not available, still waiting...')

        self.command_number += 1
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Is action completed: {0}'.format(result.result))
        if self.command_number < len(self.commands):
            self.send_goal()
        elif self.command_number >= len(self.commands):
            sys.exit()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: odometry = {0}'.format(feedback.odom))



def main(args=None):
    rclpy.init(args=args)
    try:
        action_client = ActionTurtleClient(['forward', 'turn_left', 'forward', 'backward', 'turn_left', 'forward', 'turn_right'])
        action_client.send_goal()

        rclpy.spin(action_client)
    except:
        print("Shutting down an action client.")
    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()