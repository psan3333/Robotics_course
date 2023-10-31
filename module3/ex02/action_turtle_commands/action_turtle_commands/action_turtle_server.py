import time
import rclpy
import numpy as np
import math

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from message_turtle_commands.action import MessageTurtleCommands


current_turtle_position = Pose()
starting_turtle_position = Pose()

class PositionCapturer(Node):
    def __init__(self):
        super().__init__("turtle1_pose_caturing")
        self.subscriber_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_callback,
            10
        )

    def turtle1_pose_callback(self, msg):
        global current_turtle_position, starting_turtle_position
        current_turtle_position = msg


class ActionTurtleServer(Node):
    def __init__(self):
        super().__init__('action_turtle_server')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            action_type=MessageTurtleCommands,
            action_name='MessageTurtleCommands',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )
        self.goal_position = Pose()

    def calculate_goal_point_forward_backward(self, d):
        global current_turtle_position
        self.goal_position.x = current_turtle_position.x + d * np.cos(current_turtle_position.theta)
        self.goal_position.y = current_turtle_position.y + d * np.sin(current_turtle_position.theta)
        self.goal_position.theta = current_turtle_position.theta

    def calculate_angle_and_speed(self, request):
        command = request.command
        s = float(request.s)
        theta = math.radians(request.angle)
        msg = Twist()

        if command == "forward":
            msg.linear.x = s
            self.calculate_goal_point_forward_backward(s)
        elif command == "backward":
            msg.linear.x = -s
            self.calculate_goal_point_forward_backward(-s)
        elif command == "turn_left":
            msg.angular.z = theta
            self.goal_position.theta += theta
        elif command == "turn_right":
            msg.angular.z = -theta
            self.goal_position.theta -= theta
        return msg

    def positive_angle(self, angle):
        if angle < 0:
            return 2 * np.pi + angle
        return angle

    def has_reached_the_goal(self):
        global current_turtle_position
        return (np.abs(current_turtle_position.x - self.goal_position.x) < 0.1 and
            np.abs(current_turtle_position.y - self.goal_position.y) < 0.1 and
            np.abs(self.positive_angle(current_turtle_position.theta) - self.positive_angle(self.goal_position.theta)) < 0.05)

    def execute_callback(self, goal_handle):
        global current_turtle_position, starting_turtle_position
        self.get_logger().info('Executing goal...')

        while current_turtle_position is None:  # wait for subscriber to receive at least one Pose()
            pass

        starting_turtle_position = Pose(
            x=current_turtle_position.x,
            y=current_turtle_position.y,
            theta=current_turtle_position.theta
        )
        self.goal_position = Pose(
            x=current_turtle_position.x,
            y=current_turtle_position.y,
            theta=current_turtle_position.theta
        )

        feedback_msg = MessageTurtleCommands.Feedback()
        result = MessageTurtleCommands.Result()

        turtle_movement_message = self.calculate_angle_and_speed(goal_handle.request)
        self.publisher_.publish(turtle_movement_message)

        while not self.has_reached_the_goal():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.result = False
                return result
            current_odometry = np.sqrt((current_turtle_position.x - starting_turtle_position.x) ** 2 + 
                                        (current_turtle_position.y - starting_turtle_position.y) ** 2)
            feedback_msg.odom = int(current_odometry)
            # goal_handle.publish_feedback(feedback_msg)
            # print(f"Position: {current_turtle_position.x} {current_turtle_position.y} {current_turtle_position.theta}")
            # print()
            # print(f"Goal Position: {self.goal_position.x} {self.goal_position.y} {self.goal_position.theta}")
            # print()
            # print(f"Starting Position: {starting_turtle_position.x} {starting_turtle_position.y} {starting_turtle_position.theta}")
            # print()
            time.sleep(0.2)

        goal_handle.succeed()
        starting_turtle_position = current_turtle_position

        result.result = True
        return result
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    try:
        action_turtle_server = ActionTurtleServer()
        pos_capturer = PositionCapturer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(action_turtle_server)
        executor.add_node(pos_capturer)

        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            action_turtle_server.destroy_node()
            pos_capturer.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()