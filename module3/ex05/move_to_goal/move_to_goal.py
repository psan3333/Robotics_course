import rclpy
import sys
import numpy as np
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# черепаха: linaer.x - скорость вперед, angular.z - поворот на угол, в радианах
# сначала опубликовать угол, а потом скорость

class MoveToGoal(Node):
    def __init__(self, x, y, theta, turtle_number):
        super().__init__('move_to_goal')
        self.x = x
        self.y = y
        self.theta = theta
        if theta < 0:
            self.theta = 2 * np.pi - theta
        self.not_moved = True
        self.publisher_ = self.create_publisher(Twist, f'/turtle{turtle_number}/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            Pose,
            f'/turtle{turtle_number}/pose',
            self.turtle1_pose_callback,
            1
        )

    def rotation_angle(self):
        x_turtle, y_turtle = self.turtle_pose.x, self.turtle_pose.y
        rotation_angle = self.turtle_pose.theta  # угол, на который черепаха повернута сейчас
        speed_vector = np.array([self.x - x_turtle, self.y - y_turtle])

        basis = np.array([1, 0])
        speed_vector_angle = 2. * np.pi - np.arccos(np.dot(speed_vector, basis) / (np.linalg.norm(speed_vector) * np.linalg.norm(basis)))
        if speed_vector[1] >= 0:
            speed_vector_angle = np.arccos(np.dot(speed_vector, basis) / (np.linalg.norm(speed_vector) * np.linalg.norm(basis)))

        # угол, на который черепаха будет повернута в результате
        turtle_angle = speed_vector_angle
        # сделано так, что сначала якобы вектор basis лежит под углом theta_turtle к оси Ox, 
        # затем мы поворачиваем его так, чтобы он лежал на положительной полуоси Ox, 
        # и находим угол между ним и вектором скорости черепахи
        speed_vector_angle -= rotation_angle
        if speed_vector_angle >= 2 * np.pi:
            speed_vector_angle -= 2 * np.pi
        elif speed_vector_angle < 0:
            speed_vector_angle += 2 * np.pi

        rotation_angle = speed_vector_angle if speed_vector_angle <= np.pi else -(2 * np.pi - speed_vector_angle)
        return rotation_angle, turtle_angle

    def move_turtle_to_coord(self):
        '''
        Алгоритм:
        1. Поввернуть черепашку лицом к точке (x, y)
        2. Направить черепашку в точку (x, y)
        3. Повернуть черепашку на угол theta
        '''
        # calculate new theta
        x_turtle, y_turtle = self.turtle_pose.x, self.turtle_pose.y
        
        speed_vector = np.array([self.x - x_turtle, self.y - y_turtle])
        speed_value = np.linalg.norm(speed_vector) # pass to linear.x of Twist message2
        rotation_angle, turtle_angle = self.rotation_angle() # pass to angular.z of Twist message1

        # rotate
        msg1 = Twist()
        msg1.angular.z = rotation_angle
        self.publisher_.publish(msg1)
        time.sleep(1)
        # move to (x, y) coordinate
        msg2 = Twist()
        msg2.linear.x = speed_value
        self.publisher_.publish(msg2)
        time.sleep(1)
        # rotate to angle theta
        rotate_to_theta_angle = self.theta - turtle_angle
        rotate_to_theta_angle = rotate_to_theta_angle if rotate_to_theta_angle <= np.pi else -(2 * np.pi - rotate_to_theta_angle)
        msg3 = Twist()
        msg3.angular.z = rotate_to_theta_angle
        self.publisher_.publish(msg3)
        self.not_moved = False

    def turtle1_pose_callback(self, msg):
        # position (x, y, z), orientation (x, y, z, w)
        # тут только надо position.x и orientation.z
        self.turtle_pose = msg
        if self.not_moved:
            self.move_turtle_to_coord()
            self.get_logger().info(f'Pose received: {msg.x}, {msg.y}, {msg.theta}')
            self.destroy_node()  # учистить данные о текущей ноде
            sys.exit()  # принудительный выход во избежание вызова новых callback-ов

        
def main(args=None):
    rclpy.init(args=args)

    # аргументы командной строки: x, y - координаты черпахи, theta - угол поворота черепахи относительно горизонта
    x, y, theta = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

    process = MoveToGoal(x, y, theta, 1)

    rclpy.spin(process)

    process.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

