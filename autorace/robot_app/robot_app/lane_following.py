import rclpy
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import UInt8
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SignDetection(Node):
    def __init__(self):
        super().__init__('sing_detector_node')
        self.signs_order = {
            0: ('traffic_intersection.png', 16), 
            1: ('traffic_construction.png', 30), 
            2: ('parking_lot.png', 10), 
            3: ('pedestrian_crossing.png', 5), 
            4: ('tunnel.png', 10)
        }
        self.current_sign_number = 0
        self.prepare_detector()

        self.image_camera_subscription = self.create_subscription(
            Image,
            '/color/image',
            self.find_traffic_sign,
            10
        )
        self.image_camera_subscription

        self.sign_number_publisher = self.create_publisher(
            UInt8,
            '/sign_detection',
            10
        )
        self.cv_bridge = CvBridge()
        self.frame = None

    def detect_and_compute_current(self):
        self.min_match_count = self.signs_order[self.current_sign_number][1]
        self.current_sign = cv2.imread(self.signs_path + '/' + self.signs_order[self.current_sign_number][0], 0)
        # self.current_sign = cv2.cvtColor(self.current_sign, cv2.COLOR_BGR2RGB)
        self.curr_kp, self.curr_des = self.orb.detectAndCompute(self.current_sign, None)

    def prepare_detector(self):
        # Initiate SIFT detector
        self.orb = cv2.ORB_create()

        self.signs_path = os.path.dirname(os.path.realpath(__file__)) + '/signs_to_detect'
        print(os.listdir(os.path.dirname(os.path.realpath(__file__))))
        self.sign_names = os.listdir(self.signs_path)
        self.detect_and_compute_current()
        self.brute_force_macth = cv2.BFMatcher()

    def MSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
        err = sum / num_all
        return err

    def find_traffic_sign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.

        cv_image_input = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # cv_image_input = cv2.cvtColor(cv_image_input, cv2.COLOR_BGR2RGB)
        cv_image_input = cv2.cvtColor(cv_image_input, cv2.COLOR_BGR2GRAY)
        if self.current_sign_number in [2, 3]:
            cv_image_input = cv_image_input[:, cv_image_input.shape[1]-300:]
        elif self.current_sign_number == 0:
            cv_image_input = cv_image_input[:, cv_image_input.shape[1]-400:]

        try:

            # find the keypoints and descriptors with SIFT
            _, input_des = self.orb.detectAndCompute(cv_image_input,None)

            matches = self.brute_force_macth.knnMatch(self.curr_des, input_des, k=2)

            good = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good.append(m)

            if len(good)>self.min_match_count:
                print(f'found matches: {len(good)}')
                # msg_sign_number = UInt8()
                # msg_sign_number.data = self.current_sign_number
                # self.sign_number_publisher.publish(msg_sign_number)
                self.current_sign_number += 1
                self.detect_and_compute_current()
                cv2.waitKey(5000)
                
        except Exception:
            pass
            
    #         msg_sign_number = UInt8()
    #         msg_sign_number.data = self.current_sign_number
    #         self.sign_number_publisher.publish(msg_sign_number)
    #         # self.current_sign_number += 1
        cv2.waitKey(100)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetection()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

