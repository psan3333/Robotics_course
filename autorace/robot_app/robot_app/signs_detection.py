import rclpy
import cv2
import os
import sys
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
            0: ('traffic_intersection.png', 12), 
            1: ('traffic_construction.png', 24), 
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
        if self.current_sign_number > len(self.signs_order.keys()):
            print('All of the signs were successfully detected!')
            print('Detection execution ends.')
            sys.exit()
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

    def check_dst_points(self, area_points):
        x_y_adjacent_vertexes_diff = np.array([
            np.abs(area_points[0, 0] - area_points[1, 0]),
            np.abs(area_points[0, 1] - area_points[2, 1]),
            np.abs(area_points[2, 0] - area_points[3, 0]),
            np.abs(area_points[1, 1] - area_points[3, 1])
        ])
        return False if np.max(x_y_adjacent_vertexes_diff) > 30 else True

    def find_dst_area(self, area_points):
        area_points = area_points[np.argsort(area_points[:, 0])]
        if area_points[2, 1] > area_points[3, 1]:
            temp = area_points[2].copy()
            area_points[2] = area_points[3]
            area_points[3] = temp
        if area_points[0, 1] > area_points[1, 1]:
            temp = area_points[0].copy()
            area_points[0] = area_points[1]
            area_points[1] = temp
        if not self.check_dst_points(area_points):
            print('wrong matches detected and eliminated (bad shape)')
            raise Exception()
        a1, b1, c1 = np.linalg.norm(area_points[0] - area_points[1]), np.linalg.norm(area_points[1] - area_points[2]), np.linalg.norm(area_points[0] - area_points[2])
        a2, b2, c2 = np.linalg.norm(area_points[1] - area_points[2]), np.linalg.norm(area_points[2] - area_points[3]), np.linalg.norm(area_points[3] - area_points[1])
        p1 = (a1 + b1 + c1) / 2
        p2 = (a2 + b2 + c2) / 2
        return np.sqrt(p1 * (p1 - a1) * (p1 - b1) * (p1 - c1)) + np.sqrt(p2 * (p2 - a2) * (p2 - b2) * (p2 - c2))

    def find_traffic_sign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        cv_image_input = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # cv_image_input = cv2.cvtColor(cv_image_input, cv2.COLOR_BGR2RGB)
        cv_image_input = cv2.cvtColor(cv_image_input, cv2.COLOR_BGR2GRAY)
        if self.current_sign_number in [2, 3]:
            cv_image_input = cv_image_input[:, cv_image_input.shape[1]//2:]
        elif self.current_sign_number == 0:
            cv_image_input = cv_image_input[:, cv_image_input.shape[1]-400:]

        cv2.imshow('picture', cv_image_input)

        try:

            # find the keypoints and descriptors with SIFT
            input_kp, input_des = self.orb.detectAndCompute(cv_image_input,None)

            matches = self.brute_force_macth.knnMatch(self.curr_des, input_des, k=2)

            good = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good.append(m)

            if len(good)>self.min_match_count:
                print(f'found matches: {len(good)}')
                src_pts = np.float32([ self.curr_kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ input_kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                h, w = self.current_sign.shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M).reshape(4, 2)
                found_matching_area = self.find_dst_area(dst)
                pattern_area = self.find_dst_area(pts.reshape(4, 2))
                print(found_matching_area, pattern_area)
                if pattern_area / 2 > found_matching_area:
                    print('wrong matches detected and eliminated (to low area)')
                    raise Exception()
                msg_sign_number = UInt8()
                msg_sign_number.data = self.current_sign_number
                self.sign_number_publisher.publish(msg_sign_number)
                self.current_sign_number += 1
                self.detect_and_compute_current()
                print('Waiting 5 seconds')
                cv2.waitKey(5000)
                print('End waiting')
                print()
            else:
                print(f'matches amount: {len(good)}')
                
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

