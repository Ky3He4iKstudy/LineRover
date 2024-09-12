#! /usr/bin/env python

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pid_controller import PID

# Constants
FULL_SPEED = 0.2
SLOW_SPEED = 0.01
MAX_ROT_SPEED = 5

SCANLINE = 700  # scan only last 100 rows on camera
Kp = 2.0
Kd = 0.05
Ki = 0.03

class SimpleMover:
    def __init__(self):
        self.target_angle = 0  # -1 to 1
        
        self.bridge = CvBridge()
        rospy.init_node('move_robot')
        rospy.Subscriber('/robot/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.pid = PID(Kp, Ki, Kd, (-1, 1))

    def spin(self):
        rospy.sleep(1)  # move after full init
        time_prev = rospy.get_time()
        while not rospy.is_shutdown():
            try:
                t = rospy.get_time()
                dt = t - time_prev
                time_prev = t
                if dt == 0:
                    dt = 0.03
                elif dt > 0.1:
                    dt = 0.1
                
                move = Twist()
                # get target direction [-1..1]
                ang = self.pid(self.target_angle, dt)
                # set angular and linear speed
                move.angular.z = -ang * MAX_ROT_SPEED
                # move faster on straight lines
                move.linear.x = FULL_SPEED * (1 - abs(ang)) + SLOW_SPEED
                #print(self.target_angle, ang, move.angular.z, move.linear.x)
                
                self.pub.publish(move)
                self.rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                break
            except KeyboardInterrupt:
                break
            except BaseException as e:
                print(e)
        self.shutdown()

    def shutdown(self):
        self.pub.publish(Twist())
        rospy.sleep(1)
        
    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        
        grey_image = cv2.cvtColor(cv_image[SCANLINE:], cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 15, 255, cv2.THRESH_BINARY)
        cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR).THRESH_BINARY_INV)

        y, x = self.center_of_mass(255 - mask)
        y += SCANLINE
        #print(self.target_angle, x, y)

        if not (np.isnan(x) or np.isnan(y)):
            self.target_angle = np.clip(2 * x / msg.width - 1, -1, 1)
            cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), 3)
            cv2.line(cv_image, (0, SCANLINE), (msg.width, SCANLINE), 
                (255, 0, 0), 1)
        # no center found -> no target_angle change
        self.show_image(cv_image, 'Line')
        
    @staticmethod
    def show_image(img, title):
        cv2.imshow(title, img)
        cv2.waitKey(3)
        
    @staticmethod
    def center_of_mass(data):
        normalizer = np.sum(data, None, None)
        grids = np.ogrid[[slice(0, i) for i in data.shape]]
        return [np.sum(data * grids[dir].astype(float), None, None) / normalizer
                   for dir in range(data.ndim)]
        

if __name__ == '__main__':
    mover = SimpleMover()
    mover.spin()
