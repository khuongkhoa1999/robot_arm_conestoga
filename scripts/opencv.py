#!/usr/bin/env python

import rospy
import cv2
from rospy.core import rospyinfo
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np


class OpenCv(object):
    def __init__(self):
        
        self.bridge = CvBridge()
        self.lower_color_bounds = np.array([17, 15, 100])
        self.upper_color_bounds = np.array([50, 56, 200])

        # Publishers
        self.pub = rospy.Publisher('/opencv/image_raw', Image,queue_size=10)
        self.pub2 = rospy.Publisher('/opencv/coordinates', Float32MultiArray,queue_size=1)
        # Subscribers
        rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        

    def callback(self, ros_image):
        #convert ros_image into an opencv-compatible image
        try:
            #rospy.loginfo('Image received...')
            self.image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
            rospy.logfatal(e)
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        
        # Draw rectangle around the faces
        #for (x, y, w, h) in :
        #    cv2.rectangle(self.image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        #mask = cv2.inRange(self.image,self.lower_color_bounds,self.upper_color_bounds )
        #output = cv2.bitwise_and(self.image,self.image, mask = mask)
        #if output is not None:
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.image ,"bgr8"))
        #print("opencv")
        coordinates_msg = Float32MultiArray()
        coordinates_msg.data = [0,0.25]
        self.pub2.publish(coordinates_msg)


def main(args):
    rospy.init_node('opencv', anonymous=True)
    opencv = OpenCv()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
