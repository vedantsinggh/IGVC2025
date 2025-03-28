#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/atom/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect edges using Canny
        edges = cv2.Canny(blurred, 50, 150)
        
        # Show only the Canny Edges window
        cv2.imshow("Canny Edges", edges)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

