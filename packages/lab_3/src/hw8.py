#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class HoughTransform:
    def __init__(self):
        self.bridge = CvBridge()
        self.crop = rospy.Publisher("/image_cropped", Image, queue_size=10)        
        self.yellow = rospy.Publisher("/image_yellow", Image, queue_size = 10) 
        self.white = rospy.Publisher("/image_white", Image, queue_size = 10)
        cv2.HoughLinesP(image, rho, theta, threshold[, lines[, minLineLength[, maxLineGap}}})
        
        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
if __name__=="__main__":
    rospy.init_node("hough_transform", anonymous=True)
    HoughTransform()
    rospy.spin()
