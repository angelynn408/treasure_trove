#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageFlipper:
    def __init__(self):
        self.bridge = CvBridge()
        self.crop = rospy.Publisher("/image_cropped", Image, queue_size=10)        
        self.yellow = rospy.Publisher("/image_yellow", Image, queue_size = 10) 
        self.white = rospy.Publisher("/image_white", Image, queue_size = 10)
        self.sub = rospy.Subscriber("/image", Image, self.crop)
        self.erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        self.dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        
    def crop(self, msg):
        ros_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ros
        y_len = len(cv_img)
        x_len = len(cv_img[0])
        
        bottom_half = cvg_img[y_len/2:y_len, 0:x_len]
        cv_hsv = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(cv_hsv, (0,0,0), (180,25,255))
        yellow = cv2.inRange(cv_hsv, (30,100,150),(40,255,255))
        
        white = cv2.erode(white, self.erode)
        yellow = cv2.erode(yellow, self.erode)
        white = cv2.dilate(white, self.dilate)
        yellow = cv2.dilate(yellow, self.dilate)
        
        white = cv2.bitwise_and(bottom_half, bottom_half, mask=white) #mask
        yellow = cv2.bitwise_and(bottom_half, bottom_half, mask=yellow) #mask
        cv_ros = self.bridge.cv2_to_imgmsg(bottom_half, "bgr8")
        white = self.bridge.cv2_to_imgmsg(white, "bgr8")
        yellow = self.bridge.cv2_to_imgmsg(yellow, "bgr8")
        
        self.bottom_half.publish(cv_ros)
        mask_white = self.bridge.cv2_to_imgmsg(white, "bgr8")
        mask_yellow = self.bridge.cv2_to_imgmsg(yellow, "bgr8")
        self.white.publish(mask_white)
        self.yellow.publish(mask_yellow)
        
if __name__=="__main__":
    rospy.init_node("image_flipper", anonymous=True)
    ImageFlipper()
    rospy.spin()
