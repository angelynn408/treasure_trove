#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LineFilter:
    def __init__(self):
        self.bridge = CvBridge()
        self.cropped = rospy.Subscriber("/image_cropped", Image, self.Cropped)        #subscribes to cropped image from hw7 /image_cropped
        self.yellow = rospy.Subscriber("/image_yellow", Image, self.Yellow) 
        self.white = rospy.Subscriber("/image_white", Image, self.White)          #subs to /image_white and /image_yellow
        self.edges = rospy.Publisher("/image_edges", Image, queue_size=10)
        self.yellow_lines = rospy.Publisher("/image_yellow_lines", Image, queue_size=10)
        self.white_lines = rospy.Publisher("/image_white_lines", Image, queue_size=10)
        self.all_lines = rospy.Publisher("/image_all_lines", Image, queue_size=10)
       
    def Cropped(self, msg):
        self.crop_img = msg
        self.cv2_cropped = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        self.cropped_edges = cv2.Canny(self.cv2_cropped, 10, 255)
        cropped_edges = self.bridge.cv2_to_imgmsg(self.cropped_edges,"mono8")
        self.edges.publish(cropped_edges)
        
    def Yellow(self, msg):
        yellow = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        self.yellow_edges = np.array(yellow)
        
    def White(self,msg):
        white = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        self.white_edges = np.array(white)
        self.AllEdges()
    
    def AllEdges(self):
        #OpenCV bitwise_and on /image_white and /image_yellow
        yellow_edges = cv2.bitwise_and(self.yellow_edges, self.yellow_edges, mask=self.cropped_edges)      
        white_edges = cv2.bitwise_and(self.white_edges, self.white_edges, mask=self.cropped_edges)
        all_edges = cv2.bitwise_or(yellow_edges, white_edges)
        
        #hsv to rgb
        yellow_rgb = cv2.cvtColor(yellow_edge, cv2.Color_HSV2RGB)
        white_rgb = cv2.cvtColor(white_edge, cv2.Color_HSV2RGB)
        all_rgb = cv2.cvtColor(all_edges, cv2.COLOR_HSV2RGB)
        
        #rgb to gray
        yellow_gray = cv2.cvtColor(yellow_rgb, cv2.COLOR_RGB2GRAY)
        white_gray = cv2.cvtColor(white_rgb, cv2.COLOR_RGB2GRAY)
        all_gray = cv2.cvtColor(all_edges, cv2.COLOR_RGB2GRAY)
        
        #Hough transform
        yellow_hough = cv2.HoughLinesP(yellow_gray, rho=1, theta=np.pi/180, threshold=20, minLineLength=10, maxLineGap=5)
        white_hough = cv2.HoughLinesP(white_gray, rho=1, theta=np.pi/180, threshold=20, minLineLength=10, maxLineGap=5)
        all_hough = cv2.HoughLinesP(all_gray, rho=1, theta=np.pi/180, threshold=20, minLineLength=10, maxLineGap=5)
        
        #line draw
        yellow_line = self.output_lines(self.cv2_cropped, yellow_hough)
        white_line = self.output_lines(self.cv2_cropped, white_hough)
        all_line = self.output_lines(self.cv2_cropped, all_hough)
        
        #cv2 to imgmsg
        yellow_convert = self.bridge.cv2_to_imgmsg(yellow_line,"bgr8")
        white_convert = self.bridge.cv2_to_imgmsg(white_line,"bgr8")
        all_convert = self.bridge.cv2_to_imgmsg(all_line,"bgr8")
        
        #publish images
        self.yellow_lines.publish(yellow_convert)
        self.white_lines.publish(white_convert)
        self.all_lines.publish(all_convert)
       
if __name__=="__main__":
    rospy.init_node("line_filter", anonymous=True)
    LineFilter()
    rospy.spin()
