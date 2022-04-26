#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs import SegmentList, Segment

class LaneDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.line_overlay = rospy.Publisher("/image_lines", Image, queue_size=10)        
        self.line_segments = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)
        self.sub = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.Overlay, queue_size=1, buff_size=2**24)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)) 
        
    def Overlay(self, msg):
        segment = Segment()
        segmentlist = SegmentList()
        
        cv_img = self.bridge.compressedimgmsg_to_cv2(msg, "bgr8")
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(old_image, image_size, interpolation=cv2.INTER_NEAREST)
        cropped_image = new_image[offset:, :]
        
              
        
        cv_hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(cv_hsv, (0,0,0), (180,25,255))
        yellow = cv2.inRange(cv_hsv, (30,100,150),(40,255,255))
        
        white = cv2.erode(white, self.erode)
        yellow = cv2.erode(yellow, self.erode)
        white = cv2.dilate(white, self.dilate)
        yellow = cv2.dilate(yellow, self.dilate)
        
        white = cv2.bitwise_and(cropped_image, cropped_image, mask=white) #mask
        yellow = cv2.bitwise_and(cropped_image, cropped_image, mask=yellow) #mask
        cv_ros = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")

        
        self.crop.publish(cv_ros)
        mask_white = self.bridge.cv2_to_imgmsg(white, "bgr8")
        mask_yellow = self.bridge.cv2_to_imgmsg(yellow, "bgr8")
        self.white.publish(mask_white)
        self.yellow.publish(mask_yellow)
       
    def Cropped(self, msg):
        self.crop_img = msg
        self.cv2_cropped = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        self.cropped_edges = cv2.Canny(self.cv2_cropped, 10, 255)
        cropped_edges = self.bridge.cv2_to_imgmsg(self.cropped_edges,"mono8")
        self.edges.publish(cropped_edges)
        
    def Yellow(self, msg):
        yellow = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        yellow = cv2.dilate(yellow,self.dilate)
        self.yellow_edges = np.array(yellow)
        
    def White(self,msg):
        white = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        white = cv2.dilate(white,self.dilate)
        self.white_edges = np.array(white)
        self.AllEdges()
    
    def AllEdges(self):
        #OpenCV bitwise_and on /image_white and /image_yellow
        yellow_edges = cv2.bitwise_and(self.yellow_edges, self.yellow_edges, mask=self.cropped_edges)      
        white_edges = cv2.bitwise_and(self.white_edges, self.white_edges, mask=self.cropped_edges)
        all_edges = cv2.bitwise_or(yellow_edges, white_edges)
        
        #hsv to rgb
        yellow_rgb = cv2.cvtColor(yellow_edges, cv2.COLOR_HSV2RGB)
        white_rgb = cv2.cvtColor(white_edges, cv2.COLOR_HSV2RGB)
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
        
    def output_lines(self,original_image,lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range (len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
if __name__=="__main__":
    rospy.init_node("lane_detector", anonymous=True)
    LaneDetector()
    rospy.spin()
