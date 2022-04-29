#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import SegmentList, Segment

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
        
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        cropped_image = new_image[offset:, :]

        cv_hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(cv_hsv, (0,0,0), (180,25,255))
        yellow = cv2.inRange(cv_hsv, (30,100,150),(40,255,255))
        
        white = cv2.erode(white, self.kernel)
        yellow = cv2.erode(yellow, self.kernel)
        white = cv2.dilate(white, self.kernel)
        yellow = cv2.dilate(yellow, self.kernel)
        
        white = cv2.bitwise_and(cropped_image, cropped_image, mask=white) #mask
        yellow = cv2.bitwise_and(cropped_image, cropped_image, mask=yellow) #mask
           
        self.cropped_edges = cv2.Canny(cropped_image, 10, 255)
        
        yellow = cv2.dilate(yellow,self.kernel)
        self.yellow_edges = np.array(yellow)
        
        white = cv2.dilate(white,self.kernel)
        self.white_edges = np.array(white)
        
        yellow_edges = cv2.bitwise_and(self.yellow_edges, self.yellow_edges, mask=self.cropped_edges)      
        white_edges = cv2.bitwise_and(self.white_edges, self.white_edges, mask=self.cropped_edges)

        #hsv to rgb
        yellow_rgb = cv2.cvtColor(yellow_edges, cv2.COLOR_HSV2RGB)
        white_rgb = cv2.cvtColor(white_edges, cv2.COLOR_HSV2RGB)
  
        #rgb to gray
        yellow_gray = cv2.cvtColor(yellow_rgb, cv2.COLOR_RGB2GRAY)
        white_gray = cv2.cvtColor(white_rgb, cv2.COLOR_RGB2GRAY)
        
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[0], 1. / image_size[1], 1. / image_size[0], 1. / image_size[1]])
        
        #Hough transform
        yellow_hough = cv2.HoughLinesP(yellow_gray, rho=1, theta=np.pi/180, threshold=10, minLineLength=10, maxLineGap=5)
        white_hough = cv2.HoughLinesP(white_gray, rho=1, theta=np.pi/180, threshold=10, minLineLength=10, maxLineGap=5)
        
        if white_hough is not None:
            line_normalized_white = (white_hough + arr_cutoff) * arr_ratio
            for i in line_normalized_white:
                segment = Segment()
                segment.color = 0
                segment.pixels_normalized[0].x = i[0][0]
                segment.pixels_normalized[0].y = i[0][1]
                segment.pixels_normalized[1].x = i[0][2]
                segment.pixels_normalized[1].y = i[0][3]
                segmentlist.segments.append(segment)

                
        if yellow_hough is not None:
            line_normalized_yellow = (yellow_hough + arr_cutoff) * arr_ratio
            for i in line_normalized_yellow:
                segment = Segment()
                segment.color = 1
                segment.pixels_normalized[0].x = i[0][0]
                segment.pixels_normalized[0].y = i[0][1]
                segment.pixels_normalized[1].x = i[0][2]
                segment.pixels_normalized[1].y = i[0][3]
                segmentlist.segments.append(segment)
        
        #line draw
        yellow_line = self.output_lines_yellow(cropped_image, yellow_hough)
        white_line = self.output_lines_white(yellow_line, white_hough)
     
        
        #cv2 to imgmsg
        white_convert = self.bridge.cv2_to_imgmsg(white_line,"bgr8")
        self.line_overlay.publish(white_convert)
      
        if len(segmentlist.segments) != 0:
            self.line_segments.publish(segmentlist)
        
    def output_lines_white(self,original_image,lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range (len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,255,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
    def output_lines_yellow(self,original_image,lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range (len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,255,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
if __name__=="__main__":
    rospy.init_node("lane_detector", anonymous=True)
    LaneDetector()
    rospy.spin()
