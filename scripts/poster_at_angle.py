import rospy
import yaml
import os
import sys
import cv2
import numpy as np 
import rospy 
import time
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int32, Bool, Int32MultiArray
from typing import List, Union
from collections import Counter


class PosterAtAngle():
    def __init__(self, camera):
        self.camera = camera
        self.contour = None
        self.vertical_lines_image = None
        self.image_pd_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        
    def convert_to_grayscale(self, cv_image):
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        return cv_image_gray
        
    def canny_edge_detection(self, cv_image_gray, lower=0, upper=0):
        cv_image_gray = cv_image_gray.astype(np.uint8)
        cv_image_edges = cv2.Canny(cv_image_gray, lower, upper)
        return cv_image_edges
        
    def pipeline(self, cv_image):
        cv_image_blur_gray = self.convert_to_grayscale(cv_image)
        cv_image_blur_edges = self.canny_edge_detection(cv_image_blur_gray, lower=170, upper=170)
        contours, hierarchy = cv2.findContours(cv_image_blur_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv_image_blur_edges = cv2.cvtColor(cv_image_blur_edges, cv2.COLOR_GRAY2RGB)
        return contours, cv_image_blur_edges
    
    def calc_left_right_white_pixels(self):
        binary_image = self.vertical_lines_image
        height, width = binary_image.shape[:2]
        left_half = binary_image[:, :width // 2]
        right_half = binary_image[:, width // 2:]
        
        left_pixel_count = cv2.countNonZero(left_half)
        right_pixel_count = cv2.countNonZero(right_half)
        
        return left_pixel_count, right_pixel_count
        
    def get_left_right_white_pixels(self):
        if self.vertical_lines_image is None: 
            return
        results = []
        for _ in range(10000):
            lp, rp = self.calc_left_right_white_pixels()
            results.append((lp,rp))
            
        lp_avg = int(sum(pair[0] for pair in results)/ len(results))
        rp_avg = int(sum(pair[1] for pair in results)/ len(results))
        
        avg_pair = (lp_avg, rp_avg)
            
        return avg_pair
    
    def callback(self, data):
        contours, cv_image_blur_edges = self.pipeline(self.camera.cv_image)
        contour = max(contours, key=cv2.contourArea)
        self.contour = contour
        
        mask = np.zeros_like(self.camera.cv_image[:,:,0])
        cv2.drawContours(mask, [self.contour], 0, (255, 255, 255), 1)
        
        
        image_thr = cv2.adaptiveThreshold(mask, 255, cv2.THRESH_BINARY_INV, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 3, 3)
        verticalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 11))
        mask2 = cv2.morphologyEx(image_thr, cv2.MORPH_OPEN, verticalStructure)
        self.vertical_lines_image = mask2
        
        #cv2.namedWindow('vl')
        #cv2.imshow('vl', mask2)
        #cv2.waitKey(3)
        
    
        
