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

class PosterDetector():
    def __init__(self, camera):
        self.bridge = CvBridge()
        self.poster_det_pub = rospy.Publisher('/poster_detection', Int32MultiArray, queue_size = 0)
        #self.poster_dir_pub = rospy.Publisher('/poster_direction', Int32, queue_size = 0)
        self.camera = camera
        self.image_pd_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.min_contour_size = 1500
        self.rate = rospy.Rate(10)
        self.binary_thresh = 220
        self.max_area = 5000
        self.min_area = 50
        
        self.contour = None
        self.cx = None
        self.permission = False
        
        self.binary_image = None
        
        
    def get_scarlett_mask(self,cv_image, hsv_image ):
        scarlett_sensitivity = 5
        lower = np.array([0 - scarlett_sensitivity, 100 , 100])
        upper = np.array([0 + scarlett_sensitivity, 255, 255])
        mask = cv2.inRange(hsv_image, lower, upper)
        rgb_mask = cv2.bitwise_and(cv_image, hsv_image, mask=mask)
        return rgb_mask, mask

    def get_peacock_mask(self,cv_image, hsv_image):
        peacock_sensitivity = 10
        lower = np.array([110 - peacock_sensitivity, 45, 70])
        upper = np.array([110 + peacock_sensitivity, 255, 255])
        mask = cv2.inRange(hsv_image, lower, upper)
        rgb_mask = cv2.bitwise_and(cv_image, hsv_image, mask=mask)
        return rgb_mask, mask

    def get_mustard_mask(self,cv_image, hsv_image ):
        mustard_sensitivity = 5
        lower = np.array([20, 100 , 100])
        upper = np.array([25 + mustard_sensitivity, 180, 230])
        mask = cv2.inRange(hsv_image, lower, upper)
        rgb_mask = cv2.bitwise_and(cv_image, hsv_image, mask=mask)
        return rgb_mask, mask
    
    def get_plum_mask(self,cv_image, hsv_image ):
        plum_sensitivity = 20
        lower = np.array([154 - plum_sensitivity, 50 , 0])
        upper = np.array([154 + plum_sensitivity, 150, 255])
        mask = cv2.inRange(hsv_image, lower, upper)
        rgb_mask = cv2.bitwise_and(cv_image, hsv_image, mask=mask)
        return rgb_mask, mask
        
        
    def combine_mask(self, scarlett_rgb_mask, plum_rgb_mask, peacock_rgb_mask, mustard_rgb_mask):
        sp_mask = cv2.bitwise_or(scarlett_rgb_mask, plum_rgb_mask)
        spp_mask = cv2.bitwise_or(sp_mask, peacock_rgb_mask)
        sppm_mask = cv2.bitwise_or(spp_mask, mustard_rgb_mask)
        return sppm_mask
        
    
    def get_contour(self):
        return self.contour
    
    def update_thresholds(self, b_t = 127, mx_a = 5000, mn_a = 50):
        self.binary_thresh = b_t
        self.max_area = mx_a
        self.min_area = mn_a
    
    def maxAreaFilter(self, maxArea, inputImage):
        # Perform an area filter on the binary blobs:
        componentsNumber, labeledImage, componentStats, componentCentroids = cv2.connectedComponentsWithStats(inputImage, connectivity=4)
        # Get the indices/labels of the remaining components based on the area stat
        # (skip the background component at index 0)
        remainingComponentLabels = [i for i in range(1, componentsNumber) if componentStats[i][4] <= maxArea]
        # Filter the labeled pixels based on the remaining labels,
        # assign pixel intensity to 255 (uint8) for the remaining pixels
        filteredImage = np.where(np.isin(labeledImage, remainingComponentLabels) == True, 255, 0).astype('uint8')
        return filteredImage
    
    def minAreaFilter(self, minArea, inputImage):
        # Perform an area filter on the binary blobs:
        componentsNumber, labeledImage, componentStats, componentCentroids = cv2.connectedComponentsWithStats(inputImage, connectivity=4)
        # Get the indices/labels of the remaining components based on the area stat
        # (skip the background component at index 0)
        remainingComponentLabels = [i for i in range(1, componentsNumber) if componentStats[i][4] >= minArea]
        # Filter the labeled pixels based on the remaining labels,
        # assign pixel intensity to 255 (uint8) for the remaining pixels
        filteredImage = np.where(np.isin(labeledImage, remainingComponentLabels) == True, 255, 0).astype('uint8')
        return filteredImage
    
    def get_binary_from_image(self):
        img_float = self.camera.cv_image.astype(np.float_) / 255.
        k_channel = 1 - np.max(img_float, axis=2)
        k_channel = (255 * k_channel).astype(np.uint8)
        binary_thresh = self.binary_thresh
        _, binary_image = cv2.threshold(k_channel, binary_thresh, 255, cv2.THRESH_BINARY)
        max_area = self.max_area
        binary_image = self.maxAreaFilter(max_area, binary_image)
        min_area = self.min_area
        binary_image = self.minAreaFilter(min_area, binary_image)
        return binary_image
    
    def get_l2_contours(self, contours, hierarchy):
        level2_contours = []
        if len(contours) > 0:
            for h in hierarchy[0]:
                if h[2] > -1:
                    level2_contours.append(h[2])
        return level2_contours
        
    def get_largest_l2_contour(self, l2_contours, contours): 
        largest_contour_index = max(l2_contours, key=lambda i: cv2.contourArea(contours[i]))
        largest_contour = contours[largest_contour_index]
        return largest_contour, largest_contour_index
    
    def is_minimum_size(self, largest_contour):
        contour_area = cv2.contourArea(largest_contour)
        if contour_area > self.min_contour_size: return True
        if contour_area > 6000: self.update_thresholds(175)
        else: return False
        
    def get_contour_center(self, contour):
        moment = cv2.moments(contour)
        try:
            cx, cy = int(moment['m10']/moment['m00']), int(moment['m01']/moment['m00'])
            return cx, cy
        except ZeroDivisionError as e:
            pass
    
    def get_contour_area(self, contour):
        return cv2.contourArea(contour)
        
    
    def draw_contour_on_image(self, cv_image, contours, contour_index):
        cv2.drawContours(cv_image, contours, contour_index, (0, 255, 0), 1, lineType=cv2.LINE_AA)
        
    def package_callback_data(self, poster_bool, area):
        to_publish = []
        if self.cx is None:
            to_publish = [poster_bool, 0, area]
        else:
            to_publish = [poster_bool, self.cx, area]
        return to_publish
    
    def find_contours(self, binary_image):
        contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return contours
    
    def draw_contour(self, cv_image, contour):
        cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 1, lineType=cv2.LINE_AA)

    def image_callback(self, data):
        poster_detector = 0
        cx_going_into_pd = 0
        contour_area_into_pd = 0
        
        binary_image = self.get_binary_from_image()
        contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        binary_image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2RGB)
        l2_contours = self.get_l2_contours(contours, hierarchy)
        if len(l2_contours) > 0:
            largest_contour, largest_contour_index = self.get_largest_l2_contour(l2_contours, contours)
            self.contour = largest_contour
            if self.is_minimum_size(largest_contour):
                try:
                    cx = self.get_contour_center(largest_contour)
                    self.draw_contour_on_image(binary_image, contours, largest_contour_index)
                    poster_detector = 1
                    if self.permission: 
                        self.cx = cx
                    contour_area_into_pd = int(self.get_contour_area(largest_contour))
                except TypeError as e:
                    return
                
                
        
        
        
        hsv_image = cv2.cvtColor(self.camera.cv_image, cv2.COLOR_BGR2HSV)        
        
        #individual mask
        scarlett_rgb_mask, scarlett_mask = self.get_scarlett_mask(self.camera.cv_image, hsv_image)
        self.sc_mask = scarlett_mask
        plum_rgb_mask, plum_mask = self.get_plum_mask(self.camera.cv_image, hsv_image)
        self.pl_mask = plum_mask
        peacock_rgb_mask, peacock_mask  = self.get_peacock_mask(self.camera.cv_image, hsv_image)
        self.pe_mask = peacock_mask
        mustard_rgb_mask, mustard_mask = self.get_mustard_mask(self.camera.cv_image, hsv_image)
        self.mu_mask = mustard_mask
        
        # combined rgb mask
        combined_scarlett_plum_peacock_mustard_mask = self.combine_mask(scarlett_rgb_mask, plum_rgb_mask, peacock_rgb_mask, mustard_rgb_mask)
        
        #combined binary mask
        combined_binary_mask = scarlett_mask + plum_mask + peacock_mask + mustard_mask
        
        
        
        
        contours = self.find_contours(combined_binary_mask)
        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            self.contour = contour
            try:
                cx, cy = self.get_contour_center(contour)
                if cv2.contourArea(contour) > 500:
                    self.draw_contour(combined_scarlett_plum_peacock_mustard_mask, contour)
                    poster_detector = 1
                    if self.permission:
                        self.cx = cx
                    contour_area_into_pd = int(cv2.contourArea(contour))
            except TypeError as e:
                return
        
                    
        to_publish = self.package_callback_data(poster_detector, contour_area_into_pd)
        msg = Int32MultiArray(data=to_publish)
        # print(msg.data)
        self.poster_det_pub.publish(msg)
        self.rate.sleep()
        
        #cv2.namedWindow('poster detecter - edges only')
        #cv2.imshow('poster detecter - edges only', binary_image)
        
        #cv2.namedWindow('poster_detector')
        #cv2.imshow('poster_detector', combined_scarlett_plum_peacock_mustard_mask)
        
        
        cv2.waitKey(3)
