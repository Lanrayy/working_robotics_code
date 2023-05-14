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

class ColourPosterDetector():
    def __init__(self, camera):
        self.bridge = CvBridge()
        self.poster_det_pub = rospy.Publisher('/poster_detection', Int32MultiArray, queue_size = 0)
        self.camera = camera
        self.image_pd_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.rate = rospy.Rate(10)
        
        self.sc_mask = None
        self.pl_mask = None
        self.pe_mask = None
        self.mu_mask = None
        
        self.contour = None
        self.cx = None
        self.permission = False
    
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
        
    def find_contours(self, binary_image):
        contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return contours
    
    def get_contour(self):
        return self.contour
    
    def get_contour_center(self, contour):
        moment = cv2.moments(contour)
        try:
            cx, cy = int(moment['m10']/moment['m00']), int(moment['m01']/moment['m00'])
            return cx, cy
        except ZeroDivisionError as e:
            pass
        
    def draw_contour(self, cv_image, contour):
        cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 1, lineType=cv2.LINE_AA)
        
    def package_callback_data(self, poster_bool, area):
        to_publish = []
        if self.cx is None:
            to_publish = [poster_bool, 0, area]
        else:
            to_publish = [poster_bool, self.cx, area]
        return to_publish
    
    def contour_detected(self, mask):
        contours = self.find_contours(mask)
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) > 500:
            return True
        return False
    
    def find_character(self):
        if self.contour_detected(self.sc_mask): return 'scarlett'
        elif self.contour_detected(self.pl_mask): return 'plum'
        elif self.contour_detected(self.pe_mask): return 'peacock'
        elif self.contour_detected(self.mu_mask): return 'mustard'
        else: return 'no character found'

    def callback(self, data):
        poster_detector = 0
        cx_going_into_pd = 0
        contour_area_into_pd = 0
        
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
        
        # contour the area of detected contour and make sure it is greater than 500 pxs
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
                
        #print('self cx is ', self.cx)
        to_publish = self.package_callback_data(poster_detector, contour_area_into_pd)
        msg = Int32MultiArray(data=to_publish)
        self.poster_det_pub.publish(msg)
        self.rate.sleep()
                
        #cv2.namedWindow('poster_detector')
        #cv2.imshow('poster_detector', combined_scarlett_plum_peacock_mustard_mask)
        #cv2.waitKey(3)
        
        
