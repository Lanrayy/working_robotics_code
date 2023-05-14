import os
import sys
import rospy
import yaml
import os
import cv2
import numpy as np 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


from camera_class import CameraClass

class improvePositioning():
    def __init__(self, lprp, camera):
        print('lprp is', lprp)
        self.difference_between_lengths = lprp[0] - lprp[1]
        self.camera = camera
        self.min_contour_size = 1500
        self.binary_thresh = 220
        self.max_area = 5000
        self.min_area = 50
        self.cx = None
        
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        self.image_pd_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.rate = rospy.Rate(10) #10hz
    
        time.sleep(1)

        if self.difference_between_lengths > 70: self.move_right()
        if self.difference_between_lengths < -70: self.move_left()
        
    def spin_left(self):
        slow_spin = Twist()
        slow_spin.angular.z = 1.37079632679
        self.pub.publish(slow_spin)
        
    def slow_spin_left(self):
        slow_spin = Twist()
        slow_spin.angular.z = 0.1
        self.pub.publish(slow_spin)
    
    def spin_right(self):
        slow_spin = Twist()
        slow_spin.angular.z = -1.37079632679
        self.pub.publish(slow_spin)
        
        
    def slow_spin_right(self):
        slow_spin = Twist()
        slow_spin.angular.z = -0.1
        self.pub.publish(slow_spin)
        
        
    def move_forward(self):
        forward = Twist()
        forward.linear.x = 0.2 # Forward with 0.2 m/sec.
        self.pub.publish(forward)
        
    def move_backward(self):
        backward = Twist()
        backward.linear.x = -0.1 # Backward with -0.1 m/sec.
        self.pub.publish(backward)
            
    def move_right(self):
        for _ in range (20):
            self.move_backward()
            time.sleep(0.1)
        
        for _ in range(10): 
            self.spin_right()
            time.sleep(0.1)

        for _ in range (20):
            self.move_forward()
            time.sleep(0.1)

        while(self.cx < 300 or self.cx > 340):
            self.slow_spin_left()
            time.sleep(0.01)
        
        
    def move_left(self):
        # 2) Move backward 1 meter
        for _ in range (20):
            self.move_backward()
            time.sleep(0.1)
        
        for _ in range(10): 
            self.spin_left()
            time.sleep(0.1)

        # 2) Move forward 1 meter
        for _ in range (20):
            self.move_forward()
            time.sleep(0.1)

        while(self.cx < 300 or self.cx > 340):
            self.slow_spin_right()
            time.sleep(0.01)
        
            
            
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


    def image_callback(self, data):
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
                    self.cx = cx[0]
                except TypeError as e:
                    return
                
        #print('self.cx is', self.cx)
        






