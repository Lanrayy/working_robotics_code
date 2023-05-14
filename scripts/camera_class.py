import os
import sys


import rospy
import yaml
import cv2
import numpy as np 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import time


# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
import sys
sys.path.insert(0, f'{PROJECT_DIR}/scripts')


class CameraClass():
    def __init__(self):
        print("\n I am here! \n")
        self.rate = rospy.Rate(10) #10hz
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.cv_image = None
        self.img_width = None
    
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            
            self.img_width = self.cv_image.shape[1]
                
            cv2.namedWindow('turtlebot_cam')
            cv2.imshow('turtlebot_cam', self.cv_image)
            cv2.waitKey(3)
                
            self.rate.sleep()
        except CvBridgeError as e:
            print(e)