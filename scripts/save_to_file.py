import rospy
import yaml
import os

import cv2
import numpy as np 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
import sys
sys.path.insert(0, f'{PROJECT_DIR}/scripts')

from camera_class import CameraClass

# function to test that the import has been made successfully
def say_it_works_stf():
    rospy.loginfo("Success! You just have imported save to file module....")
    

# # function to test that the import has been made successfully
# def say_it_works():
#     rospy.loginfo("Success! You just have imported the saving to file module...")



def save_character_screenshot(cv_image):
    path = f'{PROJECT_DIR}/output/cluedo_character.png'
    cv2.imwrite(path, cv_image)
    
    
def write_character_id(character_id):
    path = f'{PROJECT_DIR}/output/cluedo_character.txt'
    with open(path, 'w') as opened_file:
        opened_file.write(character_id)


class SaveToFile():
    def __init__(self, camera, character):
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        # self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
        self.rate = rospy.Rate(10) #10hz
        self.bridge = CvBridge()
        self.camera = camera
        self.character_name_saved = 0
        self.character_found = character
        
        self.take_screenshot()
        self.write_character_found()
        
        
        #self.screenshot_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.screenshot_callback)
        #self.character_id_sub = rospy.Subscriber('/character_found', String, self.character_id_callback)
        
    def take_screenshot(self):
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     print(e)
        
        # cv2.namedWindow('screenshot_cam')
        # cv2.imshow('screenshot_cam', self.camera.cv_image)
        # cv2.waitKey(3)
        save_character_screenshot(self.camera.cv_image)
        
    def write_character_found(self):
        write_character_id(self.character_found)
