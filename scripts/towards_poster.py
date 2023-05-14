import os
import sys
import rospy
import yaml
import cv2
import numpy as np 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Int32MultiArray
import time
from camera_class import CameraClass

# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
import sys
sys.path.insert(0, f'{PROJECT_DIR}/scripts')


class TowardsPoster():
    def __init__(self, camera, poster_detector):
        poster_detector.permission = True
        self.camera = camera
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        self.rate = rospy.Rate(10) #10hz
        self.poster_detection = rospy.Subscriber('/poster_detection', Int32MultiArray, self.poster_detection_callback)
        self.poster_found = False
        self.in_front_of_poster = False
        self.poster_cx = 0
        self.poster_area = 0
        self.poster_detector = poster_detector
        self.done_min = False
        self.done_max = False
        
        while not self.poster_found:
            self.spin_on_spot()
            time.sleep(0.1)
            
        while not self.in_front_of_poster:
            #move towards poster using cx given by subscriber
            if self.poster_area > 7000:
                self.move_stop()
                self.in_front_of_poster = True
            else:
                self.move_towards_cx()
            time.sleep(0.1)
        
        
    
    def move_towards_cx(self):
        image_width = self.camera.img_width
        error = self.poster_cx - image_width / 2
        move_to_do = Twist()
        move_to_do.linear = Vector3(0.1,0,0)
        move_to_do.angular = Vector3(0,0,-0.5 * error / image_width)
        self.pub.publish(move_to_do)
        
        
    def move_stop(self):
        move_stop = Twist()
        self.pub.publish(move_stop)
        
        
        
    def spin_on_spot(self):
        slow_spin = Twist()
        slow_spin.angular.z = 0.2
        while not self.poster_found:
            self.pub.publish(slow_spin)
            self.rate.sleep()
            
    def poster_detection_callback(self, msg):
        self.poster_found = bool(msg.data[0])
        self.poster_area = msg.data[2]
        if self.poster_detector.permission:
            self.poster_cx = msg.data[1]
        else:
            self.poster_cx = 0
            
        