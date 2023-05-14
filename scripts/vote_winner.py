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

class voteWinner():
    def __init__(self):
        self.votes = [0,0,0,0]
        
    def get_vote(self, character_found):
        if character_found == 'scarlett':
            self.votes[0] += 1
        if character_found == 'plum':
            self.votes[1] += 1
        if character_found == 'peacock':
            self.votes[2] += 1
        if character_found == 'mustard':
            self.votes[3] += 1
            
    def get_winner(self):
        max_val = max(self.votes)
        max_ind = self.votes.index(max_val)
        if max_ind == 0:
            return 'scarlett'
        if max_ind == 1:
            return 'plum'
        if max_ind == 2:
            return 'peacock'
        if max_ind == 3:
            return 'mustard'
        
        
        