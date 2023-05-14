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

from nav_msgs.msg import Path
import time
import sys



# ** Don't change this path **


PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
import sys
sys.path.insert(0, f'{PROJECT_DIR}/scripts')


from feature_matcher import FeatureMatcher
from poster_detector import PosterDetector
from colour_poster_detector import ColourPosterDetector
from entering_green_room import EnterGreenRoom, read_input_points, say_it_works_egr
from save_to_file import SaveToFile, say_it_works_stf
from camera_class import CameraClass
from GreenRoomSearch import *
from towards_poster import TowardsPoster
from vote_winner import voteWinner
from poster_at_angle import PosterAtAngle
from improve_positioning import improvePositioning


def test_project_running():
    rospy.loginfo("Program running...")
robot_in_green_room = 'False'

def character_id_callback(data):
    print(data.data)
    
    if data.data == 'True':
        print("setting robot_in_green_room to true...")
        global robot_in_green_room
        robot_in_green_room = 'True'
        

def main():
    try:
        rospy.Subscriber('in_green_room', String, character_id_callback)
        rospy.init_node('group_project')
        test_project_running()
        say_it_works_egr()
        say_it_works_stf()
        global robot_in_green_room
        
        camera = CameraClass()
        
        
        p_d = ColourPosterDetector(camera)
        p_d = PosterDetector(camera)
        f_m = FeatureMatcher(camera)
        poster_at_angle = PosterAtAngle(camera)
        time.sleep(1)
        rospy.loginfo("Finished Entering Green Room Class")
        green = EnterGreenRoom(camera)
        rospy.loginfo("Finished Entering Green Room Class")



        print(f'Robot in green room variable: {robot_in_green_room}')

        
        doorway = green.doorway

        # # #TODO: DELETE BELOW
        # robot_in_green_room = True
        

        i = 0
        while(robot_in_green_room == 'False'):
            rospy.loginfo("Waiting...")
            time.sleep(1)
        
        
        print(f'Robot in green room variable: {robot_in_green_room}')
        print("Not Waiting..." )
        
        
        global FirstPathPassed 
        global allowed
        
        print("reached here")
        input_points = read_input_points()
        room1_entrance = input_points.get('room1_entrance_xy')
        room2_entrance = input_points.get('room2_entrance_xy')
        room1_centre = input_points.get('room1_centre_xy')
        room2_centre = input_points.get('room2_centre_xy')
        
        greenRoom_Search = GoToPoseInGreenRoom(camera, doorway)
        poster_pose = greenRoom_Search.BeginSearch(room1_centre[0], room1_centre[1])  
        poster_position, poster_quaternion = greenRoom_Search.convert_poster_pose(poster_pose)
        greenRoom_Search.goto(poster_position, poster_quaternion)
        final_pose = greenRoom_Search.current_pose
        expected_pose = greenRoom_Search.poster_pose
        
        towards_poster = TowardsPoster(camera, p_d)
        
        time.sleep(2)
        time.sleep(1)
        lprp = poster_at_angle.get_left_right_white_pixels()
        i_p = improvePositioning(lprp, camera)
        print('now in front of poster, taking picture and finding out character!')
        time.sleep(4)
        
        vote_winner = voteWinner()
        for i in range(100):
            #character_found = p_d.find_character()
            character_found = f_m.find_character()
            vote_winner.get_vote(character_found)
        print(vote_winner.votes)
        character = vote_winner.get_winner()
        save_to_file = SaveToFile(camera, character)
        rospy.signal_shutdown('Finished program')
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    # rospy.init_node('group_project')
    # Your code should go here. You can break your code into several files and
    # include them in this file. You just need to make sure that your solution 
    # can run by just running rosrun group_project main.py
    main()

    # Please note how to properly read/write to files: use similar approach for
    # the rest of your I/O in your project. You might need to use these functions
    # elsewhere this is just a demonstration of how to use them.
    

    # points = read_input_points()
    # write_text_file('mustard')
    # save_character_screenshot(cv_image)
