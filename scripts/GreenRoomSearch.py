import rospy
import yaml
import os
import cv2
import numpy as np 
import rospy 
import math
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from math import pi

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

from nav_msgs.msg import Path
import time
import sys
from std_msgs.msg import Float64
from std_msgs.msg import Bool, Int32, Int32MultiArray




PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
path = f'{PROJECT_DIR}/scripts'
sys.path.insert(0, path)

allowed = True #needs to be global




class GoToPoseInGreenRoom():
    
    def generate_points(self, start, end, num_points=10):
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append((x, y))
        return points




    def plannedPathsChecker(self, path):
        global allowed

        FirstPathPassed = False #needs to be global
        pathArray = [] #needs to be global
        if len(path.poses) == 0:
            allowed = False
            return
        
        if FirstPathPassed == False:
            for i, pose in enumerate(path.poses):
                x = pose.pose.position.x
                y = pose.pose.position.y
                pathArray.append((x,y))
            FirstPathPassed = True
        
        
            
            allowed = True
            for coordiantes in self.points:
                #print("Checking a coordiante of entrance point", coordiantes)
                for point in pathArray:
                    ##print("checking against point")
                    if abs(coordiantes[0] - point[0]) <= 0.3 and abs(coordiantes[1] - point[1]) <= 0.3:
                        #planned path does go through entrance
                        allowed = False
                        self.move_base.cancel_goal()

                        print(f"match found entrance coordiante :{coordiantes} and path coordinate: {point}")
                        break
    def __init__(self, camera, doorway):
            
        self.poster_detected = False
        self.poster_direction = None    
        self.detected = rospy.Subscriber('/poster_detection', Int32MultiArray, self.poster_detection_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_current_pose)
        self.poster_pose = None
        self.current_pose = None
        
        self.start_point = doorway[0]
        self.end_point = doorway[1]

        self.points = self.generate_points(self.start_point, self.end_point)
        self.goal_sent = False
        self.camera = camera
        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Wait for the action server to come up')

        self.move_base.wait_for_server()
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plannedPathsChecker)


    def goto(self, pos, quat):
        

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        
        
        global allowed

	    # Start moving
        self.move_base.send_goal(goal)
        time.sleep(3)#wait for path to be created and checked first before checking value of allowed
        
        if allowed == True:
            #print("path allowed")
            success = self.move_base.wait_for_result(rospy.Duration(10)) 

        else:
            print("path not allowed")
            # clear the plan for move_base
            self.move_base.send_goal(MoveBaseGoal())

            # wait for the goal to complete
            self.move_base.wait_for_result()        ##
            self.move_base.cancel_goal()

            # self.move_base.send_goal(MoveBaseGoal())
            # self.move_base.wait_for_result()   
            rospy.sleep(1)
            success = self.move_base.wait_for_result(rospy.Duration(5)) 
##

            
        # Allow TurtleBot up to 20 seconds to complete task

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo('Stop')
        rospy.sleep(1)
        
    # rotates until the difference between the cx value and the center 
    # of the robot's view is 0
    ###
    def center_object(self, cx):
        pass
        # Created my own publisher and cv_image here,
        # if they are already created elsewhere just change where necessary
        rotate = Twist()  
        move_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        
        # print(type(self.camera.img_width))
        img_width = self.camera.img_width

        
        x_diff = cx - img_width/2
        



    # TODO: get the robot to go to the entrance points using coordinates
    def go_to_location_in_green_room(self, x, y, angleValue):
        try:
            # navigator = GoToPoseInGreenRoom()
            theta = 0.02
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(angleValue/2), 'r4' : np.cos(angleValue/2)}

            rospy.loginfo('Go to (%s, %s) pose', position['x'], position['y'])
            global allowed
                    
            allowed = True
            success = self.goto(position, quaternion)
            
            if success:
                rospy.loginfo('Hooray, reached the desired pose')
                return True
            else:
                rospy.loginfo('The base failed to reach the desired pose')
                return False

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)           
        except rospy.ROSInterruptException:
            rospy.loginfo('Ctrl-C caught. Quitting')
            
            
    def convert_poster_pose(self, poster_pose):
        position = {'x': poster_pose.position.x, 'y' : poster_pose.position.y}
        quaternion = {'r1' : poster_pose.orientation.x, 'r2' : poster_pose.orientation.y, 'r3' : poster_pose.orientation.z, 'r4' : poster_pose.orientation.w}
        # print("quater:", poster_pose.or
        # ientation)
        return position, quaternion


            
    def get_current_pose(self, data):
        self.current_pose = data.pose.pose        


    def poster_detection_callback(self, msg):
        # global poster_detected
        # global poster_direction
        if bool(msg.data[0]) == 1:
            self.poster_detected = True
            self.poster_direction = msg.data[1]
            # get pose when poster is detected
            if self.poster_pose == None:
                self.poster_pose = self.current_pose
            
            # print("poster_pose:", self.poster_pose)
            # self.pose_sub.unregister()
            

        #print("checking format", msg.data[0], msg.data[1])
        #cx_going_into_pd = msg.data[1]#???
        if self.poster_direction != None:
            self.center_object(self.poster_direction) ## IMPORT THE FUNCTION


    def BeginSearch(self, centreX, centreY):
        
        

        rightSteps = 1
        upSteps = 1
        leftSteps = 2
        downSteps = 2

        step=0.2

        x=centreX
        y=centreY

        angleValue = 0.25 * math.pi
        while (self.poster_detected == False):
            # right
            for i in range(rightSteps):
                rotationAngle = 1.5*math.pi - angleValue
                x += step
                print("right to", x, y)
                self.go_to_location_in_green_room(x, y, rotationAngle)
                if self.poster_detected == True:

                    break
            if self.poster_detected == True:
                print("found and breaking outer")
                break        
            # up
            for i in range(upSteps):
                rotationAngle = 0 - angleValue
                y += step
                print("up to", x, y)
                self.go_to_location_in_green_room(x, y, rotationAngle)
                if self.poster_detected == True:

                    break
            if self.poster_detected == True:
                print("found and breaking outer")
                break
            
            # left 
            for i in range(leftSteps):
                rotationAngle = math.pi/2 - angleValue
                x -= step
                print("left to", x, y)
                self.go_to_location_in_green_room(x, y, rotationAngle)
                if self.poster_detected == True:

                    break
            if self.poster_detected == True:
                print("found and breaking outer")
                break   

            # down
            for i in range(downSteps):
                rotationAngle = math.pi - angleValue
                y -= step
                print("down to", x, y)
                self.go_to_location_in_green_room(x, y, rotationAngle)
                if self.poster_detected == True:

                    break
            if self.poster_detected == True:
                print("found and breaking outer")
                break   
            
            
            rightSteps += 2
            upSteps += 2
            leftSteps += 2
            downSteps += 2
            
        return self.poster_pose
        
            
    
