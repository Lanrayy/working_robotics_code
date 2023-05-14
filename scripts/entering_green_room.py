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


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


from camera_class import CameraClass
from doorway import Doorway

# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
path = f'{PROJECT_DIR}/scripts'
sys.path.insert(0, path)


# ** Don't change this path **
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
path = f'{PROJECT_DIR}/scripts'
sys.path.insert(0, path)

# function to test that the import has been made successfully
def say_it_works_egr():
    rospy.loginfo("Success! You just have imported entering green room...")
    
def read_input_points():
    path = f'{PROJECT_DIR}/world/input_points.yaml'
    with open(path) as opened_file:
        return yaml.safe_load(opened_file)


robot_going_into = 'None'


class EnterGreenRoom():
    def __init__(self, camera):
        self.green_room = 0
        self.green_found = False
        
        # entrances of the room
        input_points = read_input_points()
        self.room1_entrance = input_points.get('room1_entrance_xy')
        self.room2_entrance = input_points.get('room2_entrance_xy')
        self.room1_centre = input_points.get('room1_centre_xy')
        self.room2_centre = input_points.get('room2_centre_xy')
        self.camera = camera
        self.doorway = None
        # run main program
        self.main_program()
        
        
        
        
        
        
        
        
        
        
    def set_doorway(self):
        center_points = None
        entrance_points = None
        doorway1 = None
        doorway2 = None
        doorway_x = None
        doorway_y = None
        if self.green_room == 1:
            center_points = self.room1_centre
            entrance_points = self.room1_entrance
        elif self.green_room == 2:
            center_points = self.room2_centre
            entrance_points = self.room2_entrance
        else:
            print("green room not set")
            return
        gradient = (center_points[1] - entrance_points[1]) / (center_points[0] - entrance_points[0])

        if abs(gradient) < 1:
            # horizontal line from room entrance to center of room
            if center_points[0] < entrance_points[0]:
                doorway_x = entrance_points[0] - 1.5
            else:
                doorway_x = entrance_points[0] + 1.5
            print(f"Centre points {center_points}")
            doorway1 = (doorway_x, center_points[1] + 1.5)
            doorway2 = (doorway_x, center_points[1] + 1.5)
        else:
            # vertical line from room entrance to center of room
            if center_points[1] < entrance_points[1]:
                doorway_y = entrance_points[1] - 1.5
            else:
                doorway_y = entrance_points[1] + 1.5
            doorway1 = (center_points[0] - 2, doorway_y)
            doorway2 = (center_points[0] + 2, doorway_y)

        self.doorway = [doorway1, doorway2]
        print("doorway:", self.doorway)
        
    # TODO: get the robot to go to the specific points using coordinates
    def go_to_location(self, x, y):
        try:
            
            navigator = GoToPose()
            theta = 0.02
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            rospy.loginfo('Go to (%s, %s) pose', position['x'], position['y'])
            success = navigator.goto(position, quaternion)
            
            if success:
                rospy.loginfo('Hooray, reached the desired pose')
            else:
                rospy.loginfo('The base failed to reach the desired pose')
                
            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)           
        except rospy.ROSInterruptException:
            rospy.loginfo('Ctrl-C caught. Quitting')
        
        
    # get the robot to rotate on the spot
    def rotate_on_the_spot(self):
        # define publisher that'll will publish the movements to the robot
        pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        rate = rospy.Rate(10) #10hz # 20hz

        # instantiatiate a Twist class that is going to be published to the robot
        actions = Twist()

        # while not rospy.is_shutdown():
        for _ in range(10):
            actions.angular.z = 0.5 # 1.5707963268 # 0.5708

            for _ in range(20): # 10 #55
                pub.publish(actions)
                rate.sleep()
    
    # def show_camera(self):
    #     cI = colourIdentifier()
        
    def callback(self, data):
        # rospy.loginfo(f'{rospy.get_caller_id()} I heard {data.data}')
        # print(type(data.data))
        
        # rospy.loginfo(f"Green circle found at room {room} entrance...")
        global robot_going_into
        input_points = read_input_points()
        room1_centre = input_points.get('room1_centre_xy')
        room2_centre = input_points.get('room2_centre_xy')
        rospy.loginfo('Green Circle found...')
        
        if self.green_room == 1:
            rospy.loginfo('...Going into room 1')
            
            print('...Going into room 1')
            robot_going_into = 'Room'
            
            self.green_found = True
            #Karl insert code here for getting doorway values
            self.set_doorway()
            self.go_to_location(self.room1_centre[0], room1_centre[1])
            
            
        elif self.green_room == 2:
            rospy.loginfo('...Going into room 2')
            print('...Going into room 2')

            robot_going_into = 'Room'
            self.green_found = True
            self.set_doorway()
            self.go_to_location(self.room2_centre[0], room2_centre[1])
        else:
            rospy.loginfo('Global variable not working...')
            print('Global variable not working...')
            
    
    def main_program(self):
        #read entrance input points
        
        # show camera
        cI = colourIdentifier(self.camera)
        
        print(self.green_found)
        
        # Go to the first entrance, if green has not been found then try to find green circle
        if self.green_found ==  False:
            rospy.loginfo("Going to location 1 entrance")
            self.green_room = 1 # set green room to 1
            self.go_to_location(self.room1_entrance[0], self.room1_entrance[1])
            # TODO: DELETE BELOW 
            #self.go_to_location(self.room1_centre[0], self.room1_centre[1])
            rospy.loginfo('Launching subscriber')
            sub = rospy.Subscriber('green_circle', String, self.callback)
            rospy.loginfo('Spinning on the spot')
            self.rotate_on_the_spot()
            rospy.loginfo('Closing subscriber')
            sub.unregister()

        print(self.green_found)
        #Go to the second room, if green has not been found then try to find green circle
        if self.green_found == False:
            rospy.loginfo("Going to location 2 entrance")
            self.green_room = 2
            self.go_to_location(self.room2_entrance[0], self.room2_entrance[1])
            rospy.loginfo('Launching subscriber')
            sub = rospy.Subscriber('green_circle', String, self.callback) # launch subscriber
            rospy.loginfo('Spinning on the spot')
            self.rotate_on_the_spot()
            rospy.loginfo('Closing subscriber')
            sub.unregister()
        


        
        rospy.loginfo(f'We are now in the centre of room: {self.green_room}')
    

class colourIdentifier(): 
    def __init__(self, camera):
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        # self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
        self.rate = rospy.Rate(10) #10hz
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('green_circle', String, queue_size=0)
        self.camera = camera
        print("In init")
        
        #sensitivity of the image mask
        self.sensitivity = 30
        
    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler

        # Set the upper and lower bounds for the colour you wish to identify - green

        # Convert the rgb image into a hsv image

        # Filter out everything but a particular colour using the cv2.inRange() method

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # try:
            
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     print(e)
            
        
        
            
            
        # convert image from bgr to hsv
        hsv_image = cv2.cvtColor(self.camera.cv_image, cv2.COLOR_BGR2HSV)
        
        #filter anything that is not green
        # define the range of the colour to be filtered out
        hsv_green_lower = np.array([60 - self.sensitivity, 50, 0])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255 ])
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        green_rgb_mask = cv2.bitwise_and(self.camera.cv_image, self.camera.cv_image, mask=green_mask)
        
        # red mask
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        
        red_rgb_mask = cv2.bitwise_and(self.camera.cv_image, hsv_image, mask=red_mask)

        # # blue mask
        hsv_blue_lower = np.array([100 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([100 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        blue_rgb_mask = cv2.bitwise_and(self.camera.cv_image, hsv_image, mask=blue_mask)
        
        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two images at once, so multiple calls are necessary for more than two colours
        
        # #combine the red and green mask
        red_green_rgb_mask = cv2.bitwise_or(red_rgb_mask, green_rgb_mask)
        
        # #combine the red and green mask
        red_green_blue_rgb_mask = cv2.bitwise_or(red_green_rgb_mask, blue_rgb_mask)
        
        # red_green_blue black white mask
        red_green_blue_mask = red_mask + green_mask + blue_mask
        
        
        # cv2.namedWindow('red_green_blue_rgb_mask') 
        # cv2.imshow('red_green_blue_rgb_mask', red_green_blue_rgb_mask)
        # cv2.waitKey(3)
        
        
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        
        
        # contours, _ = cv2.findContours(red_green_blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # get each individual contour

        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        green_mask= cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        
        # cv2.namedWindow('green_mask')
        # cv2.imshow('green_mask', green_mask)
        # cv2.waitKey(3)
        
        # bitwise and images with themselves
        green_rgb_mask = cv2.bitwise_and(self.camera.cv_image, self.camera.cv_image, mask=green_mask)

        if len(green_contours) > 0:
            # print("Green Contour Found")
            green_contour = max(green_contours, key=cv2.contourArea)
            green_moment = cv2.moments(green_contour)
            try:
                green_cx, green_cy = int(green_moment['m10']/green_moment['m00']), int(green_moment['m01']/green_moment['m00'])
            except ZeroDivisionError:
                pass
            size = 2000
            
            
            if cv2.contourArea(green_contour) > size: #<What do you think is a suitable area?>:
                # print(cv2.contourArea(green_contour))
                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                #print("Green found")
                (x, y), radius = cv2.minEnclosingCircle(green_contour)

                cv2.circle(red_green_blue_rgb_mask,(green_cx,green_cy), int(radius),(60, 60,60),3)

                # Then alter the values of any flags
                self.green_detected = 1
                
                
                self.pub.publish("Greeeen Detected")
                self.rate.sleep()

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        hsv_image2 = cv2.cvtColor(self.camera.cv_image, cv2.COLOR_BGR2HSV)
        cv2.namedWindow('contour_camera') 
        cv2.imshow('contour_camera', red_green_blue_rgb_mask)
        # cv2.waitKey(3)
        
        
class GoToPose():
    def __init__(self):
        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Wait for the action server to come up')
        self.pub = rospy.Publisher('in_green_room', String, queue_size=0)

        self.move_base.wait_for_server()

    def goto(self, pos, quat):
        
        global robot_going_into
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            
            print(f"Robot going into...: {robot_going_into}")
            
            # self.pub.publish("True")
            if robot_going_into ==  'Room':
                print("Publishing True...")
                self.pub.publish("True")
                
            rospy.loginfo('Successful!!!!')
            result = True
        else:
            rospy.loginfo('Not successful! Cancelling goal!!!!')
            self.move_base.cancel_goal()

        self.goal_sent = False

        return result

    def shutdown(self):
        if self.goal_sent:
            rospy.loginfo('In shutdown! Cancelling goal!!!!')
            self.move_base.cancel_goal()
        rospy.loginfo('Stop')
        rospy.sleep(1)
  
