import rospy
import cv2
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import UInt8
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
     
class Doorway():
    def __init__(self):
        self.goal_sent = False
        self.bumped = False
        self.origin = None
        self.y_doorway = None
        self.center_pose = None
        self.current_pose = None
        self.doorway_coords = None
        
        # add variable to store points as list of tuples
        # self.doorway
        
        self.green_detected = False
        self.sensitivity = 10
        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.move_forward = Twist()
        self.move_forward.linear.x = 0.5
        self.stop = Twist()
        
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()

        # We covered which topic to subscribe to should you wish to receive image data
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.turn_towards_colour)

        # publishers and subscribers
        self.doorway_pub = rospy.Publisher('doorway', PoseArray, queue_size=10) 
        self.move_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=0)
        
        self.init_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_origin)
        self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.check_bumper)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_current_pose)
        self.get_doorway_sub = None
        print("initialised doorway")
        
        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Wait for the action server to come up')

        self.move_base.wait_for_server()
        
        
    # rotate robot towards the cx value
    def rotate(self, cx, cy, img_width):
        print("rotating - km")
        x_diff = cx - img_width/2
        # stop rotating once we are facing the green circle
        if x_diff == 0:
            self.image_sub.unregister()
            # move forward to bump into the green circle
            self.bump_wall()
            return
        
        print("contour is {} away from the center".format(x_diff))
        
        turn_speed = 0.5
        max_turn_angle = 1.0
        
        turn_angle = -max(min(x_diff/img_width * max_turn_angle, max_turn_angle), -max_turn_angle)
        rotate = Twist()
        rotate.angular.z = turn_speed * turn_angle
        self.pub.publish(rotate)
        
    # only faces green for now
    # may want to update to turn towards the colour circle we are currently looking at
    def turn_towards_colour(self, data):
        # Convert the received image into a opencv image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            img_width = cv_image.shape[1]
            
        # But remember that you should always wrap a call to this conversion method in an exception handler
        except CvBridgeError as e:
            print(e)
            return

        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100]) # self.sensitivity still needs to be initialised in __init__
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
        # Convert the rgb image into a hsv image
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        green_mask = cv2.inRange(hsv_img, hsv_green_lower,hsv_green_upper)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        target = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
        
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        green_contours,_ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours
        if len(green_contours) > 0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            c = max(green_contours, key=cv2.contourArea)

            #Moments can calculate the center of the contour
            M = cv2.moments(c)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            
            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>:
                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                (x, y), radius = cv2.minEnclosingCircle(c)

                cv2.circle(img=cv_image,center=(int(x), int(y)),radius=int(radius),color=(0,128,0),thickness=5)

                # Alter the value of the flag
                self.green_detected = True
            self.rotate(cx, cy, img_width)

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

        cv2.namedWindow('camera_Feed') 
        cv2.imshow('camera_Feed', target)
        cv2.waitKey(3) 
     
    # get the original location of the robot when this class is first called
    def get_origin(self, data):
        self.origin = data.pose.pose.position
        self.init_pose_sub.unregister()
        print("unregistered init pose sub")
        print("origin coords: {},{}".format(self.origin.x, self.origin.y))
    
    # check if bumper is hit
    def check_bumper(self, data):
        if data.state == 1:
            # stop moving
            desired_velocity = Twist()
            desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.
            self.move_pub.publish(desired_velocity)
            
            self.bumped = True
            rospy.sleep(3) # Sleeps for 3 sec to allow time to get latest pose
            
            print("bumped is now ", self.bumped)
            print("final coordinates: {}, {}, {}".format(self.current_pose.position.x,self.current_pose.position.y,self.current_pose.position.z))
            self.y_doorway = self.current_pose.position.y + 0.3
            self.set_doorway_coords()
            # self.pose_sub.unregister()
            self.bump_sub.unregister()
            
    
    # get current location of the robot
    def get_current_pose(self, data):
        self.current_pose = data.pose.pose
        
        
    # should be called when the robot moves to the center of the room? or within the callback function that does this
    # def set_doorway_coords(self, data):
    # def get_doorway(self, data):
    
    # altered ^ so that pose_sub is monitored and when 
    def get_doorway(self):
        data = self.pose_sub
        x_current = data.pose.pose.position.x
        y_current = data.pose.pose.position.y
        self.center_pose = data.pose.pose
        
        # check if we need to look for a smaller or larger value, then only send the pose values once the robot
        # has passed the doorway
        if self.origin.y < self.y_doorway and y_current > self.y_doorway:
            self.doorway_coords = [(self.center_pose.position.x - 2, self.center_pose.position.z),(self.center_pose.position.x + 2, self.center_pose.position.z)]
            # self.publish_doorway(self.center_pose)
            # self.get_doorway_sub.unregister()
            self.pose_sub.unregister()
            
        elif self.origin.y > self.y_doorway and y_current < self.y_doorway: 
            self.doorway_coords = [(self.center_pose.position.x - 2, self.center_pose.position.z),(self.center_pose.position.x + 2, self.center_pose.position.z)]
            # self.publish_doorway(self.center_pose)
            # self.get_doorway_sub.unregister()
            self.pose_sub.unregister()
            
           
           
    # publish coordinates of the doorway 
    def publish_doorway(self, center_pose):
        print("publishing doorway")
        
        # create new pose positions spanning either side of the doorway
        start_pose = Pose()
        start_position = (self.center_pose.position.x - 2, self.center_pose.position.y, self.center_pose.position.z)
        start_pose.position.x, start_pose.position.y, start_pose.position.z = start_position
        start_pose.orientation = self.center_pose.orientation
        
        end_pose = Pose()
        end_position = (self.center_pose.position.x + 2, self.center_pose.position.y, self.center_pose.position.z)
        end_pose.position.x, end_pose.position.y, end_pose.position.z = end_position
        end_pose.orientation = self.center_pose.orientation
        
        poses = PoseArray()
        poses.header.frame_id = 'map'
        
        poses.poses.append(start_pose)
        poses.poses.append(self.center_pose)
        poses.poses.append(end_pose)
        
        self.doorway_pub.publish(poses)
        
    # move forward until we bump into the wall
    def bump_wall(self):
        rate = rospy.Rate(10) #10hz

        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.
        
        for _ in range (400):            
            if self.bumped == True:
                rospy.sleep(3) # Sleeps for 3 sec to allow time to get latest pose
                break
            self.move_pub.publish(desired_velocity)
            rate.sleep()
        
        if self.bumped == False:
            print("doorway not found :(")
        
        print("y_doorway:", self.y_doorway)
    

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

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
        rospy.loginfo('Stop - km')
        rospy.sleep(1)

# if __name__ == '__main__':
#     try:
#         rospy.init_node('nav_test', anonymous=True)
                
#         navigator = Doorway()
        
#         navigator.turn_towards_colour()
        
#         navigator.get_doorway_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, navigator.get_doorway)
        
#         # Customize the following values so they are appropriate for your location
#         x = -2.2 # SPECIFY X COORDINATE HERE
#         y = 5.28 # SPECIFY Y COORDINATE HERE
#         theta = -0.05 # SPECIFY THETA (ROTATION) HERE
#         position = {'x': x, 'y' : y}
#         quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

#         rospy.loginfo('Go to (%s, %s) pose', position['x'], position['y'])
#         success = navigator.goto(position, quaternion)

#         if success:
#             rospy.loginfo('Hooray, reached the desired pose')
#         else:
#             rospy.loginfo('The base failed to reach the desired pose')

#         # Sleep to give the last log messages time to be sent
#         rospy.sleep(1)
#     except rospy.ROSInterruptException:
#         rospy.loginfo('Ctrl-C caught. Quitting')