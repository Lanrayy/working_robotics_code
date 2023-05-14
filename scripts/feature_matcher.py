import rospy
import yaml
import os
import sys
import cv2
import numpy as np 
import rospy 
from sensor_msgs.msg import Image 
from std_msgs.msg import String

SCARLET_DIR = os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images/scarlet.png')
PLUM_DIR = os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images/plum.png')
PEACOCK_DIR = os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images/peacock.png')
MUSTARD_DIR = os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images/mustard.png')
PROJECT_DIR = os.path.expanduser('~/catkin_ws/src/group_project')
path = f'{PROJECT_DIR}/scripts'
sys.path.insert(0, path)

class FeatureMatcher():
    def get_scarlet_image(self):
        global SCARLET_DIR
        scarlet_image = cv2.imread(SCARLET_DIR)
        scarlet_image = cv2.resize(scarlet_image, (143, 214))
        return scarlet_image
    
    def get_plum_image(self):
        global PLUM_DIR
        plum_image = cv2.imread(PLUM_DIR)
        plum_image = cv2.resize(plum_image, (143, 214))
        return plum_image
    
    def get_peacock_image(self):
        global PEACOCK_DIR
        peacock_image = cv2.imread(PEACOCK_DIR)
        peacock_image = cv2.resize(peacock_image, (143, 214))
        return peacock_image
    
    def get_mustard_image(self):
        global MUSTARD_DIR
        mustard_image = cv2.imread(MUSTARD_DIR)
        mustard_image = cv2.resize(mustard_image, (143, 214))
        return mustard_image
    
    
    def __init__(self, camera):
        self.scarlet_image = None
        self.plum_image = None
        self.peacock_image = None
        self.mustard_image = None
        self.camera = camera
        self.poster_only_image = None
        
        #self.sift = cv2.SIFT_create()
        self.orb = cv2.ORB_create()
        self.fast = cv2.FastFeatureDetector_create()
        self.brute_force_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.char_pub = rospy.Publisher('/character_found', String, queue_size=0)
        self.image_fm_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.feature_image_callback)

    def apply_gaussian_blur(self, cv_image, kernel_size = 3, sigma = 1):
        cv_image_blur =  cv2.GaussianBlur(cv_image, (kernel_size,kernel_size), sigma)
        return cv_image_blur

    def convert_to_grayscale(self, cv_image):
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        return cv_image_gray

    def canny_edge_detection(self, cv_image_gray, lower=0, upper=0):
        cv_image_gray = cv_image_gray.astype(np.uint8)
        cv_image_edges = cv2.Canny(cv_image_gray, lower, upper)
        return cv_image_edges

    def pipeline(self, cv_image):
        cv_image_blur_gray = self.convert_to_grayscale(cv_image)
        cv_image_blur_edges = self.canny_edge_detection(cv_image_blur_gray, lower=120, upper=120)
        contours, hierarchy = cv2.findContours(cv_image_blur_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv_image_blur_edges = cv2.cvtColor(cv_image_blur_edges, cv2.COLOR_GRAY2RGB)
        return contours, cv_image_blur_edges
    
    
    def poster_only_mask(self, cv_image, poster):
        masked_img = np.zeros_like(cv_image)
        if poster is not None:
            mask = np.zeros_like(cv_image[:,:,0])
            cv2.drawContours(mask, [poster], 0, (255, 255, 255), -1)
            mask = np.dstack((mask,mask,mask))
            masked_img = cv2.bitwise_and(cv_image, mask)
            self.poster_only_image = masked_img
            return masked_img
        else:
            return masked_img

    def sort_and_calc_score(self,matches):
        matches = sorted(matches, key = lambda x:x.distance)
        matches = matches[:5]
        score = 0
        for m in matches:
            score += m.distance
        return score

    def set_keypoints_and_descriptors_scarlet(self):
        self.scarlet_keypoints, self.scarlet_descriptors = self.orb.detectAndCompute(self.scarlet_image, None)
        
    def set_keypoints_and_descriptors_plum(self):
        self.plum_keypoints, self.plum_descriptors = self.orb.detectAndCompute(self.plum_image, None)
        
    def set_keypoints_and_descriptors_peacock(self):
        self.peacock_keypoints, self.peacock_descriptors = self.orb.detectAndCompute(self.peacock_image, None)
        
    def set_keypoints_and_descriptors_mustard(self):
        self.mustard_keypoints, self.mustard_descriptors = self.orb.detectAndCompute(self.mustard_image, None)
        


    def get_scarlet_score(self, descriptors):
        matches = self.brute_force_matcher.match(self.scarlet_descriptors, descriptors)
        score = self.sort_and_calc_score(matches)
        return score

    def get_plum_score(self, descriptors):
        matches = self.brute_force_matcher.match(self.plum_descriptors, descriptors)
        score = self.sort_and_calc_score(matches)
        return score
    
    def get_peacock_score(self, descriptors):
        matches = self.brute_force_matcher.match(self.peacock_descriptors, descriptors)
        score = self.sort_and_calc_score(matches)
        return score
    
    def get_mustard_score(self, descriptors):
        matches = self.brute_force_matcher.match(self.mustard_descriptors, descriptors)
        score = self.sort_and_calc_score(matches)
        return score
    

    def find_character(self):
        if self.scarlet_image is None:
            self.scarlet_image = self.get_scarlet_image()
            self.set_keypoints_and_descriptors_scarlet()
        if self.plum_image is None:
            self.plum_image = self.get_plum_image()
            self.set_keypoints_and_descriptors_plum()
        if self.peacock_image is None:
            self.peacock_image = self.get_peacock_image()
            self.set_keypoints_and_descriptors_peacock()
        if self.mustard_image is None:
            self.mustard_image = self.get_mustard_image()
            self.set_keypoints_and_descriptors_mustard()
        
        
        if (self.poster_only_image is not None):
            keypoints, descriptors = self.orb.detectAndCompute(self.poster_only_image, None)

        try:
            scarlet_score = self.get_scarlet_score(descriptors)
            plum_score = self.get_plum_score(descriptors)
            peacock_score = self.get_peacock_score(descriptors)
            mustard_score = self.get_mustard_score(descriptors)
            ans = min(scarlet_score, plum_score, peacock_score, mustard_score)
            to_publish = String()
            if ans == scarlet_score:
                to_publish.data = ['scarlett',scarlet_score, plum_score, peacock_score, mustard_score]
            elif ans == plum_score:
                to_publish.data = ['plum',scarlet_score, plum_score, peacock_score, mustard_score]
            elif ans == peacock_score:
                to_publish.data = ['peacock',scarlet_score, plum_score, peacock_score, mustard_score]
            elif ans == mustard_score:
                to_publish.data = ['mustard',scarlet_score, plum_score, peacock_score, mustard_score]
            return to_publish.data[0]
        except:
            return 'no poster detected'
        
        
    def feature_image_callback(self, data):
        contours, cv_image_blur_edges = self.pipeline(self.camera.cv_image)
        contour = max(contours, key=cv2.contourArea)
        poster_only_image = self.poster_only_mask(self.camera.cv_image, contour)
        
        #cv2.namedWindow('poster_only')
        #cv2.imshow('poster_only', poster_only_image)
        #cv2.waitKey(3)
        
        
        
