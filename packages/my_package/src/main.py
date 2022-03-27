#!/usr/bin/env python3

import os
import time
import roslib
import sys
import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from cv_bridge import CvBridgeError,CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Empty, EmptyResponse
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

###The main idea of how to use camera to follow the track is inspired by the following
###including how to derive the masked images and calculating movement of robot to follow the center of track
###source:https://www.youtube.com/watch?v=ukGa74saFfM&t=3327s


class MyNode(DTROS):
    def __init__(self,node_name):
        super(MyNode, self).__init__(node_name = node_name, node_type=NodeType.GENERIC)
        print("initializing")
        self.host = str(os.environ['VEHICLE_NAME'])
        self.start_time = time.time()
        #rospy.init_node("my_node", anonymous=True)
        rospy.on_shutdown(self.shut_down_info)
        self.bridge_object = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        self.image_sub = rospy.Subscriber("/%s/camera_node/image/compressed"%self.host, CompressedImage, self.camera_callback, queue_size=1)
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/%s/car_cmd_switch_node/cmd'%self.host,Twist2DStamped, queue_size=1)
        #self.turtlebot = Robot()
        self.default_linear_speed = 0.2         # v
        #self.current_linear_speed = 0.2
        self.match_stop_sign = False
        self.red_zone_detected = False
        self.descenter = 15
        self.descenter = 50
        self.shutdown = False
        rospy.spin()
        #self.hz = 60
        #self.rate = rospy.Rate(self.hz)
        #while not rospy.is_shutdown() and not self.shutdown:
        #    self.rate.sleep()


    def shut_down_info(self):
        print("shuting down")
        self.cmd_vel_pub.publish(Twist2DStamped())
        print("The running time of this node is %f seconds"%(time.time() - self.start_time))
		
    def camera_callback(self, data):
        if self.shutdown == True:
            return    
        try:
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except  CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        print(cv_image.shape)

        """
        crop the image so that the robot only make move decision based the closest(bottom-most) area without affected by
        roads that are far away
        """
        width_crop_start = int(0.25*width)
        width_crop_end = int(0.75*width)
        width_crop_start = 160
        width_crop_end = 480
        rows_to_watch = 100
        crop_img =cv_image[int((height)/2+ self.descenter):int((height)/2)+int(self.descenter+rows_to_watch),220:460,:]
        
        height, width, channels = crop_img.shape
        print("crop_img:",crop_img.shape)
        hsv = cv2.cvtColor(crop_img,cv2.COLOR_BGR2HSV)

        """ 
        #convert img from RGB to HSV
        #the code used to find the hsv threshold for each of blue,green,red, white color is from
        #source:https://github.com/botforge/ColorTrackbar/blob/master/HSV%20Trackbar.py
        """
        lower_all_color = np.array([0,0,193])
        upper_all_color = np.array([177,255,255])
        
        lower_red = np.array([0,18,0])
        upper_red = np.array([36,255,232])
        lower_blue = np.array([117,18,0])
        upper_blue = np.array([125,255,232])
        lower_green = np.array([53,0,141])
        upper_green = np.array([61,255,255])
        
        lower_white = np.array([0,0,193])
        upper_white = np.array([0,0,255])
        
        lower_yellow = np.array([20,150,150])
        upper_yellow = np.array([55,255,255])
        mask_path = cv2.inRange(hsv, lower_all_color,upper_all_color)
        mask_red = cv2.inRange(hsv, lower_red,upper_red)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv,lower_white, upper_white)
	
        """
        ###the idea of using contours to check if there is a center in masked image is inspired by this video
        ###source:https://www.youtube.com/watch?v=tlkWX7R-NHE&t=307s
        ###since I will detect 
        
        """
        contours_yellow, hierachy_yellow = cv2.findContours(mask_yellow, 1, cv2.CHAIN_APPROX_NONE)
        contours_red,hierachy_red = cv2.findContours(mask_red,1,cv2.CHAIN_APPROX_NONE)
        
        mask = cv2.inRange(hsv, lower_white, upper_white)

        #m_path = cv2.moments(mask_path, False)
        m_path = cv2.moments(mask_yellow, False)
        m_red = cv2.moments(mask_red, False)
        
        error_x = 0
        if len(contours_yellow) > 0:
        
            print("m",m_path)
            try:
              cx,cy = m_path['m10']/m_path['m00'],m_path['m01']/m_path['m00']
              error_x = cx - width/2
              print("yellow path")
            except ZeroDivisionError:
              cx,cy = height/2, width/2
            res = cv2.bitwise_and(crop_img,crop_img,mask=mask_path)
        
        twist_object = Twist2DStamped()

        """
        check if track of special colors other than oridinary white track
        """

        #if len(contours_red) > 0:
        #    cx = width/2
        #    twist_object.omega = 0
        #    self.red_zone_detected = True
        #    print("red path, slow down")
        #    cx,cy = m_red['m10']/m_red['m00'],m_red['m01']/m_red['m00']
        #    twist_object.v = 0.5 * self.default_linear_speed
        #self.red_zone_detected = False
        """
        #when a red track is detected, start to match the whole image captured by robot camera with the stop sign image
        #the implementation code of using SIFT feature detection and FLANN Matcher is from the openCV tutorial 
        #source:https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html
        """
        
        if self.red_zone_detected==True and self.match_stop_sign==False:
            im1 = cv2.imread("../world/stop.png",0)
            print("stop sign img:", im1.shape)
            #cv_image = crop_img
            MIN_MATCH_COUNT = 7
            sift = cv2.SIFT_create()
		
		    #find the keypoinst and descriptors with SIFT
            kp1, des1 = sift.detectAndCompute(cv_image,None)
            kp2, des2 = sift.detectAndCompute(im1, None)
            
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks = 50)
		
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            
            matches = flann.knnMatch(des1, des2,k=2)
            
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
            
            if len(good)>MIN_MATCH_COUNT:
                print("stop sign found!!!!")
                src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matchesMask = mask.ravel().tolist()
                h,w = im1.shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)
                img2 = cv2.polylines(im1,[np.int32(dst)],True,255,3, cv2.LINE_AA)
                self.match_stop_sign = True
            else:
                print( "Not enough matches are found - {}/{}".format(len(good),MIN_MATCH_COUNT))
                matchesMask = None

            draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                singlePointColor = None,matchesMask = matchesMask, # draw only inliers 
                flags = 2)
            img3 = cv2.drawMatches(cv_image,kp1,im1,kp2,good,None,**draw_params)
            #cv2.imshow("match",img3)

        
        print("error_x:",error_x)
        
        twist_object.omega = -error_x/40
        twist_object.omega = -error_x/25
        twist_object.v = self.default_linear_speed 
        print("current omega:",twist_object.omega)
        print("current speed:",twist_object.v)
        
        if self.match_stop_sign == True:
            twist_object.v = 0
            twist_object.omega = 0

        #self.turtlebot.move_robot(twist_object)
        #########################cv2.circle(res,(int(cx),int(cy)),10,(0,0,255),-1)
	
        #cv2.imshow("Original", cv_image)
        #cv2.imshow("HSV",hsv)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', mask_red)[1]).tostring()
        msg.data = np.array(cv2.imencode('.jpg', mask_yellow)[1]).tostring()
        self.image_pub.publish(msg)
        self.cmd_vel_pub.publish(twist_object)
        #cv2.imshow("Path Decision",res)
        
        #cv2.imshow("blue mask", mask_blue)
        #cv2.imshow("green mask", mask_green)
        #cv2.imshow("red mask", mask_red)
        
        """
        if self.match_stop_sign == True:
            print("ready to shut down")
            time.sleep(20)
            self.shutdown = True
        """
"""
class Robot():
    def __init__(self):
        self.host = str(os.environ['VEHICLE_NAME'])
        self.cmd_vel_pub = rospy.Publisher('/%s/car_cmd_switch_node/cmd'%self.host,Twist2DStamped, queue_size=1)
        self.cmd_vel_subs = rospy.Subscriber('/%s/car_cmd_switch_node/cmd'%self.host, Twist2DStamped, self.cmdvel_callback)
        self.last_cmdvel_command = Twist2DStamped()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.shutdown_detected = False

    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg
        print("last cmdvel command:",msg)

    def compare_twist_commands(self, twist1,twist2):
        LX = twist1.linear.x == twist2.linear.x
        LY = twist1.linear.y == twist2.linear.y
        LZ = twist1.linear.z == twist2.linear.z
        AX = twist1.angular.x == twist2.angular.x
        AY = twist1.angular.y == twist2.angular.y
        AZ = twist1.angular.z == twist2.angular.z
        equal = LX and LY and LZ and AX and AY and AZ

        return equal
    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
        self._cmdvel_pub_rate.sleep()
"""
"""
class Robot():
    def __init__(self):
        #self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        #self.cmd_vel_subs = rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback)
        self.cmd_vel_pub = rospy.Publisher('/csc22908/kinematics_node/velocity',Twist, queue_size=10)
        self.cmd_vel_subs = rospy.Subscriber('/csc22908/kinematics_node/velocity', Twist, self.cmdvel_callback)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.shutdown_detected = False

    def cmdvel_callback(self,msg):
        self.last_cmdvel_command = msg

    def compare_twist_commands(self, twist1,twist2):
        LX = twist1.linear.x == twist2.linear.x
        LY = twist1.linear.y == twist2.linear.y
        LZ = twist1.linear.z == twist2.linear.z
        AX = twist1.angular.x == twist2.angular.x
        AY = twist1.angular.y == twist2.angular.y
        AZ = twist1.angular.z == twist2.angular.z
        equal = LX and LY and LZ and AX and AY and AZ

        return equal
    def move_robot(self, twist_object):
        current_equal_to_new = False
        while (not (current_equal_to_new) and not(self.shutdown_detected)):
            self.cmd_vel_pub.publish(twist_object)
            self._cmdvel_pub_rate.sleep()
            current_equal_to_new = self.compare_twist_commands(twist1=self.last_cmdvel_command,twist2=twist_object)
"""


def main():
    print("Start following lines")
    #time.sleep(5)

    try:
        my_node = MyNode(node_name = "path_finder")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
if __name__ == '__main__':
    main()
