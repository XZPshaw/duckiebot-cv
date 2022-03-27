#!/usr/bin/python3.8

from numpy import dtype
import rospy
import cv2 as cv
from sensor_msgs.msg import Image	
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


'''
This code provides a slidebar to adjust the HSV range 
to filter out the corresponding color in turtlebot camera view.
The url is: https://github.com/botforge/ColorTrackbar/blob/master/HSV%20Trackbar.py
Great thanks to the author, it helps me resolves the HSV range of of the path
'''
bridge = CvBridge()
# optional argument for trackbars
def nothing(x):
    pass

# named ites for easy reference
barsWindow = 'Bars'
hl = 'H Low'
hh = 'H High'
sl = 'S Low'
sh = 'S High'
vl = 'V Low'
vh = 'V High'



# create window for the slidebars
cv.namedWindow(barsWindow, flags = cv.WINDOW_AUTOSIZE)

# create the sliders
cv.createTrackbar(hl, barsWindow, 0, 179, nothing)
cv.createTrackbar(hh, barsWindow, 0, 179, nothing)
cv.createTrackbar(sl, barsWindow, 0, 255, nothing)
cv.createTrackbar(sh, barsWindow, 0, 255, nothing)
cv.createTrackbar(vl, barsWindow, 0, 255, nothing)
cv.createTrackbar(vh, barsWindow, 0, 255, nothing)

# set initial values for sliders
cv.setTrackbarPos(hl, barsWindow, 0)
cv.setTrackbarPos(hh, barsWindow, 179)
cv.setTrackbarPos(sl, barsWindow, 0)
cv.setTrackbarPos(sh, barsWindow, 255)
cv.setTrackbarPos(vl, barsWindow, 0)
cv.setTrackbarPos(vh, barsWindow, 255)


def imgCallback(data):
  #cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  cap = bridge.imgmsg_to_cv2(data, "bgr8")
  #ret, frame = data.read()
  #frame = cv.GaussianBlur(frame, (5, 5), 0)
    
  # convert to HSV from BGR
  hsv = cv.cvtColor(cap, cv.COLOR_BGR2HSV)

  # read trackbar positions for all
  hul = cv.getTrackbarPos(hl, barsWindow)
  huh = cv.getTrackbarPos(hh, barsWindow)
  sal = cv.getTrackbarPos(sl, barsWindow)
  sah = cv.getTrackbarPos(sh, barsWindow)
  val = cv.getTrackbarPos(vl, barsWindow)
  vah = cv.getTrackbarPos(vh, barsWindow)

  # make array for final values
  HSVLOW = np.array([hul, sal, val])
  HSVHIGH = np.array([huh, sah, vah])

  # apply the range on a mask
  mask = cv.inRange(hsv, HSVLOW, HSVHIGH)
  maskedFrame = cv.bitwise_and(cap, cap, mask = mask)

  # display the camera and masked images
  cv.imshow('Masked', maskedFrame)
  cv.imshow('Camera', cap)

  cv.waitKey(3)

def main():
  rospy.init_node('my_planner_node')
  img_sub = rospy.Subscriber("/camera/image_raw", Image, imgCallback)
  # # clean up our resources
  # cap.release()
  # cv.destroyAllWindows()

  
  rospy.spin()

if __name__ == "__main__":
  main()

