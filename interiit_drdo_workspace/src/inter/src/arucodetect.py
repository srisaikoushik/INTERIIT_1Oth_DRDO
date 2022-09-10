#!/usr/bin/env python

import rospy
import mavros
import cv2
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##camera matrix#############
mtx = np.array([[277.191356, 0. , 0],
                [0. , 277.191356, 0],
                [0. , 0. , 1. ]])
# 320.5 , 240.5
####distortion coefficient##
dist = np.array([[0.0,0.0,0.0,0.0,0.0]])

marker_size_id4 = 0.7
marker_size_id3 = 0.2
axis_length = 0.1
rvecs = []
tvecs = []

font = cv2.FONT_HERSHEY_PLAIN
# callback method for state sub
mavros.set_namespace()
################################
#callback for image subscription
bridge = CvBridge()
arucoDict = cv2.aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters_create()
image = np.zeros((240,320,3))
def imagecv(img):
    global image
    try:
      image = bridge.imgmsg_to_cv2(img,'bgr8')
      cv2.imshow("image_window",image)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)
################################
def arucodetection(im):
    (rows,cols,channels) = im.shape
    if (rows >0) & (cols > 0) :
       corners,ids,rejected = cv2.aruco.detectMarkers(im,arucoDict,parameters = arucoParams)
    return corners,ids
################################
    
################################
position = PoseStamped()
def present_pos(data):
    global position
    position = data
################################
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)

get_present_pos = rospy.Subscriber('mavros/local_position/pose',PoseStamped,present_pos)
 
#subscribing to cam eliment
fpvimage = rospy.Subscriber('/iris/usb_cam/image_raw',Image, imagecv)

pose = PoseStamped()
#pose.pose.position.x = 0
#pose.pose.position.y = 0
pose.pose.position.z = 5

def landing_control():
    rospy.init_node('arucodetect', anonymous=True)
    global image
    global pos
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    height = position.pose.position.z
    while not rospy.is_shutdown():
         corners,ids = arucodetection(image)
         if (len(corners) >0):
            image = cv2.aruco.drawDetectedMarkers(image.copy(),corners,ids)

            if 4 in ids and not 3 in ids :
               rvecs,tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_id4, mtx, dist)
               print( "4",tvecs[0,0,2])
               pose.pose.position.x = position.pose.position.x
               pose.pose.position.y = position.pose.position.y
               pose.pose.position.z = 3
               pose.header.stamp = rospy.Time.now()
               local_pos_pub.publish(pose)
            if (3 in ids):
               rvecs,tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_id3, mtx, dist)
               print( "3",tvecs)
              
         rate.sleep()

if __name__ == '__main__':
    try:
        landing_control()
    except rospy.ROSInterruptException:
           pass
