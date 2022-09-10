#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import cv2
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##camera matrix#############
mtx = np.array([[277.191356, 0. , 3],
                [0. , 277.191356, -3],
                [0. , 0. , 1. ]])
# 320.5 , 240.5
####distortion coefficient##
dist = np.array([[0.0,0.0,0.0,0.0,0.0]])

marker_size_id4 = 0.7
marker_size_id3 = 0.2
axis_length = 0.1
rvecs = []
tvecs = []

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
       i = cv2.cvtColor(im.copy(),cv2.COLOR_BGR2GRAY)
       corners,ids,rejected = cv2.aruco.detectMarkers(i,arucoDict,parameters = arucoParams)
    return corners,ids
################################
    
################################
position = PoseStamped()
def present_pos(data):
    global position
    position = data
################################

# callback method for state sub
mavros.set_namespace()
current_state = State() 
offb_set_mode = SetMode

def state_cb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)

get_present_pos = rospy.Subscriber('mavros/local_position/pose',PoseStamped,present_pos)

#subscribing to camera eliment
fpvimage = rospy.Subscriber('/iris/usb_cam/image_raw',Image, imagecv)

def landing_control():
    rate = rospy.Rate(20.0)
    global image
    while not rospy.is_shutdown():
         height = position.pose.position.z
         x_value= position.pose.position.x
         y_value= position.pose.position.y
         corners,ids = arucodetection(image)
         if (len(corners) >0):
            image = cv2.aruco.drawDetectedMarkers(image.copy(),corners,ids)
            if height > 0.5:
               if 4 in ids :
                 rvecs,tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_id4, mtx, dist)
                 k = np.where(ids==[4])
                 a = k[0][0]
                 pose.pose.position.x = -5
                 pose.pose.position.y = 0
                 pose.pose.position.z = 1
                 pose.header.stamp = rospy.Time.now()
                 local_pos_pub.publish(pose)
                 print(4,tvecs,(x_value + 5,y_value,height))
            else:
               if 3 in ids :
                 rvecs,tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_id3, mtx, dist)
                 l = np.where(ids==[3])
                 b = l[0][0]
                 pose.pose.position.x = -5
                 pose.pose.position.y = 0
                 pose.pose.position.z = 0
                 pose.header.stamp = rospy.Time.now()
                 local_pos_pub.publish(pose)
                 print(3,tvecs)
         rate.sleep()
###################
pose = PoseStamped()
pose.pose.position.x = -5
pose.pose.position.y = 0
pose.pose.position.z = 10

def detect_landing():
    rospy.init_node('arucolanding', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        height = position.pose.position.z
        if (height < 9.9):
          now = rospy.get_rostime()
          if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
             set_mode_client(base_mode=0, custom_mode="OFFBOARD")
             last_request = now 
          else:
             if not current_state.armed and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
          if prev_state.armed != current_state.armed:
              rospy.loginfo("Vehicle armed: %r" % current_state.armed)
          if prev_state.mode != current_state.mode: 
              rospy.loginfo("Current mode: %s" % current_state.mode)
          prev_state = current_state

        # Update timestamp and publish pose 
          pose.header.stamp = rospy.Time.now()
          local_pos_pub.publish(pose)
          print(height)
          rate.sleep()
        else:
          landing_control()
          rospy.spin() 
if __name__ == '__main__':
    try:
        detect_landing()
    except rospy.ROSInterruptException:
           pass
