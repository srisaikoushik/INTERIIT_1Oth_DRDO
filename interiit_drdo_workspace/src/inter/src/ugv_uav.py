#!/usr/bin/env python

import rospy
import mavros
import cv2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from prius_msgs.msg import *
from prius_msgs.srv import *

# callback method for state sub
mavros.set_namespace()
current_state = State()
q = PoseStamped()
offb_set_mode = SetMode

def state_cb(state):
    global current_state
    current_state = state

def call(data):
    global q
    q = data
    
local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
v = rospy.Subscriber('mavros/local_position/pose',PoseStamped,call)
state_sub = rospy.Subscriber('ugv/mavros/state', State, state_cb)

local_pos_pub = rospy.Publisher('/prius',Control, queue_size=10)


y = PoseStamped()

def position_control():
    rospy.init_node('d', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        
        # older versions of PX4 always return success==True, so better to check Status instead
        prev_state = current_state

        # Update timestamp and publish pose
        y.pose.position.x = q.pose.position.x +1
        y.pose.position.y = q.pose.position.y
        y.pose.position.z = 0


        
        y.header.stamp = rospy.Time.now()
        local_pos_pub.publish(y)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
           pass
