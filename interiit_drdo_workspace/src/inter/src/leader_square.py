#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# callback method for state sub
mavros.set_namespace()
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher('uav0/mavros/setpoint_position/local',PoseStamped,queue_size = 10)
state_sub = rospy.Subscriber('uav0/mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('uav0/mavros/set_mode', SetMode) 

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

def position_control():
    rospy.init_node('of_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(25.0) # MUST be more then 2Hz
    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
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
        pose.pose.position.y = 0
        for i in range(100):
            local_pos_pub.publish(pose)
            rate.sleep()
        pose.pose.position.y = -2
        for i in range(100):
            local_pos_pub.publish(pose)
            rate.sleep()
        pose.pose.position.x = 2
        for i in range(100):
            local_pos_pub.publish(pose)
            rate.sleep()
        pose.pose.position.y = 0
        for i in range(100):
            local_pos_pub.publish(pose)
            rate.sleep()
        pose.pose.position.x = 0
        for i in range(100):
            local_pos_pub.publish(pose)
            rate.sleep()
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
           pass