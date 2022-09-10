#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# callback method for state sub
mavros.set_namespace()
current_state = State()
current_state1 = State()
q = PoseStamped()
offb_set_mode = SetMode

def state_cb(state):
    global current_state
    current_state = state
def state_cb1(state):
    global current_state1
    current_state1 = state
def call(data):
    global q
    q = data
    
local_pos_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
local_pos_pub1 = rospy.Publisher('uav2/mavros/setpoint_position/local',PoseStamped, queue_size=10)
v = rospy.Subscriber('uav0/mavros/local_position/pose',PoseStamped,call)
state_sub = rospy.Subscriber('uav1/mavros/state', State, state_cb)
state_sub1 = rospy.Subscriber('uav2/mavros/state',State, state_cb1)
arming_client = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)
arming_client1 = rospy.ServiceProxy('uav2/mavros/cmd/arming',CommandBool)
set_mode_client = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode) 
set_mode_client1 = rospy.ServiceProxy('uav2/mavros/set_mode',SetMode)

y = PoseStamped()
y1 = PoseStamped()
def position_control():
    rospy.init_node('d', anonymous=True)
    prev_state = current_state
    prev_state1 = current_state1
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()
    while not current_state1.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    last_request1 = rospy.get_rostime();
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        now1 = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 
        
        if current_state1.mode != "OFFBOARD" and (now1 - last_request1 > rospy.Duration(5.)):
            set_mode_client1(base_mode=0, custom_mode="OFFBOARD")
            last_request1 = now1 
        else:
            if not current_state1.armed and (now1 - last_request1 > rospy.Duration(5.)):
               arming_client1(True)
               last_request1 = now1
        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state
        
        if prev_state1.armed != current_state1.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state1.armed)
        if prev_state1.mode != current_state1.mode: 
            rospy.loginfo("Current mode: %s" % current_state1.mode)
        prev_state1 = current_state1

        # Update timestamp and publish pose
        y.pose.position.x = q.pose.position.x +1
        y.pose.position.y = q.pose.position.y
        y.pose.position.z = q.pose.position.z

        y1.pose.position.x = q.pose.position.x
        y1.pose.position.y = q.pose.position.y +1
        y1.pose.position.z = q.pose.position.z

        y1.header.stamp = rospy.Time.now()
        y.header.stamp = rospy.Time.now()
        local_pos_pub.publish(y)
        local_pos_pub1.publish(y1)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
           pass
