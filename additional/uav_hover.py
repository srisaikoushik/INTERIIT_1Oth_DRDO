import rospy
import mavros
import math
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped,Twist
from mavros_msgs.msg import *
from mavros_msgs.srv import *

mavros.set_namespace()

bridge = CvBridge()

lower = np.array([0,248,134])
upper = np.array([171,255,140])

X = 320
Y = 240

cx = 320
cy = 240
rospy.init_node('offb', anonymous=True)

#drone velocity
twist = Twist()

twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0

def follow(cx,cy):
    vy = -float(cx -X)/10
    vx = -float(cy - Y)/10
    a = vx**2+vy**2
    v = math.sqrt(a)
    print(vx,vy,v)
    if(v >= 0.5):
        wx = (vx/v)*0.5
        wy = (vy/v)*0.5
    else:
        wx = 0
        wy = 0
    twist.linear.x = wx
    twist.linear.y = wy
    uav_vel.publish(twist)


def image_view(data):
    img = bridge.imgmsg_to_cv2(data,desired_encoding=data.encoding)
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    # car_detect = cv2.CascadeClassifier("cars.xml")
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # cars = car_detect.detectMultiScale(gray, 1.2, 1)

    # for (x,y,w,h) in cars:
    #     cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
    #     cx = int(x+(w/2))
    #     cy = int(y+(h/2))
    #     cv2.circle(img, (cx,cy), 7, (0, 255,0), -1)      
    img_hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv,lower,upper)

    edged = cv2.Canny(mask, 170, 255) #Determine edges of objects in an image
    ret,thresh = cv2.threshold(edged,240,255,cv2.THRESH_BINARY)  
    contours = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cx = 320
    cy = 240
    if len(contours[1]) != 0:
        center = contours[1][0]
        M = cv2.moments(center)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # cv2.drawContours(img, [center], -1, (0, 255, 0), 2)
    follow(cx,cy)

    # cv2.circle(img, (cx, cy), 7, (0, 255,0), -1)
    cv2.circle(img, (X, Y), 7, (255, 255,0), -1)
    cv2.imshow("img",img)
    cv2.waitKey(1)
    # cv2.destroyAllWindows()
uav_vel = rospy.Publisher(mavros.get_topic('setpoint_velocity','cmd_vel_unstamped'), Twist,queue_size=10)
img_sub = rospy.Subscriber("/depth_camera/rgb/image_raw",Image,image_view)



def uav_hover():
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    
    while not rospy.is_shutdown():
        pass
        

if __name__ == '__main__':
    try:
        uav_hover()
    except rospy.ROSInterruptException:
           pass




