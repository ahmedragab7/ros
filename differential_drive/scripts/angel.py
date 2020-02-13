#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import atan2
from sensor_msgs.msg import Imu

x = 0.0
y = 0.0 
theta = 0.0
stop=0
def newOdom(msg):
    global x
    global y
    global theta
 
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    AZ = msg.pose.pose.orientation.z
    AX = msg.pose.pose.orientation.x
    AY =msg.pose.pose.orientation.y
    AW =msg.pose.pose.orientation.w
    orientation_list=[AX,AY,AZ,AW]
    (roll,pitch,theta)=euler_from_quaternion(orientation_list)
    rospy.loginfo("the main is %f",theta) 
rospy.init_node("speed_controller")
r = rospy.Rate(4)
sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, newOdom)
while not rospy.is_shutdown():
    r.sleep()   
