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
 
    x = 0
    y = 0
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    theta = msg.pose.pose.orientation.z
    
 
rospy.init_node("speed_controller")
 
sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
 
speed = Twist()
 
r = rospy.Rate(4)
 
goal = Point()
goal.x = 0.7
goal.y = -0.3
 
while not rospy.is_shutdown():
    
    inc_x = goal.x -x
    inc_y = goal.y -y
 
    angle_to_goal = atan2(inc_y, inc_x)
    rospy.loginfo( "the angel %f", angle_to_goal)
    if ((abs(angle_to_goal - theta) > 0.2) and stop==0):
        speed.linear.x = 0.0
        speed.angular.z = 1.0
    else:
        stop=1
        if (goal.x > x)  :
            speed.linear.x = -0.5
            speed.angular.z = 0.0
          
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0

     
    pub.publish(speed)
    r.sleep()    


