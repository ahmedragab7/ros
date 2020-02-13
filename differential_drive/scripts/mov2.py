#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import atan2, sqrt, pow
from sensor_msgs.msg import Imu

x = 0.0
y = 0.0 
theta = 0.0
stop=0
dis_now=0.0
def newOdom(msg):
    global x
    global y
    global theta
    global dis_now
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    dis_now=sqrt(pow(x,2)+pow(y,2))
    AZ = msg.pose.pose.orientation.z
    AX = msg.pose.pose.orientation.x
    AY =msg.pose.pose.orientation.y
    AW =msg.pose.pose.orientation.w
    orientation_list=[AX,AY,AZ,AW]
    (roll,pitch,theta)=euler_from_quaternion(orientation_list)
    rospy.loginfo("the main is %f",theta) 
rospy.init_node("speed_controller")
 
sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
 
speed = Twist()
 
r = rospy.Rate(4)
 
goal = Point()
goal.x = 0.4
goal.y = 0.2
distance=sqrt(pow(goal.x,2)+pow(goal.y,2))
while not rospy.is_shutdown():
    
    inc_x = goal.x -x
    inc_y = goal.y -y
 
    angle_to_goal = atan2(inc_y, inc_x)
    #angle_to_goal=angle_to_goal/3.14
    rospy.loginfo( "the angel %f", angle_to_goal)
    if ((abs(angle_to_goal - theta) > 0.2) and stop==0):
        rospy.loginfo( "in angular")
        if (angle_to_goal - theta) > 0:
            speed.linear.x = 0.0
            speed.angular.z = -1.0
            #rospy.loginfo( "in wrong")
        else:
            speed.linear.x = 0.0
            speed.angular.z = 1.0
            rospy.loginfo( "in right") 
    else:
        #rospy.loginfo( "in distance")
        stop=1
        print("i Am here")
        print(distance)
        print("dis_now")
        print(dis_now)
        if abs(distance-dis_now)>0.05  :
            speed.linear.x = 0.5
            speed.angular.z = 0.0
          
        else:
            
            speed.angular.z = 0.0
	    speed.linear.x = 0.0
     
    pub.publish(speed)
    r.sleep()    


