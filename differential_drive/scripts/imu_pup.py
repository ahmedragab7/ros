#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg  import Imu
AZ=0
def callback(data):
    global AZ
    AZ = data.data
   
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", AZ)
def listener():
    global AZ
    imu_msg=Imu()
    pub=rospy.Publisher('imu_data',Imu,queue_size=10)
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imu_puplisher', anonymous=True)
    
    rospy.Subscriber("imu_ard", Float32, callback)
    rate = rospy.Rate(10) 
    # spin() simply keeps python from exiting until this node is stopped
    
    while not rospy.is_shutdown():
        imu_msg.linear_acceleration.z =AZ
        imu_msg.header.frame_id ='odom' 
        pub.publish(imu_msg)
        rate.sleep()


  
if __name__ == '__main__':
    listener()
    
