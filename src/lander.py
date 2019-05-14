#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from collections import deque
import tf


odom_ = Odometry()
def odom_cb(data):
	global odom_
	odom_ = data

obj_ = PoseStamped()
flag_ = False
def obj_cb(data):
    global obj_
    global flag_
    obj_ = data
    flag_ = True


rospy.init_node('offboard')

# global_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, odom_cb, queue_size=100)
local_sub = rospy.Subscriber('/ironman/tfmini_odom', Odometry, odom_cb, queue_size=100)
obj_sub = rospy.Subscriber('/ironman/object/pose', PoseStamped, obj_cb, queue_size=100)
#obj_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, obj_cb, queue_size=100)

set_pub = rospy.Publisher('/ironman/command/pose', PoseStamped, queue_size=100)


rate = rospy.Rate(20.0)
count = 0

while not rospy.is_shutdown():
 
  
    if (flag_):
        pose_ = PoseStamped()
        pose_.header.stamp = rospy.Time.now()
        pose_.pose.position.x = obj_.pose.position.x 
        pose_.pose.position.y = obj_.pose.position.y 
        if((odom_.pose.pose.position.z-0.002*count)>1.5):
            pose_.pose.position.z = odom_.pose.pose.position.z - 0.002*(count)
            h =  odom_.pose.pose.position.z
        else:
            pose_.pose.position.z = 1.5
        pose_.pose.orientation = odom_.pose.pose.orientation
        count=count+1
        if(count%20==0 ):
	   # count = count +1
            flag_ = False
	    #pose_.pose.position.x = odom_.pose.pose.position.x +  obj_.pose.position.x
            #pose_.pose.position.y = odom_.pose.pose.position.y +  obj_.pose.position.y
            set_pub.publish(pose_)
            count=count+1
    rate.sleep()
