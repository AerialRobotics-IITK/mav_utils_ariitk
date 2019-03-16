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
def rovio_cb(data):
	global odom_
	odom_ = data


rospy.init_node('viimu_to_pilot')

vicon_sub = rospy.Subscriber('/ironman/rovio/odometry', Odometry, rovio_cb, queue_size=100)
odom_pub = rospy.Publisher('odometry', Odometry, queue_size=100)


rate = rospy.Rate(20.0)


while not rospy.is_shutdown():
 
    x = odom_.pose.pose.position.x
    y = odom_.pose.pose.position.y
    z = odom_.pose.pose.position.z

    odom = Odometry()
    # odom=odom_
    odom.header.frame_id = '/world'
    odom.child_frame_id = '/ironman'
    odom.header.stamp = rospy.Time.now()
    odom.pose.pose.position.x = x-0.02
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = z+0.06

    odom.pose.pose.orientation = odom_.pose.pose.orientation
    odom.twist.twist = odom_.twist.twist

        


    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),[odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,odom_.pose.pose.orientation.z,odom_.pose.pose.orientation.w],rospy.Time.now(), "/ironman","/world")

    odom_pub.publish(odom)

    rate.sleep()
