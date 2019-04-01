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
def tf_cb(data):
	global odom_
	odom_ = data


def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

rospy.init_node('pose_to_odom')

local_sub = rospy.Subscriber('/ironman/pilot/local_position/odom', Odometry, tf_cb, queue_size=100)
tf_pub = rospy.Publisher('/pose', TransformStamped, queue_size=100)


rate = rospy.Rate(20.0)


while not rospy.is_shutdown():



	if (1):
		
		tf_ = TransformStamped()
		tf_.header.frame_id = '/world'
		tf_.child_frame_id = '/imu'
		tf_.header.stamp = rospy.Time.now()

		tf_.transform.translation.x = odom_.pose.pose.position.x
		tf_.transform.translation.y = odom_.pose.pose.position.y
		tf_.transform.translation.z = odom_.pose.pose.position.z

		tf_.transform.rotation.x = odom_.pose.pose.orientation.x
		tf_.transform.rotation.y = odom_.pose.pose.orientation.y
		tf_.transform.rotation.z = odom_.pose.pose.orientation.z
		tf_.transform.rotation.w = odom_.pose.pose.orientation.w
		
		tf_pub.publish(tf_)

	rate.sleep()
