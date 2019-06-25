#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu,Range
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from collections import deque
import tf



class QueueBuffer(object):
	def __init__(self, window_size):
		self.size = window_size
		self.buffer = deque()
		self.mean = 0.
		self.count = 0.
	
	def insert(self, x):
		if self.count < self.size:
			self.buffer.append(x)
			self.count += 1
		else:
			self.buffer.popleft()
			self.buffer.append(x)
		self.mean = sum(self.buffer)/self.count
	

	def mean(self):
		return self.mean
	

	def size(self):
		return self.count


odom_px4 = Odometry()
def odom_px4_cb(data):
	global odom_px4
	odom_px4 = data


tfmini_range = Range()
def tfmini_cb(data):
	global tfmini_range
	tfmini_range = data

tfMessage = TFMessage()
def tf_cb(data):
	global tfMessage
	tfMessage = data


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

odom_sub = rospy.Subscriber('pilot/local_position/odom', Odometry, odom_px4_cb, queue_size=100)
tfmini_sub = rospy.Subscriber('tfmini_ros_node/TFmini', Range, tfmini_cb, queue_size=100)
odom_pub = rospy.Publisher('odom', Odometry, queue_size=100)


rate = rospy.Rate(20.0)
counter = 0
x = 0.
y = 0.

dt = 1./20.

window_size = 5

prev_vx = QueueBuffer(window_size)
prev_vy = QueueBuffer(window_size)
prev_vz = QueueBuffer(window_size)

while not rospy.is_shutdown():

	(v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(odom_px4.pose.pose.orientation.w, odom_px4.pose.pose.orientation.x , odom_px4.pose.pose.orientation.y, odom_px4.pose.pose.orientation.z)
	v_phi = float((v_roll))
	v_theta = float((v_pitch))
	v_psi = float((v_yaw))
		
	x = odom_px4.pose.pose.position.x
	y = odom_px4.pose.pose.position.y
	z = tfmini_range.range

	yaw = math.radians(v_psi)

	if counter > 0:
		
		twist_z = (z - z_prev) / dt


		odom = Odometry()
		odom.header.frame_id = '/world'
		odom.child_frame_id = '/f450/base_link'
		odom.header.stamp = rospy.Time.now()

		odom.pose.pose.position.x = x
		odom.pose.pose.position.y = y
		odom.pose.pose.position.z = z

		odom.pose.pose.orientation = odom_px4.pose.pose.orientation

		prev_vz.insert(twist_z)

		odom.twist.twist.linear = odom_px4.twist.twist.linear
		odom.twist.twist.linear.z = prev_vz.mean
		z_prev = z

		odom.twist.twist.angular.x = 0.
		odom.twist.twist.angular.y = 0.
		odom.twist.twist.angular.z = 0.

		

		odom_pub.publish(odom)

		br = tf.TransformBroadcaster()
		br.sendTransform((x,y,z),[odom_px4.pose.pose.orientation.x, odom_px4.pose.pose.orientation.y,odom_px4.pose.pose.orientation.z,odom_px4.pose.pose.orientation.w],rospy.Time.now(), "/ironman2/base_link","/world")

	else:
		x_prev = x
		y_prev = y
		z_prev = z
		counter += 1



	rate.sleep()
