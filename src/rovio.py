#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
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


pose = PoseWithCovarianceStamped()
def vicon_cb(data):
	global pose
	pose = data


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

vicon_sub = rospy.Subscriber('/ironman/rovio/pose_with_covariance_stamped', PoseWithCovarianceStamped, vicon_cb, queue_size=100)
odom_pub = rospy.Publisher('odometry', Odometry, queue_size=100)


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

	(v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x , pose.pose.pose.orientation.y, pose.pose.pose.orientation.z)
	v_phi = float((v_roll))
	v_theta = float((v_pitch))
	v_psi = float((v_yaw))

		
	x = pose.pose.pose.position.x
	y = pose.pose.pose.position.y
	z = pose.pose.pose.position.z

	yaw = math.radians(v_psi)

	if counter > 0:
		vel_x_world = (x - x_prev) / dt
		vel_y_world = (y - y_prev) / dt

		x_prev = x
		y_prev = y


		twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
		twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world
		twist_z = (z - z_prev) / dt


		odom = Odometry()
		odom.header.frame_id = '/world'
		odom.child_frame_id = '/ironman/base_link'
		odom.header.stamp = rospy.Time.now()

		odom.pose.pose.position.x = pose.pose.pose.position.x
		odom.pose.pose.position.y = pose.pose.pose.position.y
		odom.pose.pose.position.z = pose.pose.pose.position.z

		odom.pose.pose.orientation.x = pose.pose.pose.orientation.x
		odom.pose.pose.orientation.y = pose.pose.pose.orientation.y
		odom.pose.pose.orientation.z = pose.pose.pose.orientation.z
		odom.pose.pose.orientation.w = pose.pose.pose.orientation.w

		prev_vx.insert(twist_x)
		prev_vy.insert(twist_y)
		prev_vz.insert(twist_z)

		odom.twist.twist.linear.x = prev_vx.mean
		odom.twist.twist.linear.y = prev_vy.mean
		odom.twist.twist.linear.z = prev_vz.mean
		z_prev = z

		odom.twist.twist.angular.x = 0.
		odom.twist.twist.angular.y = 0.
		odom.twist.twist.angular.z = 0.

		

		odom_pub.publish(odom)

		br = tf.TransformBroadcaster()
		br.sendTransform((x,y,z),[pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w],rospy.Time.now(), "/ironman/base_link","/world")

	else:
		x_prev = x
		y_prev = y
		z_prev = z
		counter += 1



	rate.sleep()
