#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, Transform, TwistStamped
from tf2_msgs.msg import TFMessage
import tf
import numpy as np
from mav_msgs.msg import RollPitchYawrateThrust
from std_msgs.msg import String
import time
import argparse
from datetime import datetime


parser = argparse.ArgumentParser()
parser.add_argument("data_points",help="data_points",type=int)
args = parser.parse_args()
data_points = args.data_points


now = datetime.now()
log_dir = str(now.strftime("%Y%m%d-%H%M%S")) + '_'


odom = Odometry()
def odom_cb(data):
	global odom
	odom = data

vrpn = TwistStamped()
def vrpn_cb(data):
	global vrpn
	vrpn = data


rpyth = RollPitchYawrateThrust()
def rpyth_cb(data):
	global rpyth
	rpyth = data


state = String()
def state_cb(data):
	global state
	state = data




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


rospy.init_node('collector_node')
odom_sub = rospy.Subscriber('/f_450/odometry', Odometry, odom_cb, queue_size=100)
pose_pub = rospy.Publisher('/f_450/command/pose', PoseStamped, queue_size=100)
vrpn_sub = rospy.Subscriber('/f_450/vrpn_client_node/f_450/twist',TwistStamped,vrpn_cb, queue_size=100)
rpyt_sub = rospy.Subscriber('/f_450/mavros/setpoint_raw/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, rpyth_cb, queue_size=100)
state_sub = rospy.Subscriber('/f_450/state_machine/state_info', String, state_cb, queue_size=100)


rate = rospy.Rate(50.0)
counter = 0


x_f = []
y_f = []
z_f = []

vx_f = []
vy_f = []
vz_f = []

r_f = []
p_f = []
yaw_f = []


rs_f = []
ps_f = []
ys_f = []

x_des = []
y_des = []
z_des = []

x_sp = []
y_sp = []
z_sp = []


roll_sp = []
pitch_sp = []
yaw_rate_sp = []
thrust_sp = []

target = [-1.6, -0.3, 0.5]

target_pose = PoseStamped()

target_pose.header.stamp = rospy.Time.now()
target_pose.pose.position.x = target[0]
target_pose.pose.position.y = target[1]
target_pose.pose.position.z = target[2]

# target_pose.pose.orientation.x = 0.
# target_pose.pose.orientation.y = 0.
# target_pose.pose.orientation.z = 0.7071068
# target_pose.pose.orientation.w = 0.7071068


pose_pub.publish(target_pose)
np.random.seed(125)

while not rospy.is_shutdown():

	x_f.append(float(odom.pose.pose.position.x))
	y_f.append(float(odom.pose.pose.position.y))
	z_f.append(float(odom.pose.pose.position.z))

	vx_f.append(float(odom.twist.twist.linear.x))
	vy_f.append(float(odom.twist.twist.linear.y))
	vz_f.append(float(odom.twist.twist.linear.z))

	(roll,pitch, yaw) = quaternion_to_euler_angle(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x , odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)
	r_f.append(float(math.radians(roll)))
	p_f.append(float(math.radians(pitch)))
	yaw_f.append(float(math.radians(yaw)))

	rs_f.append(float(vrpn.twist.angular.x))
	ps_f.append(float(vrpn.twist.angular.y))
	ys_f.append(float(vrpn.twist.angular.z))


	x_sp.append(target[0])
	y_sp.append(target[1])
	z_sp.append(target[2])

	roll_sp.append(rpyth.roll)
	pitch_sp.append(rpyth.pitch)
	yaw_rate_sp.append(rpyth.yaw_rate)
	thrust_sp.append(rpyth.thrust.z)



	curr_pos = [float(odom.pose.pose.position.x),  float(odom.pose.pose.position.y), float(odom.pose.pose.position.z)]
	if abs(curr_pos[0] - target[0]) < 0.1 and  abs(curr_pos[1] - target[1]) < 0.1 and abs(curr_pos[2] - target[2]) < 0.15:
		sample_x = np.random.uniform(low=-2.2 , high=-1.0)
		sample_y = np.random.uniform(low=-2.0, high=1.0)
		sample_z = np.random.uniform(low=0.4, high=0.8)

		if abs(sample_x - curr_pos[0]) < 0.9 and abs(sample_y -curr_pos[1]) < 0.9 and state.data == 'PositionHold':
			counter += 1
			target = [sample_x, sample_y, sample_z]

			target_pose.header.stamp = rospy.Time.now()
			target_pose.pose.position.x = target[0]
			target_pose.pose.position.y = target[1]
			target_pose.pose.position.z = target[2]

			# target_pose.pose.orientation.x = 0.
			# target_pose.pose.orientation.y = 0.
			# target_pose.pose.orientation.z = 0.7071068
			# target_pose.pose.orientation.w = 0.7071068


  
	if counter > data_points:
		break
	
	pose_pub.publish(target_pose)
	print('counter :', counter)

	rate.sleep()


##################
# nn1 and nn2
###################

state_ 	 = np.array([x_f,y_f,z_f,  vx_f,vy_f,vz_f, r_f,p_f,yaw_f, rs_f,ps_f,ys_f],ndmin=2).transpose()
setpoints_ = np.array([x_sp, y_sp, z_sp], ndmin=2).transpose()
controls_ = np.array([roll_sp, pitch_sp, yaw_rate_sp , thrust_sp ],ndmin=2).transpose()

print('state_		:',state_.shape)
print('setpoints_	:',setpoints_.shape)
print('control_ 	:',controls_.shape)

np.save(log_dir+'input_state.npy',state_)
np.save(log_dir+'setpoints.npy', setpoints_)
np.save(log_dir+'controls.npy'   ,controls_)