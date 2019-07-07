#include "mav_utils/state_estimation.h"

namespace mav_utils {
StateEstimation::StateEstimation(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), range_finder_enable(false) {
  nh_private_.param("range_finder_enable", range_finder_enable,
                    range_finder_enable);
  px4_odom_sub =
      nh_.subscribe("odom_px4", 1, &StateEstimation::odomCallback, this);
  px4_odom_sub = nh_.subscribe("global_pose", 1,
                               &StateEstimation::globalPoseCallback, this);
  px4_odom_sub = nh_.subscribe("range_finder", 1,
                               &StateEstimation::rangeFinderCallback, this);

  avoidance_pose_pub =
      nh_.advertise<mav_utils_msgs::GlobalPose>("avoidance_pose", 1, true);
  range_finder_odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
}

void StateEstimation::globalPoseCallback(const sensor_msgs::NavSatFix &msg) {
  global_pose = msg;
  std::cout<<"call"<<std::endl;
  avoidancePosePubCallback();
}
void StateEstimation::odomCallback(const nav_msgs::Odometry &msg) {
  px4_odom = msg;
  odomPubCallback();
}
void StateEstimation::rangeFinderCallback(const sensor_msgs::Range &msg) {
  range_finder = msg;
}

void StateEstimation::odomPubCallback() {
  nav_msgs::Odometry odom_est;
  odom_est = px4_odom;
  if (range_finder_enable) odom_est.pose.pose.position.z = range_finder.range;
  // need to find a good velocity for rangefinder
  // odom_est.twist.twist.linear.z = odom.twist.twist.linear.z;
  range_finder_odom_pub.publish(odom_est);
}

void StateEstimation::avoidancePosePubCallback() {
  mav_utils_msgs::GlobalPose avoidance_pose;

  avoidance_pose.header = global_pose.header;
  avoidance_pose.latitude = global_pose.latitude;
  avoidance_pose.longitude = global_pose.longitude;
  avoidance_pose.orientation = px4_odom.pose.pose.orientation;
  avoidance_pose.linear_twist = px4_odom.twist.twist.linear;
  avoidance_pose_pub.publish(avoidance_pose);
}
}