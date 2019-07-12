#include "mav_utils/state_estimation.h"

namespace mav_utils {
StateEstimation::StateEstimation(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      range_finder_enable(false),
      fix_lat(26.519730),
      fix_long(80.232452) {
  nh_private_.param("range_finder_enable", range_finder_enable,
                    range_finder_enable);
  nh_private_.param("fix_lat", fix_lat, fix_lat);
  nh_private_.param("fix_long", fix_long, fix_long);

  px4_odom_sub =
      nh_.subscribe("odom_px4", 1, &StateEstimation::odomCallback, this);
  global_pose_sub =
      nh_.subscribe("global_pose", 1, &StateEstimation::globalPoseCallback,
                    this, ros::TransportHints().tcpNoDelay());
  range_finder_sub = nh_.subscribe("range_finder", 1,
                                   &StateEstimation::rangeFinderCallback, this);

  utm_pose_pub = nh_.advertise<mav_utils_msgs::UTMPose>("utm_pose", 1, true);
  range_finder_odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
  home_pose_pub =
      nh_.advertise<geometry_msgs::PointStamped>("home_pose", 1, true);

  home_reset_service = nh_.advertiseService(
      "home_reset", &StateEstimation::homePoseResetCallback, this);

  GeographicLib::UTMUPS::Forward(fix_lat, fix_long, zone_fix, northp_fix, x_fix,
                                 y_fix, conv_fix, scale_fix);
}

void StateEstimation::globalPoseCallback(const sensor_msgs::NavSatFix &msg) {
  global_pose = msg;
  utmPosePubCallback();
  homePosePubCallback();
}
void StateEstimation::odomCallback(const nav_msgs::Odometry &msg) {
  px4_odom = msg;
  odomPubCallback();
}
void StateEstimation::rangeFinderCallback(const sensor_msgs::Range &msg) {
  range_finder = msg;
}

bool StateEstimation::homePoseResetCallback(std_srvs::Empty::Request &req,
                                            std_srvs::Empty::Response &res) {
  GeographicLib::UTMUPS::Forward(global_pose.latitude, global_pose.longitude,
                                 zone_home, northp_home, x_home, y_home,
                                 conv_home, scale_home);
  ROS_INFO("home reset done");
  return true;
}

void StateEstimation::homePosePubCallback() {
  geometry_msgs::PointStamped home_pose;

  home_pose.header = global_pose.header;
  home_pose.point.x = x_home - x_fix;
  home_pose.point.y = y_home - y_fix;
  home_pose_pub.publish(home_pose);
}

void StateEstimation::odomPubCallback() {
  nav_msgs::Odometry odom_est;
  odom_est = px4_odom;
  if (range_finder_enable) odom_est.pose.pose.position.z = range_finder.range;
  /*
  TODO: need to find a method for good velocity estimate
   odom_est.twist.twist.linear.z = odom.twist.twist.linear.z;
   */
  range_finder_odom_pub.publish(odom_est);
}

void StateEstimation::utmPosePubCallback() {
  mav_utils_msgs::UTMPose utm_pose;

  int zone;
  bool northp;
  double x, y, conv, scale;
  GeographicLib::UTMUPS::Forward(global_pose.latitude, global_pose.longitude,
                                 zone, northp, x, y, conv, scale);

  utm_pose.header = global_pose.header;
  utm_pose.pose.position.x = x - x_fix;
  utm_pose.pose.position.y = y - y_fix;

  if (range_finder_enable)
    utm_pose.pose.position.z = range_finder.range;
  else
    utm_pose.pose.position.z = px4_odom.pose.pose.position.z;

  utm_pose.pose.orientation = px4_odom.pose.pose.orientation;
  utm_pose.linear_twist = px4_odom.twist.twist.linear;
  utm_pose_pub.publish(utm_pose);
}
}  // namespace mav_utils