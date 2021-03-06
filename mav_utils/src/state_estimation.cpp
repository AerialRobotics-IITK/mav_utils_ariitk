#include "mav_utils/state_estimation.h"

namespace mav_utils {
StateEstimation::StateEstimation(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), range_finder_enable(false),
      fix_lat(26.519730), fix_long(80.232452) {
  // ros params
  nh_private_.param("range_finder_enable", range_finder_enable,
                    range_finder_enable);
  nh_private_.param("fix_lat", fix_lat, fix_lat);
  nh_private_.param("fix_long", fix_long, fix_long);

  // ros subscribers
  px4_odom_sub =
      nh_.subscribe("odom_px4", 1, &StateEstimation::odomCallback, this);
  global_pose_sub =
      nh_.subscribe("global_pose", 1, &StateEstimation::globalPoseCallback,
                    this, ros::TransportHints().tcpNoDelay());
  range_finder_sub = nh_.subscribe("range_finder", 1,
                                   &StateEstimation::rangeFinderCallback, this);
  // router_sub =
  //     nh_.subscribe("router", 1, &StateEstimation::routerCallback, this);

  // ros pubishers
  utm_pose_pub = nh_.advertise<mav_utils_msgs::UTMPose>("utm_pose", 1, true);
  range_finder_odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
  home_pose_pub =
      nh_.advertise<geometry_msgs::PointStamped>("home_pose", 1, true);
  // obj_gps_pub =
  //     nh_.advertise<mav_utils_msgs::RouterInfo>("objects/gps", 1, true);

  // ros service
  home_reset_service = nh_.advertiseService(
      "home_reset", &StateEstimation::homePoseResetCallback, this);

  // UTM pose of fixed point
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
// void StateEstimation::routerCallback(const mav_utils_msgs::RouterInfo &msg) {
//   router_info = msg;
//   objGPSPubCallback();
// }

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
  if (range_finder_enable)
    odom_est.pose.pose.position.z = range_finder.range;
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

// void StateEstimation::objGPSPubCallback() {
//   int num = router_info.router_data.size();

//   if (num == 0)
//     return;
//   else {
//     mav_utils_msgs::RouterInfo gps_info;
//     mav_utils_msgs::RouterData gps_data;

//     double obj_lat, obj_lon, x, y;

//     gps_info.header = router_info.header;

//     for (int i = 0; i < num; i++) {
//       obj_lat = obj_lon = x = y = 0;

//       gps_data.id = router_info.router_data.at(i).id;
//       gps_info.object_id.push_back(gps_data.id);

//       x = gps_info.router_data.at(i).position.x + x_fix;
//       y = gps_info.router_data.at(i).position.y + y_fix;

//       // Assuming all coordinates are in same zone and northp.
//       GeographicLib::UTMUPS::Reverse(zone_fix, northp_fix, x, y, obj_lat,
//                                      obj_lon);

//       gps_data.position.x = obj_lat;
//       gps_data.position.y = obj_lon;
//       gps_data.position.z = 0;

//       gps_info.router_data.push_back(gps_data);
//     }

//     obj_gps_pub.publish(gps_info);
//   }
// }
} // namespace mav_utils