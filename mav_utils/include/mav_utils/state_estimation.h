#ifndef MAV_UTILS_STATE_ESTIMATION_H
#define MAV_UTILS_STATE_ESTIMATION_H

#include <geometry_msgs/PointStamped.h>
#include <mav_utils_msgs/UTMPose.h>
#include <mav_utils_msgs/RouterInfo.h>
#include <mav_utils_msgs/RouterData.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <GeographicLib/UTMUPS.hpp>
#include "ros/ros.h"

namespace mav_utils {
  class StateEstimation {
   public:
    StateEstimation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    // input data using ros callabacks
    void odomCallback(const nav_msgs::Odometry& msg);
    void globalPoseCallback(const sensor_msgs::NavSatFix& msg);
    void rangeFinderCallback(const sensor_msgs::Range& msg);
    void routerCallback(const mav_utils_msgs::RouterInfo& msg);
    bool homePoseResetCallback(std_srvs::EmptyRequest& request,
                               std_srvs::EmptyResponse& response);
    // publish modified odom data
    void odomPubCallback();
    // publish UTMPose data
    void utmPosePubCallback();
    // publish home UTMPose data
    void homePosePubCallback();
    // publish GPS data for objects
    void objGPSPubCallback();

    nav_msgs::Odometry px4_odom;
    // mav global pose in lat long
    sensor_msgs::NavSatFix global_pose;
    // height from LiDaR
    sensor_msgs::Range range_finder;
    // data from router
    mav_utils_msgs::RouterInfo router_info;

    bool range_finder_enable;
    float fix_lat, fix_long;

    int zone_fix, zone_home;
    bool northp_fix, northp_home;
    double x_fix, y_fix, conv_fix, scale_fix;
    double x_home, y_home, conv_home, scale_home;

    ros::NodeHandle nh_, nh_private_;

    // subscribers and publishers
    ros::Subscriber px4_odom_sub, global_pose_sub, range_finder_sub, router_sub;
    ros::Publisher utm_pose_pub, range_finder_odom_pub, home_pose_pub, obj_gps_pub;
    ros::ServiceServer home_reset_service;
  };
}  // namespace mav_utils

#endif