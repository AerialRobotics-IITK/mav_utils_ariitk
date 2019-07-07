#ifndef MAV_UTILS_STATE_ESTIMATION_H
#define MAV_UTILS_STATE_ESTIMATION_H

#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <mav_utils_msgs/GlobalPose.h>
#include "ros/ros.h"

namespace mav_utils {
    class StateEstimation {
        public: 
        StateEstimation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

        void odomCallback(const nav_msgs::Odometry& msg);
        void globalPoseCallback(const sensor_msgs::NavSatFix& msg);
        void rangeFinderCallback(const sensor_msgs::Range& msg);

        void odomPubCallback();
        void avoidancePosePubCallback();

        nav_msgs::Odometry px4_odom;
        sensor_msgs::NavSatFix global_pose;
        sensor_msgs::Range range_finder;

        bool range_finder_enable;

        ros::NodeHandle nh_, nh_private_;

        ros::Subscriber px4_odom_sub, global_pose_sub, range_finder_sub;
        ros::Publisher avoidance_pose_pub, range_finder_odom_pub;
    };
}

#endif