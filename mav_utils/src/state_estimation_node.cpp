#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <mav_utils_msgs/GlobalPose.h>
#include "ros/ros.h"

nav_msgs::Odometry odom;
sensor_msgs::NavSatFix globalPose;
sensor_msgs::Range rangeFinder;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom = *msg;
}

void globalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr &msg){
    globalPose = *msg;
}

void rangeFinderCallback(const sensor_msgs::Range::ConstPtr &msg){
    rangeFinder = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimation_node");
    ros::NodeHandle nh;

    ros::Subscriber odomSub = nh.subscribe("odometry", 10, odomCallback);
    ros::Subscriber globalPoseSub = nh.subscribe("mav_global_pose", 10, globalPoseCallback);
    ros::Subscriber rangeFinderSub = nh.subscribe("rangefinder", 10, rangeFinderCallback);

    ros::Publisher avoidancePosePub = nh.advertise<mav_utils_msgs::GlobalPose>("mav_avoidance_pose", 10);
    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("estimated_odometry", 10);

    double rate=30;
    bool rangeFinderEnable;
    mav_utils_msgs::GlobalPose avoidancePose;
    nav_msgs::Odometry odom_est;

    nh.getParam("state_estimation_node/rangefinder_enable", rangeFinderEnable);

    while (ros::ok())
    {
        nh.getParam("state_estimation_node/rate", rate);
        ros::Rate loop_rate(rate);

        avoidancePose.header = globalPose.header;
        avoidancePose.latitude = globalPose.latitude;
        avoidancePose.longitude = globalPose.longitude;
        avoidancePose.orientation = odom.pose.pose.orientation;
        avoidancePose.linear_twist = odom.twist.twist.linear;

        odom_est = odom;
        if(rangeFinderEnable) odom_est.pose.pose.position.z = rangeFinder.range;

        //need to find a good velocity for rangefinder
        // odom_est.twist.twist.linear.z = odom.twist.twist.linear.z;

        avoidancePosePub.publish(avoidancePose);
        odomPub.publish(odom_est);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}