#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

nav_msgs::Odometry odom_;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_planner");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("pilot/local_position/odom", 10, odom_cb);
    ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, arucocb);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_traject", 10);
    // rate
    ros::Rate sleep_rate(1);

    while (ros::ok())
    {



        ros::spinOnce();
        sleep_rate.sleep();
    }
    return 0;
}