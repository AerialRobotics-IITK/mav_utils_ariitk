#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Empty.h>

mav_msgs::RollPitchYawrateThrust rpyth;
geometry_msgs::PointStamped missionInfo;
mavros_msgs::State mavState;

void rpythCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr &msg) {
  rpyth = *msg;
}

void missionInfoCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  missionInfo = *msg;
}

void mavStateCallback(const mavros_msgs::State::ConstPtr &msg) {
  mavState = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_switch_node");
  ros::NodeHandle nh;

  ros::Subscriber rpythSub =
      nh.subscribe("command/roll_pitch_yawrate_thrust", 10, rpythCallback);
  ros::Subscriber missionInfoSub =
      nh.subscribe("mission_info", 10, missionInfoCallback);
  ros::Subscriber mavStateSub =
      nh.subscribe("pilot/state", 10, mavStateCallback);

  ros::ServiceClient backToPoseHoldClient =
      nh.serviceClient<std_srvs::Empty>("back_to_position_hold");

  ros::Publisher rpythPub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
      "pilot/setpoint_raw/roll_pitch_yawrate_thrust", 10);
  ros::Publisher mavPoseLocalMavrosPub =
      nh.advertise<geometry_msgs::PoseStamped>("pilot/setpoint_position/local",
                                               10);
  ros::Publisher mavPoseLocalNMPCPub =
      nh.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
  ros::Publisher mavPoseGlobalPub =
      nh.advertise<mavros_msgs::GlobalPositionTarget>(
          "pilot/setpoint_position/global", 10);

  double rate = 30;
  geometry_msgs::PoseStamped mavCommandPose;
  mavros_msgs::GlobalPositionTarget mavCommandGlobalPose;

  while (ros::ok()) {
    nh.getParam("controller_switch_node/rate", rate);
    ros::Rate loopRate(rate);

    if (1) {
      mavCommandPose.header = missionInfo.header;
      mavCommandPose.pose.position = missionInfo.point;

      if (0) {
        std_srvs::Empty backToPosition;
        if (backToPoseHoldClient.call(backToPosition))
          ROS_INFO("NMPC on");
        else
          ROS_ERROR("Failed to move in position hold");

        mavPoseLocalNMPCPub.publish(mavCommandPose);
        rpythPub.publish(rpyth);
      } else
        mavPoseLocalMavrosPub.publish(mavCommandPose);
    }

    loopRate.sleep();
    ros::spinOnce();
  }
  return 0;
}
