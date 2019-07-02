#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include "ros/ros.h"

mav_msgs::RollPitchYawrateThrust rpyth;
geometry_msgs::PoseStamped commandPose;
mavros_msgs::State mavState;
bool isNMPCOn;

void rpythCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr &msg){
    rpyth = *msg;
}

void commandPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    commandPose = *msg;
}

void mavStateCallback(const mavros_msgs::State::ConstPtr &msg){
    mavState = *msg;
}

void isNMPCOnCallback(const std_msgs::Bool::ConstPtr &msg){
    isNMPCOn = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_switch_node");
    ros::NodeHandle nh;

    ros::Subscriber rpythSub = nh.subscribe("command/roll_pitch_yawrate_thrust", 10, rpythCallback);
    ros::Subscriber commandPoseSub = nh.subscribe("command/pose", 10, commandPoseCallback);
    ros::Subscriber mavStateSub = nh.subscribe("pilot/state", 10, mavStateCallback);
    ros::Subscriber isNMPCOnSub = nh.subscribe("is_nmpc_on", 10, isNMPCOnCallback);

    ros::ServiceClient backToPoseHoldClient = nh.serviceClient<std_srvs::Empty>("back_to_position_hold");

    ros::Publisher rpythPub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("pilot/setpoint_raw/roll_pitch_yawrate_thrust", 10);
    ros::Publisher mavPoseLocalPub = nh.advertise<geometry_msgs::PoseStamped>("pilot/setpoint_position/local", 10);
    ros::Publisher mavPoseGlobalPub = nh.advertise<mavros_msgs::GlobalPositionTarget>("pilot/setpoint_position/global", 10);

    double rate=30;
    bool rangeFinderEnable;
    mav_msgs::RollPitchYawrateThrust rpythToPub;
    geometry_msgs::PoseStamped mavCommandPose;

    while (ros::ok()){
        nh.getParam("controller_switch_node/rate", rate);
        ros::Rate loop_rate(rate);

        if(isNMPCOn){
            std_srvs::Empty backToPosition;
            if (backToPoseHoldClient.call(backToPosition))
            {
                ROS_INFO("NMPC on");
            }
            else
            {
                ROS_ERROR("Failed to move in position hold");
            }
        }
        else if(global_mode){
            
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
