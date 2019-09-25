#include "mav_utils/state_estimation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_estimation_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  mav_utils::StateEstimation node(nh, nh_private);
  ROS_INFO("Initialized state_estimation node.");

  ros::spin();
  return 0;
}