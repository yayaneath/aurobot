// Standard
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Custom
#include <pa10/pa10_wrapper.h>

std::vector<float> LAST_STATE(7, 0.0);

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  std::vector<float> state;
  bool sameState = true;

  for (size_t i = 7; i < 14; ++i) {
    std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
    std::cout << "-> Position: " << inputJointsMsg->position[i] << "\n";

    state.push_back((float) inputJointsMsg->position[i]);

    if ((float) inputJointsMsg->position[i] != LAST_STATE[i-7])
      sameState = false;
  }

  if (sameState) {
    std::cout << "WE KEEP SAME POSITION\n";
    return;
  }

  LAST_STATE = state;
  Eigen::VectorXf angulars(7);
  angulars << LAST_STATE[0], LAST_STATE[1], LAST_STATE[2], LAST_STATE[3],
    LAST_STATE[4], LAST_STATE[5], LAST_STATE[6];

  std::cout << "Moving to...\n" << angulars << "\n";

  PA10Wrapper pa10(62004);
  pa10.setVel(0.3);
  pa10.goTo(angulars);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_grasp_executer");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states",
    100, jointsCallback);

  ros::spin();

  return 0;
}
