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
PA10Wrapper pa10(62005);
bool braked = false;

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  std::vector<float> state;
  bool sameState = true;

  for (size_t i = 0; i < 7; ++i) {
    std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
    std::cout << "-> Position: " << inputJointsMsg->position[i] << "\n";

    state.push_back((float) inputJointsMsg->position[i]);

    if ((float) inputJointsMsg->position[i] != LAST_STATE[i])
      sameState = false;
  }

  if (sameState) {
    std::cout << "WE KEEP SAME POSITION\n";

    if (!braked) {
      pa10.disarm();
      braked = true;
    }

    return;
  }

  if (braked) {
    pa10.arm();
    braked = false;
  }

  LAST_STATE = state;
  Eigen::VectorXf angulars(7);
  angulars << LAST_STATE[0], LAST_STATE[1], LAST_STATE[2], LAST_STATE[3],
    LAST_STATE[4], LAST_STATE[5], LAST_STATE[6];

  std::cout << "Moving to...\n" << angulars << "\n";
  
  pa10.setVel(0.15); // TODO: Read a ROS PARAM that is set by allegro_plan_grasp
  pa10.goTo(angulars);

  std::cout << "Moved!\n";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_left_pa10_mover");

  ros::NodeHandle n;
  ros::Rate r(0.5); // This rate works fine with 0.1 velocity in the PA10 and 0.50 in MoveIt (vel/acc)

  while(ros::ok()) {
    sensor_msgs::JointStateConstPtr receivedMessage = 
      ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    jointsCallback(receivedMessage);

    r.sleep();
  }

  return 0;
}
