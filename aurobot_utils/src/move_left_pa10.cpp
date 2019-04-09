// Standard
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Custom
#include <pa10/pa10_wrapper.h>

const uint PA10_PORT = 62005;

std::vector<float> lastState(7, 0.0);
PA10Wrapper pa10Client(PA10_PORT);
bool braked = false;

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  std::vector<float> state;
  bool sameState = true;

  // Left PA10's joints are between index 0 and 7
  for (size_t i = 0; i < 7; ++i) {
    std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
    std::cout << "-> Position: " << inputJointsMsg->position[i] << "\n";

    state.push_back((float) inputJointsMsg->position[i]);

    if ((float) inputJointsMsg->position[i] != lastState[i])
      sameState = false;
  }

  if (sameState) {
    std::cout << "WE KEEP SAME POSITION\n";

    if (!braked) {
      pa10Client.disarm();
      braked = true;
    }

    return;
  }

  if (braked) {
    pa10Client.arm();
    braked = false;
  }

  lastState = state;
  
  Eigen::VectorXf angulars(7);
  angulars << lastState[0], lastState[1], lastState[2], lastState[3],
    lastState[4], lastState[5], lastState[6];

  std::cout << "Moving to...\n" << angulars << "\n";
  
  // TODO: Read a ROS PARAM that is set by allegro_plan_grasp
  pa10Client.setVel(0.15);
  pa10Client.goTo(angulars);

  std::cout << "Moved!\n";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_left_pa10");

  ros::NodeHandle nh; // Required to run rate.
  ros::Rate r(0.5); // This rate works fine with 0.5 velocity in MoveIt

  while(ros::ok()) {
    sensor_msgs::JointStateConstPtr receivedMessage = 
      ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    jointsCallback(receivedMessage);

    r.sleep();
  }

  return 0;
}
