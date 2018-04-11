// Standard
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub;
std::vector<double> lastState(16, 0.0);

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  std::vector<double> state;
  bool sameState = true;

  // Left Allegro's joints are between index 7 and 23
  for (size_t i = 7; i < 23; ++i) {
    std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
    std::cout << "-> Position: " << inputJointsMsg->position[i] << "\n";

    state.push_back((double) inputJointsMsg->position[i]);

    if ((double) inputJointsMsg->position[i] != lastState[i-7])
      sameState = false;
  }

  if (sameState)
    std::cout << "SAME POSITION\n";

  lastState = state;

  sensor_msgs::JointState newStateMsg;
  newStateMsg.name = {
    "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0", // Finger 0
    "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0", // Finger 1
    "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0", // Finger 2
    "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0" // Thumb
  }; 
  newStateMsg.position = { // We received the fingers in reverse order
    lastState[12], lastState[13], lastState[14], lastState[15],
    lastState[8], lastState[9], lastState[10], lastState[11],
    lastState[4], lastState[5], lastState[6], lastState[7],
    lastState[0], lastState[1], lastState[2], lastState[3]
  };

  std::cout << "New Message: \n" << newStateMsg << "\n";

  pub.publish(newStateMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_move_left_allegro");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states",
    100, jointsCallback);
 
  pub = nh.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd", 1);

  ros::spin();

  return 0;
}
