// Standard
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub;
std::vector<double> LAST_STATE(16, 0.0);

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  std::vector<double> state;
  bool sameState = true;

  for (size_t i = 7; i < 23; ++i) {
    std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
    std::cout << "-> Position: " << inputJointsMsg->position[i] << "\n";

    state.push_back((double) inputJointsMsg->position[i]);

    if ((double) inputJointsMsg->position[i] != LAST_STATE[i-7])
      sameState = false;
  }

  if (sameState) {
    std::cout << "SAME POSITION\n";
  }

  LAST_STATE = state;

  sensor_msgs::JointState msg;
  msg.name = {
    "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0", // Finger 0
    "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0", // Finger 1
    "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0", // Finger 2
    "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0" // Thumb
  }; 
  msg.position = { // We receive the fingers in reverse order
    LAST_STATE[12], LAST_STATE[13], LAST_STATE[14], LAST_STATE[15],
    LAST_STATE[8], LAST_STATE[9], LAST_STATE[10], LAST_STATE[11],
    LAST_STATE[4], LAST_STATE[5], LAST_STATE[6], LAST_STATE[7],
    LAST_STATE[0], LAST_STATE[1], LAST_STATE[2], LAST_STATE[3]
  };

  std::cout << "New Message: \n" << msg << "\n";

  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_left_allegro_mover");

  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states",
    100, jointsCallback);
 
  pub = nh.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd", 1);

  ros::spin();

  return 0;
}
