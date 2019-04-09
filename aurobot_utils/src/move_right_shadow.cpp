// Standard
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

ros::Publisher pub;
std::vector<double> lastState(24, -1.0);

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  std::vector<double> state;
  bool sameState = true;

  // Right Shadow's joints are between index 7 and 30
  int startPos = 7, endPos = 30;

  for (size_t i = startPos; i <= endPos; ++i) {
    std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
    std::cout << "-> Position: " << inputJointsMsg->position[i] << "\n";

    state.push_back((double) inputJointsMsg->position[i]);

    if ((double) inputJointsMsg->position[i] != lastState[i - startPos])
      sameState = false;
  }

  if (sameState)
    std::cout << "SAME POSITION\n";

  lastState = state;

  trajectory_msgs::JointTrajectory newTrajMsg;

  newTrajMsg.joint_names = {
    "rh_WRJ2", "rh_WRJ1",
    "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1",
    "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1",
    "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",
    "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1",
    "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1"
  }; 

  newTrajMsg.points.resize(1);
  newTrajMsg.points[0].positions = lastState;
  newTrajMsg.points[0].time_from_start = ros::Duration(0.1);

  //std::cout << "New TrajPoint: \n" << newTrajMsg << "\n";

  pub.publish(newTrajMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_right_shadow");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states",
    100, jointsCallback);
 
  pub = nh.advertise<trajectory_msgs::JointTrajectory>("/rh_trajectory_controller/command", 1);

  ros::spin();

  return 0;
}
