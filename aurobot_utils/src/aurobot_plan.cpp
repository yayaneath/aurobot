#include <iostream>
#include <vector>
#include <map>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf/transform_listener.h>
#include <eigen_stl_containers/eigen_stl_containers.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = argv[1];
  std::cout << "= = = = = Group: " << PLANNING_GROUP << " = = = = =\n";

  // Load move group information and state
  moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);
  moveGroup.clearPathConstraints();

  moveit::core::RobotStatePtr kinematicState = moveGroup.getCurrentState();
  const robot_state::JointModelGroup *jointModelGroup = 
     moveGroup.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  std::vector<std::string> subgroups = jointModelGroup->getSubgroupNames();
  for(std::size_t i = 0; i < subgroups.size(); ++i )
    ROS_INFO("Subgroup: %s", subgroups[i].c_str());

  // Display current joint values
  const std::vector<std::string> &jointNames = jointModelGroup->getVariableNames();

  std::vector<double> jointValues;
  kinematicState->copyJointGroupPositions(jointModelGroup, jointValues);

  for (std::size_t i = 0; i < jointNames.size(); ++i)
    ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointValues[i]);

  // Define state for f0
  Eigen::Affine3d f0State;
  Eigen::Vector3d f0Trans(0.52513, 0.24236, 1.3241);//(0.56917, 0.032293, 1.2175);
  Eigen::Quaterniond f0Rot(0.50593, -0.45106, -0.67222, -0.29783);//(-0.40753, 0.61482, 0.45611, -0.49787);
  f0State.translation() = f0Trans;
  f0State.linear() = f0Rot.toRotationMatrix();

  ROS_INFO_STREAM("f0 Translation: " << f0State.translation());
  ROS_INFO_STREAM("f0 Rotation: " << f0State.rotation());

  // Define state for pa10
  Eigen::Affine3d pa10State;
  Eigen::Vector3d pa10Trans(0.55285, 0.29603, 1.3962);//(0.61121, 0.12642, 1.1583);
  Eigen::Quaterniond pa10Rot(0.57136, 0.34182, 0.64867, -0.36869);//(-0.44397, 0.16408, -0.60878, 0.63667);
  pa10State.translation() = pa10Trans;
  pa10State.linear() = pa10Rot.toRotationMatrix();

  ROS_INFO_STREAM("pa10 Translation: " << pa10State.translation());
  ROS_INFO_STREAM("pa10 Rotation: " << pa10State.rotation());

  // Define state for thumb
  Eigen::Affine3d thumbState;
  Eigen::Vector3d thumbTrans(0.47438, 0.26677, 1.3501);//(0.47753, 0.08663, 1.2431);
  Eigen::Quaterniond thumbRot(0.40338, 0.83083, 0.16893, 0.3442);//(0.86738, 0.413, -0.22734, -0.15938);
  thumbState.translation() = thumbTrans;
  thumbState.linear() = thumbRot.toRotationMatrix();

  ROS_INFO_STREAM("thumb Translation: " << thumbState.translation());
  ROS_INFO_STREAM("thumb Rotation: " << thumbState.rotation());

  // Preprare for IK solving
  EigenSTL::vector_Affine3d poses;
  poses.push_back(f0State);
  poses.push_back(pa10State);
  poses.push_back(thumbState);

  std::vector<std::string> tips;
  tips.push_back("l_allegro_link_3_tip");
  tips.push_back("l_allegro_base_link");
  tips.push_back("l_allegro_link_15_tip");

  std::vector<std::vector<double> > limits;
  std::vector<double> f0Limits(4, 0.1);
  std::vector<double> pa10Limits(7, 0.1);
  std::vector<double> thumbLimits(4, 0.1);
  limits.push_back(f0Limits);
  limits.push_back(pa10Limits);
  limits.push_back(thumbLimits);

  //bool foundIK = kinematicState->setFromIK(jointModelGroup, f0State, 10, 0.1);
  bool foundIK = kinematicState->setFromIK(jointModelGroup, poses, tips, limits, 2, 10);
  //bool foundIK = kinematicState->setFromIKSubgroups(jointModelGroup, poses, tips, limits, 20, 0.5);

  if (foundIK) {
    ROS_INFO("Solution found!");
    kinematicState->copyJointGroupPositions(jointModelGroup, jointValues);
    for (std::size_t i = 0; i < jointNames.size(); ++i)
      ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointValues[i]);
  }
  else
    ROS_INFO("Did not find IK solution");

  ros::shutdown();
  return 0;
}