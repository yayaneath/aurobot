// Standard
#include <iostream>
#include <vector>
#include <map>
#include <math.h>

// ROS
#include <ros/topic.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "shadow_amp_aprox");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::cout << "Empezando...\n";

  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("/world","/rviz_visual_markers"));
  const std::string MIDDLE_PLANNING_GROUP = "rh_middle_finger";
  const std::string MIDDLE_END_EFFECTOR_LINK = "rh_mftip"; 
  const std::string THUMB_PLANNING_GROUP = "rh_thumb";
  const std::string THUMB_END_EFFECTOR_LINK = "rh_thtip";

  int steps = 100;
  double currentMiddlePosition = 0.1466, middlePositionMax = 0.8199,
    middlePositionStep = (middlePositionMax - currentMiddlePosition) / (double) steps;
  double currentThumbPosition = 0.0, thumbPositionMax = 0.5983,
    thumbPositionStep = (thumbPositionMax - currentThumbPosition) / (double) steps;


  moveit::planning_interface::MoveGroupInterface shadowMoveGroup("right_hand");
  moveit::planning_interface::MoveGroupInterface::Plan shadowMovePlan;
  bool planSuccess = false;

  std::map<std::string, double> pregraspJoints;
  pregraspJoints = shadowMoveGroup.getNamedTargetValues("pregrasp-mf");
  pregraspJoints["rh_MFJ3"] = 0.0;
  pregraspJoints["rh_THJ5"] = 0.0;

  for (auto it = pregraspJoints.begin(); it != pregraspJoints.end(); ++it)
    std::cout << it->first << "=" << it->second << "\n";

  shadowMoveGroup.setJointValueTarget(pregraspJoints);
  planSuccess = shadowMoveGroup.plan(shadowMovePlan);

  if(!planSuccess){
    std::cout << "[ERROR] Fingers planning for joint poisition failed!\n";
    return false;
  }

  shadowMoveGroup.execute(shadowMovePlan);

  ros::shutdown();
  return 0;
}