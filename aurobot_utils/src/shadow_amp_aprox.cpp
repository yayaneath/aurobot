// Standard
#include <iostream>
#include <vector>
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

  std::vector<double> fingersPositions, thumbPositions, amplitudes;
  int steps = 100;
  double currentMiddlePosition = 0.1466, middlePositionMax = 0.8199,
    middlePositionStep = (middlePositionMax + std::abs(currentMiddlePosition)) / (double) steps;
  double currentThumbPosition = 0.0, thumbPositionMax = 0.5983,
    thumbPositionStep = (thumbPositionMax + std::abs(currentThumbPosition)) / (double) steps;

  for (size_t loop = 0; loop < steps; ++loop) {
    std::cout << "# # # LOOP " << loop << ": current middle -> " << currentMiddlePosition 
      << "  step -> " << middlePositionStep << " # # #\n";
    std::cout << "# # # LOOP " << loop << ": current thumb -> " << currentThumbPosition 
      << "  step -> " << thumbPositionStep << " # # #\n";

    fingersPositions.push_back(currentMiddlePosition);
    thumbPositions.push_back(currentThumbPosition);
    
    // Middle finger

    moveit::planning_interface::MoveGroupInterface middleMoveGroup(MIDDLE_PLANNING_GROUP);
    geometry_msgs::PoseStamped middleCurrentPose = middleMoveGroup.getCurrentPose(MIDDLE_END_EFFECTOR_LINK);
    Eigen::Vector3d middlePoint(middleCurrentPose.pose.position.x, middleCurrentPose.pose.position.y,
      middleCurrentPose.pose.position.z);

    // Thumb finger

    moveit::planning_interface::MoveGroupInterface thumbMoveGroup(THUMB_PLANNING_GROUP);
    geometry_msgs::PoseStamped thumbCurrentPose = thumbMoveGroup.getCurrentPose(THUMB_END_EFFECTOR_LINK);
    Eigen::Vector3d thumbPoint(thumbCurrentPose.pose.position.x, thumbCurrentPose.pose.position.y,
      thumbCurrentPose.pose.position.z);

    // Aproximated amplitude

    double amplitude = std::sqrt(std::pow(thumbPoint[0] - middlePoint[0], 2) + 
      std::pow(thumbPoint[1] - middlePoint[1], 2) + std::pow(thumbPoint[2] - middlePoint[2], 2));

    std::cout << "- Amplitude:" << amplitude << "\n";

    amplitudes.push_back(amplitude);

    // Visualize points
    
    visual_tools->publishSphere(middlePoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(thumbPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    visual_tools->trigger();

    // Plan and move next position

    currentMiddlePosition += middlePositionStep;
    currentThumbPosition += thumbPositionStep;

    // Middle finger

    moveit::planning_interface::MoveGroupInterface::Plan middlePlan;
    std::vector<double> middleJointsValues;
    bool middlePlanSuccess = false;

    middleJointsValues.push_back(0.0); // rh_MFJ4
    middleJointsValues.push_back(currentMiddlePosition); // rh_MFJ3
    middleJointsValues.push_back(1.0306); // rh_MFJ2
    middleJointsValues.push_back(0.0); // rh_MFJ1

    for(size_t i = 0; i < middleJointsValues.size(); ++i)
      std::cout << middleJointsValues[i] << " ";
    std::cout << std::endl;
    
    middleMoveGroup.setJointValueTarget(middleJointsValues);
    middlePlanSuccess = middleMoveGroup.plan(middlePlan);

    if(!middlePlanSuccess){
      std::cout << "[ERROR] Middle finger planning for joint poisition failed!\n";
      break;
    }

    middleMoveGroup.execute(middlePlan);

    // Thumb finger

    moveit::planning_interface::MoveGroupInterface::Plan thumbPlan;
    std::vector<double> thumbJointsValues;
    bool thumbPlanSuccess = false;

    thumbJointsValues.push_back(currentThumbPosition); // rh_THJ5
    thumbJointsValues.push_back(1.2217); // rh_THJ4
    thumbJointsValues.push_back(0.1128); // rh_THJ3
    thumbJointsValues.push_back(0.1728); // rh_THJ2
    thumbJointsValues.push_back(0.0); // rh_THJ1

    for(size_t i = 0; i < thumbJointsValues.size(); ++i)
      std::cout << thumbJointsValues[i] << " ";
    std::cout << std::endl;

    thumbMoveGroup.setJointValueTarget(thumbJointsValues);
    thumbPlanSuccess = thumbMoveGroup.plan(thumbPlan);

    if(!thumbPlanSuccess){
      std::cout << "[ERROR] Thumb finger planning for joint poisition failed!\n";
      break;
    }

    thumbMoveGroup.execute(thumbPlan);
  }

  return 0;

  for (size_t i = 0; i < amplitudes.size(); ++i) {
    std::cout << "Fingers Position: " << fingersPositions[i] 
      << " | Thumb Position: " << thumbPositions[i]
      << " | Amplitude: " << amplitudes[i] << "\n";
  }

  std::cout << "\nCerrando...\n";

  ros::shutdown();
  return 0;
}