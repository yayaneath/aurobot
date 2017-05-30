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

// Custom
#include <aurobot_utils/GraspConfiguration.h> // Custom message for publishing grasps




int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_grasp_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::cout << "Empezando...\n";

  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("/world","/rviz_visual_markers"));
  static const std::string FIRST_PLANNING_GROUP = "l_first_finger";
  static const std::string FIRST_END_EFFECTOR_LINK = "l_barrett_finger_13_tip_link"; 
  static const std::string SECOND_PLANNING_GROUP = "l_second_finger";
  static const std::string SECOND_END_EFFECTOR_LINK = "l_barrett_finger_23_tip_link"; 
  static const std::string THIRD_PLANNING_GROUP = "l_third_finger";
  static const std::string THIRD_END_EFFECTOR_LINK = "l_barrett_finger_33_tip_link"; 

  std::vector<double> positions, amplitudes;
  double currentPosition = 0.0, positionStep = 0.01, positionMax = 1.65;

  for (size_t loop = 0; loop < positionMax / positionStep; ++loop) {
    std::cout << "# # # LOOP " << loop << ": current -> " << currentPosition << " # # #\n";

    positions.push_back(currentPosition);
    
    // First finger

    moveit::planning_interface::MoveGroupInterface firstMoveGroup(FIRST_PLANNING_GROUP);
    geometry_msgs::PoseStamped firstCurrentPose = firstMoveGroup.getCurrentPose(FIRST_END_EFFECTOR_LINK);
    Eigen::Vector3d firstPoint(firstCurrentPose.pose.position.x, firstCurrentPose.pose.position.y,
      firstCurrentPose.pose.position.z);

    // Second finger

    moveit::planning_interface::MoveGroupInterface secondMoveGroup(SECOND_PLANNING_GROUP);
    geometry_msgs::PoseStamped secondCurrentPose = secondMoveGroup.getCurrentPose(SECOND_END_EFFECTOR_LINK);
    Eigen::Vector3d secondPoint(secondCurrentPose.pose.position.x, secondCurrentPose.pose.position.y,
      secondCurrentPose.pose.position.z);

    // Third finger

    moveit::planning_interface::MoveGroupInterface thirdMoveGroup(THIRD_PLANNING_GROUP);
    geometry_msgs::PoseStamped thirdCurrentPose = thirdMoveGroup.getCurrentPose(THIRD_END_EFFECTOR_LINK);
    Eigen::Vector3d thirdPoint(thirdCurrentPose.pose.position.x, thirdCurrentPose.pose.position.y,
      thirdCurrentPose.pose.position.z);

    // Midpoint between first and second fingers

    Eigen::Vector3d fingersMidPoint((firstPoint[0] + secondPoint[0]) / 2.0,
      (firstPoint[1] + secondPoint[1]) / 2.0, (firstPoint[2] + secondPoint[2]) / 2.0);

    // Aproximated amplitude

    double amplitude = std::sqrt(std::pow(thirdPoint[0] - fingersMidPoint[0], 2) + 
      std::pow(thirdPoint[1] - fingersMidPoint[1], 2) + std::pow(thirdPoint[2] - fingersMidPoint[2], 2));

    std::cout << "- Amplitude:" << amplitude << "\n";

    amplitudes.push_back(amplitude);

    // Visualize points
    
    visual_tools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(secondPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(thirdPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(fingersMidPoint, rviz_visual_tools::PINK, rviz_visual_tools::LARGE);
    visual_tools->publishLine(fingersMidPoint, thirdPoint, rviz_visual_tools::ORANGE, rviz_visual_tools::MEDIUM);
    visual_tools->trigger();

    // Plan and move next position
    currentPosition += positionStep;

    // First finger

    moveit::planning_interface::MoveGroupInterface::Plan firstPlan;
    bool firstPlanSuccess = false;
    
    firstMoveGroup.setJointValueTarget("l_barrett_j12_joint", currentPosition);
    firstPlanSuccess = firstMoveGroup.plan(firstPlan);

    if(!firstPlanSuccess){
      std::cout << "[ERROR] First finger planning for joint poisition failed!\n";
      break;
    }

    firstMoveGroup.move();

    // Second finger

    moveit::planning_interface::MoveGroupInterface::Plan secondPlan;
    bool secondPlanSuccess = false;
    
    secondMoveGroup.setJointValueTarget("l_barrett_j22_joint", currentPosition);
    secondPlanSuccess = secondMoveGroup.plan(secondPlan);

    if(!secondPlanSuccess){
      std::cout << "[ERROR] Second finger planning for joint poisition failed!\n";
      break;
    }

    secondMoveGroup.move();

    // Third finger

    moveit::planning_interface::MoveGroupInterface::Plan thirdPlan;
    bool thirdPlanSuccess = false;
    
    thirdMoveGroup.setJointValueTarget("l_barrett_j32_joint", currentPosition);
    thirdPlanSuccess = thirdMoveGroup.plan(thirdPlan);

    if(!thirdPlanSuccess){
      std::cout << "[ERROR] Third finger planning for joint poisition failed!\n";
      break;
    }

    thirdMoveGroup.move();
  }

  for (size_t i = 0; i < amplitudes.size(); ++i)
    std::cout << "Position: " << positions[i] << " | Amplitude: " << amplitudes[i] << "\n";

  std::cout << "\nCerrando...\n";

  ros::shutdown();
  return 0;
}