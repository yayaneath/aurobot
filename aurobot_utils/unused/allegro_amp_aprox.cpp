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
  ros::init(argc, argv, "allegro_amp_aprox");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::cout << "Empezando...\n";

  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("/world","/rviz_visual_markers"));
  const std::string FIRST_PLANNING_GROUP = "l_first_finger";
  const std::string FIRST_END_EFFECTOR_LINK = "l_allegro_link_3_tip"; 
  const std::string MIDDLE_PLANNING_GROUP = "l_middle_finger";
  const std::string MIDDLE_END_EFFECTOR_LINK = "l_allegro_link_7_tip"; 
  const std::string THUMB_PLANNING_GROUP = "l_thumb";
  const std::string THUMB_END_EFFECTOR_LINK = "l_allegro_link_15_tip";

  std::vector<double> fingersPositions, thumbPositions, amplitudes;
  int steps = 165;
  double currentFingerPosition = -0.1960, fingerPositionMax = 1.1500,
    fingerPositionStep = (fingerPositionMax + std::abs(currentFingerPosition)) / (double) steps;
  double currentThumbPosition = -0.1890, thumbPositionMax = 0.4000,
    thumbPositionStep = (thumbPositionMax + std::abs(currentThumbPosition)) / (double) steps;

  for (size_t loop = 0; loop < steps; ++loop) {
    std::cout << "# # # LOOP " << loop << ": current finger -> " << currentFingerPosition 
      << "  step -> " << fingerPositionStep << " # # #\n";
    std::cout << "# # # LOOP " << loop << ": current thumb -> " << currentThumbPosition 
      << "  step -> " << thumbPositionStep << " # # #\n";

    fingersPositions.push_back(currentFingerPosition);
    thumbPositions.push_back(currentThumbPosition);
    
    // First finger

    moveit::planning_interface::MoveGroupInterface firstMoveGroup(FIRST_PLANNING_GROUP);
    geometry_msgs::PoseStamped firstCurrentPose = firstMoveGroup.getCurrentPose(FIRST_END_EFFECTOR_LINK);
    Eigen::Vector3d firstPoint(firstCurrentPose.pose.position.x, firstCurrentPose.pose.position.y,
      firstCurrentPose.pose.position.z);

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

    // Midpoint between first and second fingers

    Eigen::Vector3d fingersMidPoint((firstPoint[0] + middlePoint[0]) / 2.0,
      (firstPoint[1] + middlePoint[1]) / 2.0, (firstPoint[2] + middlePoint[2]) / 2.0);

    // Aproximated amplitude

    double amplitude = std::sqrt(std::pow(thumbPoint[0] - fingersMidPoint[0], 2) + 
      std::pow(thumbPoint[1] - fingersMidPoint[1], 2) + std::pow(thumbPoint[2] - fingersMidPoint[2], 2));

    std::cout << "- Amplitude:" << amplitude << "\n";

    amplitudes.push_back(amplitude);

    // Visualize points
    
    visual_tools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(middlePoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(thumbPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    visual_tools->publishSphere(fingersMidPoint, rviz_visual_tools::PINK, rviz_visual_tools::LARGE);
    visual_tools->publishLine(fingersMidPoint, thumbPoint, rviz_visual_tools::ORANGE, rviz_visual_tools::MEDIUM);
    visual_tools->trigger();

    // Plan and move next position

    currentFingerPosition += fingerPositionStep;
    currentThumbPosition += thumbPositionStep;

    // First finger

    moveit::planning_interface::MoveGroupInterface::Plan firstPlan;
    std::vector<double> firstJointsValues;
    bool firstPlanSuccess = false;
    
    firstJointsValues.push_back(-0.1446); // l_allegro_joint_0
    firstJointsValues.push_back(currentFingerPosition); // l_allegro_joint_1
    firstJointsValues.push_back(0.6641); // l_allegro_joint_2
    firstJointsValues.push_back(0.5840); // l_allegro_joint_3

    firstMoveGroup.setJointValueTarget(firstJointsValues);
    firstPlanSuccess = firstMoveGroup.plan(firstPlan);

    if(!firstPlanSuccess){
      std::cout << "[ERROR] First finger planning for joint poisition failed!\n";
      break;
    }

    firstMoveGroup.move();

    // Middle finger

    moveit::planning_interface::MoveGroupInterface::Plan middlePlan;
    std::vector<double> middleJointsValues;
    bool middlePlanSuccess = false;
    
    middleJointsValues.push_back(-0.1446); // l_allegro_joint_4
    middleJointsValues.push_back(currentFingerPosition); // l_allegro_joint_5
    middleJointsValues.push_back(0.6641); // l_allegro_joint_6
    middleJointsValues.push_back(0.5840); // l_allegro_joint_7
    
    middleMoveGroup.setJointValueTarget(middleJointsValues);
    middlePlanSuccess = middleMoveGroup.plan(middlePlan);

    if(!middlePlanSuccess){
      std::cout << "[ERROR] Middle finger planning for joint poisition failed!\n";
      break;
    }

    middleMoveGroup.move();

    // Thumb finger

    moveit::planning_interface::MoveGroupInterface::Plan thumbPlan;
    std::vector<double> thumbJointsValues;
    bool thumbPlanSuccess = false;
    
    thumbJointsValues.push_back(1.3959); // l_allegro_joint_12
    thumbJointsValues.push_back(0.0700); // l_allegro_joint_13
    thumbJointsValues.push_back(currentThumbPosition); // l_allegro_joint_14
    thumbJointsValues.push_back(0.7372); // l_allegro_joint_15
    
    thumbMoveGroup.setJointValueTarget(thumbJointsValues);
    thumbPlanSuccess = thumbMoveGroup.plan(thumbPlan);

    if(!thumbPlanSuccess){
      std::cout << "[ERROR] Thumb finger planning for joint poisition failed!\n";
      break;
    }

    thumbMoveGroup.move();
  }

  for (size_t i = 0; i < amplitudes.size(); ++i) {
    std::cout << "Fingers Position: " << fingersPositions[i] 
      << " | Thumb Position: " << thumbPositions[i]
      << " | Amplitude: " << amplitudes[i] << "\n";
  }

  std::cout << "\nCerrando...\n";

  ros::shutdown();
  return 0;
}