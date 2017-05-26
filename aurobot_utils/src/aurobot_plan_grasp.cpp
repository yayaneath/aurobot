// Standard
#include <iostream>
#include <vector>

// ROS
#include <ros/topic.h>
#include <tf/transform_listener.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

// Custom
#include <aurobot_utils/GraspConfiguration.h> // Custom message for publishing grasps



// Grasp planner
void planGrasp(const aurobot_utils::GraspConfigurationConstPtr & inputGrasp) {
  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Transform the coordinates from the camera frame to the world

  tf::TransformListener transformer;
  tf::Stamped<tf::Point> firstPointIn, firstPointOut, secondPointIn, secondPointOut;
  firstPointIn.setX(inputGrasp->first_point_x);
  firstPointIn.setY(inputGrasp->first_point_y);
  firstPointIn.setZ(inputGrasp->first_point_z);
  firstPointIn.frame_id_ = "/head_link";
  secondPointIn.setX(inputGrasp->second_point_x);
  secondPointIn.setY(inputGrasp->second_point_y);
  secondPointIn.setZ(inputGrasp->second_point_z);
  secondPointIn.frame_id_ = "/head_link";

  transformer.waitForTransform("/world", "/head_link", ros::Time(0), ros::Duration(3.0));
  transformer.transformPoint("/world", firstPointIn, firstPointOut);
  transformer.transformPoint("/world", secondPointIn, secondPointOut);

  std::cout << "First point in:\n" << "> x = " << firstPointIn.x() << "\n"
    << "> y = " << firstPointIn.y() << "\n"
    << "> z = " << firstPointIn.z() << "\n";

  std::cout << "First point out:\n" << "> x = " << firstPointOut.x() << "\n"
    << "> y = " << firstPointOut.y() << "\n"
    << "> z = " << firstPointOut.z() << "\n";

  std::cout << "Second point in:\n" << "> x = " << secondPointIn.x() << "\n"
    << "> y = " << secondPointIn.y() << "\n"
    << "> z = " << secondPointIn.z() << "\n";

  std::cout << "Second point out:\n" << "> x = " << secondPointOut.x() << "\n"
    << "> y = " << secondPointOut.y() << "\n"
    << "> z = " << secondPointOut.z() << "\n";

  Eigen::Vector3d midPoint((firstPointOut.getX() + secondPointOut.getX()) / 2.0,
    (firstPointOut.getY() + secondPointOut.getY()) / 2.0,
    (firstPointOut.getZ() + secondPointOut.getZ()) / 2.0);

  // [TODO]
  // Depending on the orientation of the object, this should be performed
  // in a different way. 
  // - For objects standing it can be done by substracting a distance to the X axis.
  // - For objects laying on the table, it should be substracgint to the Z axis.
  midPoint[0] = midPoint[0] - 0.12; // 12cm backwards in X of the world to leave space for the fingers

  std::cout << "Min point out:\n" << midPoint << "\n";

  // [TODO]
  // Orientation of the midPoint should place the Z axis of the gripper forward from
  // the pregrasp midPoint to the original midPoint (the point between the contact points).
  // In addition, the X axis must be in the same direction as the object orientation. If it
  // is standing, that is: pointing upwards. 

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Visualize points

  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("/world","/rviz_visual_markers"));
  
  Eigen::Vector3d firstPoint(firstPointOut.getX(), firstPointOut.getY(), firstPointOut.getZ());
  Eigen::Vector3d secondPoint(secondPointOut.getX(), secondPointOut.getY(), secondPointOut.getZ());

  visual_tools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);
  visual_tools->publishSphere(secondPoint, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
  visual_tools->publishSphere(midPoint, rviz_visual_tools::GREEN, rviz_visual_tools::XLARGE);
  visual_tools->trigger();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Moving arm + palm

  static const std::string PLANNING_GROUP = "l_arm_palm";
  static const std::string END_EFFECTOR_LINK = "l_barrett_base_link"; 

  moveit::planning_interface::MoveGroupInterface barrettPalmMoveGroup(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan barrettPalmPlan;
  geometry_msgs::PoseStamped barrettPalmPose;
  bool successBarretPalmPlan = false;
  
  barrettPalmMoveGroup.clearPathConstraints();

  ROS_INFO("Reference frame: %s", barrettPalmMoveGroup.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", barrettPalmMoveGroup.getEndEffectorLink().c_str());

  std::cout << "** Current Pose **\n" << 
    barrettPalmMoveGroup.getCurrentPose(END_EFFECTOR_LINK);

  barrettPalmPose.header.stamp = ros::Time::now();
  barrettPalmPose.header.frame_id = "/world";
  barrettPalmPose.pose.position.x = midPoint[0];
  barrettPalmPose.pose.position.y = midPoint[1];
  barrettPalmPose.pose.position.z = midPoint[2];
  barrettPalmPose.pose.orientation.x = 0.77129; 
  barrettPalmPose.pose.orientation.y = 0.014485;
  barrettPalmPose.pose.orientation.z = 0.63544;
  barrettPalmPose.pose.orientation.w = -0.03349;

  std::cout << "** Target Pose **\n" << barrettPalmPose << "\n";

  barrettPalmMoveGroup.setPoseTarget(barrettPalmPose, END_EFFECTOR_LINK);
  barrettPalmMoveGroup.setPlannerId("TRRTkConfigDefault");
  barrettPalmMoveGroup.setPlanningTime(5.0);
  barrettPalmMoveGroup.setNumPlanningAttempts(10);
  barrettPalmMoveGroup.setMaxVelocityScalingFactor(1.0);
  barrettPalmMoveGroup.setMaxAccelerationScalingFactor(1.0);

  successBarretPalmPlan = barrettPalmMoveGroup.plan(barrettPalmPlan);

  ROS_INFO("Plan 1 (pose goal) %s", successBarretPalmPlan ? "" : "FAILED");

  /*if (successBarretPalmPlan)
    barrettPalmMoveGroup.move();*/

  //ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_grasp_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::cout << "Empezando...\n";

  aurobot_utils::GraspConfigurationConstPtr receivedMessage = 
    ros::topic::waitForMessage<aurobot_utils::GraspConfiguration>("/aurobot_utils/grasp_configuration");
  planGrasp(receivedMessage);

  std::cout << "Cerrando...\n";

  return 0;
}