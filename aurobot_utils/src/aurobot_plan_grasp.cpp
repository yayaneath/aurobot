// Standard
#include <iostream>
#include <vector>

// ROS
#include <ros/topic.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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
  tf::Stamped<tf::Point> firstPointIn, firstPointOut, secondPointIn, secondPointOut,
    graspNormalIn, graspNormalOut;
  firstPointIn.setX(inputGrasp->first_point_x);
  firstPointIn.setY(inputGrasp->first_point_y);
  firstPointIn.setZ(inputGrasp->first_point_z);
  firstPointIn.frame_id_ = "/head_link";
  secondPointIn.setX(inputGrasp->second_point_x);
  secondPointIn.setY(inputGrasp->second_point_y);
  secondPointIn.setZ(inputGrasp->second_point_z);
  secondPointIn.frame_id_ = "/head_link";
  graspNormalIn.setX(inputGrasp->grasp_normal_x);
  graspNormalIn.setY(inputGrasp->grasp_normal_y);
  graspNormalIn.setZ(inputGrasp->grasp_normal_z);
  graspNormalIn.frame_id_ = "/head_link";

  transformer.waitForTransform("/world", "/head_link", ros::Time(0), ros::Duration(3.0));
  transformer.transformPoint("/world", firstPointIn, firstPointOut);
  transformer.transformPoint("/world", secondPointIn, secondPointOut);
  transformer.transformPoint("/world", graspNormalIn, graspNormalOut);

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

  std::cout << "Grasp normal in:\n" << "> x = " << graspNormalIn.x() << "\n"
    << "> y = " << graspNormalIn.y() << "\n"
    << "> z = " << graspNormalIn.z() << "\n";

  std::cout << "Grasp normal out:\n" << "> x = " << graspNormalOut.x() << "\n"
    << "> y = " << graspNormalOut.y() << "\n"
    << "> z = " << graspNormalOut.z() << "\n";

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Calcule grasper palm position and orientation

  Eigen::Vector3d axeX, axeY, axeZ, aux;
  Eigen::Vector3d firstPoint(firstPointOut.getX(), firstPointOut.getY(), firstPointOut.getZ());
  Eigen::Vector3d secondPoint(secondPointOut.getX(), secondPointOut.getY(), secondPointOut.getZ());
  Eigen::Vector3d graspNormal(graspNormalOut.getX(), graspNormalOut.getY(), graspNormalOut.getZ());
  Eigen::Vector3d midPoint((firstPoint[0] + secondPoint[0]) / 2.0,
    (firstPoint[1] + secondPoint[1]) / 2.0, (firstPoint[2] + secondPoint[2]) / 2.0);
  Eigen::Vector3d worldNormal(0, 0, 1);
  float threshold = 0.9, graspCos = std::abs((worldNormal.dot(graspNormal)) / (worldNormal.norm() * graspNormal.norm()));
  
  std::cout << "Mid point out:\n" << midPoint << "\n";
  std::cout << "Grasp angle cosine to world surface: " << graspCos << "\n";

  if (graspCos > threshold) {
    axeY = secondPoint - firstPoint;
    axeZ = axeY.cross(-graspNormal);
    axeX = axeY.cross(axeZ);
  }
  else {
    axeY = secondPoint - firstPoint;
    axeZ = axeY.cross(-graspNormal);
    axeX = axeY.cross(axeZ);

    aux = axeZ;
    axeZ = -axeX;
    axeX = aux;
  }

  axeX.normalize();  axeY.normalize();  axeZ.normalize();

  std::cout << "Axe X: " << axeX << "\n";
  std::cout << "Axe Y: " << axeY << "\n";
  std::cout << "Axe Z: " << axeZ << "\n";

  Eigen::Affine3d midPointPose;
  Eigen::Matrix3d midPointRotation;
  midPointRotation << axeX[0], axeY[0], axeZ[0],
                      axeX[1], axeY[1], axeZ[1],
                      axeX[2], axeY[2], axeZ[2];
  midPointPose.translation() = midPoint;
  midPointPose.linear() = midPointRotation;

  ROS_INFO_STREAM("Mid point Translation: " << midPointPose.translation());
  ROS_INFO_STREAM("Mid point Rotation: " << midPointPose.rotation());

  // Moving the pose backwards 

  Eigen::Vector3d midPointCentered(0, 0, -0.1);
  tf::Transform midPointTransform;
  tf::Vector3 midPointTF, midPointTFed;

  tf::transformEigenToTF(midPointPose, midPointTransform);
  tf::vectorEigenToTF(midPointCentered, midPointTF);

  midPointTFed = midPointTransform(midPointTF);

  tf::vectorTFToEigen(midPointTFed, midPointCentered);

  //midPointPose.translation() = midPointCentered;

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Visualize points

  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("/world","/rviz_visual_markers"));
  
  visual_tools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visual_tools->publishSphere(secondPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visual_tools->publishSphere(midPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
  visual_tools->publishSphere(midPointCentered, rviz_visual_tools::PINK, rviz_visual_tools::LARGE);
  visual_tools->publishAxis(midPointPose, rviz_visual_tools::MEDIUM);
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

  /*barrettPalmPose.header.stamp = ros::Time::now();
  barrettPalmPose.header.frame_id = "/world";
  barrettPalmPose.pose.position.x = midPoint[0];
  barrettPalmPose.pose.position.y = midPoint[1];
  barrettPalmPose.pose.position.z = midPoint[2];
  barrettPalmPose.pose.orientation.x = 0.77129; 
  barrettPalmPose.pose.orientation.y = 0.014485;
  barrettPalmPose.pose.orientation.z = 0.63544;
  barrettPalmPose.pose.orientation.w = -0.03349;

  std::cout << "** Target Pose **\n" << barrettPalmPose << "\n";*/

  barrettPalmMoveGroup.setPoseTarget(midPointPose, END_EFFECTOR_LINK);
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