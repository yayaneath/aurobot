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


//
// CONST VARIABLES
//

const std::string FINGERS_PLANNING_GROUP = "l_fingers";
const std::string FIRST_FINGER_PLANNING_GROUP = "l_first_finger";
const std::string FIRST_FINGER_END_EFFECTOR_LINK = "l_barrett_finger_13_tip_link"; 
const std::string PALM_PLANNING_GROUP = "l_arm_palm";
const std::string PALM_END_EFFECTOR_LINK = "l_barrett_base_link"; 
const double FINGERS_POSITION_PREGRASP_DIFF = 0.1;

//
//  AUXILIAR POINTS TRANSFORMER FUNCTION
//
//  This function transform a given point from a source frame to a target frame.
//

Eigen::Vector3d transformPoint(const tf::Stamped<tf::Point> & tfPointIn, 
    const std::string & sourceFrame, const std::string & targetFrame) {
  tf::TransformListener transformer;
  tf::Stamped<tf::Point> tfPointOut;

  transformer.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(3.0));
  transformer.transformPoint(targetFrame, tfPointIn, tfPointOut);

  Eigen::Vector3d outputPoint(tfPointOut.getX(), tfPointOut.getY(), tfPointOut.getZ());

  return outputPoint;
}


//
//  AUXILIAR JOINTS POSITION FINDING FUNCTION
//
//  This function holds the relationship between a certain tips width and 
//  the jx2 joints of the Barrett fingers.
//

double getJointsPosition(double width) {
  double a = -32.3411575486, b = 11.0048328179, c = -5.41125268, d = 1.6406172149;
  return a * std::pow(width, 3) + b  * std::pow(width, 2) + c * width + d;
}


//
//  AUXILIAR GRASPING FUNCTION
//
//  This functions positions the fingers second joint regarding the object's width.
//

bool moveFingersGrasp(double objectWidth, bool isPregrasp) {
  double jointsPosition = getJointsPosition(objectWidth);

  if (isPregrasp)
    jointsPosition -= FINGERS_POSITION_PREGRASP_DIFF; // A little bit open so we later finish the grasp

  moveit::planning_interface::MoveGroupInterface fingersMoveGroup(FINGERS_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan firstPlan, secondPlan, thirdPlan;
  bool firstPlanSuccess = false, secondPlanSuccess = false, thirdPlanSuccess = false;
  
  fingersMoveGroup.setJointValueTarget("l_barrett_j12_joint", jointsPosition);
  firstPlanSuccess = fingersMoveGroup.plan(firstPlan);

  if(!firstPlanSuccess){
    std::cout << "[ERROR] First finger planning for joint poisition failed!\n";
    return false;
  }
  
  fingersMoveGroup.setJointValueTarget("l_barrett_j22_joint", jointsPosition);
  secondPlanSuccess = fingersMoveGroup.plan(secondPlan);

  if(!secondPlanSuccess){
    std::cout << "[ERROR] Second finger planning for joint poisition failed!\n";
    return false;
  }
  
  fingersMoveGroup.setJointValueTarget("l_barrett_j32_joint", jointsPosition);
  thirdPlanSuccess = fingersMoveGroup.plan(thirdPlan);

  if(!thirdPlanSuccess){
    std::cout << "[ERROR] Third finger planning for joint poisition failed!\n";
    return false;
  }

  fingersMoveGroup.move();

  return true;
}


//
//  GRASPER FUNCTION
//
//  This functions executes the whole grasping process given a tuple of contact points.
//  The process is the following:
//  - Transform the contact points to the /world frame
//  - Preprosition the robotic fingers to a pre grasping position
//  - Calculate the palm pose regarding the object orientation and the contact points
//  - Move the arm and the palm to such pose and close the fingers
//

void planGrasp(const aurobot_utils::GraspConfigurationConstPtr & inputGrasp) {
  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Transform the coordinates from the camera frame to the world

  tf::Stamped<tf::Point> firstPointIn, secondPointIn, graspNormalIn;
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

  Eigen::Vector3d firstPoint = transformPoint(firstPointIn, "/head_link", "/world");
  Eigen::Vector3d secondPoint = transformPoint(secondPointIn, "/head_link", "/world");
  Eigen::Vector3d graspNormal = transformPoint(graspNormalIn, "/head_link", "/world");

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Preposition gripper joints for reaching

  double pointsDistance = std::sqrt(std::pow(firstPoint[0] - secondPoint[0], 2) + 
      std::pow(firstPoint[1] - secondPoint[1], 2) + std::pow(firstPoint[2] - secondPoint[2], 2));
  
  if (!moveFingersGrasp(pointsDistance, true)){
    std::cout << "[ERROR] Fingers movement for pregrasp pose failed!\n";
    return;
  }
    
  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Calcule grasper palm position and orientation

  Eigen::Vector3d axeX, axeY, axeZ;
  Eigen::Vector3d worldNormal(0, 0, 1);
  Eigen::Vector3d midPoint((firstPoint[0] + secondPoint[0]) / 2.0,
    (firstPoint[1] + secondPoint[1]) / 2.0, (firstPoint[2] + secondPoint[2]) / 2.0);
  float threshold = 0.9, graspCos = std::abs((worldNormal.dot(graspNormal)) / 
    (worldNormal.norm() * graspNormal.norm()));

  if (graspCos > threshold) {
    axeY = secondPoint - firstPoint;
    axeZ = axeY.cross(-graspNormal);
    axeX = axeY.cross(axeZ);
  }
  else {
    Eigen::Vector3d aux;

    axeY = secondPoint - firstPoint;
    axeZ = axeY.cross(-graspNormal);
    axeX = axeY.cross(axeZ);

    aux = axeZ;
    axeZ = -axeX;
    axeX = aux;
  }

  axeX.normalize();  axeY.normalize();  axeZ.normalize();

  Eigen::Affine3d midPointPose;
  Eigen::Matrix3d midPointRotation;
  midPointRotation << axeX[0], axeY[0], axeZ[0],
                      axeX[1], axeY[1], axeZ[1],
                      axeX[2], axeY[2], axeZ[2];
  midPointPose.translation() = midPoint;
  midPointPose.linear() = midPointRotation;

  // Moving the pose backwards 

  moveit::planning_interface::MoveGroupInterface firstMoveGroup(FIRST_FINGER_PLANNING_GROUP);
  geometry_msgs::PoseStamped firstCurrentPose = firstMoveGroup.getCurrentPose(FIRST_FINGER_END_EFFECTOR_LINK);
  tf::Stamped<tf::Point> fingerTipIn;
  fingerTipIn.setX(firstCurrentPose.pose.position.x);
  fingerTipIn.setY(firstCurrentPose.pose.position.y);
  fingerTipIn.setZ(firstCurrentPose.pose.position.z);
  fingerTipIn.frame_id_ = "/world";

  Eigen::Vector3d fingerTipVector = transformPoint(fingerTipIn, "/world", "/l_barrett_base_link");
  Eigen::Vector3d midPointCentered(0, 0, -fingerTipVector[2]);
  tf::Transform midPointTransform;
  tf::Vector3 midPointTF, midPointTFed;

  tf::transformEigenToTF(midPointPose, midPointTransform);
  tf::vectorEigenToTF(midPointCentered, midPointTF);

  midPointTFed = midPointTransform(midPointTF);

  tf::vectorTFToEigen(midPointTFed, midPointCentered);

  midPointPose.translation() = midPointCentered;

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
  // Moving arm + palm and close grasp

  moveit::planning_interface::MoveGroupInterface barrettPalmMoveGroup(PALM_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan barrettPalmPlan;
  bool successBarretPalmPlan = false;

  barrettPalmMoveGroup.setPoseTarget(midPointPose, PALM_END_EFFECTOR_LINK);
  barrettPalmMoveGroup.setPlannerId("TRRTkConfigDefault");
  barrettPalmMoveGroup.setPlanningTime(5.0);
  barrettPalmMoveGroup.setNumPlanningAttempts(10);
  barrettPalmMoveGroup.setMaxVelocityScalingFactor(1.0);
  barrettPalmMoveGroup.setMaxAccelerationScalingFactor(1.0);

  successBarretPalmPlan = barrettPalmMoveGroup.plan(barrettPalmPlan);

  ROS_INFO("Palm plan %s", successBarretPalmPlan ? "" : "FAILED");

  if (successBarretPalmPlan) {
    barrettPalmMoveGroup.move();
    
    if (!moveFingersGrasp(pointsDistance, false))
      std::cout << "[ERROR] Fingers movement for closing grasp failed!\n";
  }

  ros::shutdown();
}


//
//  MAIN FUNCTION
//
//  Waits for the last published message with the contact points and passes it to the
//  function in charge of planning and moving the hand to them.
//

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_grasper");
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