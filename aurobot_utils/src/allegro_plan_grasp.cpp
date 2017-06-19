// Standard
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <Eigen/Geometry>

// ROS
#include <ros/topic.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

// Custom
#include <aurobot_utils/GraspConfiguration.h> // Custom message for publishing grasps


//
// CONST VARIABLES
//

const std::string FIRST_PLANNING_GROUP = "l_first_finger";
const std::string FIRST_END_EFFECTOR_LINK = "l_allegro_link_3_tip"; 
const std::string MIDDLE_PLANNING_GROUP = "l_middle_finger";
const std::string MIDDLE_END_EFFECTOR_LINK = "l_allegro_link_7_tip"; 
const std::string THUMB_PLANNING_GROUP = "l_thumb";
const std::string THUMB_END_EFFECTOR_LINK = "l_allegro_link_15_tip";
const std::string PALM_PLANNING_GROUP = "l_armpalm";
const std::string PALM_END_EFFECTOR_LINK = "l_allegro_base_link"; 
const double FINGERS_POSITION_PREGRASP_DIFF = 0.1;


// 
// AUXLIIAR SCENE COLLISIONS PUBLISHER
// 

void publishRoomCollisions() {
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  moveit_msgs::CollisionObject collisionObject;
  collisionObject.header.frame_id = "/world";
  collisionObject.id = "room_walls";

  shape_msgs::SolidPrimitive leftWall;
  leftWall.type = leftWall.BOX;
  leftWall.dimensions.resize(3);
  leftWall.dimensions[0] = 3.1;  leftWall.dimensions[1] = 0.1;  leftWall.dimensions[2] = 2.80;

  geometry_msgs::Pose leftWallPose;
  leftWallPose.orientation.w = 1.0;
  leftWallPose.position.x = 0;  leftWallPose.position.y = 1.85;  leftWallPose.position.z = 1.40;

  shape_msgs::SolidPrimitive backWall;
  backWall.type = backWall.BOX;
  backWall.dimensions.resize(3);
  backWall.dimensions[0] = 0.1;  backWall.dimensions[1] = 3.7;  backWall.dimensions[2] = 2.80;

  geometry_msgs::Pose backWallPose;
  backWallPose.orientation.w = 1.0;
  backWallPose.position.x = -1.58;  backWallPose.position.y = 0;  backWallPose.position.z = 1.40;

  shape_msgs::SolidPrimitive wardrobe;
  wardrobe.type = wardrobe.BOX;
  wardrobe.dimensions.resize(3);
  wardrobe.dimensions[0] = 0.80;  wardrobe.dimensions[1] = 0.40;  wardrobe.dimensions[2] = 1.97;

  geometry_msgs::Pose wardrobePose;
  wardrobePose.orientation.w = 1.0;
  wardrobePose.position.x = -1.13;  wardrobePose.position.y = 1.60;  wardrobePose.position.z = 0.985;

  collisionObject.primitives.push_back(leftWall);
  collisionObject.primitives.push_back(backWall);
  collisionObject.primitives.push_back(wardrobe);
  collisionObject.primitive_poses.push_back(leftWallPose);
  collisionObject.primitive_poses.push_back(backWallPose);
  collisionObject.primitive_poses.push_back(wardrobePose);
  collisionObject.operation = collisionObject.ADD;
  planningSceneInterface.applyCollisionObject(collisionObject);
}


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

double getThumbJointPosition(double width) {
  double a = -17.7363395438, b = 4.4662678438, c = -3.5024945072, d = 0.4839644326;
  return a * std::pow(width, 3) + b  * std::pow(width, 2) + c * width + d;
}

double getFingersJointPosition(double width) {
  double a = -40.5300629773, b = 10.205855059, c = -8.0039323331, d = 1.3418754138;
  return a * std::pow(width, 3) + b  * std::pow(width, 2) + c * width + d;
}


//
//  AUXILIAR GRASPING FUNCTION
//
//  This functions positions the fingers second joint regarding the object's width.
//

bool moveFingersGrasp(double objectWidth, bool isPregrasp) {
  double fingersJointPosition = getFingersJointPosition(objectWidth);
  double thumbJointPosition = getThumbJointPosition(objectWidth);

  if (isPregrasp) { // A little bit open so we later finish the grasp
    fingersJointPosition -= FINGERS_POSITION_PREGRASP_DIFF;
    thumbJointPosition -= FINGERS_POSITION_PREGRASP_DIFF;
  }

  moveit::planning_interface::MoveGroupInterface firstMoveGroup(FIRST_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface middleMoveGroup(MIDDLE_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface thumbMoveGroup(THUMB_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan firstPlan, middlePlan, thumbPlan;
  std::vector<double> firstJointsValues, middleJointsValues, thumbJointsValues;
  bool firstPlanSuccess = false, middlePlanSuccess = false, thumbPlanSuccess = false;
  
  firstJointsValues.push_back(-0.1446); // l_allegro_joint_0
  firstJointsValues.push_back(fingersJointPosition); // l_allegro_joint_1
  firstJointsValues.push_back(0.6641); // l_allegro_joint_2
  firstJointsValues.push_back(0.5840); // l_allegro_joint_3

  firstMoveGroup.setJointValueTarget(firstJointsValues);
  firstPlanSuccess = firstMoveGroup.plan(firstPlan);

  if(!firstPlanSuccess){
    std::cout << "[ERROR] First finger planning for joint poisition failed!\n";
    return false;
  }
  
  middleJointsValues.push_back(-0.1446); // l_allegro_joint_4
  middleJointsValues.push_back(fingersJointPosition); // l_allegro_joint_5
  middleJointsValues.push_back(0.6641); // l_allegro_joint_6
  middleJointsValues.push_back(0.5840); // l_allegro_joint_7
  
  middleMoveGroup.setJointValueTarget(middleJointsValues);
  middlePlanSuccess = middleMoveGroup.plan(middlePlan);

  if(!middlePlanSuccess){
    std::cout << "[ERROR] Second finger planning for joint poisition failed!\n";
    return false;
  }
  
  thumbJointsValues.push_back(1.3959); // l_allegro_joint_12
  thumbJointsValues.push_back(0.0700); // l_allegro_joint_13
  thumbJointsValues.push_back(thumbJointPosition); // l_allegro_joint_14
  thumbJointsValues.push_back(0.7372); // l_allegro_joint_15

  thumbMoveGroup.setJointValueTarget(thumbJointsValues);
  thumbPlanSuccess = thumbMoveGroup.plan(thumbPlan);

  if(!thumbPlanSuccess){
    std::cout << "[ERROR] Third finger planning for joint poisition failed!\n";
    return false;
  }

  firstMoveGroup.move();
  middleMoveGroup.move();
  thumbMoveGroup.move();

  return true;
}

void drawReferencePoints(rviz_visual_tools::RvizVisualToolsPtr visualTools){ 
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

  // Palm 

  moveit::planning_interface::MoveGroupInterface palmMoveGroup(PALM_PLANNING_GROUP);
  geometry_msgs::PoseStamped palmCurrentPose = palmMoveGroup.getCurrentPose(PALM_END_EFFECTOR_LINK);
  Eigen::Vector3d palmPoint(palmCurrentPose.pose.position.x, palmCurrentPose.pose.position.y,
    palmCurrentPose.pose.position.z);

  // Midpoint between first and second fingers

  Eigen::Vector3d fingersMidPoint((firstPoint[0] + middlePoint[0]) / 2.0,
    (firstPoint[1] + middlePoint[1]) / 2.0, (firstPoint[2] + middlePoint[2]) / 2.0);

  Eigen::Vector3d graspMiddle((fingersMidPoint[0] + thumbPoint[0]) / 2.0,
    (fingersMidPoint[1] + thumbPoint[1]) / 2.0, (fingersMidPoint[2] + thumbPoint[2]) / 2.0);  

  visualTools->publishSphere(fingersMidPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visualTools->publishSphere(thumbPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visualTools->publishSphere(graspMiddle, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
  visualTools->publishSphere(palmPoint, rviz_visual_tools::BROWN, rviz_visual_tools::LARGE);
  visualTools->trigger();
}


Eigen::Vector3d computeMiddlePoint() {
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

  Eigen::Vector3d graspMiddle((fingersMidPoint[0] + thumbPoint[0]) / 2.0,
    (fingersMidPoint[1] + thumbPoint[1]) / 2.0, (fingersMidPoint[2] + thumbPoint[2]) / 2.0);

  return graspMiddle;
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
  rviz_visual_tools::RvizVisualToolsPtr visualTools;
  visualTools.reset(new rviz_visual_tools::RvizVisualTools("/world", "/rviz_visual_markers"));
  moveit::planning_interface::MoveGroupInterface allegroPalmMoveGroup(PALM_PLANNING_GROUP);

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

  visualTools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visualTools->publishSphere(secondPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visualTools->trigger();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Read the object's cloud message, transform it and get the boundary coordinates

  sensor_msgs::PointCloud2 objectcloudsMsgIn = inputGrasp->object_cloud, objectCloudMsgOut;
  tf::TransformListener tfListener;
  tf::StampedTransform transform;

  tfListener.waitForTransform("/world", "/head_link", ros::Time(0), ros::Duration(3.0));
  tfListener.lookupTransform("/world", "/head_link", ros::Time(0), transform);
  pcl_ros::transformPointCloud("/world", transform, objectcloudsMsgIn, objectCloudMsgOut);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg<pcl::PointXYZRGB>(objectCloudMsgOut, *objectCloud);

  std::cout << "Object's cloud size: " << objectCloud->width * objectCloud->height << "\n";

  float minX, minY, minZ, maxX, maxY, maxZ;
  minX = minY = minZ = std::numeric_limits<float>::max();
  maxX = maxY = maxZ = -std::numeric_limits<float>::max();

  for (size_t i = 0; i < objectCloud->points.size(); ++i) {
    if (objectCloud->points[i].x < minX)
      minX = objectCloud->points[i].x;
    if (objectCloud->points[i].x > maxX)
      maxX = objectCloud->points[i].x;

    if (objectCloud->points[i].y < minY)
      minY = objectCloud->points[i].y;
    if (objectCloud->points[i].y > maxY)
      maxY = objectCloud->points[i].y;

    if (objectCloud->points[i].z < minZ)
      minZ = objectCloud->points[i].z;
    if (objectCloud->points[i].z > maxZ)
      maxZ = objectCloud->points[i].z;
  }

  Eigen::Vector3d minBoundingPoint(minX, minY, minZ), maxBoundingPoint(maxX, maxY, maxZ);

  visualTools->publishCuboid(minBoundingPoint, maxBoundingPoint, rviz_visual_tools::TRANSLUCENT);
  visualTools->trigger();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Add the cloud as a collision object

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  moveit_msgs::CollisionObject collisionObject;
  collisionObject.header.frame_id = allegroPalmMoveGroup.getPlanningFrame();

  // The id of the object is used to identify it.
  collisionObject.id = "grasping_object";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // Scaled a little bit down so the bounding box does not exceed the real object boundaries
  primitive.dimensions[0] = 0.5 * std::abs(minBoundingPoint[0] - maxBoundingPoint[0]);
  primitive.dimensions[1] = 0.5 * std::abs(minBoundingPoint[1] - maxBoundingPoint[1]);
  primitive.dimensions[2] = 0.5 * std::abs(minBoundingPoint[2] - maxBoundingPoint[2]);

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose boxPose;
  boxPose.orientation.w = 1.0;
  boxPose.position.x = (minBoundingPoint[0] + maxBoundingPoint[0]) / 2.0;
  boxPose.position.y = (minBoundingPoint[1] + maxBoundingPoint[1]) / 2.0;
  boxPose.position.z = (minBoundingPoint[2] + maxBoundingPoint[2]) / 2.0;

  collisionObject.primitives.push_back(primitive);
  collisionObject.primitive_poses.push_back(boxPose);
  collisionObject.operation = collisionObject.ADD;

  // Now, let's add the collision object into the world
  ROS_INFO("[AUROBOT] Add collision object into the world");
  //planningSceneInterface.applyCollisionObject(collisionObject);

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Place gripper joints for reaching

  double pointsDistance = std::sqrt(std::pow(firstPoint[0] - secondPoint[0], 2) + 
      std::pow(firstPoint[1] - secondPoint[1], 2) + std::pow(firstPoint[2] - secondPoint[2], 2));
  
  if (!moveFingersGrasp(pointsDistance, true)){
    std::cout << "[ERROR] Fingers movement for pregrasp pose failed!\n";
    return;
  }

  //drawReferencePoints(visualTools);
  ROS_INFO("[AUROBOT] FINGERS POSITIONED IN PRE GRASPING POSE");
    
  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Calculate grasper palm position and orientation

  Eigen::Vector3d midPoint((firstPoint[0] + secondPoint[0]) / 2.0,
    (firstPoint[1] + secondPoint[1]) / 2.0, (firstPoint[2] + secondPoint[2]) / 2.0);

  visualTools->publishSphere(midPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
  visualTools->trigger();

  Eigen::Vector3d axeX, axeY, axeZ;
  Eigen::Vector3d worldNormal(0, 0, 1);
  float threshold = 0.9, graspCos = std::abs((worldNormal.dot(graspNormal)) / 
    (worldNormal.norm() * graspNormal.norm()));

  axeY = secondPoint - firstPoint;
  axeZ = axeY.cross(-graspNormal);
  axeX = axeY.cross(axeZ);

  if (graspCos < threshold && -axeX[0] >= 0) {
    Eigen::Vector3d aux;

    aux = axeZ;
    axeZ = -axeX;
    axeX = aux;
  }

  if (axeZ[2] > 0) { // In case the axe is pointing up
    axeZ = -axeZ;
    axeY = -axeY;
  }

  axeX.normalize();  axeY.normalize();  axeZ.normalize();

  Eigen::Affine3d midPointPose;
  Eigen::Matrix3d midPointRotation;
  midPointRotation << axeZ[0], -axeX[0], -axeY[0], //Re-arrenged for the allegro pose
                      axeZ[1], -axeX[1], -axeY[1],
                      axeZ[2], -axeX[2], -axeY[2];
  midPointPose.translation() = midPoint;
  midPointPose.linear() = midPointRotation;

  visualTools->publishAxis(midPointPose, rviz_visual_tools::MEDIUM);
  visualTools->trigger();

  Eigen::Vector3d allegroMiddle = computeMiddlePoint();
  tf::Stamped<tf::Point> allegroMiddleIn;
  allegroMiddleIn.setX(allegroMiddle[0]);
  allegroMiddleIn.setY(allegroMiddle[1]);
  allegroMiddleIn.setZ(allegroMiddle[2]);
  allegroMiddleIn.frame_id_ = "/world";

  allegroMiddle = transformPoint(allegroMiddleIn, "/world", "/l_allegro_base_link");

  // Moving the pose backwards 
  Eigen::Vector3d midPointCentered = -allegroMiddle;
  tf::Transform midPointTransform;
  tf::Vector3 midPointTF, midPointTFed;

  tf::transformEigenToTF(midPointPose, midPointTransform);
  tf::vectorEigenToTF(midPointCentered, midPointTF);

  midPointTFed = midPointTransform(midPointTF);

  tf::vectorTFToEigen(midPointTFed, midPointCentered);

  Eigen::Affine3d allegroMidPointPose = midPointPose;
  allegroMidPointPose.translation() = midPointCentered;

  visualTools->publishSphere(midPointCentered, rviz_visual_tools::PINK, rviz_visual_tools::LARGE);
  visualTools->publishAxis(allegroMidPointPose, rviz_visual_tools::MEDIUM);
  visualTools->trigger();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Moving arm + palm and close grasp

  moveit::planning_interface::MoveGroupInterface::Plan allegroPalmPlan;
  bool successAllegroPalmPlan = false;

  allegroPalmMoveGroup.setPoseTarget(allegroMidPointPose, PALM_END_EFFECTOR_LINK);
  allegroPalmMoveGroup.setPlannerId("TRRTkConfigDefault");
  allegroPalmMoveGroup.setPlanningTime(5.0);
  allegroPalmMoveGroup.setNumPlanningAttempts(10);
  allegroPalmMoveGroup.setMaxVelocityScalingFactor(1.0);
  allegroPalmMoveGroup.setMaxAccelerationScalingFactor(1.0);

  successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

  ROS_INFO("Palm plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

  if (successAllegroPalmPlan) {
    allegroPalmMoveGroup.move();
    ROS_INFO("[AUROBOT] ARM PALM POSITIONED IN GRASPING POSE");
    
    // Esta mano no parece necesitar reajustar los dedos por el error en la función
    // que relaciona joints con amplitud.
    /*if (!moveFingersGrasp(pointsDistance, false)) 
      std::cout << "[ERROR] Fingers movement for closing grasp failed!\n";*/

    ROS_INFO("[AUROBOT] GRASP COMPLETED");
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
  ros::init(argc, argv, "allegro_grasp_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::cout << "Empezando...\n";

  //publishRoomCollisions();

  aurobot_utils::GraspConfigurationConstPtr receivedMessage = 
    ros::topic::waitForMessage<aurobot_utils::GraspConfiguration>("/aurobot_utils/grasp_configuration");
  planGrasp(receivedMessage);

  std::cout << "Cerrando...\n";

  return 0;
}