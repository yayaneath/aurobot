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

const std::string ALLEGRO_HAND_PLANNING_GROUP = "l_hand";
const std::string THREE_FINGER_PLANNING_GROUP = "l_three_finger_grasp";
const std::string PALM_PLANNING_GROUP = "l_armpalm";
const std::string PALM_END_EFFECTOR_LINK = "l_allegro_base_link";

const std::string COLLISION_OBJECT_ID = "grasping_object";


/* PREGRASP PARA EL BRAZO
name: ['pa10_green_shoulder_roll_joint', 'pa10_green_shoulder_lift_joint', 'pa10_green_upper_arm_roll_joint', 
'pa10_green_elbow_flex_joint', 'pa10_green_forearm_roll_joint', 'pa10_green_wrist_flex_joint', 
'pa10_green_wrist_roll_joint', 'l_allegro_joint_12', 'l_allegro_joint_13', 'l_allegro_joint_14', 
'l_allegro_joint_15', 'l_allegro_joint_8', 'l_allegro_joint_9', 'l_allegro_joint_10', 'l_allegro_joint_11', 
'l_allegro_joint_4', 'l_allegro_joint_5', 'l_allegro_joint_6', 'l_allegro_joint_7', 'l_allegro_joint_0', 
'l_allegro_joint_1', 'l_allegro_joint_2', 'l_allegro_joint_3', 'pa10_blue_shoulder_roll_joint', 
'pa10_blue_shoulder_lift_joint', 'pa10_blue_upper_arm_roll_joint', 'pa10_blue_elbow_flex_joint', 
'pa10_blue_forearm_roll_joint', 'pa10_blue_wrist_flex_joint', 'pa10_blue_wrist_roll_joint', 'WRJ2', 'WRJ1', 'FFJ4', 
'FFJ3', 'FFJ2', 'FFJ1', 'MFJ4', 'MFJ3', 'MFJ2', 'MFJ1', 'RFJ4', 'RFJ3', 'RFJ2', 'RFJ1', 'LFJ5', 'LFJ4', 'LFJ3', 
'LFJ2', 'LFJ1', 'THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1']
position: [-0.39501526247481616, 0.783829417742977, 1.3926185444240184, -0.8339432724677648, 4.020341569198964, 
0.6141949835704216, -3.5589956365624116, 0.3635738998060001, -2.956993915140629e-05, 3.2753014657646424e-05, 
3.212741934694351e-05, 7.999491612426938e-05, -3.394329003058375e-05, 1.567064370028675e-05, 6.98618661146611e-05, 
9.431893564760685e-05, 2.1826591156423093e-05, 3.579758736304939e-05, 6.723192846402526e-06, 5.145225096493961e-05, 
4.858152368105948e-05, -1.3675045734271412e-05, 9.300455474294722e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
*/

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

Eigen::Vector3d transformVector(const tf::Stamped<tf::Vector3> & tfVectorIn, 
    const std::string & sourceFrame, const std::string & targetFrame) {
  tf::TransformListener transformer;
  tf::Stamped<tf::Vector3> tfVectorOut;

  transformer.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(3.0));
  transformer.transformVector(targetFrame, tfVectorIn, tfVectorOut);

  Eigen::Vector3d outputVector(tfVectorOut.getX(), tfVectorOut.getY(), tfVectorOut.getZ());

  return outputVector;
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

bool moveAllegroPregrasp() {
  moveit::planning_interface::MoveGroupInterface fingersMoveGroup(ALLEGRO_HAND_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan fingersPlan;
  std::vector<double> fingersJointsValues;
  bool planSuccess = false;

  // First finger
  fingersJointsValues.push_back(0.0); // l_allegro_joint_0
  fingersJointsValues.push_back(0.0); // l_allegro_joint_1
  fingersJointsValues.push_back(0.0); // l_allegro_joint_2
  fingersJointsValues.push_back(0.0); // l_allegro_joint_3
  
  // Thumb
  fingersJointsValues.push_back(1.4968131524486665); // l_allegro_joint_12
  fingersJointsValues.push_back(0.0); // l_allegro_joint_13
  fingersJointsValues.push_back(0.0); // l_allegro_joint_14
  fingersJointsValues.push_back(0.0); // l_allegro_joint_15
  
  // Middle finger
  fingersJointsValues.push_back(0.0); // l_allegro_joint_4
  fingersJointsValues.push_back(0.0); // l_allegro_joint_5
  fingersJointsValues.push_back(0.0); // l_allegro_joint_6
  fingersJointsValues.push_back(0.0); // l_allegro_joint_7
  
  // Ring finger
  fingersJointsValues.push_back(0.0); // l_allegro_joint_8
  fingersJointsValues.push_back(0.0); // l_allegro_joint_9
  fingersJointsValues.push_back(0.0); // l_allegro_joint_10
  fingersJointsValues.push_back(0.0); // l_allegro_joint_11

  fingersMoveGroup.setJointValueTarget(fingersJointsValues);

  planSuccess = fingersMoveGroup.plan(fingersPlan);

  if(!planSuccess){
    std::cout << "[ERROR] Fingers planning for joint poisition failed!\n";
    return false;
  }

  fingersMoveGroup.move();

  return true;
}

bool moveFingersGrasp(double objectWidth) {
  double fingersJointPosition = getFingersJointPosition(objectWidth);
  double thumbJointPosition = getThumbJointPosition(objectWidth);

  moveit::planning_interface::MoveGroupInterface fingersMoveGroup(THREE_FINGER_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan fingersPlan;
  std::vector<double> fingersJointsValues;
  bool planSuccess = false;

  // First finger
  fingersJointsValues.push_back(-0.1446); // l_allegro_joint_0
  fingersJointsValues.push_back(fingersJointPosition); // l_allegro_joint_1
  fingersJointsValues.push_back(0.6641); // l_allegro_joint_2
  fingersJointsValues.push_back(0.5840); // l_allegro_joint_3
  
  // Thumb
  fingersJointsValues.push_back(1.3959); // l_allegro_joint_12
  fingersJointsValues.push_back(0.0700); // l_allegro_joint_13
  fingersJointsValues.push_back(thumbJointPosition); // l_allegro_joint_14
  fingersJointsValues.push_back(0.7372); // l_allegro_joint_15
  
  // Middle finger
  fingersJointsValues.push_back(-0.1446); // l_allegro_joint_4
  fingersJointsValues.push_back(fingersJointPosition); // l_allegro_joint_5
  fingersJointsValues.push_back(0.6641); // l_allegro_joint_6
  fingersJointsValues.push_back(0.5840); // l_allegro_joint_7

  fingersMoveGroup.setJointValueTarget(fingersJointsValues);
  fingersMoveGroup.setMaxVelocityScalingFactor(0.50);

  planSuccess = fingersMoveGroup.plan(fingersPlan);

  if(!planSuccess){
    std::cout << "[ERROR] Fingers planning for joint poisition failed!\n";
    return false;
  }

  fingersMoveGroup.move();

  return true;
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


void drawReferencePoints(rviz_visual_tools::RvizVisualToolsPtr visualTools){ 
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

  Eigen::Vector3d fingersMidPoint = computeMiddlePoint();

  Eigen::Vector3d graspMiddle((fingersMidPoint[0] + thumbPoint[0]) / 2.0,
    (fingersMidPoint[1] + thumbPoint[1]) / 2.0, (fingersMidPoint[2] + thumbPoint[2]) / 2.0);  

  visualTools->publishSphere(fingersMidPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visualTools->publishSphere(thumbPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visualTools->publishSphere(graspMiddle, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
  visualTools->publishSphere(palmPoint, rviz_visual_tools::BROWN, rviz_visual_tools::LARGE);
  visualTools->trigger();
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
  visualTools->trigger();
  moveit::planning_interface::MoveGroupInterface allegroPalmMoveGroup(PALM_PLANNING_GROUP);

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Transform the coordinates from the camera frame to the world

  tf::Stamped<tf::Point> firstPointIn, secondPointIn, objAxisCenterIn;
  firstPointIn.setX(inputGrasp->first_point_x);
  firstPointIn.setY(inputGrasp->first_point_y);
  firstPointIn.setZ(inputGrasp->first_point_z);
  firstPointIn.frame_id_ = "/head_link";
  secondPointIn.setX(inputGrasp->second_point_x);
  secondPointIn.setY(inputGrasp->second_point_y);
  secondPointIn.setZ(inputGrasp->second_point_z);
  secondPointIn.frame_id_ = "/head_link";
  objAxisCenterIn.setX(inputGrasp->obj_axis_coeff_0);
  objAxisCenterIn.setY(inputGrasp->obj_axis_coeff_1);
  objAxisCenterIn.setZ(inputGrasp->obj_axis_coeff_2);
  objAxisCenterIn.frame_id_ = "/head_link";

  Eigen::Vector3d firstPoint = transformPoint(firstPointIn, "/head_link", "/world");
  Eigen::Vector3d secondPoint = transformPoint(secondPointIn, "/head_link", "/world");
  Eigen::Vector3d objAxisCenter = transformPoint(objAxisCenterIn, "/head_link", "/world");

  tf::Stamped<tf::Vector3> objAxisVectorIn;
  objAxisVectorIn.setX(inputGrasp->obj_axis_coeff_3);
  objAxisVectorIn.setY(inputGrasp->obj_axis_coeff_4);
  objAxisVectorIn.setZ(inputGrasp->obj_axis_coeff_5);
  objAxisVectorIn.frame_id_ = "/head_link";

  Eigen::Vector3d objAxisVector = transformVector(objAxisVectorIn, "/head_link", "/world");

  if (objAxisVector[2] < 0) // We need it always pointing upwards
    objAxisVector = -objAxisVector;

  visualTools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visualTools->publishSphere(secondPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visualTools->publishSphere(objAxisCenter, rviz_visual_tools::GREY, rviz_visual_tools::LARGE);
  visualTools->publishLine(objAxisCenter, objAxisCenter + objAxisVector, rviz_visual_tools::GREY,
    rviz_visual_tools::MEDIUM);
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
  collisionObject.id = COLLISION_OBJECT_ID;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // Scaled a little bit down so the bounding box does not exceed the real object boundaries
  primitive.dimensions[0] = 0.8 * std::abs(minBoundingPoint[0] - maxBoundingPoint[0]);
  primitive.dimensions[1] = 0.8 * std::abs(minBoundingPoint[1] - maxBoundingPoint[1]);
  primitive.dimensions[2] = 0.8 * std::abs(minBoundingPoint[2] - maxBoundingPoint[2]);

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
  planningSceneInterface.applyCollisionObject(collisionObject);

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Place gripper joints for reaching

  double pointsDistance = std::sqrt(std::pow(firstPoint[0] - secondPoint[0], 2) + 
      std::pow(firstPoint[1] - secondPoint[1], 2) + std::pow(firstPoint[2] - secondPoint[2], 2));
  
  if (!moveFingersGrasp(pointsDistance)){
    std::cout << "[ERROR] Fingers movement for pregrasp pose failed!\n";
    return;
  }

  drawReferencePoints(visualTools);
  ROS_INFO("[AUROBOT] FINGERS POSITIONED IN PRE GRASPING POSE");
   
  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Calculate grasper palm position and orientation

  Eigen::Vector3d midPoint((firstPoint[0] + secondPoint[0]) / 2.0,
    (firstPoint[1] + secondPoint[1]) / 2.0, (firstPoint[2] + secondPoint[2]) / 2.0);

  visualTools->publishSphere(midPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
  visualTools->trigger();

  Eigen::Vector3d axeX, axeY, axeZ;
  Eigen::Vector3d worldZ(0, 0, 1), worldY(0, 1, 0);
  float standingThreshold = 0.9, xAxisThreshold = 0.5, 
    graspCos = std::abs((worldZ.dot(objAxisVector)) / (worldZ.norm() * objAxisVector.norm()));

  std::cout << "Cosine grasp - world: " << graspCos << "\n";
  
  // TODO: LAYING OBJECTS IN THE CAMERA X AXIS
  //Re-arrenged for the allegro pose
  axeZ = secondPoint - firstPoint;
  axeX = axeZ.cross(-objAxisVector);
  axeY = axeZ.cross(axeX);

  axeY = -axeY;
  axeZ = -axeZ;

  if (graspCos < standingThreshold && axeZ[0] <= 0) {
    std::cout << "Change to Axe X and Z (reverse fingers)\n";
    
    axeX = -axeX;
    axeZ = -axeZ;
  }

  // In case the object is standing up and the axe is pointing up
  if (graspCos >= standingThreshold &&
    std::abs((worldY.dot(axeX)) / (worldY.norm() * axeX.norm())) <= xAxisThreshold) {
    std::cout << "Change to Axe Y and X (palm pointing up)\n";

    Eigen::Vector3d aux;

    aux = axeX;
    axeX = axeY;
    axeY = -aux;
  }
  
  axeX.normalize();
  axeY.normalize();
  axeZ.normalize();    

  Eigen::Affine3d midPointPose;
  Eigen::Matrix3d midPointRotation;
  midPointRotation << axeX[0], axeY[0], axeZ[0], 
                      axeX[1], axeY[1], axeZ[1],
                      axeX[2], axeY[2], axeZ[2];
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

  // Moving the pose backwards to set the palm position 
  // TODO: DEPENDING ON THE OBJECT'S SIZE WE SHOULD ADD 0.005 OR 0.01...
  Eigen::Vector3d midPointCentered(-allegroMiddle[0] + 0.005, -allegroMiddle[1], -allegroMiddle[2]);
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

  // Pregrasp pose, a litle bit further
  Eigen::Vector3d midPointCenteredPregrasp(-allegroMiddle[0] - 0.1, -allegroMiddle[1], -allegroMiddle[2]);

  tf::vectorEigenToTF(midPointCenteredPregrasp, midPointTF);
  midPointTFed = midPointTransform(midPointTF);
  tf::vectorTFToEigen(midPointTFed, midPointCenteredPregrasp);

  Eigen::Affine3d allegroMidPointPregraspPose = midPointPose;
  allegroMidPointPregraspPose.translation() = midPointCenteredPregrasp;

  visualTools->publishSphere(midPointCenteredPregrasp, rviz_visual_tools::ORANGE, rviz_visual_tools::LARGE);
  visualTools->publishAxis(allegroMidPointPregraspPose, rviz_visual_tools::MEDIUM);
  visualTools->trigger();

  // Postgrasp pose, a litle bit upwards
  Eigen::Vector3d midPointCenteredPostgrasp = midPointCentered + Eigen::Vector3d(0, 0, 0.15);

  Eigen::Affine3d allegroMidPointPostgraspPose = midPointPose;
  allegroMidPointPostgraspPose.translation() = midPointCenteredPostgrasp;

  visualTools->publishSphere(midPointCenteredPostgrasp, rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
  visualTools->publishAxis(allegroMidPointPostgraspPose, rviz_visual_tools::MEDIUM);
  visualTools->trigger();

  moveAllegroPregrasp();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Moving arm + palm and close grasp

  moveit::planning_interface::MoveGroupInterface::Plan allegroPalmPlan;
  bool successAllegroPalmPlan = false;

  allegroPalmMoveGroup.setPoseTarget(allegroMidPointPregraspPose, PALM_END_EFFECTOR_LINK);
  allegroPalmMoveGroup.setPlannerId("TRRTkConfigDefault");
  allegroPalmMoveGroup.setPlanningTime(5.0);
  allegroPalmMoveGroup.setNumPlanningAttempts(20);
  allegroPalmMoveGroup.setMaxVelocityScalingFactor(0.50);
  allegroPalmMoveGroup.setMaxAccelerationScalingFactor(0.50);
  // TODO: DO NOT ALLOW REPLANNING

  successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

  ROS_INFO("Palm pregrasp plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

  if (successAllegroPalmPlan) { // Successful plan for the pre grasping arm position
    std::cout << "PRESS ENTER TO MOVE PA10 TO PREGRASPING POSE\n";
    std::getchar();

    allegroPalmMoveGroup.move();
    ROS_INFO("[AUROBOT] ARM PALM POSITIONED IN PREGRASPING POSE");
  
    std::vector<std::string> objectId;
    objectId.push_back(COLLISION_OBJECT_ID);
    planningSceneInterface.removeCollisionObjects(objectId);

    allegroPalmMoveGroup.setPoseTarget(allegroMidPointPose, PALM_END_EFFECTOR_LINK);
    successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

    ROS_INFO("Palm grasp plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

    if (successAllegroPalmPlan) { // Successful plan for the grasping arm position
      std::cout << "PRESS ENTER TO MOVE PA10 TO GRASPING POSE\n";
      std::getchar();

      allegroPalmMoveGroup.move();
      ROS_INFO("[AUROBOT] ARM PALM POSITIONED IN GRASPING POSE");
      
      std::cout << "PRESS ENTER TO GRASP\n";
      std::getchar();

      if (!moveFingersGrasp(pointsDistance * 0.85)) // TODO: CLOSE TIGHTER?
        std::cout << "[ERROR] Fingers movement for closing grasp failed!\n";

      ROS_INFO("[AUROBOT] GRASP COMPLETED");

      allegroPalmMoveGroup.setPoseTarget(allegroMidPointPostgraspPose, PALM_END_EFFECTOR_LINK);
      successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

      ROS_INFO("Palm grasp plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

      if (successAllegroPalmPlan) { // Successful plan for the post grasping arm position
        std::cout << "PRESS ENTER TO MOVE PA10 TO POSTGRASPING POSE\n";
        std::getchar();

        allegroPalmMoveGroup.move();
      }
    }
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

  publishRoomCollisions();

  aurobot_utils::GraspConfigurationConstPtr receivedMessage = 
    ros::topic::waitForMessage<aurobot_utils::GraspConfiguration>("/aurobot_utils/grasp_configuration");
  planGrasp(receivedMessage);

  std::cout << "Cerrando...\n";

  return 0;
}