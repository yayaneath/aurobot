// Standard
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
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
#include <geograsp/GraspConfigMsg.h>

//
// CONST VARIABLES
//

const std::string CAMERA_FRAME = "/top_camera_link";

const std::string MIDDLE_PLANNING_GROUP = "rh_middle_finger";
const std::string MIDDLE_END_EFFECTOR_LINK = "rh_mftip";
const std::string MIDDLE_MOVING_JOINT = "rh_MFJ3";

const std::string THUMB_PLANNING_GROUP = "rh_thumb";
const std::string THUMB_END_EFFECTOR_LINK = "rh_thtip";
const std::string THUMB_MOVING_JOINT = "rh_THJ5";

const std::string SHADOW_HAND_PLANNING_GROUP = "right_hand";
//const std::string PALM_PLANNING_GROUP = "l_armpalm";
//const std::string PALM_END_EFFECTOR_LINK = "l_allegro_base_link";

const std::string PREGRASP_NAMED_TARGET = "pregrasp-mf";

const std::string COLLISION_OBJECT_ID = "grasping_object";

const std::string GRASP_CONFIG_TOPIC = "/geograsp/grasp_config";


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
//  the joints of the Shadow.
//

double getMiddleJointPosition(double width) {
  double a = 0.0487254446, b = -0.0628793987, c = -0.124767184, d = 0.1423194601;
  return a * std::pow(width, 3) + b  * std::pow(width, 2) + c * width + d;
}

double getThumbJointPosition(double width) {
  double a = 0.0694423878, b = -0.0524932202, c = -0.1576193457, d = 0.122830732;
  return a * std::pow(width, 3) + b  * std::pow(width, 2) + c * width + d;
}



//
//  AUXILIAR GRASPING FUNCTION
//
//  These functions positions the fingers regarding the object's width.
//

bool moveShadowPregrasp() {
  moveit::planning_interface::MoveGroupInterface shadowMoveGroup(SHADOW_HAND_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan shadowMovePlan;
  bool planSuccess = false;
  
  shadowMoveGroup.setJointValueTarget(shadowMoveGroup.getNamedTargetValues(PREGRASP_NAMED_TARGET));
  planSuccess = shadowMoveGroup.plan(shadowMovePlan);

  if(!planSuccess){
    std::cout << "[ERROR] Fingers planning for joint poisition failed!\n";
    return false;
  }

  shadowMoveGroup.execute(shadowMovePlan);

  return true;
}

bool moveShadowGrasp(double objectWidth) {
  moveit::planning_interface::MoveGroupInterface shadowMoveGroup(SHADOW_HAND_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan shadowMovePlan;
  std::map<std::string, double> graspJoints;
  bool planSuccess = false;
  double middleJointPosition = getMiddleJointPosition(objectWidth);
  double thumbJointPosition = getThumbJointPosition(objectWidth);

  graspJoints = shadowMoveGroup.getNamedTargetValues(PREGRASP_NAMED_TARGET);
  graspJoints[MIDDLE_MOVING_JOINT] = middleJointPosition;
  graspJoints[THUMB_MOVING_JOINT] = thumbJointPosition;

  shadowMoveGroup.setJointValueTarget(graspJoints);
  //shadowMoveGroup.setMaxVelocityScalingFactor(0.50);
  planSuccess = shadowMoveGroup.plan(shadowMovePlan);

  if(!planSuccess){
    std::cout << "[ERROR] Fingers planning for joint poisition failed!\n";
    return false;
  }

  shadowMoveGroup.execute(shadowMovePlan);

  return true;
}


Eigen::Vector3d computeMiddlePoint() {
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

  Eigen::Vector3d graspMiddle((middlePoint[0] + thumbPoint[0]) / 2.0,
    (middlePoint[1] + thumbPoint[1]) / 2.0, (middlePoint[2] + thumbPoint[2]) / 2.0);

  return graspMiddle;
}


void drawReferencePoints(rviz_visual_tools::RvizVisualToolsPtr visualTools){
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
  /*moveit::planning_interface::MoveGroupInterface palmMoveGroup(PALM_PLANNING_GROUP);
  geometry_msgs::PoseStamped palmCurrentPose = palmMoveGroup.getCurrentPose(PALM_END_EFFECTOR_LINK);
  Eigen::Vector3d palmPoint(palmCurrentPose.pose.position.x, palmCurrentPose.pose.position.y,
    palmCurrentPose.pose.position.z);*/

  // Midpoint between thumb and middle finger
  Eigen::Vector3d graspMiddle((middlePoint[0] + thumbPoint[0]) / 2.0,
    (middlePoint[1] + thumbPoint[1]) / 2.0, (middlePoint[2] + thumbPoint[2]) / 2.0);  

  visualTools->publishSphere(middlePoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visualTools->publishSphere(thumbPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visualTools->publishSphere(graspMiddle, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
  //visualTools->publishSphere(palmPoint, rviz_visual_tools::BROWN, rviz_visual_tools::LARGE);
  visualTools->trigger();
}



//
//  GRASPER FUNCTION
//
//  This function executes the whole grasping process given a tuple of contact points.
//  The process is the following:
//  - Transform the contact points to the /world frame
//  - Preprosition the robotic fingers to a pre grasping position
//  - Calculate the palm pose regarding the object orientation and the contact points
//  - Move the arm and the palm to such pose and close the fingers
//

void planGrasp(const aurobot_utils::GraspConfiguration & inputGrasp, const std::string & collisionId) {
  rviz_visual_tools::RvizVisualToolsPtr visualTools;
  visualTools.reset(new rviz_visual_tools::RvizVisualTools("/world", "/rviz_visual_markers"));
  visualTools->trigger();
  moveit::planning_interface::MoveGroupInterface allegroPalmMoveGroup(PALM_PLANNING_GROUP);

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Transform the coordinates from the camera frame to the world

  tf::Stamped<tf::Point> firstPointIn, secondPointIn, objAxisCenterIn;
  firstPointIn.setX(inputGrasp.first_point_x);
  firstPointIn.setY(inputGrasp.first_point_y);
  firstPointIn.setZ(inputGrasp.first_point_z);
  firstPointIn.frame_id_ = CAMERA_FRAME;
  secondPointIn.setX(inputGrasp.second_point_x);
  secondPointIn.setY(inputGrasp.second_point_y);
  secondPointIn.setZ(inputGrasp.second_point_z);
  secondPointIn.frame_id_ = CAMERA_FRAME;
  objAxisCenterIn.setX(inputGrasp.obj_axis_coeff_0);
  objAxisCenterIn.setY(inputGrasp.obj_axis_coeff_1);
  objAxisCenterIn.setZ(inputGrasp.obj_axis_coeff_2);
  objAxisCenterIn.frame_id_ = CAMERA_FRAME;

  Eigen::Vector3d firstPoint = transformPoint(firstPointIn, CAMERA_FRAME, "/world");
  Eigen::Vector3d secondPoint = transformPoint(secondPointIn, CAMERA_FRAME, "/world");
  Eigen::Vector3d objAxisCenter = transformPoint(objAxisCenterIn, CAMERA_FRAME, "/world");

  tf::Stamped<tf::Vector3> objAxisVectorIn;
  objAxisVectorIn.setX(inputGrasp.obj_axis_coeff_3);
  objAxisVectorIn.setY(inputGrasp.obj_axis_coeff_4);
  objAxisVectorIn.setZ(inputGrasp.obj_axis_coeff_5);
  objAxisVectorIn.frame_id_ = CAMERA_FRAME;

  Eigen::Vector3d objAxisVector = transformVector(objAxisVectorIn, CAMERA_FRAME, "/world");

  visualTools->publishSphere(firstPoint, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
  visualTools->publishSphere(secondPoint, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visualTools->publishSphere(objAxisCenter, rviz_visual_tools::GREY, rviz_visual_tools::LARGE);
  visualTools->publishLine(objAxisCenter, objAxisCenter + objAxisVector, rviz_visual_tools::GREY,
    rviz_visual_tools::MEDIUM);
  visualTools->trigger();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Read the object's cloud message, transform it and get the boundary coordinates

  sensor_msgs::PointCloud2 objectcloudsMsgIn = inputGrasp.object_cloud, objectCloudMsgOut;
  tf::TransformListener tfListener;
  tf::StampedTransform transform;

  tfListener.waitForTransform("/world", CAMERA_FRAME, ros::Time(0), ros::Duration(3.0));
  tfListener.lookupTransform("/world", CAMERA_FRAME, ros::Time(0), transform);
  pcl_ros::transformPointCloud("/world", transform, objectcloudsMsgIn, objectCloudMsgOut);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg<pcl::PointXYZRGB>(objectCloudMsgOut, *objectCloud);

  std::cout << "Object's cloud size: " << objectCloud->width * objectCloud->height << "\n";

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Place gripper joints for reaching

  double pointsDistance = std::sqrt(std::pow(firstPoint[0] - secondPoint[0], 2) + 
      std::pow(firstPoint[1] - secondPoint[1], 2) + std::pow(firstPoint[2] - secondPoint[2], 2));
  
  if (!moveShadowGrasp(pointsDistance)){
    std::cout << "[ERROR] Fingers movement for pregrasp pose failed!\n";
    return;
  }

  drawReferencePoints(visualTools);
  ROS_INFO("[AUROBOT] FINGERS POSITIONED IN PRE GRASPING POSE");
   
  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Calculate grasper palm position and orientation

  Eigen::Vector3d midPoint((firstPoint[0] + secondPoint[0]) / 2.0,
    (firstPoint[1] + secondPoint[1]) / 2.0, (firstPoint[2] + secondPoint[2]) / 2.0);

  /*visualTools->publishSphere(midPoint, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
  visualTools->trigger();*/

  Eigen::Vector3d axeX, axeY, axeZ;
  Eigen::Vector3d worldZ(0, 0, 1), worldY(0, 1, 0);
  float standingThreshold = 0.9, xAxisThreshold = 0.5, 
    graspCos = std::abs((worldZ.dot(objAxisVector)) / (worldZ.norm() * objAxisVector.norm()));

  std::cout << "Cosine grasp - world: " << graspCos << "\n";
  
  //Re-arrenged for the allegro pose
  axeZ = firstPoint - secondPoint;
  axeX = axeZ.cross(objAxisVector);
  axeY = axeZ.cross(axeX);

  float axeXdotWorldY = axeX.dot(worldY);
  float axeYdotObjAxis = axeY.dot(objAxisVector);

  std::cout << "Dot entre axeY y ObjAxis:" << axeYdotObjAxis << "\n";
  std::cout << "Dot entre axeX y worldY:" << axeXdotWorldY << "\n";

  std::cout << "Axe X:" << axeX << "\n";
  std::cout << "Axe Y:" << axeY << "\n";
  std::cout << "Axe Z:" << axeZ << "\n";

  // Hand pointing towards the camera
  // TODO: CHECK ONLY AXEXDOTWORLDY?
  if (axeXdotWorldY > 0 && axeYdotObjAxis < 0) {
    std::cout << "Change to Axe X and Z (reverse fingers)\n";
    
    axeX = -axeX;
    axeZ = -axeZ;
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

  /*visualTools->publishAxis(midPointPose, rviz_visual_tools::MEDIUM);
  visualTools->trigger();*/

  Eigen::Vector3d allegroMiddle = computeMiddlePoint();
  tf::Stamped<tf::Point> allegroMiddleIn;
  allegroMiddleIn.setX(allegroMiddle[0]);
  allegroMiddleIn.setY(allegroMiddle[1]);
  allegroMiddleIn.setZ(allegroMiddle[2]);
  allegroMiddleIn.frame_id_ = "/world";

  allegroMiddle = transformPoint(allegroMiddleIn, "/world", "/l_allegro_base_link");

  // Moving the pose backwards to set the palm position 
  // TODO: DEPENDING ON THE OBJECT'S SIZE WE SHOULD ADD 0.005 OR 0.01...
  Eigen::Vector3d midPointCentered(-allegroMiddle[0] + 0.000, -allegroMiddle[1], -allegroMiddle[2]);
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

  moveShadowPregrasp();

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Moving arm + palm and close grasp

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  moveit::planning_interface::MoveGroupInterface::Plan allegroPalmPlan;
  bool successAllegroPalmPlan = false;

  allegroPalmMoveGroup.setPoseTarget(allegroMidPointPregraspPose, PALM_END_EFFECTOR_LINK);
  //TRRTkConfigDefault //RRTConnectkConfigDefault // Lento! PRMstarkConfigDefault
  allegroPalmMoveGroup.setPlannerId("PRMstarkConfigDefault");
  allegroPalmMoveGroup.setPlanningTime(5.0);
  allegroPalmMoveGroup.setNumPlanningAttempts(20);
  allegroPalmMoveGroup.setMaxVelocityScalingFactor(0.50);
  allegroPalmMoveGroup.setMaxAccelerationScalingFactor(0.50);

  successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

  ROS_INFO("Palm pregrasp plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

  if (successAllegroPalmPlan) { // Successful plan for the pre grasping arm position
    std::cout << "PRESS ENTER TO MOVE PA10 TO PREGRASPING POSE\n";
    std::getchar();

    allegroPalmMoveGroup.execute(allegroPalmPlan);
    ROS_INFO("[AUROBOT] ARM PALM POSITIONED IN PREGRASPING POSE");
  
    std::vector<std::string> objectId;
    objectId.push_back(collisionId);
    planningSceneInterface.removeCollisionObjects(objectId);

    allegroPalmMoveGroup.setPoseTarget(allegroMidPointPose, PALM_END_EFFECTOR_LINK);
    successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

    ROS_INFO("Palm grasp plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

    if (successAllegroPalmPlan) { // Successful plan for the grasping arm position
      std::cout << "PRESS ENTER TO MOVE PA10 TO GRASPING POSE\n";
      std::getchar();

      allegroPalmMoveGroup.execute(allegroPalmPlan);
      ROS_INFO("[AUROBOT] ARM PALM POSITIONED IN GRASPING POSE");
      
      std::cout << "PRESS ENTER TO GRASP\n";
      std::getchar();

      if (!moveShadowGrasp(pointsDistance * 0.75))
        std::cout << "[ERROR] Fingers movement for closing grasp failed!\n";

      ROS_INFO("[AUROBOT] GRASP COMPLETED");

      allegroPalmMoveGroup.setPoseTarget(allegroMidPointPostgraspPose, PALM_END_EFFECTOR_LINK);
      successAllegroPalmPlan = allegroPalmMoveGroup.plan(allegroPalmPlan);

      ROS_INFO("Palm grasp plan %s", successAllegroPalmPlan ? "SUCCEED" : "FAILED");

      if (successAllegroPalmPlan) { // Successful plan for the post grasping arm position
        std::cout << "PRESS ENTER TO MOVE PA10 TO POSTGRASPING POSE\n";
        std::getchar();

        allegroPalmMoveGroup.execute(allegroPalmPlan);
      }
    }
  }

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Clear published objects and shutdown

  std::vector<std::string> objectsNames = planningSceneInterface.getKnownObjectNames();

  std::cout << "Cleaning " << objectsNames.size() << " objects\n";

  for(int i = 0; i < objectsNames.size(); ++i)
    std::cout << "Object " << i << ": '" << objectsNames[i] << "'\n";
  
  planningSceneInterface.removeCollisionObjects(objectsNames);

  ros::shutdown();
}



//
//  SCENE PROCESSING FUNCTION
//
//  This module is in charge of publishing collision objects by reading the scene input.
//  It calls the grasper main function with the closest object.
//

void processGraspMsg(const geograsp::GraspConfigMsgConstPtr & graspConfig) {
  //moveit::planning_interface::MoveGroupInterface allegroPalmMoveGroup(PALM_PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  std::string objCollId = "0";

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Read the object's cloud message, transform it and get the boundary coordinates

  sensor_msgs::PointCloud2 objectCloudMsgOut, objectcloudsMsgIn = graspConfig->object_cloud;
  tf::TransformListener tfListener;
  tf::StampedTransform transform;

  tfListener.waitForTransform("/world", CAMERA_FRAME, ros::Time(0), ros::Duration(3.0));
  tfListener.lookupTransform("/world", CAMERA_FRAME, ros::Time(0), transform);
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

  // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  // Add the cloud as a collision object

  moveit_msgs::CollisionObject collisionObject;
  //collisionObject.header.frame_id = allegroPalmMoveGroup.getPlanningFrame();
  collisionObject.header.frame_id = "/world";

  // The id of the object is used to identify it.
  /*std::stringstream sstm;
  sstm << COLLISION_OBJECT_ID << objectNum;
  collisionObject.id = sstm.str();*/
  collisionObject.id = objCollId;

  // Define a bounding box
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  // Scaled a little bit down so the bounding box does not exceed the real object boundaries
  primitive.dimensions[0] = 0.7 * std::abs(minBoundingPoint[0] - maxBoundingPoint[0]);
  primitive.dimensions[1] = 0.7 * std::abs(minBoundingPoint[1] - maxBoundingPoint[1]);
  primitive.dimensions[2] = 0.7 * std::abs(minBoundingPoint[2] - maxBoundingPoint[2]);

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose boxPose;
  boxPose.orientation.w = 1.0; // TODO: USE OBJECT'S AXIS?
  boxPose.position.x = (minBoundingPoint[0] + maxBoundingPoint[0]) / 2.0;
  boxPose.position.y = (minBoundingPoint[1] + maxBoundingPoint[1]) / 2.0;
  boxPose.position.z = (minBoundingPoint[2] + maxBoundingPoint[2]) / 2.0;

  collisionObject.primitives.push_back(primitive);
  collisionObject.primitive_poses.push_back(boxPose);
  collisionObject.operation = collisionObject.ADD;

  // Now, let's add the collision object into the world
  ROS_INFO("[AUROBOT] Add collision object into the world");
  planningSceneInterface.applyCollisionObject(collisionObject);

  std::cout << "Planning grasp to object ...\n";
  //planGrasp(graspConfig, objCollId);
}


//
//  MAIN FUNCTION
//
//  Waits for the last published message with the contact points and passes it to the
//  function in charge of planning and moving the hand to them.
//

int main(int argc, char **argv) {
  ros::init(argc, argv, "shadow_plan_grasp");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::cout << "Empezando...\n";

  geograsp::GraspConfigMsgConstPtr receivedMessage = 
    ros::topic::waitForMessage<geograsp::GraspConfigMsg>(GRASP_CONFIG_TOPIC);
  processGraspMsg(receivedMessage);

  std::cout << "Cerrando...\n";

  return 0;
}