#include <iostream>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf/transform_listener.h>

#include <aurobot_utils/GraspConfiguration.h> // Custom message for publishing grasps



ros::Subscriber sub;

// Grasp planner
void planGrasp(const aurobot_utils::GraspConfigurationConstPtr & inputGrasp) {
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

  // Plan the first point for the pa10_thumb move group
  /*moveit::planning_interface::MoveGroupInterface moveGroupFirstContact("left_pa10_gripper");

  ROS_INFO_NAMED("aurobot_plan_grasp", "Reference frame: %s", moveGroupFirstContact.getPlanningFrame().c_str());
  ROS_INFO_NAMED("aurobot_plan_grasp", "End effector link: %s", moveGroupFirstContact.getEndEffectorLink().c_str());

  geometry_msgs::Pose targetPoseThumb;
  targetPoseThumb.position.x = firstPointOut.x();
  targetPoseThumb.position.y = firstPointOut.y();
  targetPoseThumb.position.z = firstPointOut.z();

  moveGroupFirstContact.setPoseTarget(targetPoseThumb);

  moveit::planning_interface::MoveGroupInterface::Plan firstPointPlan;

  moveGroupFirstContact.setPlannerId("TRRTkConfigDefault");
  bool success = moveGroupFirstContact.plan(firstPointPlan);

  ROS_INFO_NAMED("aurobot_plan_grasp", "PA10 Thumb plan %s", success ? "SUCCEED" : "FAILED");

  //moveGroupFirstContact.move();*/

  sub.shutdown(); // Unsubscribe so we do no longer receive messages
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_grasp_planner");
  ros::NodeHandle nh;
  /*ros::AsyncSpinner spinner(1);
  spinner.start();*/

  sub = nh.subscribe<aurobot_utils::GraspConfiguration>("/aurobot_utils/grasp_configuration",
    1, planGrasp);

  ros::spin();  
  ros::shutdown();
  return 0;
}