#include <iostream>
#include <vector>
#include <map>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf/transform_listener.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = argv[1];
  static const std::string END_EFFECTOR_LINK = argv[2]; 
  std::cout << "= = = = = Group: " << PLANNING_GROUP << " = = = = =\n";
  std::cout << "= = = = = End effector: " << END_EFFECTOR_LINK << " = = = = =\n";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.clearPathConstraints();

  // Raw pointers are frequently used to refer to the planning group for improved performanc
  /*const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);*/

  moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();
  const robot_state::JointModelGroup *joint_model_group = 
     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

  Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform(END_EFFECTOR_LINK);

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  std::cout << "** Current Pose **\n" << move_group.getCurrentPose(END_EFFECTOR_LINK);

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  //geometry_msgs::PoseStamped target_pose1 = move_group.getRandomPose();
  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.stamp = ros::Time::now();
  target_pose1.header.frame_id = "/world";

  if (PLANNING_GROUP == "l_arm_palm") {
    target_pose1.pose.position.x = 0.468008;
    target_pose1.pose.position.y = 0.26961;
    target_pose1.pose.position.z = 1.09587;
    target_pose1.pose.orientation.x = -0.687511;
    target_pose1.pose.orientation.y = -0.0424757;
    target_pose1.pose.orientation.z = -0.724707;
    target_pose1.pose.orientation.w = 0.0180148;
  }
  else if (END_EFFECTOR_LINK == "l_barrett_finger_13_link") {
    target_pose1.pose.position.x = 0.594424;
    target_pose1.pose.position.y = 0.317063;
    target_pose1.pose.position.z = 1.06479;
    target_pose1.pose.orientation.x = -0.0850373;
    target_pose1.pose.orientation.y = 0.141775;
    target_pose1.pose.orientation.z = 0.940728;
    target_pose1.pose.orientation.w = 0.296141;
  }
  else if (END_EFFECTOR_LINK == "l_barrett_finger_23_link") {
    target_pose1.pose.position.x = 0.590324;
    target_pose1.pose.position.y = 0.317075;
    target_pose1.pose.position.z = 1.13665;
    target_pose1.pose.orientation.x = 0.0274441;
    target_pose1.pose.orientation.y = -0.126509;
    target_pose1.pose.orientation.z = 0.941023;
    target_pose1.pose.orientation.w = 0.312598;
  }
  else if (END_EFFECTOR_LINK == "l_barrett_finger_33_link") {
    target_pose1.pose.position.x = 0.58082;
    target_pose1.pose.position.y = 0.17584; 
    target_pose1.pose.position.z = 1.1049;
    target_pose1.pose.orientation.x = 0.31122;
    target_pose1.pose.orientation.y = 0.94982;
    target_pose1.pose.orientation.z = -0.0070576;
    target_pose1.pose.orientation.w = 0.030593;
  }

  std::cout << "=== Target Pose ===\n" << target_pose1 << "\n";

  move_group.setPoseTarget(target_pose1, END_EFFECTOR_LINK);
  //move_group.setRandomTarget();

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group.setPlannerId("TRRTkConfigDefault");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  bool success = move_group.plan(my_plan);

  ROS_INFO("Plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  
  ros::shutdown();
  return 0;
}