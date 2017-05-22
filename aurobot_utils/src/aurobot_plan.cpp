#include <iostream>
#include <vector>
#include <map>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

// Grasp planner
/*void planGrasp(GraspConfiguration grasp) {
  tf::TransformListener transformer;
  tf::Stamped<tf::Point> firstPointIn, firstPointOut;;
  firstPointIn.setX(grasp.firstPoint.x);
  firstPointIn.setY(grasp.firstPoint.y);
  firstPointIn.setZ(grasp.firstPoint.z);
  firstPointIn.frame_id_ = "/head_link";

  transformer.waitForTransform("/world", "/head_link", ros::Time(0), ros::Duration(3.0));
  transformer.transformPoint("/world", firstPointIn, firstPointOut);

  std::cout << "First point in:\n" << "> x = " << firstPointIn.x() << "\n"
    << "> y = " << firstPointIn.y() << "\n"
    << "> z = " << firstPointIn.z() << "\n";

  std::cout << "First point out:\n" << "> x = " << firstPointOut.x() << "\n"
    << "> y = " << firstPointOut.y() << "\n"
    << "> z = " << firstPointOut.z() << "\n";


  //tf::StampedTransform transform;

  //tfListener.waitForTransform("/world", "/head_link", ros::Time(0), ros::Duration(3.0));
  //tfListener.lookupTransform("/world", "/head_link", ros::Time(0), transform);
}*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = argv[1]; // "left_allegro" "left_pa10"
  std::cout << "= = = = = Planing for group " << PLANNING_GROUP << " = = = = =\n";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performanc
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  /*moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();*/

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  std::cout << "** Current Pose **\n" << move_group.getCurrentPose();

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  //geometry_msgs::PoseStamped target_pose1 = move_group.getRandomPose();//"l_allegro_link_3_tip");
  /*geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.stamp = ros::Time::now();
  target_pose1.header.frame_id = "/world";
  target_pose1.pose.position.x = 0.0656247;
  target_pose1.pose.position.y = 0.636605;
  target_pose1.pose.position.z = 2.10012;
  target_pose1.pose.orientation.x = 0.627422;
  target_pose1.pose.orientation.y = 0.470274;
  target_pose1.pose.orientation.z = -0.60507;
  target_pose1.pose.orientation.w = 0.138107;*/
  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.stamp = ros::Time::now();
  target_pose1.header.frame_id = "/world";
  target_pose1.pose.position.x = -0.0553096;
  target_pose1.pose.position.y = 1.47976;
  target_pose1.pose.position.z = 2.00281;
  target_pose1.pose.orientation.x = -0.326506;
  target_pose1.pose.orientation.y = -0.379928;
  target_pose1.pose.orientation.z = -0.596368;
  target_pose1.pose.orientation.w = 0.627211;

  std::cout << "=== Pose l_allegro_link_3_tip ===\n" << target_pose1 << "\n";

  move_group.setPoseTarget(target_pose1);
  //move_group.setRandomTarget();

  /*geometry_msgs::PoseStamped target_pose2;
  target_pose2.header.stamp = ros::Time::now();
  target_pose2.header.frame_id = "/world";
  target_pose2.pose.position.x = 0.26469;
  target_pose2.pose.position.y = 0.681387;
  target_pose2.pose.position.z = 1.39567;
  target_pose2.pose.orientation.x = 0.843809;
  target_pose2.pose.orientation.y = 0.387723;
  target_pose2.pose.orientation.z = 0.191163;
  target_pose2.pose.orientation.w = 0.317983;*/
  geometry_msgs::PoseStamped target_pose2;
  target_pose2.header.stamp = ros::Time::now();
  target_pose2.header.frame_id = "/world";
  target_pose2.pose.position.x = -0.115082;
  target_pose2.pose.position.y = 1.24525;
  target_pose2.pose.position.z = 1.97917;
  target_pose2.pose.orientation.x = -0.073392;
  target_pose2.pose.orientation.y = -0.43582;
  target_pose2.pose.orientation.z = 0.594675;
  target_pose2.pose.orientation.w = 0.671592;

  std::cout << "=== Pose l_allegro_link_15_tip ===\n" << target_pose2 << "\n";

  //move_group.setPoseTarget(target_pose2, "l_allegro_link_15_tip");

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

  ROS_INFO_NAMED("tutorial", "Plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");*/

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
  move_group.move();

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  /*moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (unsigned int i = 0; i < joint_group_positions.size(); ++i)
    std::cout << "Joint " << i << ": " << joint_group_positions[i] << "\n";

  std::map<std::string, std::string> params = move_group.getPlannerParams("TRRTkConfigDefault", PLANNING_GROUP);

  std::cout << "** Planner params **\n";

  for (std::map<std::string, std::string>::const_iterator it = params.begin();
    it != params.end(); ++it)
    std::cout << it->first << ":" << it->second << "\n";

  std::cout << "* *\n";

  current_state->printStatePositions();

  std::cout << "* *\n";

  current_state->printStateInfo();

  std::cout << "* *\n";

  current_state->printDirtyInfo();*&

  // Home
  /*joint_group_positions[1] = 1.349;
  joint_group_positions[3] = -2.389;

  move_group.setJointValueTarget(joint_group_positions);
  bool success = move_group.plan(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");*/
  
  ros::shutdown();
  return 0;
}