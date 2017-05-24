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

  /*Eigen::Vector3d left_pa10_translation(0.6306, 0.350007, 1.3018);
  Eigen::Quaterniond left_pa10_rotation(0.788613, -0.511256, 0.160694, -0.301472);
  end_effector_state.translation() = left_pa10_translation;
  end_effector_state.linear() = left_pa10_rotation.toRotationMatrix();
  
  ROS_INFO_STREAM("Target Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Target Rotation: " << end_effector_state.rotation());

  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  if (found_ik) {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  else
    ROS_INFO("Did not find IK solution");*/

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

  if (PLANNING_GROUP == "left_pa10") {
    target_pose1.pose.position.x = 0.6306;
    target_pose1.pose.position.y = 0.350007;
    target_pose1.pose.position.z = 1.3018;
    target_pose1.pose.orientation.x = -0.511256;
    target_pose1.pose.orientation.y = 0.160694;
    target_pose1.pose.orientation.z = -0.301472;
    target_pose1.pose.orientation.w = 0.788613;
  } else if (PLANNING_GROUP == "left_arm") {
    target_pose1.pose.position.x = 0.50452;
    target_pose1.pose.position.y = 0.09943;
    target_pose1.pose.position.z = 1.1732;
    target_pose1.pose.orientation.x = 0.28852;
    target_pose1.pose.orientation.y = 0.51667;
    target_pose1.pose.orientation.z = 0.80401;
    target_pose1.pose.orientation.w = -0.058152;
  } else if (PLANNING_GROUP == "left_thumb") {
    target_pose1.pose.position.x = 0.61009;
    target_pose1.pose.position.y = 0.137;
    target_pose1.pose.position.z = 1.1753;
    target_pose1.pose.orientation.x = 0.097206;
    target_pose1.pose.orientation.y = 0.699;
    target_pose1.pose.orientation.z = 0.70511;
    target_pose1.pose.orientation.w = -0.069102;
  } else if (PLANNING_GROUP == "left_f1") {
    target_pose1.pose.position.x = 0.62705;
    target_pose1.pose.position.y = 0.25687;
    target_pose1.pose.position.z = 1.2062;
    target_pose1.pose.orientation.x = -0.42915;
    target_pose1.pose.orientation.y = 0.72759;
    target_pose1.pose.orientation.z = -0.30149;
    target_pose1.pose.orientation.w = 0.4422;
  }

  std::cout << "=== Target Pose ===\n" << target_pose1 << "\n";

  move_group.setPoseTarget(target_pose1, END_EFFECTOR_LINK);
  //move_group.setRandomTarget();

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group.setPlannerId("TRRTkConfigDefault");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  bool success = move_group.plan(my_plan);

  ROS_INFO("Plan 1 (pose goal) %s", success ? "" : "FAILED");

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
  joint_group_positions[1] = 1.349;
  joint_group_positions[3] = -2.389;

  move_group.setJointValueTarget(joint_group_positions);
  bool success = move_group.plan(my_plan);
  ROS_INFO("Visualizing plan (joint space goal) %s", success ? "" : "FAILED");*/
  
  ros::shutdown();
  return 0;
}