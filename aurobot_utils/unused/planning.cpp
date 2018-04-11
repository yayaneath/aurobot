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
  } else if (PLANNING_GROUP == "left_hand_thumb") {
    target_pose1.pose.position.x = 0.674316;
    target_pose1.pose.position.y = 0.256652;
    target_pose1.pose.position.z = 1.34674;
    target_pose1.pose.orientation.x = 0.234702;
    target_pose1.pose.orientation.y = 0.626569;
    target_pose1.pose.orientation.z = 0.305365;
    target_pose1.pose.orientation.w = 0.677553;
  } else if (PLANNING_GROUP == "left_hand_f0") {
    /*robot_state::RobotState start_state(*move_group.getCurrentState());
    move_group.setStartState(start_state);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "l_allegro_link_15_tip";
    ocm.header.frame_id = "/world";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);*/

    target_pose1.pose.position.x = 0.68558;//0.708291;
    target_pose1.pose.position.y = 0.22738;//0.377647;
    target_pose1.pose.position.z = 1.4598;//1.37516;
    target_pose1.pose.orientation.x = -0.48084;//-0.141341;
    target_pose1.pose.orientation.y = -0.30661;//0.822899;
    target_pose1.pose.orientation.z = 0.82143;//-0.444766;
    target_pose1.pose.orientation.w = -0.0053439;//0.324103;
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
  /*bool success = move_group.plan(my_plan);

  ROS_INFO("Plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();*/

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


#include <iostream>
#include <vector>
#include <map>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf/transform_listener.h>
#include <eigen_stl_containers/eigen_stl_containers.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = argv[1];
  std::cout << "= = = = = Group: " << PLANNING_GROUP << " = = = = =\n";

  // Load move group information and state
  moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);
  moveGroup.clearPathConstraints();

  moveit::core::RobotStatePtr kinematicState = moveGroup.getCurrentState();
  const robot_state::JointModelGroup *jointModelGroup = 
     moveGroup.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  std::vector<std::string> subgroups = jointModelGroup->getSubgroupNames();
  for(std::size_t i = 0; i < subgroups.size(); ++i )
    ROS_INFO("Subgroup: %s", subgroups[i].c_str());

  // Display current joint values
  const std::vector<std::string> &jointNames = jointModelGroup->getVariableNames();

  std::vector<double> jointValues;
  kinematicState->copyJointGroupPositions(jointModelGroup, jointValues);

  for (std::size_t i = 0; i < jointNames.size(); ++i)
    ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointValues[i]);


  // Define state for lArm
  Eigen::Affine3d lArmState;
  Eigen::Vector3d lArmTrans(0.81487, -0.4565, 1.1645);
  Eigen::Quaterniond lArmRot(-0.15709, 0.65954, 0.73273, -0.058614);
  lArmState.translation() = lArmTrans;
  lArmState.linear() = lArmRot.toRotationMatrix();

  ROS_INFO_STREAM("lArm Translation: " << lArmState.translation());
  ROS_INFO_STREAM("lArm Rotation: " << lArmState.rotation());

  // Define state for lArmPalm
  Eigen::Affine3d lArmPalmState;
  Eigen::Vector3d lArmPalmTrans(0.73176, -0.22601, 1.1702);
  Eigen::Quaterniond lArmPalmRot(0.39806, 0.70643, 0.33846, -0.47744);
  lArmPalmState.translation() = lArmPalmTrans;
  lArmPalmState.linear() = lArmPalmRot.toRotationMatrix();

  ROS_INFO_STREAM("lArmPalm Translation: " << lArmPalmState.translation());
  ROS_INFO_STREAM("lArmPalm Rotation: " << lArmPalmState.rotation());

  // Define state for firstFinger
  Eigen::Affine3d firstFingerState;
  Eigen::Vector3d firstFingerTrans(0.62524, -0.1586, 1.1605);
  Eigen::Quaterniond firstFingerRot(0.31394, -0.44123, 0.67504, 0.50108);
  firstFingerState.translation() = firstFingerTrans;
  firstFingerState.linear() = firstFingerRot.toRotationMatrix();

  ROS_INFO_STREAM("firstFinger Translation: " << firstFingerState.translation());
  ROS_INFO_STREAM("firstFinger Rotation: " << firstFingerState.rotation());

  // Define state for thumb
  Eigen::Affine3d thumbState;
  Eigen::Vector3d thumbTrans(0.67232, -0.16916, 1.2437);
  Eigen::Quaterniond thumbRot(0.20622, -0.42953, -0.70455, 0.52591);
  thumbState.translation() = thumbTrans;
  thumbState.linear() = thumbRot.toRotationMatrix();

  ROS_INFO_STREAM("thumb Translation: " << thumbState.translation());
  ROS_INFO_STREAM("thumb Rotation: " << thumbState.rotation());

  // Preprare for IK solving
  EigenSTL::vector_Affine3d poses;
  poses.push_back(lArmState);
  poses.push_back(lArmPalmState);
  poses.push_back(firstFingerState);
  poses.push_back(thumbState);

  std::vector<std::string> tips;
  tips.push_back("r_jr3_link");
  tips.push_back("palm");
  tips.push_back("ffdistal");
  tips.push_back("thdistal");

  std::vector<std::vector<double> > limits;
  std::vector<double> lArmLimits(7, 0.1);
  std::vector<double> lArmPalmLimits(9, 0.1);
  std::vector<double> firstFingerLimits(4, 0.1);
  std::vector<double> thumbLimits(5, 0.1);
  limits.push_back(lArmLimits);
  limits.push_back(lArmPalmLimits);
  limits.push_back(firstFingerLimits);
  limits.push_back(thumbLimits);

  //bool foundIK = kinematicState->setFromIK(jointModelGroup, firstFingerState, 10, 0.1);
  bool foundIK = kinematicState->setFromIK(jointModelGroup, poses, tips, 10, 0.1);
  //bool foundIK = kinematicState->setFromIKSubgroups(jointModelGroup, poses, tips, limits, 20, 0.5);

  if (foundIK) {
    ROS_INFO("Solution found!");
    kinematicState->copyJointGroupPositions(jointModelGroup, jointValues);
    for (std::size_t i = 0; i < jointNames.size(); ++i)
      ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointValues[i]);
  }
  else
    ROS_INFO("Did not find IK solution");

  ros::shutdown();
  return 0;
}