// Standard
#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <moveit/move_group_interface/move_group_interface.h>

// Custom
#include <pa10/pa10_wrapper.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

const std::string green_arm_joints_names[7] = {"pa10_green_shoulder_roll_joint", "pa10_green_shoulder_lift_joint", "pa10_green_upper_arm_roll_joint",
  "pa10_green_elbow_flex_joint", "pa10_green_forearm_roll_joint", "pa10_green_wrist_flex_joint",
  "pa10_green_wrist_roll_joint"};

Eigen::VectorXf pa10_last_angulars(7);
Eigen::VectorXf LAST_STATE(7);
Eigen::VectorXf zero_velocity(7); 
PA10Wrapper pa10(62005);
bool paused = false;
bool armed = false;
bool controlling = false;
bool finish=false;
bool sameState = true;
Eigen::VectorXf state(7);

int odrDelay = 0;
int rosRate = 10;

int seq = 0;
//
ros::Publisher fakeControllerPub;
moveit::planning_interface::MoveGroupInterface* fingersMoveGroup;

void updateCurrentStatePA10();

void updatePA10LastAngulars(){
  pa10.getAngles(pa10_last_angulars);
}

void armPA10(){
  pa10.arm();
  armed = true;
}

void disarmPA10(){
  pa10.disarm();
  armed = false; 
}

void pausePA10(){
  pa10.susArm();
  controlling=false;
  paused = true;
}

void resumePA10(){
  pa10.rsmArm();
  paused = false;
}

void wait(int milliseconds)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds{milliseconds});
}

void thread(){
  while(!finish){
    updatePA10LastAngulars();
      //TEST
    if(sameState){
      std::cout << "SAME STATE\n";
      if(controlling){
        pa10.stpArm();
        controlling = false;
        updateCurrentStatePA10();
      }
    }else{
      if(!controlling){
        pa10.rsmArm();

        pa10.odrAxs(pa10_last_angulars);
        pa10.modAxs();
        controlling = true;
      }
        std::cout << "WE KEEP SAME POSITION\n";
        //Eigen::VectorXf artVel((state-pa10_last_angulars)/rosRate);


        std::cout << "Moving to...\n" << state << "\n";
        pa10.odrAxs(state);

        std::cout << "Moved!\n";
    }
    //std::cout << "READING ANGLES" << std::endl;
    wait(50);
  }
}

void updateCurrentStatePA10(){
  //updatePA10LastAngulars();
  if(!armed)
    std::cout << "PA10 needs to be armed" << std::endl;

  sensor_msgs::JointState js;
  js.header.seq = seq++;
  js.header.stamp = ros::Time::now();

  for(size_t arm_joints=0; arm_joints<7; ++arm_joints){
    js.position.push_back(pa10_last_angulars[arm_joints]);
    js.name.push_back(green_arm_joints_names[arm_joints]);
  }

  fakeControllerPub.publish(js);
}

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  sameState = true;

  //Find green arm joints
  int i=0;
  size_t arm_joints=0;
  //updatePA10LastAngulars();
  while(i<inputJointsMsg->name.size() && arm_joints < 7){
    if (inputJointsMsg->name[i].find("green") != std::string::npos){
      std::cout << "Joint " << i << " - " << inputJointsMsg->name[i] << ":\n";
      std::cout << "-> Position: " << inputJointsMsg->position[i] << 
        " REAL: " << pa10_last_angulars[arm_joints] << 
        " DIFF: " << fabs((float) inputJointsMsg->position[i]-(float)pa10_last_angulars[arm_joints]) << "\n";

      //state.push_back((float) inputJointsMsg->position[i]);
      state[arm_joints] = (float) inputJointsMsg->position[i];

      //if ((float) inputJointsMsg->position[i] != LAST_STATE[arm_joints])
      if (fabs((float) inputJointsMsg->position[i]-pa10_last_angulars[arm_joints]) > 0.01)
        sameState = false;

      ++arm_joints;
    }
    ++i;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aurobot_left_odr_axs_pa10_mover");

  ros::NodeHandle n;
  //ros::Rate r(rosRate); // This rate works fine with 0.1 velocity in the PA10 and 0.50 in MoveIt (vel/acc)
  zero_velocity << 0,0,0,0,0,0,0;
  fakeControllerPub = n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

  //fingersMoveGroup= new  moveit::planning_interface::MoveGroupInterface("l_arm");

  //Arm pa10 to get angulars positions and receive one /joint_states message to get initial information
  boost::thread *update_angulars_pa10;
  if(ros::ok()){
    armPA10();
    updatePA10LastAngulars();
    updateCurrentStatePA10();
    update_angulars_pa10 = new boost::thread{thread};
    
    pa10.stpArm();

  }

  message_filters::Subscriber<sensor_msgs::JointState> sub(n, "/joint_states", 10);
  sub.registerCallback(jointsCallback);
  ros::spin();

  finish = true;
  update_angulars_pa10->join();
  disarmPA10();

  return 0;
}
