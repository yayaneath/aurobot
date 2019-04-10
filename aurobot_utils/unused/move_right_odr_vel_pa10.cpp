// DANGEROUS CONTROLLER:
// SOMETIMES ON INIT THE ARM RAPIDLY MOVES TO AN UNKNOWN POSITION
// HIGH RISK OF COLLISIONS OR BREAKING STUFF

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

Eigen::VectorXf pa10_last_angulars(7);
Eigen::VectorXf LAST_STATE(7);
Eigen::VectorXf zero_velocity(7); 
PA10Wrapper pa10(62005, PA10Wrapper::EARM::RIGHT);// Standard
bool paused = false;
bool armed = false;
bool controlling = false;
bool finish=false;

Eigen::VectorXf error_total(7);//I
Eigen::VectorXf last_error(7);
int kp = 0;//P
int ki = 0;
int kd = 0;

int odrDelay = 0;
int rosRate = 15;//P
int velOdr;

ros::Time last_time;

//
ros::Publisher fakeControllerPub;

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
    //std::cout << "READING ANGLES" << std::endl;
    wait(50);
  }
}

void updateCurrentStatePA10(const sensor_msgs::JointStateConstPtr & inputJointsMsg){
  armPA10();
  //updatePA10LastAngulars();
  if(!armed)
    std::cout << "PA10 needs to be armed" << std::endl;

  std::cout << "ANGULARS PA10 RECEIVED" << std::endl;
  sensor_msgs::JointState js;
  //sensor_msgs::JointState;
  js.header.seq = inputJointsMsg->header.seq;
  js.header.stamp = ros::Time::now();
  js.header.frame_id=inputJointsMsg->header.frame_id;

  /*for(int i=0; i<inputJointsMsg->name.size(); ++i){
    js.name.push_back(inputJointsMsg->name[i]);
  }*/

  //Find blue joints index
  int blue_index = 0;
  while(blue_index < inputJointsMsg->name.size()){
    if (inputJointsMsg->name[blue_index].find("blue") != std::string::npos)
      break;
    //js.position.push_back(inputJointsMsg->position[blue_index]);
    ++blue_index;
  }


  std::cout << "Updating current position..." << std::endl;
  for(size_t arm_joints=0; arm_joints<7; ++arm_joints){
    js.position.push_back(pa10_last_angulars[arm_joints]);
    js.name.push_back(inputJointsMsg->name[blue_index]);
    std::cout << "Joint " << arm_joints << " - " << pa10_last_angulars[arm_joints] << ":\n";
    std::cout << "-> Position: " << pa10_last_angulars[arm_joints] << "\n";
    ++blue_index;
  }

  /*while (blue_index < inputJointsMsg->position.size()){
    js.position.push_back(inputJointsMsg->position[blue_index]);
    ++blue_index;
  }*/

  /*for(int i=0 ; i<7; ++i){
    LAST_STATE[i] = pa10_last_angulars[i];
  }*/

  fakeControllerPub.publish(js);
}

void jointsCallback(const sensor_msgs::JointStateConstPtr & inputJointsMsg) {
  std::cout << "########################\n";
  std::cout << "* Seq: " << inputJointsMsg->header.seq << "\n";
  std::cout << "* Stamp: " << inputJointsMsg->header.stamp << "\n";
  std::cout << "* Frame ID: " << inputJointsMsg->header.frame_id << "\n";

  ros::Time actual_time = inputJointsMsg->header.stamp;

  Eigen::VectorXf state(7);
  bool sameState = true;

  //Find blue arm joints
  int i=0;
  size_t arm_joints=0;
  //updatePA10LastAngulars();
  while(i<inputJointsMsg->name.size() && arm_joints < 7){
    if (inputJointsMsg->name[i].find("blue") != std::string::npos){
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

  //TEST
  if(sameState){
    std::cout << "SAME STATE\n";
    if(controlling){
      pa10.odrVel(zero_velocity);
      pa10.stpArm();
      controlling = false;
      updateCurrentStatePA10(inputJointsMsg);
    }
  }else{
    if(!controlling){
      pa10.rsmArm();
      //velOdr = rosRate;
      //pa10.odrVel(zero_velocity);
      pa10.modVel(VM_ONE, AXISALL);
      controlling = true;
      last_error << 0,0,0,0,0,0,0;
    }else{
      //updatePA10LastAngulars();
      std::cout << "WE KEEP SAME POSITION\n";
      //Eigen::VectorXf artVel((state-LAST_STATE)*5);
      //rosRate

      double dt = (actual_time - last_time).toSec();
      Eigen::VectorXf error_actual = state-pa10_last_angulars;
      error_total += error_actual * dt;

      Eigen::VectorXf proportional = (error_actual) * kp;
      Eigen::VectorXf integral = error_total * ki;//ks de psd
      Eigen::VectorXf derivative = (error_actual - last_error)/dt * kd;
      //Eigen::VectorXf artVel(proportional + integral + derivative);
      //Eigen::VectorXf artVel(zero_velocity);
      //Eigen::VectorXf artVel(proportional + integral + derivative);
      Eigen::VectorXf artVel(error_actual/rosRate);

      std::cout << "Difference time: " << dt << std::endl;
      //artVel=state-LAST_STATE;

      //artVel << LAST_STATE[0], LAST_STATE[1], LAST_STATE[2], LAST_STATE[3],
      //  LAST_STATE[4], LAST_STATE[5], LAST_STATE[6];

      LAST_STATE = state;

      std::cout << "Moving to...\n" << artVel << "\n";
      if(!pa10.odrVel(artVel) && armed){
        //++velOdr;
        armPA10();
        pa10.odrVel(zero_velocity);
        updateCurrentStatePA10(inputJointsMsg);
      }

      last_error = error_actual;
      std::cout << "Moved!\n";
    }
    last_time = actual_time;
  }

  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_right_odr_vel_pa10");

  ros::NodeHandle n;

  zero_velocity << 0,0,0,0,0,0,0;
  error_total << 0,0,0,0,0,0,0;
  last_error << 0,0,0,0,0,0,0;
  
  fakeControllerPub = n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

  //Arm pa10 to get angulars positions and receive one /joint_states message to get initial information
  boost::thread *update_angulars_pa10;

  if(ros::ok()){
    armPA10();

    update_angulars_pa10 = new boost::thread{thread};

    sensor_msgs::JointStateConstPtr jointStateMessage = 
      ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
      
    std::cout << "JOINT STATE MESSAGE RECEIVED" << std::endl;

    //updatePA10LastAngulars();
    updateCurrentStatePA10(jointStateMessage);
    
    //disarmPA10();
    pa10.stpArm();

    //r.sleep();
  }
  /*while(ros::ok()) {
    sensor_msgs::JointStateConstPtr receivedMessage = 
      ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    jointsCallback(receivedMessage);

    r.sleep();
  }*/

  //message_filters::Subscriber<sensor_msgs::JointState> sub(n, "/joint_states", rosRate);
  message_filters::Subscriber<sensor_msgs::JointState> sub(n, "/joint_states", 10);
  sub.registerCallback(jointsCallback);
  ros::spin();

  finish = true;
  update_angulars_pa10->join();
  disarmPA10();

  return 0;
}
