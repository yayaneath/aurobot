// Standard
#include <iostream>

// ROS
#include <ros/topic.h>
#include <geometry_msgs/Quaternion.h>

// CONST VARIABLES
const std::string HAND_CORRECTION_TOPIC = "/emgsensor/move";

//
//  MAIN FUNCTION
//

int main(int argc, char **argv) {
  ros::init(argc, argv, "testing_control_pub");
  ros::NodeHandle node_handle;
  ros::Publisher pub;
  
  pub = node_handle.advertise<geometry_msgs::Quaternion>(HAND_CORRECTION_TOPIC, 1);

  ros::Rate loop_rate(0.2);

  while (ros::ok()){
    geometry_msgs::Quaternion msg;

    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.0;
    msg.w = 0.0;

    ROS_INFO("Publishing new message");

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  } 

  return 0;
}