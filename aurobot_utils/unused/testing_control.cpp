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
  ros::init(argc, argv, "testing_control");
  ros::NodeHandle node_handle;
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  
  std::cout << "Empezando...\n";

  while (true) { // HAND POSE CORRECTION
    std::cout << "Starting...\n";

    char REPEAT_PLAN = 'r';
    char inputChar = REPEAT_PLAN;

    do {
      ROS_INFO("[AUROBOT] PLANNING CORRECTION");
      
      std::cin >> inputChar;
    } while(inputChar == REPEAT_PLAN);

    std::cout << "Done!\n";

    geometry_msgs::QuaternionConstPtr correctionMsg =
      ros::topic::waitForMessage<geometry_msgs::Quaternion>(HAND_CORRECTION_TOPIC, node_handle);

    float correctionStep = 0.02;
    float xCorrection = correctionMsg->x, yCorrection = correctionMsg->y, 
      zCorrection = correctionMsg->z, closeHand = correctionMsg->w;

    if (closeHand == 1.0)
      break;

    // Correct grasp pose
    std::cout << "Moving!\n";
  }

  std::cout << "Cerrando...\n";

  return 0;
}