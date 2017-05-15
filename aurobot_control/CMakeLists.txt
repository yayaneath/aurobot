cmake_minimum_required(VERSION 2.8.3)
project(aurobot_control)
find_package(catkin REQUIRED)

catkin_package(
  DEPENDS 
    gazebo 
    SDF
)

install(
  DIRECTORY
    config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
