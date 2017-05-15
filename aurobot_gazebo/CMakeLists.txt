cmake_minimum_required(VERSION 2.8.3)
project(aurobot_gazebo)
find_package(catkin REQUIRED)

catkin_package(DEPENDS gazebo SDF)

#############
## Install ##
#############
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY worlds/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)