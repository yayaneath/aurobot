cmake_minimum_required(VERSION 2.8.3)
project(aurobot_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grasping_clouds
  cmake_modules
  pcl_conversions
  pcl_ros
  roscpp
  tf
  sensor_msgs
  moveit_msgs
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)#chrono)
find_package(Eigen REQUIRED)

include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS grasping_clouds cmake_modules pcl_conversions pcl_ros roscpp tf sensor_msgs
  moveit_core moveit_ros_planning_interface moveit_msgs
  DEPENDS boost eigen
)

add_executable(realsense_to_world src/realsense_to_world.cpp)
add_executable(grasp_objects src/grasp_objects.cpp)
add_executable(aurobot_plan src/aurobot_plan.cpp)

target_link_libraries(realsense_to_world ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(grasp_objects ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(aurobot_plan ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
