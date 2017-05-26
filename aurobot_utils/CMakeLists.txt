cmake_minimum_required(VERSION 2.8.3)
project(aurobot_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
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
  rviz_visual_tools
  eigen_stl_containers
)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)#chrono)
find_package(Eigen REQUIRED)

include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_message_files (DIRECTORY msg
  FILES
  GraspConfiguration.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime grasping_clouds cmake_modules pcl_conversions pcl_ros roscpp tf sensor_msgs
  moveit_core moveit_ros_planning_interface moveit_msgs rviz_visual_tools eigen_stl_containers
  DEPENDS boost eigen
)

add_executable(realsense_to_world src/realsense_to_world.cpp)
add_executable(pointcloud_listener src/pointcloud_listener.cpp)
add_executable(aurobot_plan src/aurobot_plan.cpp)
add_executable(aurobot_plan_grasp src/aurobot_plan_grasp.cpp)

target_link_libraries(realsense_to_world ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_listener ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(aurobot_plan ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(aurobot_plan_grasp ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
