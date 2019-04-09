cmake_minimum_required(VERSION 2.8.3)
project(aurobot_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  grasping_clouds
  pa10
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
  SceneObjects.msg
)

generate_messages(
  DEPENDENCIES sensor_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime grasping_clouds pa10 cmake_modules pcl_conversions pcl_ros roscpp tf sensor_msgs moveit_core moveit_ros_planning_interface moveit_msgs rviz_visual_tools eigen_stl_containers
  DEPENDS boost eigen
)

add_executable(realsense_to_world src/realsense_to_world.cpp)
add_executable(pointcloud_listener src/pointcloud_listener.cpp)

add_executable(allegro_plan_grasp src/allegro_plan_grasp.cpp)
add_executable(allegro_control_grasp src/allegro_control_grasp.cpp)

add_executable(move_left_pa10 src/move_left_pa10.cpp)
add_executable(move_left_odr_axs_pa10 src/move_left_odr_axs_pa10.cpp)

add_executable(move_left_allegro src/move_left_allegro.cpp)

add_executable(move_right_odr_vel_pa10 src/move_right_odr_vel_pa10.cpp)

add_executable(move_right_shadow src/move_right_shadow.cpp)

target_link_libraries(realsense_to_world ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_listener ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(allegro_plan_grasp ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(allegro_control_grasp ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(move_left_pa10 ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(move_left_odr_axs_pa10 ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(move_left_allegro ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(move_right_odr_vel_pa10 ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(move_right_shadow ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})