cmake_minimum_required(VERSION 2.8.3)
project(aurobot_description)
find_package(catkin REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_package(DEPENDS xacro)

foreach(dir meshes gazebo robots urdf launch)
   install(DIRECTORY ${dir}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
