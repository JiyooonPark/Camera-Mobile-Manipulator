cmake_minimum_required(VERSION 3.5)
project(cmm_iiwa)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(init_pose src/init_pose.cpp)
add_dependencies(init_pose iiwa_msgs_generate_messages_cpp)
target_link_libraries(init_pose ${catkin_LIBRARIES})
