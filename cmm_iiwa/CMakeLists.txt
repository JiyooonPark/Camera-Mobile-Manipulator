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

add_executable(cmm_init_pose src/init_pose.cpp)
add_dependencies(cmm_init_pose iiwa_msgs_generate_messages_cpp)
target_link_libraries(cmm_init_pose ${catkin_LIBRARIES})

add_executable(moveit_camera src/moveit_camera.cpp)
add_dependencies(moveit_camera iiwa_msgs_generate_messages_cpp)
target_link_libraries(moveit_camera ${catkin_LIBRARIES})

add_executable(updown src/updown_moveit.cpp)
add_dependencies(updown iiwa_msgs_generate_messages_cpp)
target_link_libraries(updown ${catkin_LIBRARIES})

add_executable(end_effector src/end_effector.cpp)
add_dependencies(end_effector iiwa_msgs_generate_messages_cpp)
target_link_libraries(end_effector ${catkin_LIBRARIES})

add_executable(filming_methods src/filming_methods.cpp)
add_dependencies(filming_methods iiwa_msgs_generate_messages_cpp)
target_link_libraries(filming_methods ${catkin_LIBRARIES})
