cmake_minimum_required(VERSION 2.8.3)
project(cmm_viz)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()


install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
