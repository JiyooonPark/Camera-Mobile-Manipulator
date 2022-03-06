# :movie_camera: Camera Mobile Manipulator Project :movie_camera:


### Project Start: 2022.02.02


## Quick Start

    roslaunch cmm_gazebo ridgeback_iiwa_gazebo.launch 
    roslaunch cmm_viz mobile_manipulation_interactive_demo.launch
    roslaunch darknet_ros darknet_ros.launch 

## Default World 
![run environment](/doc/img/env.png)

## Darknet 
![run environment](/doc/img/darknet.png)


## LINKS 

Visual Servoing:

    https://github.com/savnani5/Visual-Servoing/

Object Detection cpp:

    https://docs.openvino.ai/latest/omz_demos_object_detection_demo_cpp.html

RGBD Camera:

    https://www.stereolabs.com/docs/ros/depth-sensing/


Gazebo person 

    - walking robot: http://gazebosim.org/tutorials?tut=actor&cat=build_robot


## TODO
- add RGBD camera instead of rgb camera
- move custom darknet to cmm_darknet
- make ridgebcak follow person 
- 

## Launch
    roslaunch cmm_gazebo ridgeback_iiwa_gazebo.launch 
    roslaunch cmm_viz mobile_manipulation_interactive_demo.launch 
    roslaunch darknet_ros darknet_ros.launch 
    rosrun cmm_darknet bounding_box_subscriber.py 
