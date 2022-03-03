#include "iiwa_ros/iiwa_ros.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";

bool sim;

void pan1(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main (int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "UpdownMoveit");
    ros::NodeHandle nh("~");
    nh.param("sim", sim, true);

    sub = nh.subscribe("/cmm/darknet_ros", 10, pan1);

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string movegroup_name, ee_link;
    geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position, start, end;
    std::string joint_position_topic, cartesian_position_topic;
    std::vector<geometry_msgs::Pose> linear_path;

    // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
    nh.param<std::string>("ee_link", ee_link, EE_LINK);    

    // Create Move Group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Configure Move Group
    move_group.setPlanningTime(0.5);
    move_group.setPlannerId(PLANNING_GROUP+"[RRTConnectkConfigDefault]");
    move_group.setEndEffectorLink(ee_link);
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Dynamic parameter to choose the rate at wich this node should run
    double ros_rate;
    nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
    ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

    int direction = 1;
    MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;
    
    double positions[3] = {0.1, -0.3, 0.5};
    
    tf2::Quaternion q;
    double rotz, roty, rotx;
    rotx = 0;
    roty = 0;
    rotz = 0;

    // while (ros::ok()){

    //     command_cartesian_position = move_group.getCurrentPose(ee_link);   
    //     // command_cartesian_position.pose.position.z -= direction * 0.40;
    //     cerr<<command_cartesian_position<<endl;
    //     // command_cartesian_position.pose.position.y = positions[i]; 
    //     // command_cartesian_position.pose.position.z += positions[i]; 
    //     // direction *= -1;
    //     // command_cartesian_position.pose.position.x += direction * 0.3; 
    //     // command_cartesian_position.pose.position.y += direction * -0.5; 
    //     // command_cartesian_position.pose.position.z += direction * 0.1; 


    //     // rotx += 0.3;
    //     // roty += 0.3;
    //     // rotz += 0.3;
    //     q.setEulerZYX(rotz,roty,rotx);


    //     command_cartesian_position.pose.orientation.x = q.x();
    //     command_cartesian_position.pose.orientation.y = q.y();
    //     command_cartesian_position.pose.orientation.z = q.z();
    //     command_cartesian_position.pose.orientation.w = q.w();

    //     move_group.setStartStateToCurrentState();
    //     move_group.setPoseTarget(command_cartesian_position, ee_link);
    //     success_plan = move_group.plan(my_plan);
    //     if (success_plan == MoveItErrorCode::SUCCESS) {
    //         motion_done = move_group.execute(my_plan);
    //     }
    //     if (motion_done == MoveItErrorCode::SUCCESS) {
    //         direction *= -1; // In the next iteration the motion will be on the opposite direction
    //         // loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
    //     }
    // }

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}