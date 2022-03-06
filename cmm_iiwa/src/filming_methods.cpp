#include "iiwa_ros/iiwa_ros.hpp"
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/conversions.hpp>

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
double PI = 3.14;

int main (int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "UpdownMoveit");
    ros::NodeHandle nh("~");
    nh.param("sim", sim, true);

    // sub = nh.subscribe("/cmm/darknet_ros", 10, pan1);

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

    double pan1[3][6] = {{0.7, 0, 1.5, PI/5*3, 0, -PI/2},{0.65, 0, 1.3, PI/2, 0, -PI/2}, {0.8, 0, 0.8, PI/4, 0, -PI/2}};
    // {-0.15, 0, 1.6, PI/2, 0, -PI/2};
    double pan2[3][6] = {{0.45, 0.5, 1.5, PI/2, PI/3.5, -PI/2},{-0.15, 0, 1.6, PI/2, 0, -PI/2},{0.45, -0.5, 1.5, PI/2, -PI/3.5, -PI/2}};

    double motion[2][6] = {{0.6, 0, 1.3-0.5, PI/2, 0, -PI/2},{-0.6, 0, 1.3-0.5, -PI/2, 0, PI/2}};
    // {0, 0, 1.9-0.7, 0, 0, 0}, 
    // double tilt[][]={};
    int motion_len = 2;
    tf2::Quaternion q;
    double rotz, roty, rotx;
    rotx = PI/2- 0.3;
    roty = 0;
    rotz = 0;
    int i = 0;


    double x, y, z, fraction;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 0.0
    const double eef_step = 0.001; // 0.001

    // for Cartesian Impedance Control
    iiwa_ros::state::CartesianPose iiwa_pose_state;
    iiwa_ros::service::ControlModeService iiwa_control_mode;

    // Low stiffness only along Z.
    iiwa_msgs::CartesianQuantity cartesian_stiffness = iiwa_ros::conversions::CartesianQuantityFromFloat(1500,1500,350,300,300,300);
    iiwa_msgs::CartesianQuantity cartesian_damping = iiwa_ros::conversions::CartesianQuantityFromFloat(0.7);


    iiwa_pose_state.init("iiwa");
    iiwa_control_mode.init("iiwa");

    iiwa_control_mode.setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping);
    iiwa_control_mode.setPositionControlMode();

    while (ros::ok()){

        command_cartesian_position = move_group.getCurrentPose(ee_link);  
        // command_cartesian_position.pose.position.x =0.66; 
        // command_cartesian_position.pose.position.z =1.1;
        // cerr<<command_cartesian_position<<endl;
        // command_cartesian_position.pose.position.y = positions[i]; 
        // command_cartesian_position.pose.position.z += positions[i]; 
        // direction *= -1;
        // command_cartesian_position.pose.position.x += direction * 0.3; 
        // command_cartesian_position.pose.position.y += direction * -0.5; 
        // command_cartesian_position.pose.position.z += direction * 0.1; 

        command_cartesian_position.pose.position.x = motion[i][0]; 
        command_cartesian_position.pose.position.y = motion[i][1]; 
        command_cartesian_position.pose.position.z = motion[i][2]; 


        rotx = motion[i][3]; 
        roty = motion[i][4]; 
        rotz = motion[i][5]; 

        cerr<<rotx<<endl;

        // rotx += 0.6 * direction; 
        // roty = 0; 
        // rotz = -PI/2; 

        // if (rotx >= PI/2 + 0.5){
        //     direction *= -1;
        // }
        // if (rotx <= PI/2 - 0.5){
        //     direction *= -1;
        // }

        q.setEuler(rotx,roty,rotz);


        command_cartesian_position.pose.orientation.x = q.x();
        command_cartesian_position.pose.orientation.y = q.y();
        command_cartesian_position.pose.orientation.z = q.z();
        command_cartesian_position.pose.orientation.w = q.w();

        linear_path.push_back(command_cartesian_position.pose);

        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(command_cartesian_position, ee_link);

        linear_path.push_back(command_cartesian_position.pose);
        fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory); // loosen the eef_step as moving backward does not need precision
        my_plan.trajectory_ = trajectory;

        success_plan = move_group.plan(my_plan);
        i++;
        if (i == motion_len){
            i = 0;
        }
        if (success_plan == MoveItErrorCode::SUCCESS) {
            motion_done = move_group.execute(my_plan);
        }
        if (motion_done == MoveItErrorCode::SUCCESS) {
            direction *= -1; // In the next iteration the motion will be on the opposite direction
            // loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
        }
    
    }

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}