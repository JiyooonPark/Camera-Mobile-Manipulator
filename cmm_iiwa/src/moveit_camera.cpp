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

#define TXT_FILE "/input/test.txt"

using namespace std;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";  
static const std::string REFERENCE_FRAME = "arm_mount_link";

bool sim;

vector<string> split(string input, char delimiter){
    vector<string> ans;
    stringstream str(input);
    string temp;
    
    while(getline(str, temp, delimiter)){
        ans.push_back(temp);
    }
    return ans;
}

int main (int argc, char **argv) {

    // Initialize ROS
    ros::init(argc, argv, "CommandRobotMoveit");
    ros::NodeHandle nh("~");
    nh.param("sim", sim, true);

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // for Cartesian Impedance Control
    iiwa_ros::state::CartesianPose iiwa_pose_state;
    iiwa_ros::service::ControlModeService iiwa_control_mode;

    if(!sim){
        iiwa_pose_state.init("iiwa");
        iiwa_control_mode.init("iiwa");
    }

    std::string movegroup_name, ee_link, planner_id, reference_frame;
    geometry_msgs::PoseStamped command_cartesian_position, start, end, init_cartesian_position;
    std::string joint_position_topic, cartesian_position_topic;
    std::vector<geometry_msgs::Pose> drawing_stroke;
    geometry_msgs::Pose drawing_point;
    geometry_msgs::Pose path_point;

    // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
    nh.param<std::string>("ee_link", ee_link, EE_LINK);
    nh.param<std::string>("planner_id", planner_id, PLANNER_ID);
    nh.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);

    // Create Move Group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;

    const double jump_threshold = 0.0; // 0.0
    const double eef_step = 0.001; // 0.001
    const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const moveit::core::LinkModel* link_model =
      move_group.getCurrentState()->getLinkModel(ee_link);

    // Configure Move Group
    move_group.setPlanningTime(0.5);
    move_group.setPlannerId(planner_id);  
    move_group.setEndEffectorLink(ee_link);
    move_group.setPoseReferenceFrame(reference_frame);
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    // TXT file with list of coordinates
    ifstream txt(ros::package::getPath("cmm_iiwa")+TXT_FILE);
    // check if text file is well opened
    if(!txt.is_open()){
        cout << "FILE NOT FOUND" << endl;
        return 1;
    }

    string line;
    bool init = false;
    double x, y, z, rotz, roty, rotx;
    tf2::Quaternion q;
    MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;
    
    // initialization before start drawing
    while (ros::ok() && !init){      

        // move to init pose
        // set all the joint values to the init joint position
        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget("iiwa_joint_1", 0.0);
        move_group.setJointValueTarget("iiwa_joint_2", 0.435332);
        move_group.setJointValueTarget("iiwa_joint_3", 0.0);
        move_group.setJointValueTarget("iiwa_joint_4", -1.91986);
        move_group.setJointValueTarget("iiwa_joint_5", 0.0);
        move_group.setJointValueTarget("iiwa_joint_6", -0.785399);
        move_group.setJointValueTarget("iiwa_joint_7", -1.57);
        success_plan = move_group.plan(my_plan);

        if (success_plan == MoveItErrorCode::SUCCESS) {
            motion_done = move_group.execute(my_plan);
        }

        ROS_INFO("Moved to the initial position");
        ros::Duration(1).sleep(); // wait for 3 sec
        init = true;
    }
    
    int stroke_num = 0;

    while(ros::ok() && getline(txt, line) && init){

        if(line == "End"){
            stroke_num++;

            ROS_INFO("Drawing %d th stroke ...", stroke_num);
            move_group.computeCartesianPath(drawing_stroke, eef_step, jump_threshold, trajectory);
            my_plan.trajectory_ = trajectory;
            
            motion_done = move_group.execute(my_plan); //ros::Duration(0.1).sleep();

            if (sim == true){   // Rviz drawing visualization
                if (motion_done == MoveItErrorCode::SUCCESS){
                    ROS_INFO("Successfully executed!");
                }
                else {
                    ROS_WARN_STREAM("LINE EXECUTION ERROR");
                }
            }
            drawing_stroke.clear();
        }
        else{
            // read drawing
            vector<string> tempSplit = split(line, ' ');

            x = stod(tempSplit[0]);
            y = stod(tempSplit[1]);
            z = stod(tempSplit[2]);

            rotx = stod(tempSplit[3]); 
            roty = stod(tempSplit[4]); 
            rotz = stod(tempSplit[5]); 

            q.setEuler(rotx,roty,rotz);

            drawing_point.position.x = x;
            drawing_point.position.y = y;
            drawing_point.position.z = z;

            drawing_point.orientation.x = q.x();
            drawing_point.orientation.y = q.y();
            drawing_point.orientation.z = q.z();
            drawing_point.orientation.w = q.w();

            cerr<<drawing_point<<endl;

            drawing_stroke.push_back(drawing_point); // push the point
        }
    }

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}