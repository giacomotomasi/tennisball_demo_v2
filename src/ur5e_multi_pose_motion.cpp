#include <iostream>
#include <ros/ros.h>

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv){
    
    ros::init(argc, argv, "ur5e_motion_node");
    ros::NodeHandle node_handle;
  
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    static const std::string PLANNING_GROUP = "manipulator"; // name for UR robots
    
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // this is useful for add/remove collision objects
    
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // create a target pose for moving the robot
    geometry_msgs::PoseStamped current_poseStamped = move_group_interface.getCurrentPose();
    geometry_msgs::Pose current_pose;
    current_pose = current_poseStamped.pose;
    
    // generate waypoints list
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose1 = current_pose;
    pose1.position.z += 0.1;
    waypoints.push_back(pose1);
    geometry_msgs::Pose pose2 = pose1;
    pose2.position.x -= 0.1;
    waypoints.push_back(pose2);
    geometry_msgs::Pose pose3 = pose2;
    pose3.position.z -= 0.1;
    waypoints.push_back(pose3);
    geometry_msgs::Pose pose4 = pose3;
    pose4.position.x += 0.1;
    waypoints.push_back(pose4);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    int count {0};
    for (std::vector<geometry_msgs::Pose>::iterator it = waypoints.begin(); it != waypoints.end(); it++){
        // set the target pose
        move_group_interface.setPoseTarget(*it);
        move_group_interface.plan(my_plan);
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            std::cout << "Moving to waypoint " << ++count << std::endl;
            move_group_interface.move();
            ros::Duration(0.5).sleep();
        }

    move_group_interface.setPoseTarget(current_pose);
    move_group_interface.plan(my_plan);
    move_group_interface.move();
    
    return 0;
}