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
    
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    
/*    // setup visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    
    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);*/
    
    // create a target pose for moving the robot
    geometry_msgs::PoseStamped current_poseStamped = move_group_interface.getCurrentPose();
    geometry_msgs::Pose current_pose;
    current_pose = current_poseStamped.pose;
    
    // set the target pose
    //move_group_interface.setPoseTarget(pose1);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose1 = current_pose;
    pose1.position.z += 0.2;
    waypoints.push_back(pose1);
    geometry_msgs::Pose pose2 = pose1;
    pose2.position.x -= 0.2;
    waypoints.push_back(pose2);
    geometry_msgs::Pose pose3 = pose2;
    pose3.position.z -= 0.2;
    waypoints.push_back(pose3);
    geometry_msgs::Pose pose4 = pose3;
    pose4.position.x += 0.2;
    waypoints.push_back(pose4);
    std::cout << "waypoints size: " << waypoints.size() << std::endl;
    
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
/*    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();*/
    
    move_group_interface.execute(trajectory);
    
    
    
    return 0;
}