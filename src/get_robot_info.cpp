#include <iostream>
#include <ros/ros.h>

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/tf.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv){
    
    ros::init(argc, argv, "ur5e_motion_node");
    ros::NodeHandle node_handle;
  
    ros::AsyncSpinner spinner(1);
    spinner.start();
      
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    std::cout << "===================================" << std::endl;
    std::cout << "Planning frame: " << move_group.getPlanningFrame() << std::endl;
    std::cout << "End effector link: " << move_group.getEndEffectorLink() << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "===================================" << std::endl;
    std::cout << "Default planner: " << move_group.getDefaultPlannerId() << std::endl;
    std::cout << "Planner: " << move_group.getPlannerId() << std::endl;
    std::cout << "Planning frame: " << move_group.getPlanningFrame() << std::endl;
    std::cout << "Goal position tolerance: " << move_group.getGoalPositionTolerance () << std::endl;
    std::cout << "Goal orientation tolerance: " << move_group.getGoalOrientationTolerance () << std::endl;
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "===================================" << std::endl;
    std::cout << "Robot current pose:" << std::endl;
    std::cout << move_group.getCurrentPose().pose << std::endl;
    
    return 0;
}