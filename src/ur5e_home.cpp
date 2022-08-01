#include <iostream>
#include <ros/ros.h>

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/tf.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv){
    
    ros::init(argc, argv, "ur5e_motion_node");
    ros::NodeHandle node_handle;
  
    ros::AsyncSpinner spinner(1);
    spinner.start();
      
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    std::cout << "===================================" << std::endl;
    std::cout << "Planning frame: " << move_group.getPlanningFrame() << std::endl;
    std::cout << "End effector link: " << move_group.getEndEffectorLink() << std::endl;
    std::cout << std::endl;
    
    tf::Quaternion q;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ADD TABLE OBJECT
    moveit_msgs::CollisionObject table;
    table.header.frame_id = move_group.getPlanningFrame();
    
    table.id = "table1";
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.8;
    primitive.dimensions[primitive.BOX_Y] = 0.8;
    primitive.dimensions[primitive.BOX_Z] = 0.02;
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.3677;
    table_pose.position.z = -0.01;
    q.setRPY(0.0, 0.0, 0.7854);
    q = q.normalize();
    table_pose.orientation.x = q.x();
    table_pose.orientation.y = q.y();
    table_pose.orientation.z = q.z();
    table_pose.orientation.w = q.w();
    
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    
    // ADD WALL OBJECT
    moveit_msgs::CollisionObject wall;
    wall.header.frame_id = move_group.getPlanningFrame();
    
    wall.id = "wall1";
    // Define a box to add to the world.
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.02;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 1.2;
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose wall_pose;
    wall_pose.position.x = 0.2758;
    wall_pose.position.y = -0.2758;
    wall_pose.position.z = 0.6;
    q.setRPY(0.0, 0.0, -0.7854);
    q = q.normalize();
    wall_pose.orientation.x = q.x();
    wall_pose.orientation.y = q.y();
    wall_pose.orientation.z = q.z();
    wall_pose.orientation.w = q.w();
    
    wall.primitives.push_back(primitive);
    wall.primitive_poses.push_back(wall_pose);
    wall.operation = wall.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(table);
    collision_objects.push_back(wall);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    std::cout << "Add an object into the world" << std::endl;
    //planning_scene_interface.addCollisionObjects(collision_objects);
    planning_scene_interface.applyCollisionObjects(collision_objects);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    move_group.setPlanningPipelineId("ompl");
    std::cout << "===================================" << std::endl;
    std::cout << "Default planner: " << move_group.getDefaultPlannerId() << std::endl;
    move_group.setPlannerId("RRTConnect");
    std::cout << "Planner: " << move_group.getPlannerId() << std::endl;
    
    move_group.setPoseReferenceFrame("base_link");
    std::cout << "Planning frame: " << move_group.getPlanningFrame() << std::endl;
    std::cout << "Goal position tolerance: " << move_group.getGoalPositionTolerance () << std::endl;
    std::cout << "Goal orientation tolerance: " << move_group.getGoalOrientationTolerance () << std::endl;
    std::cout << "===================================" << std::endl;
    
//    std::cout << "Robot start pose:" << std::endl;
//    std::cout << move_group.getCurrentPose().pose << std::endl;
    
    // create a target pose for moving the robot
    geometry_msgs::Pose home;
    q.setRPY(-2.9270193, 0.1965133, 1.5635691);
    q = q.normalize();
    
    home.position.x = 0.0238696;
    home.position.y = 0.213341;
    home.position.z = 0.392223;
//    home.orientation.x = q.x();
//    home.orientation.y = q.y();
//    home.orientation.z = q.z();
//    home.orientation.w = q.w();
    home.orientation.x = -0.69481;
    home.orientation.y = 0.704532;
    home.orientation.z = 0.00591454;
    home.orientation.w = 0.144353;
    
    // set the target pose
    move_group.setJointValueTarget(home, "wrist_3_link"); // if you use tool0 frame with gripper consig it would result in a different pose

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setStartStateToCurrentState();
    auto plan_result = move_group.plan(my_plan);
    
    bool success = (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "Planning result: " << success << std::endl;
    if(success)
        move_group.execute(my_plan);
    
    return 0;
}