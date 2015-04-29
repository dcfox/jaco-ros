
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>

#include "ros/ros.h"


void angular_pose_callback(sensor_msgs::JointState joints_desired) {
    moveit::planning_interface::MoveGroup group("arm");
    group.setStartStateToCurrentState();
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::NodeHandle node_handle;

    //optional (display)
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
        
    std::vector<double> joints_current = group.getCurrentJointValues();
    ROS_INFO("Initial joint values: [%f,%f,%f,%f,%f,%f]", joints_current[0], joints_current[1], joints_current[2], joints_current[3], joints_current[4], joints_current[5]);
 
    group.setJointValueTarget(joints_desired.position);
    
    //set tolerances
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(.1);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    
    if (success) {
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
        sleep(5.0);        // Sleep to give Rviz time to visualize the plan.
        group.move();   // Actually moves real arm
        ROS_INFO("Moving complete.");
        std::vector<double> joints_current = group.getCurrentJointValues();
        ROS_INFO("Final joint values: [%f,%f,%f,%f,%f,%f]", joints_current[0], joints_current[1], joints_current[2], joints_current[3], joints_current[4], joints_current[5]);
    } else {
        ROS_WARN("Planning FAILED AGAIN!");
    }
}


void cartesian_pose_callback(geometry_msgs::PoseStamped pose) {
    moveit::planning_interface::MoveGroup group("arm");
    group.setStartStateToCurrentState();
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //not sure if this will work or if we will need to pass
    //it as an arguement, or make a .h file
    ros::NodeHandle node_handle;
    //optional (display)
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink frame: %s", group.getEndEffectorLink().c_str());
    ROS_INFO("PoseReferenceFrame frame: %s", group.getPoseReferenceFrame().c_str());
    ROS_INFO("EndEffector frame: %s", group.getEndEffector().c_str());
        
    geometry_msgs::PoseStamped p = group.getCurrentPose("");
        
    ROS_INFO("Initial pose: [%f,%f,%f],[%f,%f,%f,%f]", p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
    std::vector<double> joints_current = group.getCurrentJointValues();
    ROS_INFO("Initial joint values: [%f,%f,%f,%f,%f,%f]", joints_current[0], joints_current[1], joints_current[2], joints_current[3], joints_current[4], joints_current[5]);

    
    
    geometry_msgs::Pose target_pose1;
    
    //set pose 1 equal to object location
    /*target_pose1.position.x= obj_pose.pose.position.x;
    target_pose1.position.y= obj_pose.pose.position.y;
    target_pose1.position.z= obj_pose.pose.position.z;
    target_pose1.orientation.x= obj_pose.pose.orientation.x;
    target_pose1.orientation.y= obj_pose.pose.orientation.y;
    target_pose1.orientation.z= obj_pose.pose.orientation.z;
    target_pose1.orientation.w= obj_pose.pose.orientation.w;*/
    
        
    // pose 1
    /* Use this pose for testing
    target_pose2.position.x= -0.50;
    target_pose2.position.y= -0.12;
    target_pose2.position.z= 0.49;
    target_pose2.orientation.x= 0.894835;
    target_pose2.orientation.y= -0.438469;
    target_pose2.orientation.z= -0.048611;
    target_pose2.orientation.w= 0.068190;
    */
    
    //Note: this approx converts cartesian point to angular control
    //May want to try ans replace this with
    //group.setJointvalueTarget(target_pose1);
    //group.setApproximateJointValueTarget(target_pose1);
    //group.setApproximateJointValueTarget(pose.pose);
    group.setPoseTarget(pose.pose);
    
    //set tolerances
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(.1);
    

    moveit::planning_interface::MoveGroup::Plan my_plan;
    
    ROS_INFO("1111");
    
    bool success = group.plan(my_plan);
    
    ROS_INFO("222");
    
    if (success) {
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
        // Sleep to give Rviz time to visualize the plan.
        sleep(5.0);

        // Actually moves real arm
        group.move();
        ROS_INFO("Moving complete");
        geometry_msgs::PoseStamped p = group.getCurrentPose("");
        
        ROS_INFO("Current pose: [%f,%f,%f],[%f,%f,%f,%f]", p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
    } else {
        ROS_WARN("Planning FAILED AGAIN!");
    }
    //close hand
    //please let this work
    //system("rosrun jaco_demo close_hand.py");
    
    //move to eat position
    //TODO
    
    /*This is an experiment born of sleep deprivation
     * Only uncomment this code is you are feeling lucky
     * */
     /*
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();
    
    //create object
    collision_object.id = "baller";
    shape_msgs::SolidPrimitive baller;
    baller.type = baller.SPHERE;
    baller.dimensions[0]=0.02; //random size guess
    
    //place object
    geometry_msgs::Pose baller_pose;
    baller_pose.position.x= obj_pose.pose.position.x;
    baller_pose.position.y= obj_pose.pose.position.y;
    baller_pose.position.z= obj_pose.pose.position.z;
    
    //actually place object
    collision_object.primitives.push_back(baller);
    collision_object.primitive_poses.push_back(baller_pose);
    collision_object.operation = collision_object.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(collision_object);  

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");  
    planning_scene_interface.addCollisionObjects(collision_objects);
  
    // Sleep so we have time to see the object in RViz
    sleep(2.0);
    
    //Note: this code is unfinished until I can see the moveit tutorial
    //for pick & place. moveit.ros.org picked the worst week to go down
    
    */
    
    
    
}

int main (int argc, char **argv)
{
    sleep(12.5); //wait for rviz
    ROS_INFO("move_group_interface.cpp is starting");
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle node_handle; 
    ros::Subscriber cartesian_sub = node_handle.subscribe<geometry_msgs::PoseStamped>("/moveit_desired_pose/cartesian", 10, cartesian_pose_callback);
    ros::Subscriber angular_sub = node_handle.subscribe<sensor_msgs::JointState>("/moveit_desired_pose/angular", 10, angular_pose_callback);
    ros::AsyncSpinner spinner(10);
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
}
