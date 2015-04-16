
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "ros/ros.h"


void callBack(const geometry_msgs::PoseStamped &obj_pose)
{
    moveit::planning_interface::MoveGroup group("arm");
    group.setStartStateToCurrentState();
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //not sure if this will work or if we will need to pass
    //it as an arguement, or make a .h file
    ros::NodeHandle node_handle;
    //optional (display)
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    
    geometry_msgs::Pose target_pose1, target_pose2;
    
    //set pose 2 equal to object location
    target_pose2.position.x= obj_pose.pose.position.x;
    target_pose2.position.y= obj_pose.pose.position.y;
    target_pose2.position.z= obj_pose.pose.position.z;
    target_pose2.orientation.x= obj_pose.pose.orientation.x;
    target_pose2.orientation.y= obj_pose.pose.orientation.y;
    target_pose2.orientation.z= obj_pose.pose.orientation.z;
    target_pose2.orientation.w= obj_pose.pose.orientation.w;
    
    //set pose 1 equal to object location +10cm in z
    target_pose1 = target_pose2;
    target_pose1.position.z += 0.2;
        
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
    
    //Note: this approx converts cartesian point to angulat control
    //May want to try ans replace this with
    //group.setJointvalueTarget(target_pose1);
    group.setApproximateJointValueTarget(target_pose1);
    //group.setPoseTarget(target_pose1);
    
    //set tolerances
    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(.1);
    

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    // Sleep to give Rviz time to visualize the plan.
    sleep(5.0);

    // Actually moves real arm
    group.move();
    
    
    // Second movement
    group.setApproximateJointValueTarget(target_pose1);
    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(.1);
    moveit::planning_interface::MoveGroup::Plan my_plan2;
    bool success2 = group.plan(my_plan2);
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");
    sleep(5.0);
    group.move();
    
    
    //close hand
    //please let this work
    system("rosrun jaco_demo close_hand.py");
    
    //move to eat position
    //TODO
    
    
    ros::shutdown(); 
}

int main (int argc, char **argv)
{
    sleep(12.5); //wait for rviz
    ROS_INFO("move_group_interface.cpp is starting");
    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle node_handle; 
    //Not sure if the following line is written correctly
    ros::Subscriber sub = node_handle.subscribe("/object_position", 10, callBack);
    ros::AsyncSpinner spinner(1);
    spinner.start();
}
