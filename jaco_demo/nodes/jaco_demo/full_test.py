#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO arm."""
"""Daniel Fox"""

import roslib; roslib.load_manifest('jaco_demo')
import rospy
from std_msgs.msg import String
from array import *

import sys
import numpy as np

import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import goal_generators

#Example of cartesian goal setting
def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + 'jaco' + '_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=('jaco' + '_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None


#Example of gripper goal setting
def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + 'jaco' + '_arm_driver/fingers/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    goal.fingers.finger3 = float(finger_positions[2])

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the gripper action timed-out')
        return None

#NOTE: Unfinished; TODO
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pose = data.pose
    return pose
    
   #NOTE: Unfinished; TODO 
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("object_position", geometry_msgs.msg.PoseStamped, callback)

    rospy.spin()
	
	
	
	
if __name__ == '__main__':
    try:
	rospy.init_node('jaco' + '_full_test')
        
        raw_z_offset = [0.0.0.0 0.1, 0.0 0.0 0.0 0.0]
        z_offset = [(raw_z_offset[:3], raw_z_offset[3:])]
       
       #READ GOAL POSITION HERE
       pose1 = listener() #ASSUMING FORMAT IS geometry_msg/pose
       #pose1 = pose2 + z_offset
       #NOTE: READ 2 POSITIONS: THE FIRST WITH A HIGHER Z VALUE
       poses = [ pose1 ]
        #Move above the goal and then to goal
        for pos, orient in poses:
            print('    position: {},  orientation: {}'.format(pos, orient))
            result = cartesian_pose_client(pos, orient)  
            
       	#Finger positions by angle (0 to ~60), realistically (0.25 to 56)
        print('Using the specified JACO finger positions:')
        raw_positions = [55 55 55]
		positions = [ raw_positions ]

		#Close the fingers
        for position in positions:
            print('    position: {}'.format(position))
            result = gripper_client(position)

		#Cartesian points chosen from file homePos.txt
        print('Using poses from file: home/dan/catkin_ws/src/jaco-ros/homePos.txt')
        #NOTE: THIS FILE LOCATION WILL CHANGE ON DIFFERENT COMPUTERS TODO
        poses = goal_generators.poses_from_file('/home/dan/catkin_ws/src/jaco-ros/homePos.txt')         
       
        
     except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        
