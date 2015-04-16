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

#NOTE: Unfinished;
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #NOTE: The Pose does not include a Quaternion, only a point
    pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    #NOTE: This can be changed in the future
    orient = [0.732062321466, 0.0569922916504, -0.0673723200983, 0.675498043422]
    
    print('    position: {},  orientation: {}'.format(pos, orient))
    result = cartesian_pose_client(pos, orient)  
           
    #Finger positions by angle (0 to ~60), realistically (0.25 to 56)
    print('Using the specified JACO finger positions:')
    raw_positions = [55, 55, 55]
    positions = [ raw_positions ]

    #Close the fingers
    for position in positions:
        print('    position: {}'.format(position))
        result = gripper_client(position)
        
    poses = goal_generators.poses_from_file('/home/sd/ws/src/jaco-ros/homePos.txt')   


if __name__ == '__main__':
    
    rospy.init_node('jaco' + '_full_test')
    rospy.Subscriber("/object_position", geometry_msgs.msg.PoseStamped, callback)
    rospy.spin()
        
