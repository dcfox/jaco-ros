#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO arm."""
"""Daniel Fox"""

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys
import numpy as np

import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import goal_generators

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

if __name__ == '__main__':
    try:
        rospy.init_node('jaco' + '_close_hand')

	#Finger positions by angle (0 to ~60)
        print('Using the specified JACO finger positions:')
        raw_positions = [0, 0, 0]
	positions = [ raw_positions ]

	#Close the fingers
        for position in positions:
            print('    position: {}'.format(position))
            result = gripper_client(position)

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"




