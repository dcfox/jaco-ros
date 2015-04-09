#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_driver'); roslib.load_manifest('jaco_msgs')
import rospy

import actionlib

import jaco_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/jaco_arm_driver/arm_pose/arm_pose', jaco_msgs.msg.ArmPoseAction)

    goal = jaco_msgs.msg.ArmPoseGoal()

    goal.pose.header.frame_id = "/jaco_api_origin"
    pose = goal.pose.pose

    if len(sys.argv) < 3: # default pose
        pose.position.x = -0.314269423485
        pose.position.y = -0.339179039001
        pose.position.z = 0.600132465363

        pose.orientation.x = -0.590686044496
        pose.orientation.y = -0.519369415388
        pose.orientation.z = 0.324703360925
        pose.orientation.w = 0.525274342226

        rospy.logwarn("Using test goal: \n%s", goal)
    else: # use pose from command line
        pose.position.x = float(sys.argv[1])
        pose.position.y = float(sys.argv[2])
        pose.position.z = float(sys.argv[3])

		#TODO: These will have to be set as known values
        pose.orientation.x = float(0.978398782865)
        pose.orientation.y = float(0.203091216798)
        pose.orientation.z = float(-0.0162236010533)
        pose.orientation.w = float(0.0350224801514)

    client.wait_for_server()
    rospy.loginfo("Connected to Pose server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()
    
def pose_client2():
    client = actionlib.SimpleActionClient('/jaco_arm_driver/arm_pose/arm_pose', jaco_msgs.msg.ArmPoseAction)

    goal = jaco_msgs.msg.ArmPoseGoal()

    goal.pose.header.frame_id = "/jaco_api_origin"
    pose = goal.pose.pose

    if len(sys.argv) < 3: # default pose
        pose.position.x = -0.314269423485
        pose.position.y = -0.339179039001
        pose.position.z = 0.600132465363

        pose.orientation.x = -0.590686044496
        pose.orientation.y = -0.519369415388
        pose.orientation.z = 0.324703360925 - 0.1
        pose.orientation.w = 0.525274342226

        rospy.logwarn("Using test goal: \n%s", goal)
    else: # use pose from command line
        pose.position.x = float(sys.argv[1])
        pose.position.y = float(sys.argv[2])
        pose.position.z = float(sys.argv[3]) - 0.1

		#TODO: These will have to be set as known values
        pose.orientation.x = float(0.978398782865)
        pose.orientation.y = float(0.203091216798)
        pose.orientation.z = float(-0.0162236010533)
        pose.orientation.w = float(0.0350224801514)

    client.wait_for_server()
    rospy.loginfo("Connected to Pose server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()
    
def finger_pose_client():
    client = actionlib.SimpleActionClient('/jaco_arm_driver/fingers/finger_joint_angles', jaco_msgs.msg.SetFingersPositionAction)

    goal = jaco_msgs.msg.SetFingersPositionGoal()

    if len(sys.argv) < 3:
        goal.fingers.finger1 = 4000
        goal.fingers.finger2 = 4000
        goal.fingers.finger3 = 4000

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
		#Note: This was changed
        goal.fingers.finger1 = float(55.0)
        goal.fingers.finger2 = float(55.0)
        goal.fingers.finger3 = float(55.0)

    client.wait_for_server()
    rospy.loginfo("Connected to Finger server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
	print "Hello"
	try:
		print "I am trying"
		rospy.init_node('arm_pose_client')
		#TODO: Not sure if this line will work
		result = pose_client2()
		print "Poses, how do they work?"
		rospy.loginfo("Result: %s", result)
	except rospy.ROSInterruptException: 
		rospy.loginfo("Program interrupted before completion Move A")
        
	try:
		rospy.init_node('arm_pose_client')
		#TODO: Not sure if this line will work
		result = pose_client()
		rospy.loginfo("Result: %s", result)
	except rospy.ROSInterruptException: 
		rospy.loginfo("Program interrupted before completion Move B")
		
	try:
		rospy.init_node('finger_pose_client')
		result = finger_pose_client()
		rospy.loginfo("Result: \n%s", result)
	except rospy.ROSInterruptException: 
		rospy.loginfo("Program interrupted before completion Hand A")
        

