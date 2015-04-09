#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def move():
	
	print "PROGRAM BEGINNNING"

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move',
		anonymous=True)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("arm")

	display_trajectory_publisher = rospy.Publisher(
		                    '/move_group/display_planned_path',
		                    moveit_msgs.msg.DisplayTrajectory)
		                    
	print "============ Waiting for RVIZ..."
	rospy.sleep(10)

	print "============ Reference frame: %s" % group.get_planning_frame()

	print "============ Reference frame: %s" % group.get_end_effector_link()

	print "============ Robot Groups:"
	print robot.get_group_names()

	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"

	print "END PROGRAM"

	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 0.687
	pose_target.position.x = 0.047
	pose_target.position.y = -0.291
	pose_target.position.z = 0.841
	group.set_pose_target(pose_target)
	
	plan1 = group.plan()

	print "============ Waiting while RVIZ displays plan1..."
	rospy.sleep(5)
	
	print "============ Visualizing plan1"
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan1)
	display_trajectory_publisher.publish(display_trajectory);

	print "============ Waiting while plan1 is visualized (again)..."
	rospy.sleep(5)
	
	# Uncomment below line when working with a real robot
	group.go(wait=True)
	
	
	moveit_commander.roscpp_shutdown()
	
if __name__=='__main__':
  try:
	  move()
  except rospy.ROSINterruptException:
	  print "ERROR ERROR ERROR"
	  pass
