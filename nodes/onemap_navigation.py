#!/usr/bin/env python

import rospy
import actionlib
import tf
import os
import sys
import math
import json
from subprocess import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
import dynamic_reconfigure.client

waypoints = PoseArray()
change_params = []

def goal_pose(pose):
   goal_pose = MoveBaseGoal()
   goal_pose.target_pose.header.frame_id = 'map'
   goal_pose.target_pose.pose = pose

   return goal_pose

def load_file(fileName):
   with open(fileName, "r") as f:
	waypointFile = json.load(f)
	for line in waypointFile:
		pose = Pose()
		pose.position.x = line[0][0]
		pose.position.y = line[0][1]
		pose.position.z = line[0][2]
		pose.orientation.x = line[1][0]
		pose.orientation.y = line[1][1]
		pose.orientation.z = line[1][2]
		pose.orientation.w = line[1][3]
		waypoints.poses.append(pose)
		if len(line) >= 3:
			change_params.append(line[2])
		else:
			change_params.append(None)
	waypoints.header.frame_id = "map"

def angle_dif(target, current):
   diff = target - current
   if (diff > math.pi):
	diff -= 2 * math.pi
   elif(diff < -math.pi):
	diff += 2 * math.pi
   return abs(diff)

if __name__ == '__main__':
   rospy.init_node('arno_navigation')
   pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
   yo_tolerance = rospy.get_param('move_base/DWAPlannerROS/yaw_goal_tolerance')
   xy_tolerance = rospy.get_param('move_base/DWAPlannerROS/xy_goal_tolerance')

   argvs = sys.argv
   argc = len(argvs)

   if argc < 2 :
	print('\007')
	print("Usage : rosrun arno onemap_navigation.py fileName_1 filename_2 ..... filename_N")
	quit()

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')
   load_file(argvs[1]+'_waypoint.json')
   listener = tf.TransformListener()

   client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
   client.wait_for_server()
   listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
   dynamic_client = dynamic_reconfigure.client.Client("amcl", timeout=30)

   for i in range(argc-1):
	if len(waypoints.poses) == 0 :
		load_file(argvs[i+1]+'_waypoint.json')
		rospy.sleep(2.0)   
    
	for i in range(len(waypoints.poses)):
		pose = waypoints.poses[0]
		pub.publish(waypoints)
		goal = goal_pose(pose)
		client.send_goal(goal)
		if not change_params[0] == None:
			dynamic_client.update_configuration(change_params[i])

		while not rospy.is_shutdown():
			now = rospy.Time.now()
			listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))

			position, quaternion = listener.lookupTransform("map", "base_link", now)
			euler = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
			goal_euler = tf.transformations.euler_from_quaternion((goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))
			print("dis =  "+str((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ))
			print("rad = "+str(angle_dif(goal_euler[2], euler[2])))
			client.wait_for_result(rospy.Duration(0.1))
			if (math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= xy_tolerance) or (client.get_result()):
				print("next!!")
				waypoints.poses.pop(0)
				break
