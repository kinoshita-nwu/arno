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
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
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
		if len(waypoints.poses) == 0 :
			set_initpose(pose)
		waypoints.poses.append(pose)
		if len(line) >= 3:
			change_params.append(line[2])
		else:
			change_params.append(None)
	waypoints.header.frame_id = "map"

def load_map(yamlFile,num):
   num1 = str(num)
   p = Popen(['rosrun','map_server','map_server',yamlFile+'_'+num1+'.yaml'])
   rospy.sleep(0.5)
   p.terminate()

def set_initpose(pose) :
   init = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
   init_pose = PoseWithCovarianceStamped()

   init_pose.header.stamp = rospy.Time.now()
   init_pose.header.frame_id = "map"
   init_pose.pose.pose.position.x = pose.position.x
   init_pose.pose.pose.position.y = pose.position.y
   init_pose.pose.pose.orientation.z = pose.orientation.z
   init_pose.pose.pose.orientation.w = pose.orientation.w 
     
   init.publish(init_pose)

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
   last_xy_tolerance = 0.1
   count = 0

   if len(sys.argv) < 2 :
	print('\007')
        print("Usage : rosrun arno multimap_navigation.py fileName")
        quit()

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')
   load_file(sys.argv[1]+'_waypoint.json')
   listener = tf.TransformListener()

   client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
   client.wait_for_server()
   listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
   dynamic_client = dynamic_reconfigure.client.Client("amcl", timeout=30)

   while not rospy.is_shutdown(): 
	path = sys.argv[1]+'_'+str(count)+'.yaml'
    	is_file = os.path.isfile(path)

	if not is_file :
		print("finish!!")
		quit()

	if len(waypoints.poses) == 0 :
		load_map(sys.argv[1],count)
		j = str(count)
		load_file(sys.argv[1]+'_'+j+'_waypoint.json')
		count = int(count) 
		count +=1   
    
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
			
			xy = math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2)
			if ((len(waypoints.poses) == 1) and ((xy <= last_xy_tolerance) or (client.get_result()))):
				print("next!!!!!!!!!!")
			        waypoints.poses.pop(0)
	                        break
	                
			if ((xy <= xy_tolerance) or (client.get_result())):
	                    print("next!!")
			    waypoints.poses.pop(0)
	                    break
