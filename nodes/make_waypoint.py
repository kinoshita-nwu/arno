#!/usr/bin/env python

import rospy
import tf
import os
import sys
import math
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

Dif1 = 0
Dif2 = 0
counter = 0
write = None
file_name = None

pos = [0,0,0]
qu = [0,0,0,0]

make_waypoints_out = PoseArray()
make_waypoints_in = PoseArray()
make_numbers = MarkerArray()


def PrintArrow() :
   global Dif1
   global counter
   global pos
   global qu
    
   if Dif1 >= 10.0 :
    	pub = rospy.Publisher('make_waypoints_out', PoseArray, queue_size = 10)
    	pose = Pose()
    	pose.position.x = pos[0]
    	pose.position.y = pos[1]
    	pose.orientation.z = qu[2]
    	pose.orientation.w = qu[3]

    	make_waypoints_out.header.frame_id = 'map'
    	make_waypoints_out.poses.append(pose)
   	pub.publish(make_waypoints_out)

   else :
	pub = rospy.Publisher('make_waypoints_in', PoseArray, queue_size = 10)
    	pose = Pose()
    	pose.position.x = pos[0]
    	pose.position.y = pos[1]
    	pose.orientation.z = qu[2]
    	pose.orientation.w = qu[3]

    	make_waypoints_in.header.frame_id = 'map'
    	make_waypoints_in.poses.append(pose)
   	pub.publish(make_waypoints_in)

   num = rospy.Publisher('make_numbers', MarkerArray, queue_size = 10)
   marker_data = Marker()
   marker_data.header.frame_id = 'map'
   marker_data.header.stamp = rospy.Time.now()

   marker_data.ns = 'basic_shapes'
   marker_data.id = counter

   marker_data.action = Marker.ADD
    
   marker_data.pose.position.x = pos[0]
   marker_data.pose.position.y = pos[1]
   marker_data.pose.orientation.z = qu[2]
   marker_data.pose.orientation.w = qu[3]
   marker_data.text = str(counter)

   marker_data.color.a = 1.0
   marker_data.color.r = 1.0
   marker_data.color.g = 0.1
   marker_data.color.b = 0.5

   if Dif1 <= 9.0 :
    	marker_data.scale.z = 0.5
   else :
	marker_data.scale.z = 2.0

   marker_data.lifetime = rospy.Duration()
   marker_data.type = Marker.TEXT_VIEW_FACING

   make_numbers.markers.append(marker_data)
   num.publish(make_numbers)
   counter +=1

def WriteFile() :
   global Dif1
   global counter
   global write
   global file_name
   global pos
   global qu
   global make_waypoints_out
   global make_waypoints_in
   
   if write == 'start' :
   	file=open(sys.argv[1]+file_name, 'w')
	file.write('[')
   else :
	file=open(sys.argv[1]+file_name, 'a')
	if write == 'goal' :
		file.write('\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n]' .format(pos[0],pos[1],qu[2],qu[3]))
   	else :
                if Dif1 >= 10.0 :
   		        waypoint = make_waypoints_out
                else :
                        waypoint = make_waypoints_in
                for i in range(counter) :
                        x = waypoint.poses[i].position.x
   		        y = waypoint.poses[i].position.y
   		        z = waypoint.poses[i].orientation.z
   		        w = waypoint.poses[i].orientation.w

   		        file.write('\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],'.format(x,y,z,w))
   
   file.close()

def AngleDif(target, current):
   diff = target - current
   if (diff > math.pi):
        diff -= 2 * math.pi
   elif(diff < -math.pi):
	diff += 2 * math.pi
   return abs(diff)

def Question():
   global Dif1
   global Dif2
   global file_name

   way_set = [[1.0,0.4],[15.0,1.2]]
   pass_set = [[30.0,1.2],[60.0,1.2]]

   if (len(sys.argv) < 2):
        print('\007')
        print('Usage ' + sys.argv[0] + ' fileName ')
        quit()

   while True:
	print('\007')
        file_name = raw_input('waypoint or passpoint ? (way/pass) ')
	if file_name == 'way' :
		file_name = '_waypoint.json'
		break
	elif file_name == 'pass' :
		file_name = '_passpoint.json'
		break
	else :
		print('Enter way or pass.')

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')
   is_file = os.path.isfile(sys.argv[1]+file_name)

   if is_file:
	while True:
		answer = raw_input('Overwrite '+sys.argv[1]+file_name+'? (y/n) ')
		if answer == 'y' :
			break
		elif answer == 'n' :
			quit()
		else :
			print('Enter y or n.')

   while True:
	print('\007')
    	answer = raw_input('in or out or self? ')
    	if answer == 'in' :
		if file_name == '_waypoint.json':
			Dif1 = way_set[0][0]
			Dif2 = way_set[0][1]
		else :
			Dif1 = pass_set[0][0]
			Dif2 = pass_set[0][1]
	   	break
    	elif answer == 'out' :
		if file_name == '_waypoint.json':
			Dif1 = way_set[1][0]
			Dif2 = way_set[1][1]
		else :
			Dif1 = pass_set[1][0]
			Dif2 = pass_set[1][1]
	   	break
    	elif answer == 'self':
		if file_name == '_waypoint.json':
			dif_set =  way_set
		else :
			dif_set = pass_set
	   	print('[ex]in   Dif1 = {0}  Dif2 = {1}\n[ex]out   Dif1 = {2}  Dif2 = {3}\n' 
		.format(dif_set[0][0],dif_set[0][1],dif_set[1][0],dif_set[1][1]))
	   	Dif1 = float(raw_input('Dif1 = '))
	   	Dif2 = float(raw_input('Dif2 = '))
	   	break
    	else :
	   	print('Enter y or n.')

   print('\n[Start make{0}] Dif1 = {1} , Dif2 = {2}' .format(file_name,Dif1,Dif2))

if __name__ == '__main__':
   rospy.init_node('arno_position')
   listener = tf.TransformListener()

   Question()

   now = rospy.Time.now()
   listener.waitForTransform('map', 'base_link', now, rospy.Duration(4.0))
   position, quaternion = listener.lookupTransform('map', 'base_link', now)
    
   pos[0] = position[0] 
   pos[1] = position[1]
   qu[0] = quaternion[0]
   qu[1] = quaternion[1]
   qu[2] = quaternion[2]
   qu[3] = quaternion[3]

   write = 'start'
   WriteFile()

   while not rospy.is_shutdown() :
	now = rospy.Time.now()
	listener.waitForTransform('map', 'base_link', now, rospy.Duration(4.0))
	position, quaternion = listener.lookupTransform('map', 'base_link', now)
        
	dif1 = (position[0] - pos[0])**2 + (position[1] - pos[1])**2
	euler1 = tf.transformations.euler_from_quaternion((quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
	euler2 = tf.transformations.euler_from_quaternion((qu[0],qu[1],qu[2],qu[3]))
	dif2 = AngleDif(euler1[2],euler2[2])
	
       	if dif1 >= Dif1 or dif2 >= Dif2 :
		PrintArrow()  
		print('{0}:[[{1},{2},0.0],[0.0,0.0,{3},{4}]]' .format(counter-1,pos[0],pos[1],qu[2],qu[3]))
	
		pos[0] = position[0]
		pos[1] = position[1]
		qu[2] = quaternion[2]
		qu[3] = quaternion[3]

   while True :
	print('\007')
    	remove = raw_input('Remove? (y/n)')
    	if remove == 'y' :
		while True :
			removenum = raw_input('number: ')
	     		if not removenum :
		 		break
	      		else :
                        	if Dif1 >= 10.0 :
					make_waypoints_out.poses.pop(int(removenum))
				else :
					make_waypoints_in.poses.pop(int(removenum))
				counter -= 1
				print(' Remove waypoint {0}' .format(removenum))
		break
    	elif remove == 'n' :
	  	break
	else :
	  	print('Enter y or n .')

   write = None
   WriteFile()

   listener.waitForTransform('map', 'base_link', now, rospy.Duration(4.0))
   position, quaternion = listener.lookupTransform('map', 'base_link', now)
	
   pos[0] = position[0]
   pos[1] = position[1]
   qu[2] = quaternion[2]
   qu[3] = quaternion[3]

   write = 'goal'
   WriteFile()
