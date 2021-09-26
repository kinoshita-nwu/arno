#!/usr/bin/env python

import rospy
import tf
import os
import sys
import math
import json
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import Marker ,MarkerArray
from geometry_msgs.msg import PoseArray, Pose
from move_base_msgs.msg import MoveBaseActionGoal


q = 0
grid = 0
free = 0
occupancy = 0
count = 0
state = None
frame = None
map_state = None

rows = []
lists = []
map_xy = []
past_state = []
x_y = [0,0,0,0,0]

start = Pose()
goal = Pose()
passpoint = PoseArray()
waypoint = PoseArray()
label = MarkerArray()


def PassCallback(data):
   global passpoint
   global frame

   pub = rospy.Publisher('passpoint', PoseArray, queue_size = 10)
   pose = Pose()

   passpoint.header.frame_id = frame
   pose.position.x = data.goal.target_pose.pose.position.x
   pose.position.y = data.goal.target_pose.pose.position.y
   pose.orientation.z = data.goal.target_pose.pose.orientation.z
   pose.orientation.w = data.goal.target_pose.pose.orientation.w

   print('[[{0},{1},0.0],[0.0,0.0,{2},{3}]]' 
	.format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))

   passpoint.poses.append(pose)
   pub.publish(passpoint)

def MapCallback(msg):
   global map_xy
   global rows
   
   count = 0
   row = []

   data = msg.data
   width = msg.info.width
   height = msg.info.height
	
   map_xy.append(msg.info.origin.position.x)
   map_xy.append(msg.info.origin.position.y)
  
   for i in range(height):
	for i in range(width):
		row.append(data[count])
		count += 1
	rows.append(row)
	row = []

def GoalPose():
   global goal
   global passpoint

   pose = passpoint.poses[-1]
   goal.position.x = pose.position.x
   goal.position.y = pose.position.y
   goal.orientation.z = pose.orientation.z
   goal.orientation.w = pose.orientation.w

def StartPose():
   global count
   global start
   global waypoint

   pose = waypoint.poses[count]
   start.position.x = pose.position.x
   start.position.y = pose.position.y
   start.orientation.z = pose.orientation.z
   start.orientation.w = pose.orientation.w

def WaypointPose():
   global count
   global state
   global frame
   global start
   global waypoint
   global x_y

   pub = rospy.Publisher('waypoint', PoseArray, queue_size = 10)
   pose = Pose()
	
   waypoint.header.frame_id = frame
   pose.position.x = x_y[0]
   pose.position.y = x_y[1]
   pose.orientation.z = start.orientation.z
   pose.orientation.w = start.orientation.w
   
   print('\nwaypoint {0}\n   {1}' .format(count,state))

   waypoint.poses.append(pose)
   pub.publish(waypoint)

def ArrowLabel():
   global count
   global state
   global frame
   global start
   global goal
   global waypoint
   global label
   
   pub = rospy.Publisher('label', MarkerArray, queue_size = 10)
   marker_data = Marker()

   marker_data.header.frame_id = frame
   marker_data.header.stamp = rospy.Time.now()  
   
   marker_data.ns = 'basic_shapes'
   marker_data.action = Marker.ADD
   marker_data.lifetime = rospy.Duration()
   marker_data.type = Marker.TEXT_VIEW_FACING
   
   marker_data.color.r = 1.0
   marker_data.color.g = 0.2
   marker_data.color.b = 0.5
   marker_data.color.a = 1.0

   marker_data.scale.x = 1.0
   marker_data.scale.y = 1.0
   marker_data.scale.z = 1.0

   if state == 'goal' :
	pose = goal
	marker_data.text = 'Goal'
	marker_data.id = -1
   elif state == 'start' :
	pose = start
	marker_data.text = 'Start'
	marker_data.id = -2
   else :
	pose = waypoint.poses[count]
	marker_data.text = str(count)
	marker_data.id = count
	count +=1

   marker_data.pose.position.x = pose.position.x
   marker_data.pose.position.y = pose.position.y
   marker_data.pose.orientation.z = pose.orientation.z
   marker_data.pose.orientation.w = pose.orientation.w
   
   label.markers.append(marker_data)
   pub.publish(label)

def WriteFile():
   global count
   global state
   global start
   global waypoint
   global passpoint

   if state == 'start' :
	pose = start
   else :
	pose = waypoint.poses[count]
     
   x = pose.position.x
   y = pose.position.y
   z = pose.orientation.z
   w = pose.orientation.w
   
   file=open(sys.argv[1]+'_waypoint.json', 'a')
   file.write('\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],'.format(x,y,z,w))
   file.close()

def Init_XY():
   global start
   global goal
   global passpoint
   global x_y

   x_y[0] = start.position.x
   x_y[1] = start.position.y
   if len(passpoint.poses) == 0 :
	x_y[2] = goal.position.x
	x_y[3] = goal.position.y
   else :
	x_y[2] = passpoint.poses[0].position.x
	x_y[3] = passpoint.poses[0].position.y
   x_y[4] = math.sqrt(((x_y[2]-x_y[0])**2)+((x_y[3]-x_y[1])**2))

def Calculation_XY():
   global q
   global lists
   global x_y
   global map_state

   unknown = 0
   t_state = ['right','left','up','down','upper_right','upper_left','lower_right','lower_left']
   xy_list = []
   x_list = []
   y_list = []
   state_list = []

   q1 = 0.7 * q
  
   print('--------------------------------------------------------------')

   for i in range(8):
	x1 = x_y[0]
	y1 = x_y[1]
	x2 = x_y[2]
	y2 = x_y[3]
       
	if t_state[i] == 'right' :
		x1 += q
	elif t_state[i] == 'left' :
		x1 -= q
	elif t_state[i] == 'up' :
		y1 += q
	elif t_state[i] == 'down' :
		y1 -= q
	elif t_state[i] == 'upper_right' :
		x1 += q1
		y1 += q1
	elif t_state[i] == 'lower_right' :
		x1 += q1
		y1 -= q1
	elif t_state[i] == 'upper_left' :
		x1 -= q1
		y1 += q1
	elif t_state[i] == 'lower_left':
		x1 -= q1
		y1 -= q1
  
	print(' {0}' .format(t_state[i]))
	
	OccupancyMap(x1,y1)
	if map_state != 'free' :
		unknown += 1
		if unknown >= 7:
			print('--------------------------------------------------------------')
			print('Can not proceed!!')
			quit()
		continue
		
	xy = math.sqrt(((x_y[2]-x1)**2)+((x_y[3]-y1)**2))
	xy_list.append(xy)
	x_list.append(x1)
	y_list.append(y1)
	state_list.append(t_state[i])
	print('  (x,y,z)=({0},{1},0.0)  dis={2}' .format(x1,y1,xy))
	
   lists.append([state_list,x_list,y_list,xy_list])
   ChangeState()

def OccupancyMap(x,y):
   global map_xy
   global rows
   global frame
   global map_state
   global grid
   global free
   global occupancy

   value = [0,0,0]

   if frame == 'base_link':
	x = int(20*x)
	y = int(20*y)
   else :
	x = int(20*math.sqrt((x-map_xy[0])**2))
	y = int(20*math.sqrt((y-map_xy[1])**2))

   for i in range(grid):
	for j in range(grid):
		if rows[y+i][x+j] == 0:
			value[0] += 1
	     	elif rows[y+i][x+j] == 100:
			value[1] += 1
             	else :
			value[2] += 1
	     	if rows[y+i][x-j] == 0:
			value[0] += 1
	     	elif rows[y+i][x-j] == 100:
			value[1] += 1
	     	else :
			value[2] += 1
	     	if rows[y-i][x+j] == 0:
			value[0] += 1
	     	elif rows[y-i][x+j] == 100:
			value[1] += 1
	     	else :
			value[2] += 1
	     	if rows[y-i][x-j] == 0:
			value[0] += 1
	     	elif rows[y-i][x-j] == 100:
			value[1] += 1
	     	else :
			value[2] += 1 
        
   v_sum = float(sum(value))
   for i in range(3):
	value[i] = round((value[i]/v_sum)*100,2)

   if  value[1] >= occupancy:
	map_state = 'occupancy'
   elif value[0] >= free:
	map_state = 'free'
   else :
	map_state = 'unknown'

   print('  free={0}%  occupancy={1}%  unknown={2}%' .format(value[0],value[1],value[2]))
   print('  Grid(width,height)=({0},{1}) map_state:{2}' .format(x,y,map_state))

def ChangeState():
   global q
   global grid
   global free
   global occupancy
   global count
   global state
   global past_state
   global lists
   global x_y
   global passpoint
	
   past_state.append(state)

   while True :
		list = lists[count]
		x_y[4] = min(list[3]) 
		state = list[0][list[3].index(min(list[3]))] 
		x_y[0] = list[1][list[3].index(min(list[3]))]
		x_y[1] = list[2][list[3].index(min(list[3]))]
		check = 1
		C = ChangeAngle(check)
		if C == 1 :
			del list[0][list[3].index(min(list[3]))]
			del list[1][list[3].index(min(list[3]))]
			del list[2][list[3].index(min(list[3]))]
			del list[3][list[3].index(min(list[3]))]
		else :
			break
   Update()
    
   if x_y[4] <= q :
	print('   xy <= q : {0} <= {1}\n'.format(x_y[4],q))
	if len(passpoint.poses) == 1 :
		file=open(sys.argv[1]+'_waypoint.json', 'a')
		file.write('\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n]' 
				.format(goal.position.x,goal.position.y,goal.orientation.z,goal.orientation.w))
		file.close()
		print('--------------------------------------------------------------')
		for i in range(3) :
  			remove = raw_input('\nRemove? (y/n) ')
			if remove == 'y' :
				while True :
					removenum = raw_input('number: ')
					if removenum == '':
						break
					else :
						Number = int(removenum)
						RemovePoint(Number)
				break
			elif remove == 'n' :
				break
			else :
				if i == 2 :
					quit()
				print('Enter y or n!!\n')

		print('\n--------------------------------------------------------------')
		print('\nGoal\n  (x,y,z)=({0},{1},0.0)\n  (x,y,w,z)=(0.0,0.0,{2},{3})\n'
				.format(goal.position.x,goal.position.y,goal.orientation.z,goal.orientation.w))
		print('\nq = {0}  grid = {1}  free = {2}%  occupancy = {3}%' .format(q,grid,free,occupancy))
		print('Save '+sys.argv[1]+'_waypoint.json.....')
		quit()
	else : 
		passpoint.poses.pop(0)
		print('--------------------------------------------------------------')
		print('\nNext !!!!!!!!!!!\n')
   else :  
	print('   xy > q : {0} > {1}\n' .format(x_y[4],q))

def RemovePoint(RemoveNum):
    global state

    with open(sys.argv[1]+'_waypoint.json','r') as f:
	waypointfile = json.load(f)
	waypointfile.pop(RemoveNum)
    with open(sys.argv[1]+'_waypoint.json','w') as f:
	json.dump(waypointfile,f)  
    print(' Remove waypoint {0}' .format(RemoveNum))

def Update() :
   check = 0
   c = ChangeAngle(check) 
   WaypointPose() 
   WriteFile() 
   StartPose() 
   ArrowLabel()

def ChangeAngle(check):
   global state
   global start
   global past_state
   global x_y

   angle = 0
   p_angle = 0
   p_state = past_state[-1]

   if p_state == 'right' :
        p_angle = 0
   elif p_state == 'left' :
        p_angle = 1
   elif p_state == 'up' :
        p_angle = -0.5
   elif p_state == 'down' :
        p_angle = 0.5
   elif p_state == 'upper_right' :
        p_angle = -0.25 
   elif p_state == 'lower_right' :
        p_angle = 0.25
   elif p_state == 'upper_left' :
        p_angle = -0.75
   elif p_state == 'lower_left':
        p_angle = 0.75 

   yaw = p_angle * math.pi 
   
   if state == 'right' :
        angle = 0
   elif state == 'left' :
        angle = 1
   elif state == 'up' :
        angle = 0.5
   elif state == 'down' :
        angle = -0.5
   elif state == 'upper_right' :
        angle = 0.25 
   elif state == 'lower_right' :
        angle = -0.25
   elif state == 'upper_left' :
        angle = 0.75 
   elif state == 'lower_left':
        angle = -0.75 

   if check == 1 :
	yaw = round(abs(angle * math.pi + yaw),2)
	if yaw == 3.14 :
		print('Delete\n')
		return 1
	else :
		return 0
	
   quaternion = start.orientation
   euler = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
   yaw = euler[2] + (angle * math.pi) + yaw

   quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1],yaw)
   start.orientation.z = quaternion[2]
   start.orientation.w = quaternion[3]
	
   return 0

def SetUp():
   global q
   global grid
   global free
   global occupancy
   global state
   global frame
   
   if (len(sys.argv) < 2):
	print('\007')
        print('Usage ' + sys.argv[0] + ' fileName ')
        quit()

   for i in range(3):
	print('\007')
	robot = raw_input('With robot ? (y/n) ')
	if robot == 'y' :
		frame = 'map'
		break
	elif robot == 'n' :
		frame = 'base_link'
		break
	else :
		if i == 2 :
			quit()
		print('Enter y or n.')

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')
   is_file = os.path.isfile(sys.argv[1]+'_waypoint.json')
   if is_file :
	for i in range(3):
		answer = raw_input('Overwrite '+sys.argv[1]+'_waypoint.json? (y/n) ')
		if answer == 'y':
			break		
		elif answer == 'n':
			quit()
		else:
			if i== 2 :
				quit()
			print('Enter y or n.')

   print('wait.....')
   rospy.Subscriber('/map', OccupancyGrid,MapCallback)
   rospy.sleep(3.0)

   file=open(sys.argv[1]+'_waypoint.json', 'w')
   file.write('[')
   file.close()

   for i in range(3):
	print('\007')
   	param = raw_input('in or out or self ? ')
   	if param == 'in':
   		q = 1.7
        	grid = 5
        	free = 80
		occupancy = 20
		break
   	elif param == 'out':
        	q = 4
        	grid = 30
        	free = 80
        	occupancy = 5
		break
   	elif param == 'self':
		print('[ex]in\n  q = 1.7\n  grid = 5\n  free = 90\n  occupancy = 30')
        	print('[ex]out\n  q = 4\n  grid = 15\n  free = 90\n  occupancy = 20')
        	print('self')
		q = int(raw_input('  q= '))
        	grid = int(raw_input('  grid= '))
        	free = int(raw_input('  free= '))
        	occupancy = int(raw_input('  occupancy= '))
		break
   	else :
		if i == 2 :
			quit()
		print('Enter y or n.')

   print('Make Passpoints and Goal with [2DNavGoal]')
   rospy.sleep(2.0)
   print('Put [Enter] to start make waypoints')
   rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,PassCallback)
   rospy.sleep(0.1)

   end = raw_input()
   if not end:
        state = 'goal'
	GoalPose() 
 
   ArrowLabel() 
   
if __name__ == '__main__':
   rospy.init_node('auto_waypoint')
   
   SetUp()

   state = 'start'
   start.position.x = passpoint.poses[0].position.x
   start.position.y = passpoint.poses[0].position.y
   start.orientation.z = passpoint.poses[0].orientation.z
   start.orientation.w = passpoint.poses[0].orientation.w
   passpoint.poses.pop(0)

   ArrowLabel() 
   WriteFile() 
   
   print('\nStart\n  (x,y,z)=({0},{1},0.0)\n  (x,y,w,z)=(0.0,0.0,{2},{3})\n'
         .format(start.position.x,start.position.y,start.orientation.z,start.orientation.w))
   
   while not rospy.is_shutdown() :
	Init_XY()
	Calculation_XY()
	rospy.sleep(1.0)
