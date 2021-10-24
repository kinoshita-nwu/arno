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
direction = 0
count = 0
p_count = 0
state = None
frame = None
map_state = None

rows = []
lists = []
map_xy = []
t_state = []
past_state = []
x_y = [0] * 5

start = Pose()
goal = Pose()
waypoint = PoseArray()
passpoint = PoseArray()
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

def PassFileRead():
   global frame
   global map_xy
   global passpoint

   with open(sys.argv[1]+'_passpoint.json', 'r') as f:
	passpointFile = json.load(f)
   	for i in passpointFile :
		pub = rospy.Publisher('passpoint', PoseArray, queue_size = 10)
   		pose = Pose()

		if frame == 'map' :
			init_x = 0
			init_y = 0
		else :
			init_x = map_xy[0]
			init_y = map_xy[1]
			
   		pose.position.x = i[0][0] - init_x
   		pose.position.y = i[0][1] - init_y
		pose.position.z = i[0][2]
		pose.orientation.x = i[1][0]
   		pose.orientation.y = i[1][1]
   		pose.orientation.z = i[1][2]
   		pose.orientation.w = i[1][3]

		passpoint.poses.append(pose)
		print('[[{0},{1},0.0],[0.0,0.0,{2},{3}]]' 
			.format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))

		passpoint.header.frame_id = frame
		pub.publish(passpoint)
		rospy.sleep(0.3)

def MapCallback(msg):
   global rows
   global map_xy
   
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
   global x_y
   global waypoint

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
   global frame
   global map_xy
   global start
   global waypoint
   global passpoint

   if frame == 'map' :
	init_x = 0
	init_y = 0
   else :
	init_x = map_xy[0]
	init_y = map_xy[1]

   if state == 'start' :
	x = start.position.x
   	y = start.position.y
   	z = start.orientation.z
   	w = start.orientation.w

	file=open(sys.argv[1]+'_waypoint.json', 'w')
   	file.write('[\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],'.format(x + init_x,y + init_y,z,w))
   else :
	file=open(sys.argv[1]+'_waypoint.json', 'a')
	for i in range(count) :
   		x = waypoint.poses[i].position.x + init_x
   		y = waypoint.poses[i].position.y + init_y
   		z = waypoint.poses[i].orientation.z
   		w = waypoint.poses[i].orientation.w

   		file.write('\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],'.format(x,y,z,w))
   	file.write('\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n]'
		.format(goal.position.x + init_x,goal.position.y + init_y,goal.orientation.z,goal.orientation.w))
   file.close()

def Init_XY():
   global x_y
   global start
   global goal
   global passpoint

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
   global free
   global occupancy
   global direction
   global count
   global p_count
   global state
   global map_state
   global lists
   global t_state
   global x_y

   unknown = 0
   t_state = ['right','left','up','down',
		'upper_right','lower_right','upper_left','lower_left',
		'right_upper_right','right_lower_right',
		'left_upper_left','left_lower_left',
		'upper_upper_right','lower_lower_right',
		'upper_upper_left','lower_lower_left']

   xy_list = []
   x_list = []
   y_list = []
   state_list = []

   q0 = 0.5 * q
   q1 = 1.3 *q
  
   print('--------------------------------------------------------------')

   for i in range(direction):
	x1 = x_y[0]
	y1 = x_y[1]
	x2 = x_y[2]
	y2 = x_y[3]
       
	if t_state[i] == 'right' :
		x1 += q1
	elif t_state[i] == 'right_upper_right' :
		x1 += q
		y1 += q0
	elif t_state[i] == 'upper_right' :
		x1 += q
		y1 += q
	elif t_state[i] == 'upper_upper_right' :
		x1 += q0
		y1 += q
	elif t_state[i] == 'up' :
		y1 += q1
	elif t_state[i] == 'upper_upper_left' :
		x1 -= q0
		y1 += q
	elif t_state[i] == 'upper_left' :
		x1 -= q
		y1 += q
	elif t_state[i] == 'left_upper_left' :
		x1 -= q
		y1 += q0
	elif t_state[i] == 'left' :
		x1 -= q1
	elif t_state[i] == 'left_lower_left' :
		x1 -= q
		y1 -= q0
	elif t_state[i] == 'lower_left' :
		x1 -= q
		y1 -= q
	elif t_state[i] == 'lower_lower_left' :
		x1 -= q0
		y1 -= q
	elif t_state[i] == 'down' :
		y1 -= q1
	elif t_state[i] == 'lower_lower_right' :
		x1 += q0
		y1 -= q
	elif t_state[i] == 'lower_right' :
		x1 += q
		y1 -= q
	elif t_state[i] == 'right_lower_right' :
		x1 += q
		y1 -= q0
  
	print(' {0}' .format(t_state[i]))
	
	OccupancyMap(x1,y1)
	if map_state != 'free' :
		unknown += 1
		if unknown >= direction-1 :
			if count == 0 :
				print('\n--------------------------------------------------------------')
				print('\007You must change q, grid, free and occupancy.')
				print('--------------------------------------------------------------')
				quit()
			print('\n--------------------------------------------------------------')
			print('Can not proceed and return !!')
			print('--------------------------------------------------------------\n')
			p_count += 1
			if p_count == 2 :
				free -= 10
				occupancy -= 1
				if free <= 40 or occupancy <= 0 :
					print('\n--------------------------------------------------------------')
					print('\007You must change q, grid, free and occupancy.')
					print('--------------------------------------------------------------')
					quit()
				p_count = 0
			waypoint.poses.pop(-1)
			past_state.append(state)
			count -= 2
			state = None
			StartPose()
			return
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
   global frame
   global grid
   global free
   global occupancy
   global map_state
   global map_xy
   global rows

   value = [0] * 3

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
   global waypoint
	
   if state == None :
	count += 1
	i = 0
	while True :
		if past_state[-1] == lists[count][0][i] :
			del lists[count][0][i]
			del lists[count][1][i]
			del lists[count][2][i]
			del lists[count][3][i]
			past_state.pop(-1)
			break
		i += 1
   else :
	   past_state.append(state)

   list = lists[count]
   while True :
		state = list[0][list[3].index(min(list[3]))] 
		x_y[0] = list[1][list[3].index(min(list[3]))]
		x_y[1] = list[2][list[3].index(min(list[3]))]
		x_y[4] = min(list[3])
		check = 1
		check = ChangeAngle(check)
		if check == 1 and len(list[0]) >= 2:
			print('\n--------------------------------------------------------------')
			print('Delete {0}' .format(list[0][list[3].index(min(list[3]))]))
			print('--------------------------------------------------------------')
			del list[0][list[3].index(min(list[3]))]
			del list[1][list[3].index(min(list[3]))]
			del list[2][list[3].index(min(list[3]))]
			del list[3][list[3].index(min(list[3]))]
		elif check == 1 and len(list[0]) <= 1:
			print('\n--------------------------------------------------------------')
			print('Can not proceed!!')
			print('--------------------------------------------------------------\n')
			quit()
		else :
			break
   Update()
    
   if x_y[4] <= q :
	print('   xy <= q : {0} <= {1}\n'.format(x_y[4],q))
	if len(passpoint.poses) == 1 :
		print('--------------------------------------------------------------')
		for i in range(3) :
  			remove = raw_input('\nRemove? (y/n) ')
			if remove == 'y' :
				while True :
					removenum = raw_input('number: ')
					if removenum == '':
						break
					else :
						waypoint.poses.pop(int(removenum))
						past_state.pop(int(removenum))
						count -= 1
   						print(' Remove waypoint {0}' .format(removenum))
				break
			elif remove == 'n' :
				 break
			else :
				if i == 2 :
					quit()
				print('Enter y or n!!\n')

		WriteFile()
		print('\n--------------------------------------------------------------')
		print('\nGoal\n  (x,y,z)=({0},{1},0.0)\n  (x,y,w,z)=(0.0,0.0,{2},{3})'
				.format(goal.position.x,goal.position.y,goal.orientation.z,goal.orientation.w))
		print('\nq = {0}  grid = {1}  free = {2}%  occupancy = {3}%' .format(q,grid,free,occupancy))
		print('Save '+sys.argv[1]+'_waypoint.json.....')
		CountState()
		quit()
	else : 
		passpoint.poses.pop(0)
		print('--------------------------------------------------------------')
		print('\nNext !!!!!!!!!!!\n')
   else :  
	print('   xy > q : {0} > {1}\n' .format(x_y[4],q))

def Update() :
   check = 0
   check = ChangeAngle(check) 
   WaypointPose() 
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
   elif p_state == 'right_upper_right' :
        p_angle = -0.16
   elif p_state == 'upper_right' :
        p_angle = -0.25 
   elif p_state == 'upper_upper_right' :
        p_angle = -0.33
   elif p_state == 'up' :
        p_angle = -0.5
   elif p_state == 'upper_upper_left' :
        p_angle = -0.67
   elif p_state == 'upper_left' :
        p_angle = -0.75
   elif p_state == 'left_upper_left' :
        p_angle = -0.83
   elif p_state == 'left' :
        p_angle = 1
   elif p_state == 'left_lower_left' :
        p_angle = 0.83
   elif p_state == 'lower_left':
        p_angle = 0.75
   elif p_state == 'lower_lower_left':
        p_angle = 0.67
   elif p_state == 'down' :
        p_angle = 0.5
   elif p_state == 'lower_lower_right':
        p_angle = 0.33
   elif p_state == 'lower_right' :
        p_angle = 0.25 
   elif p_state == 'right_lower_right':
        p_angle = 0.16

   yaw = p_angle * math.pi 
   
   if state == 'right' :
        angle = 0
   elif state == 'right_upper_right' :
        angle = 0.16
   elif state == 'upper_right' :
        angle = 0.25 
   elif state == 'upper_upper_right' :
        angle = 0.33
   elif state == 'up' :
        angle = 0.5
   elif state == 'upper_upper_left' :
        angle = 0.67
   elif state == 'upper_left' :
        angle = 0.75
   elif state == 'left_upper_left' :
        angle = 0.83
   elif state == 'left' :
        angle = 1
   elif state == 'left_lower_left' :
        angle = -0.83
   elif state == 'lower_left':
        angle = -0.75
   elif state == 'lower_lower_left':
        angle = -0.67
   elif state == 'down' :
        angle = -0.5
   elif state == 'lower_lower_right':
        angle = -0.33
   elif state == 'lower_right' :
        angle = -0.25 
   elif state == 'right_lower_right':
        angle = -0.16

   if check == 1 :
	yaw = round(abs(angle * math.pi + yaw),2)
	if yaw == 3.14 :
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

def CountState():
   global direction
   global past_state
   global t_state
   global count

   st_count = [0] * direction

   for i in past_state :
	for j in range(direction) :
		if i == t_state[j] :
			st_count[j] += 1
			break

   if direction >= 9 :
	rjust = 17
   elif direction >= 5 :
	rjust = 11
   else :
	rjust = 5

   print('\n--------------------------------------------------------------\n**Report**\n')
   for i in range(direction) :
	a = round(st_count[i]/float(len(past_state))*100,3)
	print('{0} : {1}   {2}%' .format(t_state[i].rjust(rjust),st_count[i],a))

   print('\n')

   for i, state in enumerate(past_state) :
	print('{0} : {1}' .format(str(i).rjust(3),state))
   print('\n--------------------------------------------------------------')

def SetUp():
   global q
   global grid
   global free
   global occupancy
   global direction
   global state
   global frame

   if (len(sys.argv) < 3):
        print('\007***Usage : rosrun arno auto_waypoint.py [fileName] [Number_of_directions]***')
        quit()

   direction = int(sys.argv[2])
   if direction <= 3 or direction >= 17 :
	print('\007***3 < directions < 17***\n')
	quit()

   while True :
	print('\007')
	Q = raw_input('With robot ? (y/n) ')
	if Q == 'y' :
		frame = 'map'
		break
	elif Q == 'n' :
		frame = 'base_link'
		break
	else :
		print('\007***Enter y or n.***')

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide/kino')
   is_file = os.path.isfile(sys.argv[1]+'_waypoint.json')

   if is_file :
	while True :
		Q = raw_input('Overwrite '+sys.argv[1]+'_waypoint.json? (y/n) ')
		if Q == 'y':
			break		
		elif Q == 'n':
			print('***\007'+sys.argv[1]+' already exists, so exit...***')
			quit()
		else:
			print('\007***Enter y or n.***')

   print('wait.....')
   rospy.Subscriber('/map', OccupancyGrid,MapCallback)
   rospy.sleep(3.0)

   param_in = [1.5,3,70,30]
   param_out = [4.0,30,70,30]

   while True :
	print('\007')
   	Q = raw_input('in or out or self ? ')
   	if Q == 'in':
   		q = param_in[0]
        	grid = param_in[1]
        	free = param_in[2]
		occupancy = param_in[3]
		break
   	elif Q == 'out':
        	q = param_out[0]
        	grid = param_out[1]
        	free = param_out[2]
        	occupancy = param_out[3]
		break
   	elif Q == 'self':
		print('ex)\n in  q={0} grid={1} free={2} occupancy={3}\n out q={4} grid={5} free={6} occupancy={7}' 
			.format(param_in[0],param_in[1],param_in[2],param_in[3],param_out[0],param_out[1],param_out[2],param_out[3]))
        	print('\nself')
		while True :
			q = float(raw_input(' q='))
			if q <= 0 or q >= 10 :
				print('\007***0 < q < 10***\n')
			else :
				break
		while True :
        		grid = float(raw_input(' grid='))
			if grid <= 5 or grid >= 50 :
				print('\007***5 < grid < 50***\n')
			else : 
				if grid.is_integer() == True :
					grid = int(grid)
					break
				else :
					print('\007***grid is int!!***\n')
		while True :
        		free = float(raw_input(' free= '))
        		occupancy = float(raw_input(' occupancy= '))
			if ( 0 <= free and free <= 100 ) and ( 0 <= occupancy and occupancy <= 100 ) :
				break
			else :
				print('\007***0 < free < 100 and 0 < occupancy < 100***\n')
		break
   	else :
		print('\007***Enter y or n.***')

   while True :
	Q = raw_input('Read passpoint file ? (y/n) ')
	if Q == 'y' :
		PassFileRead()
		rospy.sleep(2.0)
		break
	elif Q == 'n' :
   		print('\nMake Passpoints and Goal with [2DNavGoal]')
   		rospy.sleep(2.0)
   		print('\007Put [Enter] to start make waypoints')
   		rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,PassCallback)
   		rospy.sleep(0.1)
		end = raw_input()
   		if not end:
			break
	else :
		print('\007***Enter y or n.***')

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
   rospy.sleep(2.0)
   
   while not rospy.is_shutdown() :
	Init_XY()
	Calculation_XY()
	rospy.sleep(1.0)
