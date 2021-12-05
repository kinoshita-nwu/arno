#!/usr/bin/env python

import rospy
import sys
import os
import json
import math
from nav_msgs.msg import OccupancyGrid

width = 0
height = 0
occupancy = 0
mode = None
rows = []
map_xy = []

def mapCallback(msg):
   global width
   global height
   global mode
   global rows
   global map_xy

   count = 0
   row = []

   data = msg.data
   width = msg.info.width
   height = msg.info.height
   map_xy.append(msg.info.origin.position.x)
   map_xy.append(msg.info.origin.position.y)

   print('\nwidth:{0}\nheight:{1}\n' .format(width,height))
   print('wait...')

   rows.append('P2')
   rows.append([width,height])
   rows.append('15')

   for i in range(height):
	for j in range(width):
		if data[count] == 100 : #black occupancy
			row.append(0)
		elif data[count] == -1 : #gray unknown
			row.append(11)
		elif data[count] == 0 : #white free
			row.append(15) 
		count += 1
	rows.append(row)
        row = []
   
   WriteYAML(msg)
   print('\nwait...')
   if mode == sys.argv[1]+'_remake' :
	RemakeMap()
   elif mode == sys.argv[1]+'_foot' :
        RemoveFootprint()
   #PrintScreen()

def WriteYAML(msg):
   global mode

   resolution = str(round(float(msg.info.resolution),6))
   x = str(round(float(msg.info.origin.position.x),6))
   y = str(round(float(msg.info.origin.position.y),6))

   file=open(mode+'.yaml', 'w')
   file.write('image: '+mode+'.pgm\n')
   file.write('resolution: '+resolution+'\n')
   file.write('origin: ['+x+','+y+', 0.000000]\n')
   file.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
   file.close()

   print('Make '+mode+'.yaml')

def RemakeMap():
   global occupancy

   for i in range(height):
	for j in range(width):
		if rows[i+3][j] == 0 :
			for k in range(occupancy) :
				for l in range(occupancy) :
					if rows[i+k+1][j+l-1] == 15:
						rows[i+k+1][j+l-1] = 2

def RemoveFootprint():
   global rows
   global map_xy
   
   count = 0
   waypoint_file = sys.argv[1]+'_waypoint.json'
   with open(waypoint_file, "r") as f:
        waypointFile = json.load(f)
        for line in waypointFile:
            x = int(math.sqrt((line[0][0]-map_xy[0])**2) * 20)
            y = int(math.sqrt((line[0][1]-map_xy[1])**2) * 20)
	    count += 1
	    #print('{0} x:{1} y:{2}'.format(count,x,y))
	    for i in range(100):
		for j in range(100):
			if rows[y+i][x+j] == 0  :
				rows[y+i][x+j] = 15
	    for i in range(10):
		for j in range(10):
			if rows[y+i][x+j] == 11 :
				rows[y+i][x+j] = 15

def PrintScreen():
   print(rows[0])
   print('{0} {1}'.format(rows[1][0],rows[1][1]))
   for i in range(height):
	for j in range(width-1):
		print rows[-1-i][j],
	print('{0}' .format(rows[-1-i][width-1]))

def WritePGM():
   global mode

   file=open(mode+'.pgm', 'w')
   file.write(rows[0]+'\n'+str(rows[1][0])+' '+str(rows[1][1])+'\n'+rows[2]+'\n')
   for i in range(height):
	for j in range(width):
		file.write(str(rows[-1-i][j])+' ')
	file.write('\n')
   file.close()

   print('Make '+mode+'.pgm')

def SetUp():
   global occupancy
   global mode

   if (len(sys.argv) < 2):
        print("\007Usage: rosrun arno map_remake.py fileName ")
        quit()

   while True :
	Q = raw_input('copy or remake or remove_footprint ? (copy/remake/foot) ')
	if Q == 'copy':
		mode = sys.argv[1]+'_copy'
		break
	elif Q == 'remake':
		mode = sys.argv[1]+'_remake'
		while True :
			Q = raw_input('in or out or self? (in/out/self) ')
			if Q == 'in' :
				occupancy = 3
				break
			elif Q == 'out' :
				occupancy = 10
				break
			elif Q == 'self' :
				print('ex)in...3   out...10')
				occupancy = int(raw_input('occupancy expansion width? \nself... '))
				break
			else :
				print('\007Enter in, out or self.')
		break
        elif Q == 'foot':
            mode = sys.argv[1]+'_foot'
            break
	else:
		print('\007Enter copy or remake or foot.')

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide/kino')
   is_file = os.path.isfile(mode+'.pgm')
   if is_file :
	while True :
		Q = raw_input('\n\007Overwrite '+mode+'.pgm? (y/n) ')
		if Q == 'y':
			break		
		elif Q == 'n':
			quit()
		else:
			print('\007Enter y or n.')
			
if __name__ == '__main__':
   rospy.init_node('map_sub')

   SetUp()

   rospy.Subscriber('/map', OccupancyGrid,mapCallback)
   if height >= 5000 :
   	rospy.sleep(10.0)
   else :
	rospy.sleep(3.0)
   WritePGM()
   quit()
