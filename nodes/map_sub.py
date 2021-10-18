#!/usr/bin/env python

import rospy
import sys
import os
from nav_msgs.msg import OccupancyGrid

width = 0
height = 0
occupancy = 0
mode = None
rows = []

def mapCallback(msg):
   global width
   global height
   global mode
   global rows

   count = 0
   row = []

   data = msg.data
   width = msg.info.width
   height = msg.info.height

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
	Q = raw_input('copy or remake? (c/r) ')
	if Q == 'c':
		mode = sys.argv[1]+'_copy'
		break
	elif Q == 'r':
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
	else:
		print('\007Enter c or r.')

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
