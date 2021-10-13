#!/usr/bin/env python

import rospy
import sys
import os
from nav_msgs.msg import OccupancyGrid

width = 0
height = 0
rows = []

def mapCallback(msg):
   global width
   global height
   global rows

   count = 0
   row = []

   data = msg.data
   width = msg.info.width
   height = msg.info.height
   print('width:{0}\nheight:{1}\n' .format(width,height))  [Enter] to start "make waypoints"
[[35

   rows.append('P2')
   rows.append([width,height])
   rows.append('15')

   for i in range(height):
   	for i in range(width):
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
   #PrintScreen()

def PrintScreen():
   print(rows[0])
   print('{0} {1}'.format(rows[1][0],rows[1][1]))
   for i in range(height):
	for j in range(width-1):
		print rows[-1-i][j],
	print('{0}' .format(rows[-1-i][width-1]))

def WriteYAML(msg):
   resolution = str(round(float(msg.info.resolution),6))
   x = str(round(float(msg.info.origin.position.x),6))
   y = str(round(float(msg.info.origin.position.y),6))

   file=open(sys.argv[1]+'.yaml', 'w')
   file.write('image: '+sys.argv[1]+'.pgm\n')
   file.write('resolution: '+resolution+'\n')
   file.write('origin: ['+x+','+y+', 0.000000]\n')
   file.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
   file.close()
   print('Make '+sys.argv[1]+'.yaml')

def WritePGM(): 
   file=open(sys.argv[1]+'.pgm', 'w')
   file.write(rows[0]+'\n'+str(rows[1][0])+' '+str(rows[1][1])+'\n'+rows[2]+'\n')
   for i in range(height):
	for j in range(width):
		file.write(str(rows[-1-i][j])+' ')
        file.write('\n')
   file.close()
   print('Make '+sys.argv[1]+'.pgm')

if __name__ == '__main__':
    rospy.init_node('map_sub')

    if (len(sys.argv) < 2):
        print('\007')
        print("Usage: rosrun arno map_sub.py fileName ")
        quit()

    os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')

    rospy.Subscriber('/map', OccupancyGrid,mapCallback)
    rospy.sleep(3.0)
    WritePGM()
    quit()
