#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import rospy
import tf
import os
import sys
import math
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

make_waypoints_out = PoseArray()
make_waypoints_in = PoseArray()
make_numbers = MarkerArray()
counter = 0

def PrintArrow(pos,qu,dif):
    global counter
    
    if dif >= 10.0 :
    	pub = rospy.Publisher("make_waypoints_out", PoseArray, queue_size = 10)
    	pose = Pose()
    	pose.position.x = pos[0]
    	pose.position.y = pos[1]
    	pose.orientation.z = qu[2]
    	pose.orientation.w = qu[3]

    	make_waypoints_out.header.frame_id = "map"
    	make_waypoints_out.poses.append(pose)
   	pub.publish(make_waypoints_out)

    else :
	pub = rospy.Publisher("make_waypoints_in", PoseArray, queue_size = 10)
    	pose = Pose()
    	pose.position.x = pos[0]
    	pose.position.y = pos[1]
    	pose.orientation.z = qu[2]
    	pose.orientation.w = qu[3]

    	make_waypoints_in.header.frame_id = "map"
    	make_waypoints_in.poses.append(pose)
   	pub.publish(make_waypoints_in)

    num = rospy.Publisher("make_numbers", MarkerArray, queue_size = 10)
    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "basic_shapes"
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

    if dif <= 9.0 :
    	marker_data.scale.z = 0.5
    else :
	marker_data.scale.z = 2.0

    marker_data.lifetime = rospy.Duration()
    marker_data.type = Marker.TEXT_VIEW_FACING

    make_numbers.markers.append(marker_data)
    num.publish(make_numbers)
    counter +=1

def WriteFile(pos, qu):
    x = pos[0]
    y = pos[1]
    z = qu[2]
    w = qu[3]
   
    file=open(sys.argv[1]+'_waypoint.json', 'a')
    file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(x,y,z,w))
    file.close()

def AngleDif(target, current):
    diff = target - current
    if (diff > math.pi):
        diff -= 2 * math.pi
    elif(diff < -math.pi):
	diff += 2 * math.pi
    return abs(diff)

def RemovePoint(RemoveNum):
    with open(sys.argv[1]+'_waypoint.json','r') as f:
    	waypointfile = json.load(f)
	waypointfile.pop(RemoveNum)
    with open(sys.argv[1]+'_waypoint.json','w') as f:
	json.dump(waypointfile,f)

def Question():
    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName ")
        quit()

    os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')
    path = sys.argv[1]+'_waypoint.json'
    is_file = os.path.isfile(path)

    for i in range(3):
    	if is_file:
        	answer1 = raw_input(sys.argv[1]+"_waypoint.jsonを上書きしますか？ (y/n):")
        	if answer1 == 'y' :
			break
		elif answer1 == 'n' :
			quit()
		else :
			print("y か n を入力してください。")

    for i in range(3):
    	answer2 = raw_input("屋外で作成しますか？ (y/n): ")
    	if answer2 == 'y' :
		Dif1 = 15.0
		Dif2 = 1.2
		break
    	elif answer2 == 'n' :
		answer3 = raw_input("屋内で作成しますか？ (y/n): ")
		if answer3 == 'y' :
			Dif1 = 1.8
			Dif2 = 0.4
			break
    		elif answer3 == 'n':
			print("\nウェイポイントの間隔を設定してください。\n")
			print(" ex)屋外...Dif1 = 15.0 , Dif2 = 1.2\n ex)屋内...Dif1 = 1.8 , Dif2 = 0.4\n")
    		    	dif1 = raw_input("Dif1 = ")
			dif2 = raw_input("Dif2 = ")
			Dif1 = float(dif1)
			Dif2 = float(dif2)
			break
    	else :
		print("y か n を入力してください。")

    print("\n[Start make_waypoint] dif1 = {0} , dif2 = {1}".format(Dif1,Dif2))
    return (Dif1,Dif2)

if __name__ == '__main__':
    rospy.init_node('arno_position')
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    Dif1, Dif2 = Question()

    file=open(sys.argv[1]+'_waypoint.json', 'w')
    file.write("[")
    file.close()

    now = rospy.Time.now()
    listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
    position, quaternion = listener.lookupTransform("map", "base_link", now)
    
    pos_x = position[0] 
    pos_y = position[1]
    qu_x = quaternion[0]
    qu_y = quaternion[1]
    qu_z = quaternion[2]
    qu_w = quaternion[3]

    WriteFile(position, quaternion)
    PrintArrow(position,quaternion,Dif1)
    print([position[0],position[1],0.0],[0.0,0.0,quaternion[2],quaternion[3]])

    while not rospy.is_shutdown() :
        now = rospy.Time.now()
	listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
	position, quaternion = listener.lookupTransform("map", "base_link", now)
        
	dif1 = (position[0] - pos_x)**2 + (position[1] - pos_y)**2
	euler1 = tf.transformations.euler_from_quaternion((quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
	euler2 = tf.transformations.euler_from_quaternion((qu_x,qu_y,qu_z,qu_w))
	dif2 = AngleDif(euler1[2],euler2[2])
	
       	if(dif1 >= Dif1 or dif2 >= Dif2):
		WriteFile(position, quaternion)
		PrintArrow(position,quaternion,Dif1)
        	   
		print([position[0],position[1],0.0],[0.0,0.0,quaternion[2],quaternion[3]])
	
		pos_x = position[0]
		pos_y = position[1]
		qu_x = quaternion[0]
		qu_y = quaternion[1]
		qu_z = quaternion[2]
		qu_w = quaternion[3]

    listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
    position, quaternion = listener.lookupTransform("map", "base_link", now)
    x=position[0]
    y=position[1]
    z=quaternion[2]
    w=quaternion[3]

    file=open(sys.argv[1]+'_waypoint.json', 'a')
    file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n]" .format(x,y,z,w))
    file.close()

    for i in range(3) :
    	remove = raw_input("\nウェイポイントの削除を行いますか？ (y/n):")
    	if remove == 'y' :
		print("end と入力すると終了します。")
		while True :
			removenum = raw_input("number: ")
			if removenum == 'end':
				quit()
			else :
				Number = int(removenum)
				RemovePoint(Number)
    	elif remove == 'n' :
		quit()
	else :
		print("y か n を入力してください。\n")

    rospy.spin()
