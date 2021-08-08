#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import rospy
import os
import math
import sys
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray,Pose

make_waypoints = PoseArray()

def PrintArrow(pos,qu):
    global counter

    pub = rospy.Publisher("make_waypoints", PoseArray, queue_size = 10)
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.orientation.z = qu[2]
    pose.orientation.w = qu[3]
    make_waypoints.header.frame_id = "map"
    make_waypoints.poses.append(pose)
    pub.publish(make_waypoints)

def WriteFile(pos, qu):
    x = pos[0]
    y = pos[1]
    z = qu[2]
    w = qu[3]
   
    file=open(sys.argv[1], 'a')
    file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(x,y,z,w))
    file.close()

def AngleDif(target, current):
    diff = target - current
    if (diff > math.pi):
        diff -= 2 * math.pi
    elif(diff < -math.pi):
        diff += 2 * math.pi
    return abs(diff)

if __name__ == '__main__':
    rospy.init_node('arno_position')
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName_waypoint.json")
        quit()

    path = sys.argv[1]
    is_file = os.path.isfile(path)
    if is_file:
        answer = raw_input(sys.argv[1]+"を上書きしますか？\n(yes/no):")
        if answer == 'no' :
		quit()

    pos_x = -20 
    pos_y = -20 
    qu_x = 0
    qu_y = 0
    qu_z = 0
    qu_w = 0

    file=open(sys.argv[1], 'w')
    file.write("[")
    file.close()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
	position, quaternion = listener.lookupTransform("map", "base_link", now)

        dif1 = (position[0] - pos_x)**2 + (position[1] - pos_y)**2
	euler1 = tf.transformations.euler_from_quaternion((quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
	euler2 = tf.transformations.euler_from_quaternion((qu_x,qu_y,qu_z,qu_w))
	dif2 = AngleDif(euler1[2],euler2[2])
	

        if(dif1 >= 10.0 or dif2 >= 1.2):
	    data=(position,quaternion)
            WriteFile(position, quaternion)
            PrintArrow(position,quaternion)
           
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

    file=open(sys.argv[1], 'a')
    file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n]" .format(x,y,z,w))
    file.close()

    rospy.spin()
