#!/usr/bin/env python

import rospy
import time
import math
import sys
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

def PrintArrow(data):
    pub = rospy.Publisher("Arrow", Marker, queue_size = 100)
	
    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "basic_shapes"
    marker_data.id = 0
    marker_data.action = Marker.ADD
   
    marker_data.color.r = 1.0
    marker_data.color.g = 0.0
    marker_data.color.b = 1.0
    marker_data.color.a = 1.0
   
    marker_data.scale.x = 1
    marker_data.scale.y = 0.1
    marker_data.scale.z = 0.1

    marker_data.lifetime = rospy.Duration()
    marker_data.type = 0
       
    marker_data.pose.position.x = data[0][0]
    marker_data.pose.position.y = data[0][1]
    marker_data.pose.orientation.z = data[1][2]
    marker_data.pose.orientation.w = data[1][3]
   
    pub.publish(marker_data)

def WriteFile(pos, qu):
    x = pos[0]
    y = pos[1]
    z = qu[2]
    w = qu[3]
   
    file=open(sys.argv[1], 'a')
    file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(x,y,z,w))
    file.close()

if __name__ == '__main__':
    rospy.init_node('arno_position')
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName_waypoint.json")
        quit()

    old_x = 0 
    old_y = 0 

    file=open(sys.argv[1], 'w')
    file.write("[")
    file.close()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
	position, quaternion = listener.lookupTransform("map", "base_link", now)

        dis = (position[0] - old_x)**2 + (position[1] - old_y)**2

        if(dis>=0.5):
	    data=(position,quaternion)
            WriteFile(position, quaternion)
            PrintArrow(data)
           
            print([position[0],position[1],0.0],[0.0,0.0,quaternion[2],quaternion[3]])

            old_x = position[0]
            old_y = position[1]
            old_z = quaternion[2]
            old_w = quaternion[3]

	    rospy.sleep(1)

    file=open(sys.argv[1], 'a')
    file.write("\n]")
    file.close()

    rospy.spin()
           


