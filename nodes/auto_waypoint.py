#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import rospy
import tf
import os
import sys
import math
import json
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import Marker ,MarkerArray
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal

q = 0
count = 0
state = None
start = Pose()
goal = Pose()
passpoint = PoseArray()
waypoint = PoseArray()
label = MarkerArray()
x_y = [0,0,0,0,0]

def PassCallback(data):
   global passpoint

   pub = rospy.Publisher("passpoint", PoseArray, queue_size = 10)
   pose = Pose()
   pose.position.x = data.goal.target_pose.pose.position.x
   pose.position.y = data.goal.target_pose.pose.position.y
   pose.orientation.z = data.goal.target_pose.pose.orientation.z
   pose.orientation.w = data.goal.target_pose.pose.orientation.w

   print("[[{0},{1},0.0],[0.0,0.0,{2},{3}]]" 
	.format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))

   passpoint.header.frame_id = "map"
   passpoint.poses.append(pose)
   pub.publish(passpoint)

def GoalPose():
   global goal
   global passpoint

   data = passpoint.poses[-1]
   goal.position.x = data.position.x
   goal.position.y = data.position.y
   goal.orientation.z = data.orientation.z
   goal.orientation.w = data.orientation.w

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
   global start
   global waypoint
   global x_y

   pub = rospy.Publisher("waypoint", PoseArray, queue_size = 10)
   pose = Pose()
   pose.position.x = x_y[0]
   pose.position.y = x_y[1]
   pose.orientation.z = start.orientation.z
   pose.orientation.w = start.orientation.w
   
   print("{0}:[[{1},{2},0.0],[0.0,0.0,{3},{4}]]\n" 
	.format(state,pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))

   waypoint.header.frame_id = "map"
   waypoint.poses.append(pose)
   pub.publish(waypoint)

def ArrowLabel():
   global count
   global state
   global start
   global goal
   global waypoint
   global label
   
   pub = rospy.Publisher("label", MarkerArray, queue_size = 10)
   marker_data = Marker()
   marker_data.header.frame_id = "map"
   marker_data.header.stamp = rospy.Time.now()  
   
   marker_data.ns = "basic_shapes"

   marker_data.action = Marker.ADD
   
   marker_data.color.r = 1.0
   marker_data.color.g = 0.2
   marker_data.color.b = 0.5
   marker_data.color.a = 1.0

   marker_data.scale.x = 2.0
   marker_data.scale.y = 2.0
   marker_data.scale.z = 2.0
   
   marker_data.lifetime = rospy.Duration()
   marker_data.type = Marker.TEXT_VIEW_FACING

   if state == "goal" :
      data = goal
      marker_data.text = "Goal"
      marker_data.id = -1
   elif state == "start" :
      data = start
      marker_data.text = "Start"
      marker_data.id = -2
   else :
      data = waypoint.poses[count]
      marker_data.text = str(count)
      marker_data.id = count
      count +=1

   marker_data.pose.position.x = data.position.x
   marker_data.pose.position.y = data.position.y
   marker_data.pose.orientation.z = data.orientation.z
   marker_data.pose.orientation.w = data.orientation.w
   
   label.markers.append(marker_data)
   pub.publish(label)

def WriteFile():
   global count
   global state
   global start
   global waypoint

   if state == "start" :
      data = start
   else :
      data = waypoint.poses[count]
     
   x = data.position.x
   y = data.position.y
   z = data.orientation.z
   w = data.orientation.w
   
   file=open(sys.argv[1]+'_waypoint.json', 'a')
   file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(x,y,z,w))
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

def ChangeState():
   global q
   global state
   global passpoint
   global x_y

   t_state = ["right","left","up","down","upper_right","upper_left","lower_right","lower_left"]
   counter = 0
   s_x1 = 0
   s_y1 = 0
   s_xy = x_y[4]
   s_st = None
   p = state
   q = float(q)
   
   for i in range(8):
     t_st = t_state[counter]

     x1 = x_y[0]
     y1 = x_y[1]
     x2 = x_y[2]
     y2 = x_y[3]
       
     if t_st == "right" :
        x1 += q
     elif t_st == "left" :
        x1 -= q
     elif t_st == "up" :
        y1 += q
     elif t_st == "down" :
        y1 -= q
     elif t_st == "upper_right" :
        x1 += q
        y1 += q
     elif t_st == "lower_right" :
        x1 += q
        y1 -= q
     elif t_st == "upper_left" :
        x1 -= q
        y1 += q
     elif t_st == "lower_left":
        x1 -= q
        y1 -= q  
     
     t_xy = math.sqrt(((x_y[2]-x1)**2)+((x_y[3]-y1)**2))
     print("  {0} {1} [{2},{3},0.0]" .format(t_st,t_xy,x1,y1))
     if  t_xy < s_xy :
        s_st = t_st
        s_xy = t_xy
        s_x1 = x1
        s_y1 = y1
     
     counter += 1

   state = s_st
   x_y[0] = s_x1
   x_y[1] = s_y1

   ChangeAngle(p)
   
   if state == None :
      q = -1
   elif len(passpoint.poses) == 0 and s_xy < q:
      q = 0
   elif  s_xy < q :
      passpoint.poses.pop(0)

def ChangeAngle(p_state):
   global state
   global start
   global x_y

   quaternion = start.orientation
   euler = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
   
   angle = 0
   p_angle = 0

   if p_state == "left" :
        p_angle = 1
   elif p_state == "up" :
        p_angle = -0.5
   elif p_state == "down" :
        p_angle = 0.5
   elif p_state == "upper_right" :
        p_angle = -0.25 
   elif p_state == "lower_right" :
        p_angle = 0.25
   elif p_state == "upper_left" :
        p_angle = -0.75
   elif p_state == "lower_left":
        p_angle = 0.75 

   p = p_angle * math.pi 

   if state == "left" :
        angle = 1
   elif state == "up" :
        angle = 0.5
   elif state == "down" :
        angle = -0.5
   elif state == "upper_right" :
        angle = 0.25 
   elif state == "lower_right" :
        angle = -0.25
   elif state == "upper_left" :
        angle = 0.75 
   elif state == "lower_left":
        angle = -0.75 

   yaw = euler[2] + (angle * math.pi) + p

   quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1],yaw)
   start.orientation.z = quaternion[2]
   start.orientation.w = quaternion[3]

def SetUp():
   global q
   global state
   
   if (len(sys.argv) < 2):
        print('\007')
        print("Usage " + sys.argv[0] + " fileName ")
        quit()

   os.chdir('/home/hokuyo/catkin_ws/src/arno/map/nide')

   file=open(sys.argv[1]+'_waypoint.json', 'w')
   file.write("[")
   file.close()
   
   q = raw_input("ウェイポイントの間隔を入力:")
  
   print('[2D Nav Goal]でゴールを指定してください。終了したら、endと入力してください。')
   rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,PassCallback)
   rospy.sleep(0.1)

   r = raw_input()
   if r == 'end':
        state = "goal"
	GoalPose()
   else :
	quit()
   ArrowLabel()
   
  
if __name__ == '__main__':
   rospy.init_node('auto_waypoint')
   listener = tf.TransformListener()
   listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
   
   SetUp()

   now = rospy.Time.now()
   listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
   position, quaternion = listener.lookupTransform("map", "base_link", now)
 
   state = "start"
   start.position.x = position[0]
   start.position.y = position[1]
   start.orientation.z = quaternion[2]
   start.orientation.w = quaternion[3]

   ArrowLabel()
   WriteFile()
   
   print("\nStart:[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n"
         .format(start.position.x,start.position.y,start.orientation.z,start.orientation.w))
   
   while not rospy.is_shutdown() :
      Init_XY()
      print(count)
      ChangeState()
      if q == -1 :
         break  
      WaypointPose()
      WriteFile()
      StartPose()
      ArrowLabel()
      rospy.sleep(1.0)
      if q == 0 :
         break  

   print("\nGoal:[[{0},{1},0.0],[0.0,0.0,{2},{3}]]]" 
	.format(goal.position.x,goal.position.y,goal.orientation.z,goal.orientation.w))
   
   file=open(sys.argv[1]+'_waypoint.json', 'a')
   file.write("\n[[{0},{1},0.0],[0.0,0.0,{2},{3}]]\n]" 
	.format(goal.position.x,goal.position.y,goal.orientation.z,goal.orientation.w))
   file.close()

   print("ファイルを"+sys.argv[1]+"_waypoint.jsonという名前で保存しました。")
