#!/usr/bin/env python

import rospy
import actionlib
import tf
import sys
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import json
from geometry_msgs.msg import PoseArray, Pose
import dynamic_reconfigure.client

waypoints = PoseArray()
change_params = []

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose = pose

    return goal_pose

def load_file(fileName):
    with open(fileName, "r") as f:
        waypointFile = json.load(f)
        for line in waypointFile:
            pose = Pose()
            pose.position.x = line[0][0]
            pose.position.y = line[0][1]
            pose.position.z = line[0][2]
            pose.orientation.x = line[1][0]
            pose.orientation.y = line[1][1]
            pose.orientation.z = line[1][2]
            pose.orientation.w = line[1][3]
            waypoints.poses.append(pose)
            if len(line) >= 3:
                change_params.append(line[2])
            else:
                change_params.append(None)
        waypoints.header.frame_id = "map"
    #print(waypoints)

def angle_dif(target, current):
    diff = target - current
    if (diff > math.pi):
        diff -= 2 * math.pi
    elif(diff < -math.pi):
        diff += 2 * math.pi
    return abs(diff)

if __name__ == '__main__':
    rospy.init_node('arno_navigation')
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    xy_tolerance = rospy.get_param('move_base/DWAPlannerROS/xy_goal_tolerance')
    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName")
        quit()

    load_file(sys.argv[1])
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
    dynamic_client = dynamic_reconfigure.client.Client("amcl", timeout=30)

    while not rospy.is_shutdown():
        for i in range(len(waypoints.poses)):
            pose = waypoints.poses[i]
            pub.publish(waypoints)
            goal = goal_pose(pose)
            client.send_goal(goal)
            if not change_params[i] == None:
                dynamic_client.update_configuration(change_params[i])

            while not rospy.is_shutdown():
                now = rospy.Time.now()
                listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))

                position, quaternion = listener.lookupTransform("map", "base_link", now)
                remaindXDist = position[0] - goal.target_pose.pose.position.x
                remaindYDist = position[1] - goal.target_pose.pose.position.y
                client.wait_for_result(rospy.Duration(0.1))
                if (math.sqrt(remaindXDist**2 + remaindYDist**2) <= xy_tolerance):
                    print("next!!")
                    break
                elif (client.get_result()):
                    # resend goal potision
                    print("send Fixed goal position!!")
                    goal.target_pose.pose.position.x += remaindXDist
                    goal.target_pose.pose.position.y += remaindYDist
                    client.send_goal(goal)
#                else:
#                     rospy.sleep(0.5)

