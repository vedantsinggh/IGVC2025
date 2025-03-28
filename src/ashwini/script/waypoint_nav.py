#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def move_to_goal(x, y):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_state()

if __name__ == "__main__":
    rospy.init_node("waypoint_nav")
    waypoints = [(2.0, 2.0), (4.0, -2.0), (6.0, 0.0)]
    
    for point in waypoints:
        result = move_to_goal(point[0], point[1])
        rospy.loginfo(f"Reached waypoint {point} with status {result}")

