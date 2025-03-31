#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def send_waypoint(x, y, yaw_deg=0):
    rospy.init_node('waypoint_sender', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow pub to register

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0

    import math
    yaw = math.radians(yaw_deg)
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)

    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    rospy.loginfo(f"Sending waypoint: x={x}, y={y}, yaw={yaw_deg}")
    pub.publish(goal)

if __name__ == '__main__':
    try:
        # Example: send_waypoint(x, y, yaw_deg)
        send_waypoint(1.0, 2.0, 0)
    except rospy.ROSInterruptException:
        pass