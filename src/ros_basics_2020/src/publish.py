#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

rospy.init_node('controller2', anonymous=True)
pub = rospy.Publisher("goal_pos", Point, queue_size=10)

while pub.get_num_connections() == 0:
    rospy.loginfo("Waiting for subscriber to connect")
    rospy.sleep(1)

point = Point()
point.x=3.0
point.y=3.0
point.z=0.0
pub.publish(point)
point.x=-2.0
point.y=3.0
point.z=0.0
pub.publish(point)
point.x=2.0
point.y=-2.0
point.z=0.0
pub.publish(point)
point.x=0.0
point.y=0.0
point.z=0.0
pub.publish(point)