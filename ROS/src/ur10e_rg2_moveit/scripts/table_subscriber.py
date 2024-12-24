#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def table_callback(msg):
    rospy.loginfo("Table Position: x=%f, y=%f, z=%f", msg.position.x, msg.position.y, msg.position.z)
    rospy.loginfo("Table Orientation: x=%f, y=%f, z=%f, w=%f", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

def table_subscriber():
    rospy.init_node('table_subscriber', anonymous=True)
    rospy.Subscriber("/table_coordinates", Pose, table_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        table_subscriber()
    except rospy.ROSInterruptException:
        pass
