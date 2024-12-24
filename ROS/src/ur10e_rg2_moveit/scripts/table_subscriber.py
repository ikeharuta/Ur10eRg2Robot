#!/usr/bin/env python
"""
    Subscribes to "/collision_object" topic.
    Logs the received positions and dimensions of Collision Object - Table.
"""
import rospy
from moveit_msgs.msg import CollisionObject

def collision_callback(msg):
    rospy.loginfo("Collision Object Position: x=%f, y=%f, z=%f", msg.x, msg.y, msg.z)
    rospy.loginfo("Collision Object Dimensions: width=%f, height=%f, depth=%f", msg.width, msg.height, msg.depth)

def table_subscriber():
    rospy.init_node('collision_subscriber', anonymous=True)
    rospy.Subscriber("/collision_object", CollisionObject, collision_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        table_subscriber()
    except rospy.ROSInterruptException:
        pass
