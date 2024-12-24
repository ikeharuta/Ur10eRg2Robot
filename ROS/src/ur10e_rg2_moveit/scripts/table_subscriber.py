#!/usr/bin/env python
"""
    Subscribes to "/collision_object" topic.
    Logs the received positions and dimensions of Collision Object - Table.
"""
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg  # Import shape_msgs for SolidPrimitive
from geometry_msgs.msg import Quaternion

try:
    from math import pi, tau, dist, fabs, cos
except ImportError:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i, q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def collision_callback(msg):
    rospy.loginfo("Collision Object Position: x=%f, y=%f, z=%f", msg.primitive_poses[0].position.x, msg.primitive_poses[0].position.y, msg.primitive_poses[0].position.z)
    rospy.loginfo("Collision Object Dimensions: width=%f, height=%f, depth=%f", msg.primitives[0].dimensions[0], msg.primitives[0].dimensions[1], msg.primitives[0].dimensions[2])

    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()
    
    # Wait for the scene to be initialized properly
    rospy.sleep(2.0)

    # Create PlanningScene message
    planning_scene = moveit_msgs.msg.PlanningScene()
    planning_scene.world.collision_objects.append(msg)
    planning_scene.is_diff = True

    # Apply the collision object to the planning scene
    scene_diff_publisher = rospy.Publisher('/planning_scene', moveit_msgs.msg.PlanningScene, queue_size=10)
    rospy.sleep(2.0)
    scene_diff_publisher.publish(planning_scene)

def table_subscriber():
    # Initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('collision_subscriber', anonymous=True)

    # Subscribe to the CollisionObject topic
    rospy.Subscriber("/collision_object", moveit_msgs.msg.CollisionObject, collision_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        table_subscriber()
    except rospy.ROSInterruptException:
        pass
