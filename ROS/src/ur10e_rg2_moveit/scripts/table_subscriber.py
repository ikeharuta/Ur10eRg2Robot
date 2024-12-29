#!/usr/bin/env python

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
from std_msgs.msg import Header

try:
    from math import pi, tau, dist, fabs, cos
except ImportError:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list

def collision_callback(msg):
    rospy.loginfo("Collision Object Position: x=%f, y=%f, z=%f", msg.primitive_poses[0].position.x, msg.primitive_poses[0].position.y, msg.primitive_poses[0].position.z)
    rospy.loginfo("Collision Object Dimensions: width=%f, height=%f, depth=%f", msg.primitives[0].dimensions[0], msg.primitives[0].dimensions[1], msg.primitives[0].dimensions[2])

    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()
    
    # # Create a box in the planning scene with the received pose and dimensions
    box_name = "table"
    size_x = msg.primitives[0].dimensions[0]
    size_y = msg.primitives[0].dimensions[1]
    size_z = msg.primitives[0].dimensions[0]
    x = msg.primitive_poses[0].position.x
    y = msg.primitive_poses[0].position.y
    z = msg.primitive_poses[0].position.z
     
    # TODO: add a box to planning scene
    # scene.add_box(box_name, size_x, size_y, size_z, x, y, z)

    rospy.sleep(2.0)  # Wait for the update to be applied

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


# #!/usr/bin/env python
# """
#     Subscribes to "/collision_object" topic.
#     Logs the received positions and dimensions of Collision Object - Table.
# """
# # Python 2/3 compatibility imports
# from __future__ import print_function
# from six.moves import input

# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# import shape_msgs.msg  # Import shape_msgs for SolidPrimitive
# from geometry_msgs.msg import Quaternion

# try:
#     from math import pi, tau, dist, fabs, cos
# except ImportError:  # For Python 2 compatibility
#     from math import pi, fabs, cos, sqrt
#     tau = 2.0 * pi

#     def dist(p, q):
#         return sqrt(sum((p_i, q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# def collision_callback(msg):
#     rospy.loginfo("Collision Object Position: x=%f, y=%f, z=%f", msg.primitive_poses[0].position.x, msg.primitive_poses[0].position.y, msg.primitive_poses[0].position.z)
#     rospy.loginfo("Collision Object Dimensions: width=%f, height=%f, depth=%f", msg.primitives[0].dimensions[0], msg.primitives[0].dimensions[1], msg.primitives[0].dimensions[2])

#     # Initialize the PlanningSceneInterface
#     scene = moveit_commander.PlanningSceneInterface()

#     box_pose = geometry_msgs.msg.PoseStamped()
#     box_pose.header.frame_id = "world"
#     box_name = "box"
#     scene.add_box(box_name, box_pose, size=(msg.primitives[0].dimensions[0], msg.primitives[0].dimensions[1], msg.primitives[0].dimensions[2]))

# def table_subscriber():
#     # Initialize moveit_commander and a rospy node
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('collision_subscriber', anonymous=True)

#     # Subscribe to the CollisionObject topic
#     rospy.Subscriber("/collision_object", moveit_msgs.msg.CollisionObject, collision_callback)

#     # Keep the node running
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         table_subscriber()
#     except rospy.ROSInterruptException:
#         pass