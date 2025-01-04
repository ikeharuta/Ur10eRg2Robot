using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TableDestinationPublisher : MonoBehaviour
{
    [SerializeField]
    private string m_TopicName = "/collision_object";

    [SerializeField]
    private GameObject m_Table;

    // ROS Connector
    private ROSConnection m_Ros;

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);
    }

    public void Publish()
    {
        var tablePose = new PoseMsg
        {
            position = m_Table.transform.position.To<FLU>(),
            orientation = m_Table.transform.rotation.To<FLU>()
        };

        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "world"
            },
            id = "table", // Ensure the ID is assigned
            operation = CollisionObjectMsg.ADD,
            primitive_poses = new List<PoseMsg> { tablePose }.ToArray(),
            primitives = new List<SolidPrimitiveMsg> {
                new SolidPrimitiveMsg
                {
                    type = SolidPrimitiveMsg.BOX,
                    dimensions = new double[] {
                        m_Table.transform.localScale.x,
                        m_Table.transform.localScale.y,
                        m_Table.transform.localScale.z
                    }
                }
            }.ToArray()
        };

        Debug.Log($"Collision Object Header: {collisionObject.header.frame_id}");
        Debug.Log($"Collision Object ID: {collisionObject.id}");
        Debug.Log($"Collision Object operation: {collisionObject.operation}");
        Debug.Log($"Collision Object primitive_poses: {collisionObject.primitive_poses}");
        Debug.Log($"Collision Object primitives: {collisionObject.primitives}");

        // Publish the collision object
        m_Ros.Publish(m_TopicName, collisionObject);
    }
}
