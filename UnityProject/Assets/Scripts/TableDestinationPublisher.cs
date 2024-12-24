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
    string m_TopicName = "/collision_object";

    [SerializeField]
    GameObject m_Table;

    // ROS Connector
    ROSConnection m_Ros;

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
            id = "table",
            operation = CollisionObjectMsg.ADD,
            primitive_poses = new List<PoseMsg> { tablePose }.ToArray(),
            primitives = new List<SolidPrimitiveMsg> {
                new SolidPrimitiveMsg
                {
                    type = SolidPrimitiveMsg.BOX,
                    dimensions = new double[] { 1.0, 1.0, 1.0 } // Example dimensions
                }
            }.ToArray()
        };

        // Publish the collision object
        m_Ros.Publish(m_TopicName, collisionObject);
    }
}
