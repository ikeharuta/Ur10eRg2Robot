using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Moveit;
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
            x = tablePose.position.x,
            y = tablePose.position.y,
            z = tablePose.position.z,
            width = 1.0, // Example width
            height = 1.0, // Example height
            depth = 1.0  // Example depth
        };

        m_Ros.Publish(m_TopicName, collisionObject);
    }
}
