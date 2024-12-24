using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TableDestinationPublisher : MonoBehaviour
{
    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/table_coordinates";

    [SerializeField]
    GameObject m_Table;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<PoseMsg>(m_TopicName);
    }

    public void Publish()
    {
        var tablePose = new PoseMsg
        {
            position = m_Table.transform.position.To<FLU>(),
            orientation = m_Table.transform.rotation.To<FLU>()
        };

        // Publish the table coordinates to ROS
        m_Ros.Publish(m_TopicName, tablePose);
    }
}
