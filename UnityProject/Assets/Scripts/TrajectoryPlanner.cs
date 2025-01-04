using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    private string m_TopicName = "/collision_object";
    [SerializeField]
    string m_RosServiceName = "ur10e_rg2_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR10e;
    public GameObject UR10e { get => m_UR10e; set => m_UR10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }
    [SerializeField]
    GameObject m_Table;

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(180, 90, 0);
    // TODO: Adjust for better position offset
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.35f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    // TODO: Handle gripper later
    // ArticulationBody m_LeftGripper;
    // ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        // Register the collision object publisher
        m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_UR10e.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // TODO: Handle gripper later
        // // Find left and right fingers
        // var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        // var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        // m_RightGripper = m_UR10e.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        // m_LeftGripper = m_UR10e.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        // TODO: Handle gripper later
        Debug.Log("Gripper Closed.");
        // var leftDrive = m_LeftGripper.xDrive;
        // var rightDrive = m_RightGripper.xDrive;

        // leftDrive.target = -0.01f;
        // rightDrive.target = 0.01f;

        // m_LeftGripper.xDrive = leftDrive;
        // m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        // TODO: Handle gripper later
        Debug.Log("Gripper Opened.");
        // var leftDrive = m_LeftGripper.xDrive;
        // var rightDrive = m_RightGripper.xDrive;

        // leftDrive.target = 0.01f;
        // rightDrive.target = -0.01f;

        // m_LeftGripper.xDrive = leftDrive;
        // m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>Ur10eMoveitJoints</returns>
    Ur10eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        // Publish the table mesh at the start
        PublishTableMesh();

        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(180, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        Debug.Log($"Pick Pose: Position: {request.pick_pose.position.x}, {request.pick_pose.position.y}, {request.pick_pose.position.z} | Orientation: {request.pick_pose.orientation.x}, {request.pick_pose.orientation.y}, {request.pick_pose.orientation.z}, {request.pick_pose.orientation.w}");

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        Debug.Log($"Place Pose: Position: {request.place_pose.position.x}, {request.place_pose.position.y}, {request.place_pose.position.z} | Orientation: {request.place_pose.orientation.x}, {request.place_pose.orientation.y}, {request.place_pose.orientation.z}, {request.place_pose.orientation.w}");

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from ur10e_rg2_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }

    void PublishTableMesh()
    {
        Debug.Log("Mesh is published");
        MeshFilter meshFilter = m_Table.GetComponent<MeshFilter>();

        Mesh mesh = meshFilter.mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        List<MeshMsg> meshMsgs = new List<MeshMsg>();
        MeshMsg meshMsg = new MeshMsg();

        List<PointMsg> points = new List<PointMsg>();
        foreach (var vertex in vertices)
        {
            Vector3 worldVertex = m_Table.transform.TransformPoint(vertex);
            points.Add(new PointMsg(worldVertex.x, worldVertex.y, worldVertex.z));
        }

        List<MeshTriangleMsg> triangleMsgs = new List<MeshTriangleMsg>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            MeshTriangleMsg triangleMsg = new MeshTriangleMsg
            {
                vertex_indices = new uint[] { (uint)triangles[i], (uint)triangles[i + 1], (uint)triangles[i + 2] }
            };
            triangleMsgs.Add(triangleMsg);
        }

        meshMsg.vertices = points.ToArray();
        meshMsg.triangles = triangleMsgs.ToArray();
        meshMsgs.Add(meshMsg);

        Vector3 unityPosition = m_Table.transform.position;
        Quaternion unityRotation = m_Table.transform.rotation;
        Quaternion adjustment = Quaternion.Euler(90, 0, 0);
        Quaternion rosQuaternion = unityRotation * adjustment;

        var tablePose = new PoseMsg
        {
            position = unityPosition.To<FLU>(),
            orientation = new QuaternionMsg(
                rosQuaternion.x,
                rosQuaternion.y,
                rosQuaternion.z,
                rosQuaternion.w
            )
        };

        var collisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "world"
            },
            id = "table",
            operation = CollisionObjectMsg.ADD,
            mesh_poses = new List<PoseMsg> { tablePose }.ToArray(),
            meshes = meshMsgs.ToArray()
        };

        Debug.Log($"Collision Object ID: {collisionObject.id}");
        Debug.Log($"Mesh Vertex Count: {mesh.vertexCount}");
        Debug.Log($"Mesh Pose: Position - {tablePose.position.x}, {tablePose.position.y}, {tablePose.position.z} | Orientation - {tablePose.orientation.x}, {tablePose.orientation.y}, {tablePose.orientation.z}, {tablePose.orientation.w}");

        m_Ros.Publish("/collision_object", collisionObject);
    }

}