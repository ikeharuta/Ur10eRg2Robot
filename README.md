# Pick and Place Tutorial

**Table of Contents**
- [Pick and Place Tutorial](#pick-and-place-tutorial)
- [Overview](#overview)
- [ROS Download](#ros-download)
- [Setting up Unity Scene](#setting-up-unity-scene)
- [Setting up the robot](#setting-up-the-robot)
- [Unity Side](#unity-side)
- [The ROS side](#the-ros-side)


# Overview
This project is a part of a module taught at Vietnamese-German University (VGU), instructed by Dr. Dong Quang Huan. The goal of this project is to adapt the pick and place for the UR10e robot arm integrated with the OnRobot RG2 gripper. The objective is to automate the process of picking an object from inside a 3D printer and placing it at a desired location in a simulated environment using Unity. The project utilizes the Open Motion Planning Library (OMPL) planner for motion planning.

If you have not already cloned this project to your local machine, do so now:

```bash
git clone --recurse-submodules https://github.com/PaulieThePlatypus/ur10e_rg2_PickAndPlace.git
```

# ROS Download
1. Navigate to the `/PATH/TO/ur10e_rg2_PickAndPlace/ROS` directory of this downloaded repo.
   - This directory will be used as the [ROS catkin workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace).
   - If you cloned the project and forgot to use `--recurse-submodules`, or if any submodule in this directory doesn't have content, you can run the command `git submodule update --init --recursive` to download packages for Git submodules.
   - Copy or download this directory to your ROS operating system if you are doing ROS operations in another machine, VM, or container.
    > Note: This contains the ROS packages for the pick-and-place task, including [ROS TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint), [MoveIt Msgs](https://github.com/ros-planning/moveit_msgs), `UR10e_moveit`, and `UR10e_urdf`.

2. The provided files require the following packages to be installed. ROS Noetic users should run:

   ```bash
   sudo apt-get update && sudo apt-get upgrade
   sudo apt-get install python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers
   sudo -H pip3 install rospkg jsonpickle
   ```
3. If you have not already built and sourced the ROS workspace since importing the new ROS packages, navigate to your ROS workplace, and run `catkin_make && source devel/setup.bash`. Ensure there are no errors.

`The ROS workspace is now ready to accept commands!`


# Setting up Unity Scene 

1. Install [Unity Hub](https://unity3d.com/get-unity/download).

2. Go it the "Installs" tab in the Unity Hub, and click the "Add" button. Select Unity **2020.3.11f1 (LTS)**. If this version is no longer available through the hub, you can find it in the [Unity Download Archive](https://unity3d.com/get-unity/download/archive).
   > Note: If you want to use another Unity version, the following versions work for the Pick-and-Place tutorial:

   > - Unity 2020.3: 2020.3.10f1 or later
   > - Unity 2021.1: 2021.1.8f1 or later
   > - Unity 2021.2: 2021.2.a16 or later

3. Go to the "Projects" tab in the Unity Hub, click the "Add" button, and navigate to and select the PickAndPlaceProject directory within this cloned repository (`/PATH/TO/ur10e_rg2_PickAndPlace/UnityProject`) to add the tutorial project to your Hub.

4. Click the newly added project to open it.

5. In Unity, double click to open the `Assets/Scenes/EmptyScene` scene if it is not already open.
    > Note: If you have some experience with Unity and would like to skip the scene setup portion, you can open the scene named `TutorialScene` now and skip ahead to [Setting up the robot](#setting-up-the-robot).

    > Note: Only one Unity scene should be open at a time. If you see multiple scenes open in the Hierarchy view, double-click the desired scene, e.g. `Assets/Scenes/EmptyScene`, to open it and close the other scenes.

    > The Hierarchy, Scene View, Game View, Play/Pause/Step toolbar, Inspector, Project, and Console windows of the Unity Editor have been highlighted below for reference, based on the default layout. Custom Unity Editor layouts may vary slightly. A top menu bar option is available to re-open any of these windows: Window > General.


6. In the Unity Project window, navigate to `Assets/Prefabs`. Select the Table prefab, and click and drag it into the Hierarchy window. The table should appear in the Scene view. Then, select and drag the Target into the Hierarchy window, as well as the TargetPlacement. They should appear to sit on the table. Then in the Inspector window re-scale the Table into: Position `(0.0, -0.64, 0)`, scale `(3, 0.64, 3)`


7. Select the `Main Camera` in the Hierarchy. Move the camera to a more convenient location for viewing the robot by assigning the `Main Camera`'s Position to `(0, 1.5, -2.5)`, and the Rotation to `(40, 0, 0)` in the Inspector, which can be found in the Transform component.



8. In the Unity Project window, navigate to `Assets/Import`. Select the Printer prefab `(imported.prefab)`, and click and drag it into the Hierarchy window. The Printer should appear in the Scene view. Then, select and drag the Target into the Hierarchy window, as well as the TargetPlacement. They should appear to sit on the table. Then in the Inspector window re-scale the Printer into: Position `(1.184, -0.01, -0.01)`, Rotation `(-90, 180, 90)` ,scale `(3.5, 2, 2.5)`. Modify the Target Position `(1.075, 0.168, 0.08)` and scale `(2, 2, 2)` such that it goes inside the Printer and set the TargetPlacement Position `(-0.73, 0, -0.5)` and scale `(0.1, 0.02, 0.1)` such that it locates ouside the Printer.

# Setting up the robot
> Note: Presumably when you opened this project, the Package Manager automatically checked out and built the URDF-Importer package for you. You can double-check this now by looking for `Packages/URDF-Importer` in the Project window or by opening the Package Manager window. See the [Quick Setup](../quick_setup.md) steps for adding this package to your own project.

1. Open the Physics Project Settings (in the top menu bar, Edit > Project Settings > Physics) and ensure the `Solver Type` is set to `Temporal Gauss Seidel`. This prevents erratic behavior in the joints that may be caused by the default solver.


2. Find and select the URDF file in the Project window (`Assets/URDF//ur10e_with_rg2.urdf`). From the menu, click `Assets -> Import Robot from URDF`, or in the Project window, right click on the selected file and click `Import Robot from URDF`.
    > Note: The file extension may not appear in the Project window. The ur10e_with_rg2.urdf file will appear in the root of the `Assets/URDF/ur10e_with_rg2` directory.

3. Keep the default Y Axis type and VHACD mesh decomposer in the Import menu and click `Import URDF`.

    > Note: Default mesh orientation is Y-up, which is supported by Unity, but some packages often use Z-up and X-up configuration.

    > Note: VHACD algorithm produces higher quality convex hull for collision detection than the default algorithm.

    > Note: The world-space origin of the robot is defined in its URDF file. In this sample, we have assigned it to sit on top of the table, which is at `(0, 0, 0)` with scale `(1, 1, 1)` in Unity coordinates.

    ```xml
    <joint name="joint_world" type="fixed">
        <parent link="world" />
            <child link="base_link" />
        <origin xyz="0 0 0" rpy="0 0 0" />
    </joint>
    ```

    > Note: Going from Unity world space to ROS world space requires a conversion. Unity's `(x,y,z)` is equivalent to the ROS `(z,-x,y)` coordinate.

4. Select the newly imported `ur10e_robot_rg2` object in the Scene Hierarchy, and from the Inspector window, find the Controller (Script) component. Set the Stiffness to `10000`, the Damping to `100` and `Force Limit` to `1000`. Set the Speed to `0.5` and the Acceleration to `0.25`.
    ![](img/1_controller.png)

5. In the Hierarchy window, click the arrow to the left of the name to expand the GameObject tree, down to `ur10e_robot_rg2/world/base_link`. Toggle on `Immovable` for the `base_link`.

    ![](img/1_base.png)

# Unity Side

**Quick Description:**

To enable communication between Unity and ROS, a TCP endpoint running as a ROS node handles all message passing. On the Unity side, a `ROSConnection` component provides the necessary functions to publish, subscribe, or call a service using the TCP endpoint ROS node. The ROS messages being passed between Unity and ROS are expected to be serialized exactly as ROS serializes them internally. This is achieved with the MessageGeneration plugin which generates C# classes, including serialization and deserialization functions, from ROS messages.

1. We will start with generating the MoveItMsg: RobotTrajectory. This file describes the trajectory contents that will be used in the sent and received trajectory messages.

   Select `Robotics -> Generate ROS Messages...` from the top menu bar.


   In the ROS Message Browser window, click `Browse` next to the ROS message path. Navigate to and select the ROS directory of this cloned repository (`ur10e_rg2_PickAndPlace/ROS/`). This window will populate with all msg and srv files found in this directory.



   Under `ROS/src/moveit_msgs/msg`, scroll to `RobotTrajectory.msg`, `CollisionObject.msg` and click its `Build msg` button. The button text will change to "Rebuild msg" when it has finished building.


	- One new C# script should populate the `Assets/RosMessages/Moveit/msg` directory: RobotTrajectoryMsg.cs. This name is the same as the message you built, with an "Msg" suffix (for message).

2. Next, the custom message scripts for this tutorial will need to be generated.

   Still in the ROS Message Browser window, expand `ROS/src/ur10e_rg2_moveit/msg` to view the msg files listed. Next to msg, click `Build 3 msgs`.

   ![](img/2_msg.png)

   > MessageGeneration generates a C# class from a ROS msg file with protections for use of C# reserved keywords and conversion to C# datatypes. Learn more about [ROS Messages](https://wiki.ros.org/Messages).


3. Finally, now that the messages have been generated, we will create the service for moving the robot.

   Still in the ROS Message Browser window, expand `ROS/src/ur10e_rg2_moveit/srv` to view the srv file listed. Next to srv, click `Build 1 srv`.

   ![](img/2_srv.png)

   > MessageGeneration generates two C# classes, a request and response, from a ROS srv file with protections for use of C# reserved keywords and conversion to C# datatypes. Learn more about [ROS Services](https://wiki.ros.org/Services).

   You can now close the ROS Message Browser window.

4. Open the `Assets/Scripts` You should now find two C# scripts in your project's `Assets/Scripts`.

   > Note: The SourceDestinationPublisher script is one of the included files. This script will communicate with ROS, grabbing the positions of the target and destination objects and sending it to the ROS Topic `"/ur10e_joints"`. The `Publish()` function is defined as follows:

   ```csharp
     public void Publish()
    {
        var sourceDestinationMessage = new UniversalRobotsJointsMsgMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = m_Target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = m_TargetPlacement.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
   ```
5. Return to the Unity Editor. Now that the message contents have been defined and the publisher script added, it needs to be added to the Unity world to run its functionality.

   Right click in the Hierarchy window and select "Create Empty" to add a new empty GameObject. Name it `Publisher`. Add the newly created TrajectoryPlanner component to the Publisher GameObject by selecting the Publisher object. Click "Add Component" in the Inspector, and begin typing "TrajectoryPlanner." Select the component when it appears.

   ![](img/2_sourcedest.gif)

6. Note that this component shows empty member variables in the Inspector window, which need to be assigned.

   Select the Target object in the Hierarchy and assign it to the `Target` field in the Publisher. Similarly, assign the TargetPlacement object to the `TargetPlacement` field. Assign the ur10e_robot_rg2 robot to the `UR10e` field. Assign `Ros Service Name` ur10e_gr2_moveit. Assign `Printer` with 2-makerbot-3. Assign `Table` with Table.

   ![](img/2_target.gif)

7. Next, the ROS TCP connection needs to be created. Select `Robotics -> ROS Settings` from the top menu bar.

   In the ROS Settings window, the `ROS IP Address` should be the IP address of your ROS machine (*not* the one running Unity).

   - Find the IP address of your ROS machine. In Ubuntu, open a terminal window, and enter `hostname -I`.

   - If you are **not** running ROS services in a Docker container, replace the `ROS IP Address` value with the IP address of your ROS machine. Ensure that the `Host Port` is set to `10000`.

   - If you **are** running ROS services in a Docker container, fill `ROS IP Address` with the loopback IP address `127.0.0.1`.

   ![](img/2_settings.png)

   The other settings can be left as their defaults. Opening the ROS Settings has created a ROSConnectionPrefab in `Assets/Resources` with the user-input settings. When the static `ROSConnection.instance` is referenced in a script, if a `ROSConnection` instance is not already present, the prefab will be instantiated in the Unity scene, and the connection will begin.

   > Note: While using the ROS Settings menu is the suggested workflow as of this version, you may still manually create a GameObject with an attached ROSConnection component.

8. Next, we will add a UI element that will allow user input to trigger the `PublishJoints()` function. In the Hierarchy window, right click to add a new UI > Button. Note that this will also create a new Canvas parent, as well as an Event System.
	> Note: In the `Game` view, you will see the button appear in the bottom left corner as an overlay. In `Scene` view the button will be rendered on a canvas object that may not be visible.

   > Note: In case the Button does not start in the bottom left, it can be moved by setting the `Pos X` and `Pos Y` values in its Rect Transform component. 

9. Select the newly made Button object, and scroll to see the Button component in the Inspector. Click the `+` button under the empty `OnClick()` header to add a new event. Select the `Publisher` object in the Hierarchy window and drag it into the new OnClick() event, where it says `None (Object)`. Click the dropdown where it says `No Function`. Select TrajectoryPlanner > `PublishJoints()`.

   ![](img/2_onclick.gif)

10. To change the text of the Button, expand the Button Hierarchy and select Text. Change the value in Text on the associated component.

   ![](img/2_text.png)

# The ROS side
1. Open a terminal window in the ROS workspace. Once again, source the workspace. Then, run the following `roslaunch` in order to set the ROS parameters, start the server endpoint, and start the trajectory subscriber.

   ```bash
   roslaunch ur10e_rg2_moveit test_TrajectoryPlanner.launch
   ```

   > Note: Running `roslaunch` automatically starts [ROS Core](http://wiki.ros.org/roscore) if it is not already running.

   

   > Note: To use a port other than 10000, or if you want to listen on a more restrictive ip address than 0.0.0.0 (e.g. for security reasons), you can pass those arguments into the roslaunch command like this:

   ```bash
   roslaunch ur10e_rg2_moveit test_TrajectoryPlanner.launch tcp_ip:=127.0.0.1 tcp_port:=10005
   ```

   This launch will print various messages to the console, including the set parameters and the nodes launched.

   Ensure that the `process[server_endpoint]` and `process[trajectory_subscriber]` were successfully started, and that a message similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000` is printed.

2. Return to Unity, and press Play. Click the UI Button in the Game view to call `Publish()` function, publishing the associated data to the ROS topic. View the terminal in which the `roslaunch` command is running. It should now print `I heard:` with the data.

ROS and Unity have now successfully connected!

   ![](img/2_echo.gif)

