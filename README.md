# Demoshop

## Introduction
This repository accompanies the paper [*Designing User-Centric Programming Aids for Kinesthetic Teaching of Collaborative Robots*](https://intuitivecomputing.jhu.edu/publications/2021-ras-ajaykumar.pdf). It contains the source code for Demoshop, an interactive robot programming tool that includes user-centric programming aids to help end-users author and edit task demonstrations. Demoshop consists of front-end software that runs on the [Unity game engine](https://unity.com/) and back-end software that runs on Ubuntu using [ROS](https://www.ros.org/). 

## Contents
- [demoshop-windows](https://github.com/intuitivecomputing/demoshop/tree/main/demoshop-windows): This directory is the Unity project folder corresponding to the front-end code for the Demoshop user interface. 
  - The primary subdirectory of interest within *demoshop-windows* is the [*Assets*](https://github.com/intuitivecomputing/demoshop/tree/main/demoshop-windows/Assets) subdirectory, which contains the information and scripts required to run Demoshop's 3-D simulation. 
- [demoshop_ubuntu](https://github.com/intuitivecomputing/demoshop/tree/main/demoshop_ubuntu): This directory is the ROS package folder corresponding to the back-end code for Demoshop.

- - - -

## Software and Hardware Requirements for Running Demoshop 

### Front-End
The Demoshop front-end code has been deployed on a system running Windows 10 (64-bit) using touchscreen and regular monitors. The software runs on Unity version 2018.3.13, which may be downloaded [here](https://unity3d.com/get-unity/download/archive).

### Back-End
The Demoshop back-end code has been deployed on a system running Ubuntu 16.04 (64-bit). The software uses [ROS Kinetic](http://wiki.ros.org/kinetic), which can be installed by following the instructions [here](http://wiki.ros.org/kinetic/Installation).

### Hardware
The Demoshop software is currently set up to work with a Universal Robots UR5 robot equipped with a ROBOTIQ 2F-140 gripper. We set up one webcam (Logitech C930e) in front of our task environment to track task objects. 

### Required ROS Packages
[file_server](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server)

[icl_ur5_setup_bringup](https://github.com/intuitivecomputing/ur5_with_robotiq_gripper/tree/master/icl_ur5_setup_bringup) (**NOTE**: You will need to modify/replace this package if you plan to use Demoshop with a different robot and/or gripper.)

[icl_ur5_setup_moveit_config](https://github.com/intuitivecomputing/ur5_with_robotiq_gripper/tree/master/icl_ur5_setup_moveit_config) (**NOTE**: You will need to modify/replace this package if you plan to use Demoshop with a different real or simulated robot and/or gripper.)

[icl_ur5_setup_gazebo](https://github.com/intuitivecomputing/ur5_with_robotiq_gripper/tree/master/icl_ur5_setup_gazebo) (**NOTE**: You will need to modify/replace this package if you plan to use Demoshop with a different simulated robot and/or gripper.)

[ar_track_alvar](https://github.com/ros-perception/ar_track_alvar) (*Use branch kinetic-devel*)

- - - -

## Usage
*Steps 1 and 7 below should be done on the Unity editor of a computer running Windows 10 with Unity 2018.13.3 installed, and Steps 2 - 6 below should be done on the terminal of a computer running Ubuntu 16.04 with ROS Kinetic installed.*
1. Open the demoshop-windows project on the Unity 2018.3.13 editor.
2. Establish communication between ROS and Unity: `roslaunch file_server ros_sharp_communication.launch`
3. Run UR5-related software corresponding to the real, physical robot or the simulated, virtual robot. (**NOTE**: The scripts below are associated with the UR5 used in the Intuitive Computing Lab. You will need to modify or replace these files for Demoshop to work with a different robot and/or gripper.) 
- For the real, physical robot:
  - Set up the gripper:
    - `roslaunch icl_ur5_setup_bringup ur5_gripper.launch`
    - `roslaunch icl_ur5_setup_bringup activate_gripper.launch`
      - Press *'r'* to reset, and then press *'a'* to activate the gripper.
  - Set up the UR5 in MoveIt!:
    - `roslaunch icl_ur5_setup_moveit_config ur5_gripper_moveit_planning_execution.launch`
    - `roslaunch icl_ur5_setup_moveit_config moveit_rviz.launch config:=true`
- For the virtual robot:
  - Set up the gripper in Gazebo:
    - `roslaunch icl_ur5_setup_gazebo icl_ur5_gripper.launch`
  - Set up the UR5 in MoveIt!:
    - `roslaunch icl_ur5_setup_moveit_config ur5_gripper_moveit_planning_execution.launch sim:=true`
    - `roslaunch icl_ur5_setup_moveit_config moveit_rviz.launch config:=true`

4.  Run the webcam driver: `roslaunch demoshop_ubuntu cam.launch` (**NOTE**: You will need to modify or replace the launch file to include the information corresponding to the camera you are using, if different from the Logitech C930e.)

5. Launch AR tag detection software: `roslaunch demoshop_ubuntu AR_tag_viewer_bundle_cube.launch` (**NOTE**: You will need to modify or replace the launch file and the bundle files it uses to include the information corresponding to your tagged task objects).

6. Run the Demoshop back-end software.
- For the real, physical robot: `rosrun demoshop_ubuntu robot_interface`
- For the virtual robot: `rosrun demoshop_ubuntu robot_interface_sim`

7. Press the play button on the upper center of the Unity editor to launch the Demoshop user interface.

- - - -

## Modifying Demoshop
We welcome development on the Demoshop software to extend its sensing capabilities and its applicability to different robots and different environments. Below, we describe some of the principal modifications that may be required to use Demoshop with a different robot or task environment. We have also included comments within the Demoshop source code to indicate areas that may required modification. 

For additional development questions that are not covered in this README, we welcome developers to open an issue or contact the author at [gopika@cs.jhu.edu](mailto:gopika@cs.jhu.edu). 

### Modifying the Robot Used With Demoshop
Demoshop cannot work with any robots that are unsupported by ROS at this time, so developers that want to use it with non-ROS-based robots will need to first develop a ROS package that exposes their robot’s manipulation capabilities and provides MoveIt! integration. Furthermore, Demoshop is meant for manipulation programs developed using demonstration, limiting its use to robots with manipulation (e.g., gripping) capabilities.

To modify Demoshop to work with a different robot, you will need to replace the 3-D model of the UR5 used in the *demoshop-windows* Unity project with the 3-D model of your robot by following the process outlined [here](https://github.com/siemens/ros-sharp/wiki/User_App_ROS_TransferURDFFromROS) to transfer the URDF corresponding to your robot to ROS (**NOTE**: You should preserve the trail game object and any of the game objects used to represent objects in your simulated environment before beginning this process and then reattach them to the appropriate game object (e.g., attach trail game object as a child to the gripper base game object, attach game objects corresponding to task objects as a child to the world game object) after your robot's game objects have been added to the scene using the URDF transfer plugin). You will then need to modify any code referring to the original UR5 game objects (e.g., replace references to *fts_toolside* with the name of your robot's end-effector game object), as well as the joint arrays used in [*RecordDemo.cs*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop-windows/Assets/RecordDemo.cs) (e.g., *jointNames*, *jointStates*, *snapJointConfig*) and the *ros_unity_joints* dictionary to reflect the number and the names of the joints of your robot, if different from the UR5’s. On the ROS backend, you will need to edit [*robot_interface.cpp*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop_ubuntu/src/robot_interface.cpp) and [*robot_interface_sim.cpp*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop_ubuntu/src/robot_interface_sim.cpp) to use the action server names used by the MoveIt! packages for your robot and gripper. The code interfacing with the gripper is meant for a two-finger gripper and will require modification for different types of end-effectors.

### Using Demoshop in a Different Environment
Our implementation currently uses AR tag tracking to display task objects (boxes) in Demoshop using the [*ar_track_alvar*](https://github.com/ros-perception/ar_track_alvar) package. We used the AR tag with ID 22 (*“ar_master_22”*) as a fixed reference point and AR tags starting with IDs 0 and IDs 24 to tag two different boxes. To modify the Demoshop source code to work with your custom objects, you must: (1) add 3-D models of the objects that you would like to track in Demoshop as prefab files into the Assets directory of the Unity project, (2) furnish the physical objects with AR tags (instructions for how to do this using the ar_track_alvar package can be found [here](http://wiki.ros.org/ar_track_alvar)), and (3) modify the code in the *transformCallback* function in [*robot_interface.cpp*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop_ubuntu/src/robot_interface.cpp) and [*robot_interface_sim.cpp*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop_ubuntu/src/robot_interface_sim.cpp) and the code in the *Update* function in [*RecordDemo.cs*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop-windows/Assets/RecordDemo.cs) to display the 3-D models of your objects when their respective tags are detected. 

All other objects (e.g., robot base, table, TV set) currently attached as children to the world game object and shown in the main Unity Scene (*SampleScene*) are static and were placed manually; these objects may be removed and replaced with custom 3D models to represent a new environment as desired. Collision objects should also be modified in the addCollisionObjects function in [*robot_interface.cpp*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop_ubuntu/src/robot_interface.cpp) and [*robot_interface_sim.cpp*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop_ubuntu/src/robot_interface_sim.cpp) according to the static obstacles in the environment in which Demoshop is used.

### “Snap” Feature
Our code includes a “snap” feature that autocompletes the grasp trajectory for the robot when they rest the robot’s end effector close to a task object for a certain time threshold. This feature is still under development. Instructions on how to disable it if desired are available in the comments in the *AddWaypoint* function in [*RecordDemo.cs*](https://github.com/intuitivecomputing/demoshop/blob/main/demoshop-windows/Assets/RecordDemo.cs).

- - - -

## BibTeX
If you use Demoshop in a scientific publication, please cite our work:
```
@article{ajaykumar2021designing,
  title={Designing user-centric programming aids for kinesthetic teaching of collaborative robots},
  author={Ajaykumar, Gopika and Stiber, Maia and Huang, Chien-Ming},
  journal={Robotics and Autonomous Systems},
  pages={103845},
  year={2021},
  publisher={Elsevier}
}

```
