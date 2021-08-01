# Demoshop

## Introduction
This repository accompanies the paper [*Designing User-Centric Programming Aids for Kinesthetic Teaching of Collaborative Robots*](https://intuitivecomputing.jhu.edu/publications/2021-ras-ajaykumar.pdf). It contains the source code for Demoshop, an interactive robot programming tool that includes user-centric programming aids to help end-users author and edit task demonstration. Demoshop consists of front-end software that runs on the [Unity game engine](https://unity.com/) and back-end software that runs on Ubuntu using [ROS](https://www.ros.org/). 

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

7. Run the Demoshop front-end software:
- Press the play button on the upper center of the Unity editor to launch the Demoshop user interface.
