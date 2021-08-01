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
