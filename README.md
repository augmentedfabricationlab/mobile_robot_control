# Mobile Robot Control

[![Github - License](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/augmentedfabricationlab/mobile_robot_control)
[![PyPI - Latest Release](https://travis-ci.org/augmentedfabricationlab/mobile_robot_control.svg?branch=master)](https://github.com/augmentedfabricationlab/mobile_robot_control)

#### Mobile platform communication interface in Python 3.9 through ROS

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 8.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Docker Community Edition](https://www.docker.com/get-started): Download it for [Windows](https://store.docker.com/editions/community/docker-ce-desktop-windows). Leave "switch Linux containers to Windows containers" disabled.
* Git: [official command-line client](https://git-scm.com/) or visual GUI (e.g. [Github Desktop](https://desktop.github.com/) or [SourceTree](https://www.sourcetreeapp.com/))
* [VS Code](https://code.visualstudio.com/) with the following `Extensions`:
  * `Python` (official extension)
  * `EditorConfig for VS Code` (optional)
  * `Docker` (official extension, optional)

<sup>(1): Windows 10 Home does not support running Docker.</sup>

## Dependencies
* [compas 2.2.0 or higher](https://compas.dev/index.html)
* [compas_fab 1.0.2 or higher](https://gramaziokohler.github.io/compas_fab/latest/)

## Getting Started

### 1. Install dependencies on Rhino with Rhinocode

After installing Rhino 8, open Rhino and run ScriptEditor on the Rhino command line to initialize Python.

Execute the commands below in Anaconda Prompt:
    
    C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe -m pip install compas compas_robots roslibpy pyserial
    C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe -m pip install --no-deps compas_fab

If you have an older version, you have to force install a new version:
    
    C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe -m pip install --upgrade compas compas_robots roslibpy pyserial
    C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe -m pip install --upgrade --no-deps compas_fab
    
### 2. Cloning the repositories

#### Clone the repository

Create a workspace directory: C:\Users\YOUR_USERNAME\workspace

Open Github Desktop and clone the repositories:
* [mobile_robot_control](https://github.com/augmentedfabricationlab/mobile_robot_control) into you workspace folder.
* [ur_fabrication_control](https://github.com/augmentedfabricationlab/ur_fabrication_control) into you workspace folder.
* [fabrication_manager](https://github.com/augmentedfabricationlab/fabrication_manager) into you workspace folder.

SWITCH TO compas2 BRANCH IN MOBILE_ROBOT_CONTROL AND UR_FABRICATION_CONTROL!!!!

### 3. Install the repositories in editable mode

Open Rhino, Grasshopper and a Python3 block. Go to Tools/Options and below add the following paths:
* C:\Users\your_user_name\workspace\mobile_robot_control\src
* C:\Users\your_user_name\workspace\ur_fabrication_control\src
* C:\Users\your_user_name\workspace\fabrication_manager\src

### 4. Run docker

Open up the Docker desktop.

Open up VScode, and install the docker extension.

On VScode, find the file: \mobile_robot_control\docker\ros-systems\rbvogui-xl-ur20-noetic\docker-compose.yml, right click and click "compose up". Wait for the images to be copied and started.

### 5. Playgrounds

Go to C:\Users\your_user_name\workspace\mobile_robot_control\rhino and use the playgrounds that start with r8...

## Robot Model and Robot Frames

* Open the file [rhino/01_mobile_robot_model.ghx](rhino/01_mobile_robot_model.ghx).
* First, you need to load a specified robot model by pressing the `load` button (you can choose the model from a list of urdf files).
* Once, the model is loaded, you can add and remove a tool in the `Tool` cluster and manipulate the joints with the sliders in the `Configuration` cluster.

Below are the coordinate frame conventions for the mobile robot.

        WCF = world coordinate frame
        BCF = base coordinate frame in WCF
        RCF = ur robot arm coordinate frame in BCF
        
        RWCF = reference world coordinate frame
        RBCF = base coordinate frame in RWCF
        RRCF = ur robot arm coordinate frame in RBCF
        
        MCF = marker coordinate frame
        
![Ros_0s](https://github.com/augmentedfabricationlab/mobile_robot_control/assets/57141347/483b082e-6de6-4806-ac21-bb626bb57671)

![Ros_60s](https://github.com/augmentedfabricationlab/mobile_robot_control/assets/57141347/e526bf5f-d359-435b-a434-517639062c44)

## Control

* tbd

## Simulation Playground

* tbd

## General Notes

### Docker Setup Information
* Docker user name: augmentedfabricationlab
* The [robotic_setups_description](https://github.com/augmentedfabricationlab/robotic_setups_description.git) repository contains the robot descriptions files for the Dockerfile building.
* The [robotic_setups](https://github.com/augmentedfabricationlab/robotic_setups.git) repository contains the catkin workspace for the urdf models and moveit packages for various robotic setups of our lab, for setting up the systems in Linux as described in [this tutorial](https://gramaziokohler.github.io/compas_fab/latest/examples/03_backends_ros/07_ros_create_urdf_ur5_with_measurement_tool.html).
* The `ros-base` and `novnc` images are remote images and drawn from the [gramaziokohler docker hub organization](https://hub.docker.com/u/gramaziokohler).

## Credits

This package was created by Gido Dielemans `@gidodielemans` at `@augmentedfabricationlab`
