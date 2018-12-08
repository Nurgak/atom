# Advanced Teleoperated Office Marauder (ATOM)

## Overview

Repository for the ATOM platform running on a Raspberry Pi and its simulation stack in V-REP, based on [ROS framework](http://www.ros.org/).


**Keywords:** raspberry pi, robot, hardware, simulation, mobile platform

### License

The source code is released under [MIT license](LICENSE).

**Author(s): Karl Kangur  
Maintainer: Karl Kangur, karl.kangur@gmail.com  
Affiliation: Personal**

The `atom` package has been tested under [ROS](http://wiki.ros.org) Indigo and Ubuntu Mate 16.04 on a Raspberry Pi 3.


![Example image](doc/example.jpg)


## Installation

### Building from Source

#### Dependencies

    sudo apt install -y ros-kinetic-robot-state-publisher
    sudo apt install -y ros-kinetic-ros-control
    sudo apt install -y ros-kinetic-ros-controllers
    sudo apt install -y ros-kinetic-diff-drive-controller
    sudo apt install -y ros-kinetic-cv-camera
    sudo apt install -y ros-kinetic-joy
    sudo apt install -y ros-kinetic-teleop-twist-joy
    sudo apt install -y ros-kinetic-map-server
    sudo apt install -y ros-kinetic-move-base

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/Nurgak/atom.git
    catkin build


### Unit Tests

Run the unit tests with

	catkin build run_tests_atom


## Usage

Run the main node with

	roslaunch atom_platform atom_platform.launch

## Config files

atom_control/config

* **atom_controllers.yaml** The joint state controller and the differential drive controller parameters for the ATOM platform.
* **base_local_planner_params.yaml** Parameters for the local planner, used with `move_base` package.
* **costmap_common_params.yaml** Common parameters for the local and global planners, used with `move_base` package.
* **global_costmap_params.yaml** Parameters specific for the global planner, used with `move_base` package.
* **local_costmap_params.yaml** Parameters specific for the local planner, used with `move_base` package.

## Launch files

* **atom_platform.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### atom_hardware_interface

Hardware interface between the Raspberry Pi and the mobile platform. It creates the controller manager and exposes the velocity joint interfaces for the left and right drive motors. The joint definitions are taken from the `diff_drive_controller`, so they only need be defined once in the controller configuration file.

This node sets the PWM signals of the GPIO to set the motor speeds, according to the joint velocity commands receieved by `diff_drive_controller`.

#### Services

* **`stop`** ([std_srvs/Trigger])

	Stops all motors

		rosservice call /atom_platform/stop


#### Parameters

* **`diff_drive_controller/left_wheel`** (array, default: [])

	Left wheel names.

* **`diff_drive_controller/right_wheel`** (array, default: [])

	Right wheel names.


### atom_control

...
