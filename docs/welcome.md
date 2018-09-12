# Getting Started
Welcome to the SPARC Project documentation. 
## Hardware Prerequisites 
- Pepper Robot running NaoQI 2.5 or higher
- RaspberryPI 3 
- RPLidar A1 
## Software Prerequisites
1.  Ubuntu 14.04 operating system (or any other operating system which is compatible with ROS Indigo)
2.  Install ROS [[http://wiki.ros.org/indigo/Installation/Ubuntu](http://wiki.ros.org/indigo/Installation/Ubuntu)]
3. Python 2.7
4.  Pytorch installed and configured on the machine running CUDA 8.0+

## Project 

The project is organized as follows: 
* The ***external_platform*** folder contains the python project used for external processing like running the Vision module and interacting with the ROS environment. 
* The ***master_catkin_ws*** is the ros workspace used on the external machine as well and contains the Navigation module and all ROS related nodes.
* The ***rpi_catkin_ws*** is the ros workspace used on the Raspberry PI 3 and it is used to perform data acquisition.

### External Platform
The python project is organized in a modular way with each module having a specific task.  
#### Navigation
The module contains all the scripts interacting with the SLAM module. It contains the publishing nodes for the location of the detected objects, people and QR codes. It also interacts with the SLAM module to send the current robot position in the system. 

#### Robot Interaction
The module contains all the scripts used to interact with the system from an external module. The module is used by the speech module to send command to the system. In a similar fashion, the graphical user interface makes use of this module to interact with the system.

#### Task Management
The module contains all the scripts used for the task management and planning. It is responsible with the management of the commands queue and ensures when and if a task is executed. All the commands received by the robot interactions module is forwarded to the task management module. 

The planning is done using a Queue of Tasks. 
A task is defined by a node in a binary tree, having a success child and a fail child, both which are defined also as Tasks. 

#### Vision
The module is responsible with all the components used for detection, recognition and segmentation of person, objects and qr codes. 

TOC:
- Running and Configuration Pepper with ROS
	- Installing ROS
	- Pepper ROS Integration
		- Installing NaoQI
		- Installing the required ROS modules
	- Testing the Installation
- Vision Module
	- Testing the Vision module
	- Face recognition module
		- Adding a new face
		- Removing existing faces
- Navigation Module
	- Creating a new map
	- Running the navigation module on a map
- Speech Module
	- Wit.ai configuration
	- Understanding new phrases
- Integration of new command nodes