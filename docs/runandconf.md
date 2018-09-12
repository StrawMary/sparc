# Running and Configuration Pepper with ROS
The Sparc system requires having a machine running ROS Indigo which will run the master node and a Raspberry PI 3 running ROS used for data acquisition from the RP Lidar A1 or similar Lidar devices. 

Any similar compatible devices which send [Laserscan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html) data to the master node can be used. The configuration for the SLAM module will have to be fine tuned and adapted with the device. 

## Installing ROS

1.  Setup your sources.list - Setup your computer to accept software from packages.ros.org. ROS Indigo ONLY supports Saucy (13.10) and Trusty (14.04) for debian packages.


		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
2.  Set up your keys
		
        sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
        sudo apt-get update

3.  Install the full package of ROS
	    sudo apt-get install ros-indigo-desktop-full
    
***If the ros-indigo cannot be found in the sources, make sure you have Ubuntu 14 or other compatible operating system! Check the ROS wrapper compatibility before continuing installing the system!***

4.  Initialise ROS DEP
	    sudo rosdep init  
	    rosdep update
5.  Setup the environment:
	    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc  
	    source ~/.bashrc
6.  Install python-ros
	    sudo apt-get install python-rosinstall
 
## Pepper ROS Integration

Pepper requires the NaoQI framework installed and running in order to be able to interact with Pepper. Choregraphe is also recommended for debugging.  After installing the NaoQI framework the Sparc system requires having Pepper integrated into the ROS framework, therefore altough Pepper cannot directly run ROS, a ROS wrapper can be installed in order to perform data acquisition from the robot and send command using ROS [common_msgs](https://github.com/ros/common_msgs). 

### Installing NaoQI

1. Download the C++ and Python SDK (2.5+) [https://community.aldebaran.com/](https://community.aldebaran.com/)
2.  Extract the python SDK (pynaoqi-python2.7-x.x.x.x-linux64.tar) and the C++ SDK (naoqi-sdk-x.x.x.x-linux64.tar) in ~/naoqi
		~/naoqi/naoqi-sdk-x.x.x.x-linux64/naoqi
3. Export the SDK path variable:

		export PYTHONPATH=~/naoqi/pynaoqi-python2.7-x.x.x.x-linux64:$PYTHONPATH
		echo 'export PYTHONPATH=~/naoqi/pynaoqi-python2.7-x.x.x.x-linux64:$PYTHONPATH' >> ~/.bashrc
4.  Run python and check that you can import the library
		
		from naoqi import ALProxy

### Installing the required ROS modules

-   The following command will install all the necessary ROS Modules that are used as wrappers over the Nao-QI framework and required to interact with Pepper. 
		
		sudo apt-get install ros-indigo-driver-base ros-indigo-move-base-msgs ros-indigo-octomap ros-indigo-octomap-msgs ros-indigo-humanoid-msgs ros-indigo-humanoid-nav-msgs ros-indigo-camera-info-manager ros-indigo-camera-info-manager-py

- Install the Pepper specific modules

		sudo apt-get install ros-indigo-pepper-.*

## Testing the Installation

1. Run `roscore` on the master node
2. Run `roslaunch pepper_bringup pepper_full.launch nao_ip:=<yourRobotIP> roscore_ip:=<roscore_ip> [network_interface:=<eth0|wlan0|vpn0>]`
	- roscore_ip is the ip of the master node
	- nao_ip is the IP of Pepper (can be found by pressing the button on Pepper's chest)
3. Run `rosrun rviz rviz`

The robot should be shown inside the rviz application. If this fails to happen make sure that the pepper meshes are installed and accessible. To install the meshes run `sudo apt-get install ros-indigo-pepper-meshes`. 

If you have any problems moving the robot with [move_base_simple](http://wiki.ros.org/move_base_simple) goals in rviz try publishing a static transform between the map frame and any other virtual frame in order to ensure that the map frame is available. This can be done by running:  `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10`.

If you still get problems, check that your laptop supports OpenGL 3 (it is displayed when opening RVIZ) and try changing the frame from the Global Settings in rviz.

The robot should be displayed correctly and you can subscribe to additional modules by adding sensors using the add button in RVIZ.