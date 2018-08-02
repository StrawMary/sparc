# sparc 
Global repo that contains the other Pepper repos as submodules, as well as code for planning the behavior of the robot in a goal driven manner.

# Hardware:
- Raspberry PI should be plugged in the battery
- Make sure the battery is fully charged
- The Lidar should be connected to the RPi
- The RPi should be in the same network as the PC running the Neural Nets

# How to start the RPLidarNode on the RPi
On the Master PC:
- roscore

On the RPi:
- SSH pi@<lidar_ip> (192.168.0.180)
  - If the IP doesn't work, please plug a monitor inside the RPi to find out the IP
- check the open_lidar.sh script and ensure the IP matches the MASTER_IP (export ROS_MASTER_URI=http://<master_pc_ip>:11311)
- cd /home/pi/ros_catkin_ws/ && ./open_lidar.sh

# How to start the Hector_SLAM module
Prerequisites:
- RPLidarNode is running
- Pepper is turned on and fully functional

On the Master PC:
- cd /home/<user>/catkin_ws 

- If you want to start mapping from scratch ./run_slam.sh
- If you want to use the saved map ./run_slam_saved.sh (Make sure that Pepper starts from the initial position)

- rviz will start and you should see the map, laser scans and Pepper

# How to run the navigation (move_base) module
Prerequisites: 
- Have the Hector_SLAM up and running

On the Master PC:
- cd /home//catkin_ws
- Run ./run_navigation.sh 

<i>The configuration files can be found in <catkin_ws>/src/pepper_sparc/nav_conf</i>

Most useful parameters:
- costmap_common_params:
  - robot_radius(0.4) = the estimated radius of the robot base
  - inflation_radius(0.35) = the radius of the occupied cells in the grid

# How to use the speech recognition module
Prerequisites:
- Plug in the microphone inside the <b>external</b> audio card
- Start roscore on the machine running the publisher or make sure you have set the ROS_MASTER_URI before

On the machine you are running the speech recognition:
- python speech_recognition_publisher.py
  - this will run a ros node publishing the detected speech on <b>speech_text</b> topic

# How to run the system
Prerequisites:
- Make sure the speech recognition, SLAM, RPLidarNode and every equipment is plugged in and functional and (optional) Pepper

- python main.py

