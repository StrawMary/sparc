# Navigation Module

The navigation module is build upon ROS modules which are integrated into the main software. The navigation module is working in two phases, the SLAM phase in which Pepper is used to map the environment and must be moved manually or with an exploration technique in order to build a map and the navigation phase, when the robot can move on the saved map, in a dynamic environment. 

## Creating a new map
A new map can be created by using the **full_slam_launch** file found in the **pepper_sparc** node. Once the SLAM process is began, the robot can be moved in the simulator in order to start the mapping process. The **hector_slam** node publishes the occupancy grid on the */map* topic. 

The map_saver node can be used at any point to save the map. The folder containing the maps can be also found inside the **pepper_sparc** node. 

In order to save the map simply run:     

	rosrun map_server map_server mymap.yaml 

Make sure you save and mark the starting position of the robot, since this will be considered the home position for the navigation phase.

## Running the navigation module on a map
Given a new map build in the previous step, **mymap.yaml** can be used at any point to navigate with Pepper. The robot should be in the home position. 
The starting home position must be edited in [sparc](https://github.com/ami-lab/sparc)/[master_catkin_ws](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws)/[src](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src)/[pepper_sparc](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc)/[nav_conf](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc/nav_conf)/**base_local_planner_params.yaml** as described in the ROS Modules chapter. 

First the map should be loaded and the robot should localize itself on the map. This can be done by running the **saved_slam_launch** file inside the **pepper_sparc** node. You should be able to see the robot in the right starting position. A small error in the starting position is not a reason to worry, since the localization will improve as soon as the robot starts to move. 

The **navigation.launch**  can be found in the same **pepper_sparc** node  and launches the **move_base** node in order to make Pepper navigate in a dynamic environment while avoiding obstacles. 