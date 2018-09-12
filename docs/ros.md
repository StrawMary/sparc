# ROS Modules List
1. Pepper ROS Integration - [pepper_bring_up](http://wiki.ros.org/pepper_bringup)
2. SLAM - [hector_slam](http://wiki.ros.org/hector_slam)
3. Localization after initial mapping - [amcl](http://wiki.ros.org/amcl)
4. Navigation - [move_base](http://wiki.ros.org/move_base)
5. Data Acquisition on the Raspberry PI 3 - [rplidar](http://wiki.ros.org/rplidar)
 
## Configuration

The two main launch files containing the full system launch can be found in [sparc](https://github.com/ami-lab/sparc)/[master_catkin_ws](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws)/[src](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src)/[pepper_sparc](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc)/[launch](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc/launch)/**saved_slam.launch** and in [sparc](https://github.com/ami-lab/sparc)/[master_catkin_ws](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws)/[src](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src)/[pepper_sparc](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc)/[launch](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc/launch)/**full_slam.launch**.

The **saved_slam_launch** starts the Pepper ROS integration with **pepper_bring_up**, loads a map from the maps folder inside the **pepper_sparc** node and runs **amcl** to localize the robot on the given map.

The **full_slam_launch** starts the Pepper ROS integration with **pepper_bring_up**,  runs **hector_slam** to build the map.

Both launch files are compatible with the launch file used for running the  **move_base** node, which can be found in the same folder under **navigation.launch**. 

The system works with both the saved map and with the the SLAM module.  


1. Hector_SLAM:

The main launch file can be found at: [sparc](https://github.com/ami-lab/sparc)/[master_catkin_ws](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws)/[src](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src)/[slam_launch](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/slam_launch)/**hector_mapping.launch**

    <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
		<param name="pub_map_odom_transform" value="true"/>
		<param name="map_frame" value="map" />
		<param name="base_frame" value="base_footprint" />
		<param name="odom_frame" value="odom" />
		<param name="map_resolution" value="0.05"/>
		<param name="map_size" value="1024"/>
		<param name="map_start_x" value="0.5"/>
		<param name="map_start_y" value="0.5" />
		<param name="laser_z_min_value" value="-1.0" />
		<param name="laser_z_max_value" value="1.0" />
		<param name="update_factor_free" value="0.9"/>
		<param name="update_factor_occupied" value="0.9" />
		<param name="map_update_distance_thresh" value="0.4"/>
		<param name="map_update_angle_thresh" value="0.06" />
		<param name="laser_min_dist" value="1.0"/>
	</node>
	<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 /base_footprint /laser 100" />

When running SLAM the map is updated every time the robot has moved over the **map_update_distance_thresh** value and every time the robot has rotated over **map_update_angle_thresh**. 

The values for **update_factor_free** and **update_factor_occupied** were increased in order to ensure that the mapping can work in dynamic environments since the navigation module will take the dynamics into account.  

2. AMCL

 The mail configuration file can be found inside the saved_slam_launch file. 

    <node pkg="amcl" type="amcl" name="amcl" output="screen">
 		<!-- Publish scans from best pose at a max of 10 Hz -->
 		<param name="odom_model_type" value="diff"/>
 		<param name="odom_alpha5" value="0.1"/>
 		<param name="transform_tolerance" value="0.2" />
 		<param name="gui_publish_rate" value="10.0"/>
 		<param name="laser_max_beams" value="180"/>
 		<param name="min_particles" value="5000"/>
 		<param name="max_particles" value="50000"/>
 		<param name="kld_err" value="0.05"/>
 		<param name="kld_z" value="0.99"/>
 		<param name="odom_alpha1" value="0.2"/>
 		<param name="odom_alpha2" value="0.2"/>
 		<!-- translation std dev, m -->
 		<param name="odom_alpha3" value="0.8"/>
 		<param name="odom_alpha4" value="0.2"/>
 		<param name="laser_z_hit" value="0.5"/>
 		<param name="laser_z_short" value="0.05"/>
 		<param name="laser_z_max" value="0.05"/>
 		<param name="laser_z_rand" value="0.5"/>
 		<param name="laser_sigma_hit" value="0.2"/>
 		<param name="laser_lambda_short" value="0.1"/>
 		<param name="laser_lambda_short" value="0.1"/>
 		<param name="laser_model_type" value="likelihood_field"/>
 		<!-- <param name="laser_model_type" value="beam"/> -->
 		<param name="laser_likelihood_max_dist" value="5.0"/>
 		<param name="update_min_d" value="0.2"/>
 		<param name="update_min_a" value="0.5"/>
 		<param name="odom_frame_id" value="odom"/>
 		<param name="base_frame_id" value="base_footprint"/>
 		<param name="resample_interval" value="0.5"/>
 		<param name="transform_tolerance" value="0.1"/>
 		<param name="recovery_alpha_slow" value="0.0"/>
 		<param name="recovery_alpha_fast" value="0.0"/>
 		<param name="initial_pose_x" value="5.5"/>
 		<param name="initial_pose_y" value="10.0"/>
 		<param name="initial_pose_a" value="-2.6"/>
 	</node>

When running **amcl** the position of the robot is updated every time the robot has moved over the **update_min_d** value and every time the robot has rotated over **update_min_a**. The initial pose x,y and z values should be set as the initial guess for the algorithm and closer to the robot's home position as possible. 

3. RPLidar

The main launch file can be found in [sparc](https://github.com/ami-lab/sparc)/[rpi_catkin_ws](https://github.com/ami-lab/sparc/tree/master/rpi_catkin_ws)/**src**/launch/rplidar.launch. 
 
4. Move_Base

The main configuration files for move_base can be found in: [sparc](https://github.com/ami-lab/sparc)/[master_catkin_ws](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws)/[src](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src)/[pepper_sparc](https://github.com/ami-lab/sparc/tree/master/master_catkin_ws/src/pepper_sparc)/**nav_conf**/.

The parameters depend a lot on the environment. 