# Installation guide for the Jackal Exploration


## update on the 03 November 2021
- added the new goal assignment function for the robot. 
- new RRT growing features and increase the collisin avoidane function for the data
- new map filtering function (may filter more points resulting less frontier for the robot to travel)
-

## update on the 13 October 2021
- download the local RRT from the src file and update each jackal. Remember to catkin_make /
- download the global RRT for the server pc and catkin_make it.


## jackal setup for the local RRT (for each jackal)
1. Download the RRT folders and files, and place them into the catkin_ws directory

        $ git clone git@github.com:hikashi/single_robot_rrt.git

2. add in the piece of the following code into the robot init launch file after downlaod the RRT file
        
           <arg name="eta" value="1.0"/>
           <arg name="first_robot" value="robot_1"/>
           
           <node pkg="rrt_exploration" type="local_rrt_detector" name="$(arg first_robot)_rrt_detector" output="screen">
             <param name="eta" value="$(arg eta)"/>
             <param name="map_topic" value="/$(arg first_robot)/map"/>
             <param name="robot_frame" value="/$(arg first_robot)/base_link"/>
           </node>
         
 Topic published for the local RRT
 - /detected_points
 - /robot_namespace+"_shapes" (** for visualization purposes)
         
## Server Side Installation
1. Download RRT folder into the WS environment

        git clone git@github.com:hikashi/global_robot_rrt.git

2. modify the parameters of the launch file.

        duo_jackal_exploration.launch
        
3. run the launch file
        
        roslaunch rrt_exploration duo_jackal_exploration.launch 


4. click the 4 points to start exploration.
        ![Instruction](/instruction2.png)
        
        The exploration relies on the correct sequence else rendering with no goal for each of the robot.
        1. Top Left
        2. Bottom Left
        3. Bottom Right
        4. Top Right
        5. Initial Point

## Things to notice
- allow the topics within the multimaster (detected_points).
- add some of the marker topic sent by the individual robot.
- visualization of the RRT at the RVIZ before executing the actual RRT exploration.
