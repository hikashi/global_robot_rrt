# Installation guide for the Jackal Exploration

## jackal setup for the local RRT
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
           
## Server Side Installation
1. Download RRT folder into the WS environment

        git clone git@github.com:hikashi/global_robot_rrt.git

2. run the launch file
        
        roslaunch rrt_exploration duo_jackal_exploration.launch 

3. click the 4 points to start exploration.
        ![Instruction](/instruction2.png)
        
        The exploration relies on the correct sequence else rendering with no goal for each of the robot.
        1. Top Left
        2. Bottom Left
        3. Bottom Right
        4. Top Right
        5. Initial Point

## Things to notice
- pending....
