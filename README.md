# RBE550_Project Package
 
1. `roslaunch RBE550_Project turtlebot3_office.launch` to launch burger turtlebot with office map
2. `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` to move robot with keyboard
3. To run the navigation interface in rviz `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml`
   `map.yaml` is located in the world folder. Move the file to `$home` to run the command or change `map_file` address




-----edit my world-----
1. `cd ./catkin_ws/src/RBE550_Project/world`
2. `gazebo world_street.world`
